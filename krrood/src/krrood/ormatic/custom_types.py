import datetime
import decimal
import enum
import fractions
import importlib
import io
import ipaddress
import json
import pathlib
import re
import zoneinfo
from typing import Generic

import numpy as np
from sqlalchemy import Dialect, TypeDecorator, types
from typing_extensions import Any, Dict, Optional, Type, TypeVar

from krrood.adapters.json_serializer import JSONData
from krrood.ormatic.exceptions import ZoneInfoWithoutKey
from krrood.utils import (
    get_generic_type_parameters,
    module_and_class_name,
    resolve_class_from_full_name,
)

T = TypeVar("T")

# %% base types


class ValueType(TypeDecorator, Generic[T]):
    """
    Column type for values of the Python type bound to its type parameter.

    ..note:: Concrete subclasses must set ``cache_ok`` themselves, as SQLAlchemy only
        reads it from the class that is used as the column type.
    """

    __class_getitem__ = Generic.__dict__["__class_getitem__"]
    """
    SQLAlchemy type classes ignore subscription, so ``Generic``'s is restored to bind the
    type parameter.
    """

    @property
    def python_type(self) -> Type[T]:
        """
        :return: The type of the values this column holds.
        """
        return get_generic_type_parameters(type(self), ValueType)[0]


class TextValueType(ValueType[T]):
    """
    Column type for values that are stored as their text form and rebuilt by calling
    their type on that text.
    """

    impl = types.Text

    def process_bind_param(self, value: Optional[T], dialect: Dialect) -> Optional[str]:
        if value is None:
            return None
        return str(value)

    def process_result_value(
        self, value: Optional[str], dialect: Dialect
    ) -> Optional[T]:
        if value is None:
            return None
        return self.python_type(value)


class JSONObjectType(ValueType[T]):
    """
    Column type for values that are stored as a JSON object of the parts needed to
    rebuild them.
    """

    impl = types.Text

    def to_json_object(self, value: T) -> Dict[str, JSONData]:
        """
        :param value: The value to store.
        :return: The parts needed to rebuild the value.
        """
        raise NotImplementedError

    def from_json_object(self, json_object: Dict[str, JSONData]) -> T:
        """
        :param json_object: The parts written by :meth:`to_json_object`.
        :return: The rebuilt value.
        """
        raise NotImplementedError

    def process_bind_param(self, value: Optional[T], dialect: Dialect) -> Optional[str]:
        if value is None:
            return None
        return json.dumps(self.to_json_object(value))

    def process_result_value(
        self, value: Optional[str], dialect: Dialect
    ) -> Optional[T]:
        if value is None:
            return None
        return self.from_json_object(json.loads(value))


class NumpyScalarType(ValueType[T]):
    """
    Column type for numpy scalars, stored as the matching Python scalar.
    """

    def process_bind_param(self, value: Optional[T], dialect: Dialect) -> Any:
        if value is None:
            return None
        return value.item()

    def process_result_value(self, value: Any, dialect: Dialect) -> Optional[T]:
        if value is None:
            return None
        return self.python_type(value)


# %% types and enums


class TypeType(ValueType[type]):
    """
    Type that casts fields that are of type `type` to their class name on serialization
    and converts the name to the class itself through the globals on load.
    """

    impl = types.String(256)
    cache_ok = True

    def process_bind_param(
        self, value: Optional[Type], dialect: Dialect
    ) -> Optional[str]:
        if value is None:
            return None
        return module_and_class_name(value)

    def process_result_value(self, value: impl, dialect: Dialect) -> Optional[Type]:
        if value is None:
            return None
        return resolve_class_from_full_name(str(value))


class PolymorphicEnumType(ValueType[enum.Enum]):
    """
    Custom type for storing polymorphic enums by their full path and member name.
    """

    impl = types.String(512)
    cache_ok = True

    def process_bind_param(
        self, value: Optional[enum.Enum], dialect: Dialect
    ) -> Optional[str]:
        if value is None:
            return None
        # Store as 'module.path.ClassName.MEMBER_NAME'
        return f"{value.__class__.__module__}.{value.__class__.__name__}.{value.name}"

    def process_result_value(
        self, value: Optional[str], dialect: Dialect
    ) -> Optional[enum.Enum]:
        if value is None:
            return None

        parts = value.rsplit(".", 2)
        module_name = parts[0]
        class_name = parts[1]
        member_name = parts[2]

        module = importlib.import_module(module_name)
        enum_class = getattr(module, class_name)
        return enum_class[member_name]


class JSONDataType(ValueType[JSONData]):
    """
    Type decorator for JSONData that stores JSON without automatic deserialization.

    Unlike regular JSON columns which use the engine's custom json_deserializer (that
    calls from_json()), this type keeps the data as raw JSON dictionaries/lists. This is
    necessary for fields that should be deserialized later in application code.
    """

    impl = types.String
    cache_ok = True

    def process_bind_param(self, value: Optional[JSONData], dialect: Dialect):
        """
        Store the value as-is (already JSON-serializable).
        """
        if value is None:
            return None
        return json.dumps(value)

    def process_result_value(self, value: impl, dialect: Dialect):
        """
        Return the value as-is (raw JSON, not deserialized).
        """
        if value is None:
            return None
        return json.loads(value)


# %% dates and times


class DateTimeType(ValueType[datetime.datetime]):
    """
    Column type for points in time, stored as ISO 8601 text of a fixed width.

    Timezone-aware values are stored as the same instant in UTC and are loaded back in
    UTC, so values of different offsets compare and sort correctly in the database.
    Naive values stay naive.
    """

    impl = types.String(32)
    cache_ok = True

    def process_bind_param(
        self, value: Optional[datetime.datetime], dialect: Dialect
    ) -> Optional[str]:
        if value is None:
            return None
        if value.tzinfo is not None:
            value = value.astimezone(datetime.timezone.utc)
        return value.isoformat(timespec="microseconds")

    def process_result_value(
        self, value: Optional[str], dialect: Dialect
    ) -> Optional[datetime.datetime]:
        if value is None:
            return None
        return datetime.datetime.fromisoformat(value)


class TimezoneKey(enum.StrEnum):
    """
    Keys of the JSON object a fixed-offset timezone is stored as.
    """

    OFFSET = "offset"
    """
    The offset from UTC in seconds.
    """

    NAME = "name"
    """
    The name the timezone was created with, if any.
    """


class TimezoneType(JSONObjectType[datetime.timezone]):
    """
    Column type for fixed-offset timezones, stored with the name they were created with.
    """

    cache_ok = True

    def to_json_object(self, value: datetime.timezone) -> Dict[str, JSONData]:
        offset, *name = value.__getinitargs__()
        return {
            TimezoneKey.OFFSET: offset.total_seconds(),
            TimezoneKey.NAME: name[0] if name else None,
        }

    def from_json_object(self, json_object: Dict[str, JSONData]) -> datetime.timezone:
        offset = datetime.timedelta(seconds=json_object[TimezoneKey.OFFSET])
        name = json_object[TimezoneKey.NAME]
        if name is None:
            return datetime.timezone(offset)
        return datetime.timezone(offset, name)


class ZoneInfoType(TextValueType[zoneinfo.ZoneInfo]):
    """
    Column type for IANA timezones, stored by their key.
    """

    impl = types.String(256)
    cache_ok = True

    def process_bind_param(
        self, value: Optional[zoneinfo.ZoneInfo], dialect: Dialect
    ) -> Optional[str]:
        if value is not None and value.key is None:
            raise ZoneInfoWithoutKey(value)
        return super().process_bind_param(value, dialect)


# %% numbers


class DecimalType(TextValueType[decimal.Decimal]):
    """
    Column type for decimal numbers, stored as text so that no digit is lost on
    databases without an exact decimal type.
    """

    cache_ok = True


class FractionType(TextValueType[fractions.Fraction]):
    """
    Column type for rational numbers, stored as text such as ``1/3``.
    """

    cache_ok = True


class ComplexType(TextValueType[complex]):
    """
    Column type for complex numbers, stored as text such as ``(1.5-2j)``.
    """

    cache_ok = True


# %% paths and network addresses


class PathType(TextValueType[pathlib.Path]):
    """
    Type decorator for pathlib.Path objects.
    """

    cache_ok = True


class PurePathKey(enum.StrEnum):
    """
    Keys of the JSON object a path is stored as.
    """

    TYPE = "type"
    """
    The full name of the path's class, which decides how the path is read.
    """

    PATH = "path"
    """
    The path as text.
    """


class PurePathType(JSONObjectType[pathlib.PurePath]):
    """
    Column type for paths of any flavour, stored with their class.
    """

    cache_ok = True

    def to_json_object(self, value: pathlib.PurePath) -> Dict[str, JSONData]:
        return {
            PurePathKey.TYPE: module_and_class_name(type(value)),
            PurePathKey.PATH: str(value),
        }

    def from_json_object(self, json_object: Dict[str, JSONData]) -> pathlib.PurePath:
        path_type = resolve_class_from_full_name(json_object[PurePathKey.TYPE])
        return path_type(json_object[PurePathKey.PATH])


class IPv4AddressType(TextValueType[ipaddress.IPv4Address]):
    """
    Column type for IPv4 addresses.
    """

    impl = types.String(15)
    cache_ok = True


class IPv6AddressType(TextValueType[ipaddress.IPv6Address]):
    """
    Column type for IPv6 addresses.
    """

    impl = types.String(45)
    cache_ok = True


class IPv4NetworkType(TextValueType[ipaddress.IPv4Network]):
    """
    Column type for IPv4 networks, stored in CIDR notation.
    """

    impl = types.String(18)
    cache_ok = True


class IPv6NetworkType(TextValueType[ipaddress.IPv6Network]):
    """
    Column type for IPv6 networks, stored in CIDR notation.
    """

    impl = types.String(49)
    cache_ok = True


# %% sequences and patterns


class ByteArrayType(ValueType[bytearray]):
    """
    Column type for mutable byte sequences.
    """

    impl = types.LargeBinary
    cache_ok = True

    def process_bind_param(
        self, value: Optional[bytearray], dialect: Dialect
    ) -> Optional[bytes]:
        if value is None:
            return None
        return bytes(value)

    def process_result_value(
        self, value: Optional[bytes], dialect: Dialect
    ) -> Optional[bytearray]:
        if value is None:
            return None
        return bytearray(value)


class RangeKey(enum.StrEnum):
    """
    Keys of the JSON object a range or slice is stored as.
    """

    START = "start"
    """
    The first index.
    """

    STOP = "stop"
    """
    The index the range or slice ends before.
    """

    STEP = "step"
    """
    The distance between consecutive indices.
    """


class RangeType(JSONObjectType[range]):
    """
    Column type for ranges of integers.
    """

    cache_ok = True

    def to_json_object(self, value: range) -> Dict[str, JSONData]:
        return {
            RangeKey.START: value.start,
            RangeKey.STOP: value.stop,
            RangeKey.STEP: value.step,
        }

    def from_json_object(self, json_object: Dict[str, JSONData]) -> range:
        return range(
            json_object[RangeKey.START],
            json_object[RangeKey.STOP],
            json_object[RangeKey.STEP],
        )


class SliceType(JSONObjectType[slice]):
    """
    Column type for slices whose bounds are JSON values, such as integers or None.
    """

    cache_ok = True

    def to_json_object(self, value: slice) -> Dict[str, JSONData]:
        return {
            RangeKey.START: value.start,
            RangeKey.STOP: value.stop,
            RangeKey.STEP: value.step,
        }

    def from_json_object(self, json_object: Dict[str, JSONData]) -> slice:
        return slice(
            json_object[RangeKey.START],
            json_object[RangeKey.STOP],
            json_object[RangeKey.STEP],
        )


class PatternKey(enum.StrEnum):
    """
    Keys of the JSON object a compiled regular expression is stored as.
    """

    PATTERN = "pattern"
    """
    The source of the regular expression.
    """

    FLAGS = "flags"
    """
    The flags the regular expression was compiled with.
    """


class PatternType(JSONObjectType[re.Pattern]):
    """
    Column type for compiled regular expressions over text.
    """

    cache_ok = True

    def to_json_object(self, value: re.Pattern) -> Dict[str, JSONData]:
        return {PatternKey.PATTERN: value.pattern, PatternKey.FLAGS: value.flags}

    def from_json_object(self, json_object: Dict[str, JSONData]) -> re.Pattern:
        return re.compile(
            json_object[PatternKey.PATTERN], json_object[PatternKey.FLAGS]
        )


# %% numpy


class NumpyType(ValueType[np.ndarray]):
    """
    Type decorator for numpy arrays, stored as raw float64 bytes.

    ..note:: The shape is not stored, so arrays are read back as one-dimensional.
        :class:`NumpyArrayType` keeps the shape and dtype.
    """

    impl = types.LargeBinary(4 * 1024 * 1024 * 1024 - 1)
    cache_ok = True

    def process_bind_param(
        self, value: Optional[np.ndarray], dialect: Dialect
    ) -> Optional[bytes]:
        if value is None:
            return None
        array = np.asarray(value, dtype=np.float64)
        return array.tobytes(order="C")

    def process_result_value(
        self, value: Optional[bytes], dialect: Dialect
    ) -> Optional[np.ndarray]:
        if value is None:
            return None
        return np.frombuffer(value, dtype=np.float64)


class NumpyArrayType(ValueType[np.ndarray]):
    """
    Column type for numpy arrays, stored in the ``.npy`` format so that their shape and
    dtype are kept.

    ..note:: Arrays of Python objects cannot be stored.
    """

    impl = types.LargeBinary(4 * 1024 * 1024 * 1024 - 1)
    cache_ok = True

    def process_bind_param(
        self, value: Optional[np.ndarray], dialect: Dialect
    ) -> Optional[bytes]:
        if value is None:
            return None
        buffer = io.BytesIO()
        np.save(buffer, value, allow_pickle=False)
        return buffer.getvalue()

    def process_result_value(
        self, value: Optional[bytes], dialect: Dialect
    ) -> Optional[np.ndarray]:
        if value is None:
            return None
        return np.load(io.BytesIO(value), allow_pickle=False)


class NumpyFloat16Type(NumpyScalarType[np.float16]):
    """
    Column type for numpy half precision floats.
    """

    impl = types.Float
    cache_ok = True


class NumpyFloat32Type(NumpyScalarType[np.float32]):
    """
    Column type for numpy single precision floats.
    """

    impl = types.Float
    cache_ok = True


class NumpyFloat64Type(NumpyScalarType[np.float64]):
    """
    Column type for numpy double precision floats.
    """

    impl = types.Float
    cache_ok = True


class NumpyInt8Type(NumpyScalarType[np.int8]):
    """
    Column type for numpy 8 bit integers.
    """

    impl = types.Integer
    cache_ok = True


class NumpyInt16Type(NumpyScalarType[np.int16]):
    """
    Column type for numpy 16 bit integers.
    """

    impl = types.Integer
    cache_ok = True


class NumpyInt32Type(NumpyScalarType[np.int32]):
    """
    Column type for numpy 32 bit integers.
    """

    impl = types.Integer
    cache_ok = True


class NumpyInt64Type(NumpyScalarType[np.int64]):
    """
    Column type for numpy 64 bit integers.
    """

    impl = types.BigInteger
    cache_ok = True


class NumpyUInt8Type(NumpyScalarType[np.uint8]):
    """
    Column type for numpy 8 bit unsigned integers.
    """

    impl = types.Integer
    cache_ok = True


class NumpyUInt16Type(NumpyScalarType[np.uint16]):
    """
    Column type for numpy 16 bit unsigned integers.
    """

    impl = types.Integer
    cache_ok = True


class NumpyUInt32Type(NumpyScalarType[np.uint32]):
    """
    Column type for numpy 32 bit unsigned integers, which need a 64 bit column.
    """

    impl = types.BigInteger
    cache_ok = True


class NumpyBoolType(NumpyScalarType[np.bool_]):
    """
    Column type for numpy booleans.
    """

    impl = types.Boolean
    cache_ok = True


class Datetime64Type(TextValueType[np.datetime64]):
    """
    Column type for numpy points in time, stored as ISO 8601 text whose precision keeps
    the time unit.
    """

    impl = types.String(64)
    cache_ok = True
