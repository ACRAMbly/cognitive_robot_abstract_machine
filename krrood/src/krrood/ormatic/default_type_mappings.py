from __future__ import annotations

import datetime
import decimal
import enum
import fractions
import ipaddress
import pathlib
import uuid
import zoneinfo
from dataclasses import dataclass
from types import NoneType

import numpy as np
import sqlalchemy
from typing_extensions import Any, Type

from krrood.adapters.json_serializer import JSONData, SubclassJSONSerializer
from krrood.ormatic.custom_types import (
    ByteArrayType,
    ComplexType,
    DateTimeType,
    Datetime64Type,
    FractionType,
    IPv4AddressType,
    IPv4NetworkType,
    IPv6AddressType,
    IPv6NetworkType,
    JSONDataType,
    NumpyArrayType,
    NumpyBoolType,
    NumpyFloat16Type,
    NumpyFloat32Type,
    NumpyFloat64Type,
    NumpyInt16Type,
    NumpyInt32Type,
    NumpyInt64Type,
    NumpyInt8Type,
    NumpyUInt16Type,
    NumpyUInt32Type,
    NumpyUInt8Type,
    PathType,
    PolymorphicEnumType,
    TypeType,
    ZoneInfoType,
)


@dataclass(frozen=True)
class TypeMapping:
    """
    A Python type and the SQLAlchemy column type that stores its values.
    """

    python_type: Any
    """
    The type of the values, a class or a typing construct such as ``typing.Type``.
    """

    column_type: Type[sqlalchemy.types.TypeEngine]
    """
    The column type that stores the values.
    """


class DefaultTypeMapping(enum.Enum):
    """
    The type mappings ORMatic uses for every type the caller gives no mapping for.
    """

    # %% types, enums, identifiers and JSON

    TYPING_TYPE = TypeMapping(Type, TypeType)
    TYPE = TypeMapping(type, TypeType)
    ENUM = TypeMapping(enum.Enum, PolymorphicEnumType)
    SUBCLASS_JSON_SERIALIZER = TypeMapping(SubclassJSONSerializer, sqlalchemy.JSON)
    UUID = TypeMapping(uuid.UUID, sqlalchemy.UUID)
    PATH = TypeMapping(pathlib.Path, PathType)
    JSON_DATA = TypeMapping(JSONData, JSONDataType)
    NONE = TypeMapping(NoneType, TypeType)

    # %% dates and times

    DATE = TypeMapping(datetime.date, sqlalchemy.Date)
    TIME = TypeMapping(datetime.time, sqlalchemy.Time)
    TIMEDELTA = TypeMapping(datetime.timedelta, sqlalchemy.Interval)
    DATETIME = TypeMapping(datetime.datetime, DateTimeType)
    ZONE_INFO = TypeMapping(zoneinfo.ZoneInfo, ZoneInfoType)

    # %% numbers

    DECIMAL = TypeMapping(decimal.Decimal, sqlalchemy.Numeric)
    FRACTION = TypeMapping(fractions.Fraction, FractionType)
    COMPLEX = TypeMapping(complex, ComplexType)

    # %% network addresses

    IPV4_ADDRESS = TypeMapping(ipaddress.IPv4Address, IPv4AddressType)
    IPV6_ADDRESS = TypeMapping(ipaddress.IPv6Address, IPv6AddressType)
    IPV4_NETWORK = TypeMapping(ipaddress.IPv4Network, IPv4NetworkType)
    IPV6_NETWORK = TypeMapping(ipaddress.IPv6Network, IPv6NetworkType)

    # %% byte sequences

    BYTE_ARRAY = TypeMapping(bytearray, ByteArrayType)

    # %% numpy

    NUMPY_ARRAY = TypeMapping(np.ndarray, NumpyArrayType)
    NUMPY_FLOAT16 = TypeMapping(np.float16, NumpyFloat16Type)
    NUMPY_FLOAT32 = TypeMapping(np.float32, NumpyFloat32Type)
    NUMPY_FLOAT64 = TypeMapping(np.float64, NumpyFloat64Type)
    NUMPY_INT8 = TypeMapping(np.int8, NumpyInt8Type)
    NUMPY_INT16 = TypeMapping(np.int16, NumpyInt16Type)
    NUMPY_INT32 = TypeMapping(np.int32, NumpyInt32Type)
    NUMPY_INT64 = TypeMapping(np.int64, NumpyInt64Type)
    NUMPY_UINT8 = TypeMapping(np.uint8, NumpyUInt8Type)
    NUMPY_UINT16 = TypeMapping(np.uint16, NumpyUInt16Type)
    NUMPY_UINT32 = TypeMapping(np.uint32, NumpyUInt32Type)
    NUMPY_BOOL = TypeMapping(np.bool_, NumpyBoolType)
    NUMPY_DATETIME64 = TypeMapping(np.datetime64, Datetime64Type)
