"""
Decides how ORMatic stores each field of a mapped class.

Every way of storing a field is a :class:`FieldStorage` member, and one
:class:`FieldStorageRule` per member states the complete condition under which it
applies. The conditions exclude each other, so no rule relies on the order in which
the rules are checked, and :class:`FieldClassifier` raises
:class:`~krrood.ormatic.exceptions.AmbiguousFieldStorage` if two rules ever claim the
same field.
"""

from __future__ import annotations

import enum
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from inspect import isclass
from types import NoneType

from typing_extensions import TYPE_CHECKING, ClassVar, List, Type

from krrood.adapters.json_serializer import JSONSerializableTypeRegistry
from krrood.class_diagrams.wrapped_field import WrappedField
from krrood.ormatic.exceptions import AmbiguousFieldStorage

if TYPE_CHECKING:
    from krrood.ormatic.ormatic import ORMatic


class FieldStorage(enum.Enum):
    """
    The ways ORMatic can store a field.
    """

    NOT_STORED = enum.auto()
    """
    The field gets no column.
    """

    TYPE = enum.auto()
    """
    A column holding a class, for fields annotated with ``type`` or ``Type[...]``.
    """

    BUILTIN = enum.auto()
    """
    A column whose type SQLAlchemy derives from the annotation by itself.
    """

    MANY_TO_ONE = enum.auto()
    """
    A foreign key to the table of a mapped class.
    """

    MANY_TO_MANY = enum.auto()
    """
    An association table to the table of a mapped class.
    """

    CUSTOM_TYPE = enum.auto()
    """
    A column of the type the type mappings give for the field's type.
    """

    JSON = enum.auto()
    """
    A JSON column written by krrood's JSON serializer.
    """


# %% rules


@dataclass
class FieldStorageRule(ABC):
    """
    The condition under which a field is stored in one particular way.
    """

    ormatic: ORMatic
    """
    The ORMatic instance whose mapped classes and type mappings the rule consults.
    """

    storage: ClassVar[FieldStorage]
    """
    The way of storing a field that this rule decides on.
    """

    sqlalchemy_builtins: ClassVar[tuple[Type, ...]] = (
        int,
        float,
        str,
        bool,
        bytes,
        NoneType,
    )
    """
    The builtins SQLAlchemy maps to a column type by itself.
    """

    @abstractmethod
    def applies_to(self, wrapped_field: WrappedField) -> bool:
        """
        :param wrapped_field: The field to decide on.
        :return: True if the field is stored the way this rule stands for.
        """

    def is_mapped(self, clazz: Type) -> bool:
        """
        :return: True if a table maps the class.
        """
        return clazz in self.ormatic.mapped_classes

    def has_type_mapping(self, clazz: Type) -> bool:
        """
        :return: True if the type mappings give a column type for the class.
        """
        return clazz in self.ormatic.type_mappings

    def has_json_serializer(self, clazz: Type) -> bool:
        """
        :return: True if a JSON serializer is registered for the class hierarchy.
        """
        return JSONSerializableTypeRegistry().has_type_specific_serializer(clazz)

    def is_stored_as_a_value(self, clazz: Type) -> bool:
        """
        Whether values of the class are stored whole in their owner's row, so a free type
        parameter of the class leaves nothing undecided about how to store them.

        :return: True if no table maps the class and a type mapping or a JSON serializer
            stores it.
        """
        return not self.is_mapped(clazz) and (
            self.has_type_mapping(clazz) or self.has_json_serializer(clazz)
        )

    def cannot_be_stored(self, wrapped_field: WrappedField) -> bool:
        """
        Whether the field's type leaves ORMatic no way to store it: a dictionary, or a
        generic class with free type parameters that nothing in the class diagram could
        fill.
        """
        type_endpoint = wrapped_field.type_endpoint
        if isclass(type_endpoint) and issubclass(type_endpoint, dict):
            return True
        return (
            wrapped_field.is_underspecified_generic
            and isclass(type_endpoint)
            and not self.is_mapped(type_endpoint)
            and not self.is_stored_as_a_value(type_endpoint)
            and not any(
                issubclass(type_endpoint, alternative_mapping.original_class())
                for alternative_mapping in self.ormatic.alternative_mappings
            )
        )

    def holds_a_storable_value(self, wrapped_field: WrappedField) -> bool:
        """
        :return: True if the field holds values rather than classes, and ORMatic can
            store them.
        """
        return not wrapped_field.is_type_type and not self.cannot_be_stored(
            wrapped_field
        )


class NotStoredRule(FieldStorageRule):
    """
    Fields whose type ORMatic has no way to store.
    """

    storage = FieldStorage.NOT_STORED

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        return self.cannot_be_stored(wrapped_field)


class TypeRule(FieldStorageRule):
    """
    Fields that hold classes.
    """

    storage = FieldStorage.TYPE

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        return wrapped_field.is_type_type and not self.cannot_be_stored(wrapped_field)


class BuiltinRule(FieldStorageRule):
    """
    Single values of a builtin that SQLAlchemy maps by itself.
    """

    storage = FieldStorage.BUILTIN

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        return (
            self.holds_a_storable_value(wrapped_field)
            and not wrapped_field.is_container
            and wrapped_field.type_endpoint in self.sqlalchemy_builtins
        )


class ManyToOneRule(FieldStorageRule):
    """
    Single references to an instance of a mapped class.
    """

    storage = FieldStorage.MANY_TO_ONE

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        return (
            self.holds_a_storable_value(wrapped_field)
            and not wrapped_field.is_container
            and self.is_mapped(wrapped_field.type_endpoint)
        )


class ManyToManyRule(FieldStorageRule):
    """
    Collections of instances of a mapped class.
    """

    storage = FieldStorage.MANY_TO_MANY

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        return (
            self.holds_a_storable_value(wrapped_field)
            and wrapped_field.is_container
            and not wrapped_field.is_optional
            and self.is_mapped(wrapped_field.type_endpoint)
        )


class CustomTypeRule(FieldStorageRule):
    """
    Single values of a type that the type mappings give a column type for.
    """

    storage = FieldStorage.CUSTOM_TYPE

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        type_endpoint = wrapped_field.type_endpoint
        return (
            self.holds_a_storable_value(wrapped_field)
            and not wrapped_field.is_container
            and type_endpoint not in self.sqlalchemy_builtins
            and not self.is_mapped(type_endpoint)
            and self.has_type_mapping(type_endpoint)
        )


class JSONRule(FieldStorageRule):
    """
    Values that krrood's JSON serializer writes: collections of builtins or of mapped
    types, and values or collections of a type with a JSON serializer of its own.
    """

    storage = FieldStorage.JSON

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        type_endpoint = wrapped_field.type_endpoint
        if not self.holds_a_storable_value(wrapped_field) or self.is_mapped(
            type_endpoint
        ):
            return False
        if wrapped_field.is_container:
            return (
                wrapped_field.is_collection_of_builtins
                or self.has_type_mapping(type_endpoint)
                or self.has_json_serializer(type_endpoint)
            )
        return (
            type_endpoint not in self.sqlalchemy_builtins
            and not self.has_type_mapping(type_endpoint)
            and self.has_json_serializer(type_endpoint)
        )


# %% classification


@dataclass
class FieldClassifier:
    """
    Decides how each field is stored by asking every rule.
    """

    ormatic: ORMatic
    """
    The ORMatic instance the rules consult.
    """

    rule_types: List[Type[FieldStorageRule]] = field(
        default_factory=lambda: [
            NotStoredRule,
            TypeRule,
            BuiltinRule,
            ManyToOneRule,
            ManyToManyRule,
            CustomTypeRule,
            JSONRule,
        ]
    )
    """
    The rules to ask. Their conditions must exclude each other.
    """

    rules: List[FieldStorageRule] = field(init=False)
    """
    The rules, bound to :attr:`ormatic`.
    """

    def __post_init__(self):
        self.rules = [rule_type(self.ormatic) for rule_type in self.rule_types]

    def classify(self, wrapped_field: WrappedField) -> FieldStorage:
        """
        :param wrapped_field: The field to decide on.
        :return: How the field is stored, or :attr:`FieldStorage.NOT_STORED` if no rule
            applies.
        :raises AmbiguousFieldStorage: If more than one rule applies.
        """
        applying_rules = [rule for rule in self.rules if rule.applies_to(wrapped_field)]
        if len(applying_rules) > 1:
            raise AmbiguousFieldStorage(wrapped_field, applying_rules)
        if not applying_rules:
            return FieldStorage.NOT_STORED
        return applying_rules[0].storage
