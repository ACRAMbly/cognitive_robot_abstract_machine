import pytest

from krrood.class_diagrams.class_diagram import ClassDiagram
from krrood.class_diagrams.wrapped_field import WrappedField
from krrood.ormatic.exceptions import AmbiguousFieldStorage
from krrood.ormatic.field_storage import (
    BuiltinRule,
    FieldClassifier,
    FieldStorage,
    FieldStorageRule,
)
from krrood.ormatic.ormatic import ORMatic
from ..dataset.field_storage_classes import FieldsOfEveryStorage, StorageLeaf

# %% helpers


@pytest.fixture
def ormatic() -> ORMatic:
    return ORMatic(ClassDiagram([StorageLeaf, FieldsOfEveryStorage]))


def field_named(ormatic: ORMatic, name: str) -> WrappedField:
    """
    :return: The field of :class:`FieldsOfEveryStorage` with the given name.
    """
    wrapped_class = ormatic.class_dependency_graph.get_wrapped_class(
        FieldsOfEveryStorage
    )
    return next(f for f in wrapped_class.fields if f.field.name == name)


# %% classification


@pytest.mark.parametrize(
    "field_name, storage",
    [
        ("number", FieldStorage.BUILTIN),
        ("kind", FieldStorage.TYPE),
        ("leaf", FieldStorage.MANY_TO_ONE),
        ("leaves", FieldStorage.MANY_TO_MANY),
        ("day", FieldStorage.CUSTOM_TYPE),
        ("complex_number", FieldStorage.CUSTOM_TYPE),
        ("span", FieldStorage.JSON),
        ("numbers", FieldStorage.JSON),
        ("days", FieldStorage.JSON),
        ("lookup", FieldStorage.NOT_STORED),
        ("anything", FieldStorage.NOT_STORED),
    ],
)
def test_field_is_stored_the_way_its_type_calls_for(ormatic, field_name, storage):
    assert (
        ormatic.field_classifier.classify(field_named(ormatic, field_name)) is storage
    )


# %% overlapping rules


class RuleClaimingEveryField(FieldStorageRule):
    """
    A rule whose condition overlaps with every other rule.
    """

    storage = FieldStorage.JSON

    def applies_to(self, wrapped_field: WrappedField) -> bool:
        return True


def test_field_claimed_by_two_rules_is_rejected(ormatic):
    classifier = FieldClassifier(
        ormatic, rule_types=[BuiltinRule, RuleClaimingEveryField]
    )

    with pytest.raises(AmbiguousFieldStorage):
        classifier.classify(field_named(ormatic, "number"))
