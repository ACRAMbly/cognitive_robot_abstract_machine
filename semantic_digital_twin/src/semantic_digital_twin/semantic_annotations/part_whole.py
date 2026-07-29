"""
Metadata and field discovery for the part-whole relation between semantic annotations.

This module holds only the vocabulary of the relation, so both the annotation mixins
that declare part-whole fields and the specification API that fills them can depend on
it without depending on each other.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

from krrood.class_diagrams.class_diagram import WrappedClass
from krrood.class_diagrams.wrapped_field import WrappedField
from krrood.patterns.field_metadata import FieldMetadata
from typing_extensions import TYPE_CHECKING, Type

if TYPE_CHECKING:
    from semantic_digital_twin.semantic_annotations.mixins import PartWholeRelationship


# %% relation metadata


@dataclass
class IsPartWholeRelationship(FieldMetadata):
    """
    Marks a field as holding a structural *part* of its owner (the part-whole relation).

    The relation is signalled by the presence of an instance of this class in the
    field's ``metadata`` mapping (attach it with :meth:`~FieldMetadata.as_dict`), and
    the instance describes how mounting a part into that field affects the whole.
    """

    removes_part_geometry_from_whole: bool = False
    """
    Whether mounting a part into this field removes the part's volume from the whole's
    collision and visual geometry.

    This is a property of the relation rather than of the part: the same
    :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.EntryWay`
    cuts the wall it is an aperture of, but not the door whose passage it marks.
    """


# %% field discovery


@lru_cache(maxsize=None)
def wrapped_part_whole_relationship_fields(
    cls: Type[PartWholeRelationship],
) -> list[WrappedField]:
    """
    Filters the fields of cls for all fields marked as a part-whole relationship (by
    carrying an :class:`IsPartWholeRelationship` in their metadata), and returns them as
    a Wrapped Class.
    """
    return [
        wrapped_part_whole_relationship_field
        for wrapped_part_whole_relationship_field in WrappedClass(cls).fields
        if IsPartWholeRelationship.of_field(
            wrapped_part_whole_relationship_field.clazz.clazz,
            wrapped_part_whole_relationship_field.name,
        )
        is not None
    ]


def part_whole_relationship_of(wrapped_field: WrappedField) -> IsPartWholeRelationship:
    """
    Read the part-whole metadata off a wrapped part-whole relationship field.

    :param wrapped_field: A field known to carry :class:`IsPartWholeRelationship`.
    :return: The metadata describing the relation.
    """
    return IsPartWholeRelationship.of_field(
        wrapped_field.clazz.clazz, wrapped_field.field.name
    )
