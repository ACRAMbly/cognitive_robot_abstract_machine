"""
Locating the parts an annotation's part-whole relationships call for in the shape of its
root body, as faces of that shape.

A :class:`PartLocator` answers a :class:`PartQuery` -- a shape and the type of annotation
it belongs to -- with the faces of each part it finds. It may run on this machine or on
a server, which is why both the query and the answer are plain data.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
import trimesh
from krrood.class_diagrams.class_diagram import WrappedClass
from krrood.class_diagrams.wrapped_field import WrappedField
from krrood.entity_query_language.utils import camel_case_to_words
from numpy.typing import NDArray
from scipy.spatial import cKDTree
from typing_extensions import Optional, Type

from semantic_digital_twin.exceptions import UnknownPartWholeRelationshipField
from semantic_digital_twin.pipeline.part_finding.splitting import PartFinder, ShapeSplit
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    PartWholeRelationship,
)
from semantic_digital_twin.semantic_annotations.part_whole import (
    IsPartWholeRelationship,
)

# %% the parts an annotation calls for


@dataclass
class PartRelationship:
    """
    A part-whole relationship of an annotation type whose parts have a body of their
    own, so that they can be split off the shape of the annotation's root body.
    """

    relationship_field: WrappedField
    """
    The field of the annotation type that holds the parts.
    """

    @classmethod
    def all_of(cls, whole_type: Type[PartWholeRelationship]) -> list[PartRelationship]:
        """
        :param whole_type: A type of annotation.
        :return: Its part-whole relationships whose parts have a body of their own, in
            declaration order.
        """
        return [
            cls(relationship_field=wrapped_field)
            for wrapped_field in WrappedClass(whole_type).fields_with_metadata(
                IsPartWholeRelationship
            )
            if issubclass(wrapped_field.type_endpoint, HasRootBody)
        ]

    @classmethod
    def named(
        cls, whole_type: Type[PartWholeRelationship], name: str
    ) -> PartRelationship:
        """
        :param whole_type: A type of annotation.
        :param name: The name of one of its part-whole relationship fields.
        :return: That relationship.
        """
        relationships = cls.all_of(whole_type)
        for relationship in relationships:
            if relationship.name == name:
                return relationship
        raise UnknownPartWholeRelationshipField(
            whole_type, name, [relationship.name for relationship in relationships]
        )

    @property
    def name(self) -> str:
        """
        :return: The name of the field that holds the parts.
        """
        return self.relationship_field.field.name

    @property
    def part_type(self) -> Type[HasRootBody]:
        """
        :return: The type of annotation each part is.
        """
        return self.relationship_field.type_endpoint

    @property
    def part_name(self) -> str:
        """
        :return: What a part is called in plain words, such as ``"handle"``.
        """
        return camel_case_to_words(self.part_type.__name__)

    @property
    def holds_many(self) -> bool:
        """
        :return: Whether the relationship holds any number of parts rather than one.
        """
        return self.relationship_field.is_many_to_many_relationship

    @property
    def may_be_empty(self) -> bool:
        """
        :return: Whether an annotation may lack the parts of this relationship, which is
            the case for an optional part and for any number of parts.
        """
        return self.relationship_field.is_optional or self.holds_many

    def is_filled_in(self, whole: PartWholeRelationship) -> bool:
        """
        :param whole: An annotation of the type this relationship belongs to.
        :return: Whether the annotation has a part in this relationship already.
        """
        value = getattr(whole, self.name)
        return bool(value) if self.holds_many else value is not None


# %% the question and its answer


@dataclass
class PartQuery:
    """
    The question which faces of a shape belong to the parts an annotation's part-whole
    relationships call for.
    """

    vertices: NDArray[np.float64]
    """
    The shape's vertices, as rows of x, y and z in the frame of the annotation's root
    body, its z-axis pointing up.
    """

    faces: NDArray[np.int64]
    """
    The shape's triangles, as rows of three indices into :attr:`vertices`.
    """

    whole_type: Type[PartWholeRelationship]
    """
    The type of annotation the shape belongs to.
    """

    relationship_names: list[str]
    """
    The names of the part-whole relationships whose parts are wanted.
    """

    @classmethod
    def of_shape(
        cls,
        shape: trimesh.Trimesh,
        whole_type: Type[PartWholeRelationship],
        relationship_names: list[str],
    ) -> PartQuery:
        """
        :param shape: The shape of an annotation's root body, in its frame.
        :param whole_type: The type of the annotation.
        :param relationship_names: The part-whole relationships whose parts are wanted.
        :return: The query for those parts.
        """
        return cls(
            vertices=np.asarray(shape.vertices, dtype=np.float64),
            faces=np.asarray(shape.faces, dtype=np.int64),
            whole_type=whole_type,
            relationship_names=list(relationship_names),
        )

    @cached_property
    def shape(self) -> trimesh.Trimesh:
        """
        :return: The shape, its faces in the order of :attr:`faces`.
        """
        return trimesh.Trimesh(vertices=self.vertices, faces=self.faces, process=False)

    @property
    def relationships(self) -> list[PartRelationship]:
        """
        :return: The part-whole relationships whose parts are wanted.
        """
        return [
            PartRelationship.named(self.whole_type, name)
            for name in self.relationship_names
        ]

    @property
    def object_name(self) -> str:
        """
        :return: What the annotation is called in plain words, such as ``"mug"``.
        """
        return camel_case_to_words(self.whole_type.__name__)


@dataclass
class LocatedPart:
    """
    A part found in the shape of a :class:`PartQuery`.
    """

    relationship_name: str
    """
    The name of the part-whole relationship the part belongs to.
    """

    faces: NDArray[np.int64]
    """
    The indices of the shape's faces that make up the part.
    """

    confidence: float
    """
    How sure the locator is that these faces are the part, from 0 to 1.
    """


class PartLocator(ABC):
    """
    Finds which faces of a shape belong to the parts of an annotation.
    """

    @abstractmethod
    def locate(self, query: PartQuery) -> list[LocatedPart]:
        """
        :param query: The shape and the parts wanted.
        :return: The parts found: for a relationship that holds a single part at most
            one, for one that holds any number each separate piece. A part that is not
            found is left out.
        """


@dataclass
class KnownPartLocator(PartLocator):
    """
    Answers every query with the same parts: a stand-in for a real locator wherever the
    answer is known beforehand, such as in tests.
    """

    parts: list[LocatedPart] = field(default_factory=list)
    """
    The parts every query is answered with.
    """

    queries: list[PartQuery] = field(default_factory=list, init=False)
    """
    The queries asked so far, oldest first.
    """

    def locate(self, query: PartQuery) -> list[LocatedPart]:
        self.queries.append(query)
        return list(self.parts)


# %% splitting a located part off


@dataclass
class LocatedFacesFinder(PartFinder):
    """
    Finds a part whose faces were located in a shape before, either in that shape or in
    what is left of it after other parts were split off, by the faces' centers.
    """

    face_centers: NDArray[np.float64]
    """
    The centers of the part's faces, as rows of x, y and z.
    """

    tolerance: float = 1e-6
    """
    How far a face's center may lie from one of :attr:`face_centers` to still be one of
    the part's faces, in meters.
    """

    def split(self, shape: trimesh.Trimesh) -> Optional[ShapeSplit]:
        distances, _ = cKDTree(self.face_centers).query(shape.triangles_center)
        return ShapeSplit.by_faces(shape, distances <= self.tolerance)
