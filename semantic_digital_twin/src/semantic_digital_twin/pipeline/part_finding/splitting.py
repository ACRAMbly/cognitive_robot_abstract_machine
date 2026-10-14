"""
Splitting a part, such as a handle, off the shape of an object into a body of its own,
annotated as that part of the object's annotation.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import cached_property
from itertools import count

import numpy as np
import trimesh
from numpy.typing import NDArray
from trimesh.visual.texture import TextureVisuals
from typing_extensions import TYPE_CHECKING, Callable, Optional, Type, TypeVar

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.pipeline.pipeline import Step
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Mesh, Shape
from semantic_digital_twin.world_description.inertial_properties import Inertial
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from semantic_digital_twin.semantic_annotations.mixins import (
        HasRootBody,
        PartWholeRelationship,
    )
    from semantic_digital_twin.world import World

T = TypeVar("T")

# %% splitting a shape


@dataclass
class ShapeSplit:
    """
    A shape cut into a part and the rest of it.
    """

    part: trimesh.Trimesh
    """
    The part.
    """

    rest: trimesh.Trimesh
    """
    The rest of the shape.
    """

    @classmethod
    def by_faces(
        cls, shape: trimesh.Trimesh, in_part: NDArray[np.bool_]
    ) -> Optional[ShapeSplit]:
        """
        :param shape: The shape to split.
        :param in_part: For each face of ``shape``, whether it belongs to the part.
        :return: The faces of the part and the remaining faces; ``None`` if either
            would be empty.
        """
        if not in_part.any() or in_part.all():
            return None
        return cls(
            part=shape.submesh([np.flatnonzero(in_part)], append=True),
            rest=shape.submesh([np.flatnonzero(~in_part)], append=True),
        )


class PartFinder(ABC):
    """
    Finds the piece of an object's shape that is one of its parts.
    """

    @abstractmethod
    def split(self, shape: trimesh.Trimesh) -> Optional[ShapeSplit]:
        """
        :param shape: The object's shape, in its root body's frame.
        :return: The shape split into the part and the rest; ``None`` if the shape has
            no such part.
        """


# %% splitting a part off a body


@dataclass
class PartSplitter:
    """
    Splits a part off the shape of a body into a new body fixed to it, optionally
    annotating the new body as a part of the body's annotation.

    The body keeps its inertial properties, which describe the whole object; each split
    is one block of the world's modification history.
    """

    part_type: Type[HasRootBody]
    """
    The type of annotation the part is given, which also names the part's body.
    """

    finder: PartFinder
    """
    Finds the part in a body's shape.
    """

    def part_of(self, whole: PartWholeRelationship) -> Optional[HasRootBody]:
        """
        The part of one annotation, split off its shape the first time it is asked for.

        :param whole: The annotation whose part is wanted.
        :return: The part the annotation already has, if any; otherwise the part split
            off its shape, ``None`` if there is none to split off.
        """
        existing = [part for part in whole.parts if isinstance(part, self.part_type)]
        if existing:
            return existing[0]
        return self.split(whole)

    def split(
        self, whole: PartWholeRelationship, field_name: str = ""
    ) -> Optional[HasRootBody]:
        """
        Split the part off the shape of one annotation's root body and add it to the
        annotation, whether or not the annotation has such a part already.

        :param whole: The annotation whose root body's shape is split.
        :param field_name: The part-whole relationship field of the annotation the part
            is added to; found by the part's type if not given.
        :return: The part, added to the annotation and its world; ``None`` if the finder
            finds none or the root body has no shape.
        """
        cut = self._cut(whole.root)
        if cut is None:
            return None
        world = whole.root._world
        with world.modify_world():
            part = self.part_type(root=cut.apply())
            world.add_semantic_annotation(part)
            whole.add(part, field_name=field_name)
        return part

    def split_body(self, body: Body) -> Optional[Body]:
        """
        Split the part off the shape of a body, without annotating either of them.

        :param body: The body whose shape is split; it keeps the rest of its shape.
        :return: The new body of the part, fixed to ``body`` in its world; ``None`` if
            the finder finds none or the body has no shape.
        """
        cut = self._cut(body)
        if cut is None:
            return None
        with body._world.modify_world():
            return cut.apply()

    def _cut(self, body: Body) -> Optional[CutBody]:
        """
        :param body: The body whose shape is split.
        :return: The part's body and the rest of the shape, not yet in the world;
            ``None`` if the finder finds no part or the body has no shape.
        """
        shapes = body.visual or body.collision
        if not shapes:
            return None
        colored = ColoredShape(shapes=shapes)
        split = self.finder.split(colored.mesh)
        if split is None:
            return None
        part = Body(name=self._part_name(body), inertial=Inertial.negligible())
        part.visual = colored.piece(split.part, part)
        rest = colored.piece(split.rest, body)
        if body.collision:
            part.collision = colored.piece(split.part, part)
            rest_collision = colored.piece(split.rest, body)
        else:
            rest_collision = ShapeCollection()
        return CutBody(body=body, part=part, visual=rest, collision=rest_collision)

    def _part_name(self, body: Body) -> PrefixedName:
        """
        :param body: The body the part is split off.
        :return: The part's type, under the body's name, numbered if the world has an
            entity of that name already.
        """
        taken = {
            str(entity.name) for entity in body._world.kinematic_structure_entities
        }
        base = self.part_type.__name__.lower()
        names = (
            PrefixedName(f"{base}_{number}" if number else base, str(body.name))
            for number in count()
        )
        return next(name for name in names if str(name) not in taken)


@dataclass
class CutBody:
    """
    A body's shape cut into the part's body and the rest, to be applied to the world.
    """

    body: Body
    """
    The body whose shape is cut.
    """

    part: Body
    """
    The part's body, not yet in the world.
    """

    visual: ShapeCollection
    """
    What the body looks like without the part.
    """

    collision: ShapeCollection
    """
    How the body collides without the part.
    """

    def apply(self) -> Body:
        """
        Give the body the rest of its shape and fix the part's body to it.

        Must be called while the world is being modified.

        :return: The part's body.
        """
        self.body.update_shapes(visual=self.visual, collision=self.collision)
        self.body._world.add_connection(
            FixedConnection(parent=self.body, child=self.part)
        )
        return self.part


@dataclass
class ColoredShape:
    """
    The shapes of a body as one mesh whose faces keep the colors of the shapes, so that
    the pieces cut from it can be colored like it.
    """

    shapes: ShapeCollection
    """
    The shapes of the body.
    """

    @cached_property
    def mesh(self) -> trimesh.Trimesh:
        """
        :return: The shapes as one mesh in the body's frame, each face colored as its
            shape looks there.
        """
        meshes = []
        for shape in self.shapes:
            mesh = shape.mesh.copy()
            if isinstance(mesh.visual, TextureVisuals):
                mesh.visual = mesh.visual.to_color()
            if mesh.visual.kind is None:
                mesh.visual.face_colors = trimesh.visual.color.to_rgba(
                    shape.color.to_rgba()
                )
            mesh.apply_transform(shape.origin.to_np())
            meshes.append(mesh)
        return trimesh.util.concatenate(meshes)

    def piece(self, mesh: trimesh.Trimesh, body: Body) -> ShapeCollection:
        """
        :param mesh: A piece cut from :attr:`mesh`, in the frame of ``body``.
        :param body: The body the piece belongs to.
        :return: The piece as the shapes of ``body``, colored like the faces it was cut
            from, and with the surface finish and simulator properties that all of
            :attr:`shapes` share.
        """
        mesh = mesh.copy()
        _, _, faces = trimesh.proximity.closest_point(self.mesh, mesh.triangles_center)
        mesh.visual = trimesh.visual.ColorVisuals(
            mesh, face_colors=self.mesh.visual.face_colors[faces]
        )
        shape = Mesh.from_trimesh(
            mesh=mesh,
            origin=HomogeneousTransformationMatrix(reference_frame=body),
            finish=self.shared(lambda shape: shape.finish),
        )
        shape.simulator_additional_properties = list(
            self.shared(lambda shape: shape.simulator_additional_properties) or []
        )
        return ShapeCollection([shape], reference_frame=body)

    def shared(self, attribute: Callable[[Shape], T]) -> Optional[T]:
        """
        :param attribute: Reads an attribute of a shape.
        :return: The attribute's value, if all :attr:`shapes` have the same; otherwise
            ``None``.
        """
        values = [attribute(shape) for shape in self.shapes]
        return values[0] if all(value == values[0] for value in values) else None


# %% the pipeline step


@dataclass
class SplitPartFromShape(Step):
    """
    Gives every annotation of a type that has no such part yet the part its shape has,
    as a body of its own.
    """

    whole_type: Type[PartWholeRelationship]
    """
    The type of annotation whose shapes are split.
    """

    splitter: PartSplitter
    """
    Splits the part off each annotation's shape.
    """

    def _apply(self, world: World) -> World:
        for whole in world.get_semantic_annotations_by_type(self.whole_type):
            self.splitter.part_of(whole)
        return world
