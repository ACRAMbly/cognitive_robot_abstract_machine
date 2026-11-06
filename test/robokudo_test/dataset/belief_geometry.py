# %% Imports

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from enum import StrEnum, auto

from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import TYPE_CHECKING, Any
from robokudo.types.belief_creation import ModelGeometry
from robokudo import world
from semantic_digital_twin.callbacks.callback import ModelChangeCallback
from semantic_digital_twin.exceptions import PathResolutionError

if TYPE_CHECKING:
    from pathlib import Path
    from uuid import UUID

    from semantic_digital_twin.datastructures.prefixed_name import PrefixedName


# %% Local model geometry


class GeometryResource(StrEnum):
    """
    Resources for deterministic generic creation tests.
    """

    OFFSET_MODEL = "offset.stl"
    MISSING_MODEL = "missing.stl"


class CreationRoute(StrEnum):
    """
    Association outcomes that create additional persistent beliefs.
    """

    LOW_SIMILARITY = auto()
    UNMATCHED = auto()


@dataclass
class OffsetModelGeometry(ModelGeometry):
    """
    Independent visual and collision meshes retaining source-local offsets.
    """

    geometry: Mesh
    """
    Loaded model template with an explicit metric scale.
    """

    def create_body(self, body_name: PrefixedName) -> Body:
        return Body(
            name=body_name,
            visual=ShapeCollection([deepcopy(self.geometry)]),
            collision=ShapeCollection([deepcopy(self.geometry)]),
        )


@dataclass
class ResourceModelGeometry(ModelGeometry):
    """
    Load geometry when a body is prepared, requiring an available file.
    """

    mesh_path: Path
    """
    Model resource whose absence prevents creation.
    """

    def create_body(self, body_name: PrefixedName) -> Body:
        if not self.mesh_path.is_file():
            raise PathResolutionError(uri=str(self.mesh_path))
        geometry = Mesh(filename=str(self.mesh_path))
        return Body(
            name=body_name,
            visual=ShapeCollection([geometry]),
            collision=ShapeCollection([deepcopy(geometry)]),
        )


# %% Insertion observations


@dataclass
class ShapeCounts:
    """
    Visual and collision shape counts observed at insertion.
    """

    visual: int
    """
    Number of visual shapes in the inserted body.
    """

    collision: int
    """
    Number of collision shapes in the inserted body.
    """


@dataclass(eq=False)
class InsertionGeometry(ModelChangeCallback):
    """
    Observe each belief body's geometry when its model change is published.
    """

    shape_counts: dict[UUID, ShapeCounts] = field(default_factory=dict, init=False)
    """
    Shape counts at the first notification containing each belief body.
    """

    def on_model_change(self, **kwargs: Any) -> None:
        for identifier, belief in world.get_object_belief_states().items():
            if identifier in self.shape_counts:
                continue
            self.shape_counts[identifier] = ShapeCounts(
                len(belief.body.visual.shapes), len(belief.body.collision.shapes)
            )
