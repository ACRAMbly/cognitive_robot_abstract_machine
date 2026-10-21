# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, StrEnum


# %% CAD model definitions


class ModelIdentifier(StrEnum):
    """Object kinds recognized in scripted Tetris observations."""

    FIRST = "child_cube_0"
    SECOND = "child_cube_2"


class MeshFilename(StrEnum):
    """PLY resources containing the supplied CAD models."""

    FIRST = "child_cube_0_colored.ply"
    SECOND = "child_cube_2_colored.ply"


@dataclass(frozen=True)
class BlockModel:
    """CAD resource and explicit metric scale for one block kind."""

    identifier: ModelIdentifier
    """Classification label identifying the object kind."""

    mesh_filename: MeshFilename
    """Resource filename relative to the configured mesh directory."""

    metres_per_mesh_unit: float = 0.001
    """Metres represented by one coordinate unit in the CAD resource."""


class TetrisModel(Enum):
    """Model definitions independent of detection identity."""

    FIRST = BlockModel(ModelIdentifier.FIRST, MeshFilename.FIRST)
    SECOND = BlockModel(ModelIdentifier.SECOND, MeshFilename.SECOND)
