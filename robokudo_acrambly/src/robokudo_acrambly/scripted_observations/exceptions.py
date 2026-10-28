# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException
from robokudo_acrambly.playback.exceptions import (
    InvalidAbsenceInterval,
    InvalidPlaybackTiming,
    InvalidPoseKeyframes,
)
from typing_extensions import TYPE_CHECKING


if TYPE_CHECKING:
    from pathlib import Path

    from robokudo.types.tf import Pose


# %% Resource errors


@dataclass
class MissingMeshResource(DataclassException):
    """
    A configured CAD resource is unavailable in the mesh directory.
    """

    mesh_path: Path
    """Expected location of the missing mesh resource."""

    def error_message(self) -> str:
        return f"The scripted observation mesh does not exist: {self.mesh_path}."

    def suggest_correction(self) -> str:
        return f"Supply a mesh directory containing {self.mesh_path.name}."


# %% Projection errors


@dataclass
class InvalidCameraProjection(DataclassException):
    """
    A configured block cannot be observed fully within the camera image.
    """

    object_pose: Pose
    """Camera-relative CAD-origin pose of the unsupported observation."""

    def error_message(self) -> str:
        return f"The block at {self.object_pose} does not project fully into the image."

    def suggest_correction(self) -> str:
        return "Place the complete mesh in front of the camera and inside the image."
