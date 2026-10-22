# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException
from typing_extensions import TYPE_CHECKING


if TYPE_CHECKING:
    from pathlib import Path

    from robokudo.scripted_observations.models import ModelIdentifier
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


# %% Playback configuration errors


@dataclass
class InvalidPlaybackTiming(DataclassException):
    """
    A finite sequence cannot include both endpoints at its configured cadence.
    """

    duration_nanoseconds: int
    """Configured time between the initial and final capture."""

    sample_period_nanoseconds: int
    """
    Configured time between consecutive captures.
    """

    def error_message(self) -> str:
        return (
            f"Invalid scripted playback duration {self.duration_nanoseconds} ns "
            f"and sample period {self.sample_period_nanoseconds} ns."
        )

    def suggest_correction(self) -> str:
        return (
            "Use a positive sample period and a nonnegative duration that is "
            "an exact multiple of the sample period."
        )


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


# %% Motion configuration errors


@dataclass
class InvalidPoseKeyframes(DataclassException):
    """
    A block's keyframes cannot form the configured sampled motion.
    """

    model_identifier: ModelIdentifier
    """Object kind whose keyframes are invalid."""

    keyframe_offsets_nanoseconds: list[int]
    """
    Configured keyframe offsets relative to the initial capture.
    """

    sample_period_nanoseconds: int
    """Scheduled time between consecutive observations."""

    def error_message(self) -> str:
        return (
            f"Invalid pose keyframe offsets for {self.model_identifier}: "
            f"{self.keyframe_offsets_nanoseconds} with sample period {self.sample_period_nanoseconds} ns."
        )

    def suggest_correction(self) -> str:
        return (
            "Use strictly increasing positive offsets; keyframes within the playback "
            "duration must be exact multiples of the sample period."
        )
