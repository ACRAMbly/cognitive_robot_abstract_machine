# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException
from typing_extensions import TYPE_CHECKING


if TYPE_CHECKING:
    from pathlib import Path


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
