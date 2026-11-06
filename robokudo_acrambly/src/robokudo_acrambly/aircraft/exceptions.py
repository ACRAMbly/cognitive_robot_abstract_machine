# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import TYPE_CHECKING

from krrood.exceptions import DataclassException

if TYPE_CHECKING:
    from pathlib import Path


# %% Model resource errors


@dataclass
class MissingAircraftResource(DataclassException):
    """
    A configured aircraft mesh is unavailable.
    """

    mesh_path: Path
    """Expected location of the unavailable resource."""

    def error_message(self) -> str:
        return f"The aircraft mesh is not a readable file: {self.mesh_path}."

    def suggest_correction(self) -> str:
        return f"Supply a mesh directory containing {self.mesh_path.name}, or replace the model resource."
