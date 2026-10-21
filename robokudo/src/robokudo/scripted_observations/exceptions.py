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
