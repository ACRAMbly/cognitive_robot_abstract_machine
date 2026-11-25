# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from krrood.exceptions import DataclassException

# %% Missing world server


class WorldServerScript(StrEnum):
    """
    Scripts that serve the robot's world, relative to the repository root.
    """

    STAND_IN = "acrambly_temp/stand_in_world_server.py"
    """Stand-in world server for runs without Giskard."""


@dataclass
class WorldServerUnavailable(DataclassException):
    """
    No world server answered within the timeout, so there is no robot world to join.
    """

    timeout_seconds: float
    """How long RoboKudo looked for a world server."""

    def error_message(self) -> str:
        return f"No world server answered within {self.timeout_seconds} seconds."

    def suggest_correction(self) -> str:
        return (
            f"Start the stand-in world server first with "
            f"'python {WorldServerScript.STAND_IN}', or start Giskard."
        )
