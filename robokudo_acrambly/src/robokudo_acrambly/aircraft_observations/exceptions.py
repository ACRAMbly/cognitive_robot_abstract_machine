# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException


# %% Missing launcher configuration


@dataclass
class MissingAircraftFrameConfiguration(DataclassException):
    """
    The aircraft launcher has no explicit camera-to-world calibration.
    """

    environment_variable: str
    """Environment setting identifying the required frame configuration file."""

    def error_message(self) -> str:
        return f"Aircraft frame configuration is missing: {self.environment_variable}."

    def suggest_correction(self) -> str:
        return f"Set {self.environment_variable} to your frame configuration JSON file, or pass explicit frames."
