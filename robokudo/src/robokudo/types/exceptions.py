# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from robokudo.exceptions import RoboKudoError

# %% Model configuration


@dataclass
class MissingModelGeometry(RoboKudoError):
    """
    An observed kind has no geometry in the enabled model configuration.
    """

    model_label: str
    """Accepted classification label lacking configured model geometry."""

    def error_message(self) -> str:
        return f"No model geometry is configured for {self.model_label!r}."

    def suggest_correction(self) -> str:
        return "supply geometry for this model label or disable model geometry configuration."
