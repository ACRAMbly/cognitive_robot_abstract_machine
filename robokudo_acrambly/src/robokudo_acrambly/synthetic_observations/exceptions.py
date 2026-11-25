"""
Errors of scripted observations.
"""

from __future__ import annotations

# %% Playback errors

from dataclasses import dataclass

from krrood.exceptions import DataclassException


@dataclass(kw_only=True)
class NoPassDue(DataclassException):
    """
    Raised when a pass is taken from a playback whose next pass is not due.
    """

    pass_index: int
    """
    The position of the next pass in the sequence.
    """

    pass_count: int
    """
    How many passes the sequence has.
    """

    def error_message(self) -> str:
        if self.pass_index >= self.pass_count:
            return (
                f"All {self.pass_count} passes of the observation sequence were "
                f"already taken."
            )
        return f"Pass {self.pass_index} of the observation sequence is not due yet."

    def suggest_correction(self) -> str:
        return "take a pass only after is_next_pass_due returned true."
