# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from robokudo_acrambly.playback.motion import AbsenceInterval


# %% Invalid timing


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


# %% Motion configuration errors


@dataclass
class InvalidPoseKeyframes(DataclassException):
    """
    An object's keyframes cannot form the configured sampled motion.
    """

    model_identifier: str
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


# %% Visibility configuration errors


@dataclass
class InvalidAbsenceInterval(DataclassException):
    """
    An object's absence interval cannot form a discrete sampled transition.
    """

    model_identifier: str
    """Object kind whose absence interval is invalid."""

    absence_interval: AbsenceInterval
    """
    Configured half-open elapsed capture interval.
    """

    sample_period_nanoseconds: int
    """Scheduled time between consecutive observations."""

    def error_message(self) -> str:
        return (
            f"Invalid absence interval for {self.model_identifier}: "
            f"{self.absence_interval} with sample period {self.sample_period_nanoseconds} ns."
        )

    def suggest_correction(self) -> str:
        return (
            "Use a nonnegative start before the end; endpoints within the playback "
            "duration must be exact multiples of the sample period."
        )
