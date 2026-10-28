# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field

from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.playback.clock import TimeUnit
from robokudo_acrambly.playback.exceptions import InvalidPlaybackTiming

if TYPE_CHECKING:
    from robokudo_acrambly.playback.clock import Clock


# %% Capture timing


@dataclass
class PlaybackTiming:
    """
    Capture cadence of a finite observation sequence.
    """

    duration_nanoseconds: int = 30 * TimeUnit.NANOSECONDS_PER_SECOND
    """
    Elapsed time from the first capture through the final one.
    """

    sample_period_nanoseconds: int = TimeUnit.NANOSECONDS_PER_SECOND // 10
    """
    Scheduled interval between consecutive captures.
    """

    def validate(self) -> None:
        """
        Reject a cadence that cannot include both sequence endpoints.
        """
        if (
            self.sample_period_nanoseconds <= 0
            or self.duration_nanoseconds < 0
            or self.duration_nanoseconds % self.sample_period_nanoseconds != 0
        ):
            raise InvalidPlaybackTiming(
                duration_nanoseconds=self.duration_nanoseconds,
                sample_period_nanoseconds=self.sample_period_nanoseconds,
            )

    def contains_unsampled_offsets(self, offsets: list[int]) -> bool:
        """
        Return whether an in-sequence boundary falls between captures.
        """
        return any(
            offset <= self.duration_nanoseconds
            and offset % self.sample_period_nanoseconds != 0
            for offset in offsets
        )


# %% Due captures


@dataclass
class ScheduledCapture:
    """
    Capture offset and Unix timestamp for one due observation.
    """

    elapsed_nanoseconds: int
    """
    Scheduled time relative to the sequence's first capture.
    """

    capture_timestamp: int
    """
    Unix timestamp of the scheduled capture.
    """


@dataclass
class CaptureSchedule:
    """
    Preserve every scheduled capture through waiting, delay, and exhaustion.
    """

    timing: PlaybackTiming
    """
    Finite duration and capture cadence.
    """

    clock: Clock
    """
    Clock separating observation delivery from scheduled capture time.
    """

    sequence_start_timestamp: int | None = field(default=None, init=False)
    """
    Unix time at the first attempted capture.
    """

    observation_index: int = field(default=0, init=False)
    """
    Position of the next scheduled observation.
    """

    def schedule_capture(self) -> ScheduledCapture | Status:
        """
        Select the next due capture or report waiting and exhaustion.
        """
        capture_offset = self.observation_index * self.timing.sample_period_nanoseconds
        if capture_offset > self.timing.duration_nanoseconds:
            return Status.FAILURE
        current_timestamp = self.clock.now_nanoseconds()
        if self.sequence_start_timestamp is None:
            self.sequence_start_timestamp = current_timestamp
        capture_timestamp = self.sequence_start_timestamp + capture_offset
        if current_timestamp < capture_timestamp:
            return Status.RUNNING
        return ScheduledCapture(capture_offset, capture_timestamp)
