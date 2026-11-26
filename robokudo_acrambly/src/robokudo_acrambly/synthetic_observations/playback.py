"""
When the passes of an observation sequence are captured, and handing them out in order
as their time comes.
"""

from __future__ import annotations

# %% Imports

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import timedelta

import numpy

from robokudo_acrambly.synthetic_observations.exceptions import NoPassDue


# %% Pass schedule


@dataclass(kw_only=True)
class PassSchedule:
    """
    The passes of an observation sequence, captured at a fixed period from its start.
    """

    duration: timedelta = timedelta(seconds=30)
    """
    How long the sequence lasts; the last pass is captured one period before its end.
    """

    pass_period: timedelta = timedelta(milliseconds=100)
    """
    The time between the captures of two consecutive passes.
    """

    @property
    def pass_count(self) -> int:
        """
        How many passes the sequence has.
        """
        return self.duration // self.pass_period

    def elapsed_at(self, *, pass_index: int) -> timedelta:
        """
        :param pass_index: The position of a pass in the sequence, from 0.
        :return: How long after the start of the sequence that pass is captured.
        """
        return self.pass_period * pass_index


@dataclass(kw_only=True)
class ScheduledPass:
    """
    One pass of an observation sequence and when it is captured.
    """

    index: int
    """
    The position of the pass in the sequence, from 0.
    """

    elapsed: timedelta
    """
    How long after the start of the sequence the pass is captured.
    """

    capture_time_nanoseconds: int
    """
    When the pass is captured, in nanoseconds since the Unix epoch.
    """


# %% Clocks


class Clock(ABC):
    """
    A source of the current time.
    """

    @abstractmethod
    def now_nanoseconds(self) -> int:
        """
        :return: The current time, in nanoseconds since the Unix epoch.
        """


@dataclass(kw_only=True)
class SystemClock(Clock):
    """
    The time of the operating system.
    """

    def now_nanoseconds(self) -> int:
        return time.time_ns()


# %% Playback


@dataclass(kw_only=True)
class Playback:
    """
    Hands out the passes of a schedule in order, each once its capture time has come.

    A caller that falls behind still gets every pass, with its scheduled capture time.
    """

    schedule: PassSchedule
    """
    The passes to hand out.
    """

    clock: Clock
    """
    The time the capture times are compared against.
    """

    start_nanoseconds: int | None = field(init=False, default=None)
    """
    When the sequence started, in nanoseconds since the Unix epoch; None before the
    first call to :meth:`is_next_pass_due`.
    """

    next_pass_index: int = field(init=False, default=0)
    """
    The position of the pass :meth:`take_next_pass` hands out next.
    """

    def is_next_pass_due(self) -> bool:
        """
        :return: Whether the next pass exists and its capture time has come. The first
            call starts the sequence.
        """
        now = self.clock.now_nanoseconds()
        if self.start_nanoseconds is None:
            self.start_nanoseconds = now
        if self.next_pass_index >= self.schedule.pass_count:
            return False
        return now >= self._capture_time_nanoseconds(pass_index=self.next_pass_index)

    def take_next_pass(self) -> ScheduledPass:
        """
        :return: The next pass, which from now on counts as handed out.
        :raises NoPassDue: If :meth:`is_next_pass_due` is false.
        """
        if not self.is_next_pass_due():
            raise NoPassDue(
                pass_index=self.next_pass_index, pass_count=self.schedule.pass_count
            )
        scheduled_pass = ScheduledPass(
            index=self.next_pass_index,
            elapsed=self.schedule.elapsed_at(pass_index=self.next_pass_index),
            capture_time_nanoseconds=self._capture_time_nanoseconds(
                pass_index=self.next_pass_index
            ),
        )
        self.next_pass_index += 1
        return scheduled_pass

    def _capture_time_nanoseconds(self, *, pass_index: int) -> int:
        """
        :param pass_index: The position of a pass in the sequence, from 0.
        :return: When that pass is captured, in nanoseconds since the Unix epoch.
        """
        elapsed = numpy.timedelta64(
            self.schedule.elapsed_at(pass_index=pass_index), "ns"
        )
        return self.start_nanoseconds + int(elapsed.astype(numpy.int64))
