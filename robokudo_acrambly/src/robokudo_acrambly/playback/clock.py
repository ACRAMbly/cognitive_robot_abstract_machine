# %% Imports

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum
from time import time_ns


# %% Capture clock


class TimeUnit(IntEnum):
    """
    Integer conversion between Unix seconds and capture nanoseconds.
    """

    NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass
class Clock(ABC):
    """
    Supply Unix capture time independently of observation delivery.
    """

    @abstractmethod
    def now_nanoseconds(self) -> int:
        """
        Return the current Unix time in integer nanoseconds.
        """


@dataclass
class SystemClock(Clock):
    """
    Read the system's Unix clock for ordinary observation captures.
    """

    def now_nanoseconds(self) -> int:
        return time_ns()
