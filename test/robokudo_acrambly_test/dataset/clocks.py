"""
Clocks whose time a test controls.
"""

from __future__ import annotations

# %% Controlled time

from dataclasses import dataclass

from robokudo_acrambly.synthetic_observations.playback import Clock


@dataclass(kw_only=True)
class SteppedClock(Clock):
    """
    A clock that only moves when a test moves it.
    """

    now: int = 1_700_000_000_000_000_000
    """
    The current time, in nanoseconds since the Unix epoch.
    """

    def now_nanoseconds(self) -> int:
        return self.now
