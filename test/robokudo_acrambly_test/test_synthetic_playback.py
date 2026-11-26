"""
Handing out the passes of an observation sequence as their capture time comes.
"""

from __future__ import annotations

# %% Imports

import pytest

from robokudo_acrambly.synthetic_observations.exceptions import NoPassDue
from robokudo_acrambly.synthetic_observations.playback import PassSchedule, Playback

from .dataset.clocks import SteppedClock


# %% Fixtures


@pytest.fixture
def playback(clock: SteppedClock) -> Playback:
    """
    A playback of the default schedule that has not started.
    """
    return Playback(schedule=PassSchedule(), clock=clock)


# %% Waiting for the capture time


def test_first_pass_is_due_when_the_sequence_starts(
    playback: Playback, clock: SteppedClock
) -> None:
    assert playback.is_next_pass_due()
    first_pass = playback.take_next_pass()
    assert first_pass.index == 0
    assert first_pass.capture_time_nanoseconds == clock.now


def test_next_pass_waits_for_its_capture_time(
    playback: Playback, clock: SteppedClock
) -> None:
    start = clock.now
    playback.take_next_pass()
    clock.now = start + 99_999_999
    assert not playback.is_next_pass_due()
    with pytest.raises(NoPassDue):
        playback.take_next_pass()
    clock.now = start + 100_000_000
    second_pass = playback.take_next_pass()
    assert second_pass.index == 1
    assert second_pass.capture_time_nanoseconds == start + 100_000_000


# %% Falling behind


def test_late_caller_gets_every_pass_in_order_with_its_scheduled_time(
    playback: Playback, clock: SteppedClock
) -> None:
    start = clock.now
    playback.is_next_pass_due()
    clock.now = start + 60_000_000_000
    taken = []
    while playback.is_next_pass_due():
        taken.append(playback.take_next_pass())
    assert [scheduled.index for scheduled in taken] == list(range(300))
    assert [scheduled.capture_time_nanoseconds for scheduled in taken] == [
        start + index * 100_000_000 for index in range(300)
    ]


def test_no_pass_follows_the_last_one(playback: Playback, clock: SteppedClock) -> None:
    start = clock.now
    playback.is_next_pass_due()
    clock.now = start + 60_000_000_000
    while playback.is_next_pass_due():
        playback.take_next_pass()
    clock.now = start + 120_000_000_000
    assert not playback.is_next_pass_due()
    with pytest.raises(NoPassDue):
        playback.take_next_pass()
