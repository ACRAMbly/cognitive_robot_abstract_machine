# %% Shared perception fixtures

"""
Reuse RoboKudo's ROS lifecycle and annotator state fixtures, and share a controlled
clock.
"""

from __future__ import annotations

import pytest

from ..robokudo_test.conftest import (
    annotator_output_buffer,
    isolate_blackboard,
    node,
    ros_default,
)
from .dataset.clocks import SteppedClock


# %% Controlled time


@pytest.fixture
def clock() -> SteppedClock:
    """
    A clock at a fixed time.
    """
    return SteppedClock()
