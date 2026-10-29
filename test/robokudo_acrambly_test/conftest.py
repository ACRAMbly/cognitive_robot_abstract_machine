# %% Shared perception fixtures

"""
Reuse RoboKudo's ROS lifecycle and annotator state fixtures.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from ..robokudo_test.conftest import (
    annotator_output_buffer,
    isolate_blackboard,
    node,
    ros_default,
)


# %% Spatial comparison precision


@dataclass
class AircraftPoseComparison:
    """Precision for comparing independently calculated aircraft poses."""

    absolute_tolerance: float = 1e-12
    """Maximum absolute error in expected translations and rotations."""


@pytest.fixture
def aircraft_pose_comparison() -> AircraftPoseComparison:
    """Supply the shared precision for aircraft pose assertions."""
    return AircraftPoseComparison()
