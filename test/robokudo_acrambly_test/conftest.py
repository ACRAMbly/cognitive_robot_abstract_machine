# %% Shared perception fixtures

"""
Reuse RoboKudo's ROS lifecycle and annotator state fixtures.
"""

from __future__ import annotations

from ..robokudo_test.conftest import (
    annotator_output_buffer,
    isolate_blackboard,
    node,
    ros_default,
)
