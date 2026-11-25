# %% Shared perception fixtures

"""
Reuse RoboKudo's ROS lifecycle and annotator state fixtures.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from enum import StrEnum
from threading import Thread

import pytest
from rclpy.executors import SingleThreadedExecutor
from typing_extensions import TYPE_CHECKING

from robokudo import world as robokudo_world
from robokudo_acrambly.shared_world.world_source import WorldServerSource
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)

from .dataset.robot_world import RobotWorldFrame, create_robot_world
from ..robokudo_test.conftest import (
    annotator_output_buffer,
    isolate_blackboard,
    node,
    ros_default,
)

if TYPE_CHECKING:
    from rclpy.node import Node
    from semantic_digital_twin.world import World


# %% Robot world


class RobotWorldMarker(StrEnum):
    """
    Markers that change how a test gets the robot's world.
    """

    SERVED_WORLD = "served_world"
    """The test fetches the robot's world from a real world server."""


def pytest_configure(config: pytest.Config) -> None:
    """
    Register the robot world markers.
    """
    config.addinivalue_line(
        "markers",
        f"{RobotWorldMarker.SERVED_WORLD}: fetch the robot's world from a world server",
    )


@pytest.fixture(autouse=True)
def robot_world(
    request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch
) -> World:
    """
    Stand in for the world server: RoboKudo starts from a robot world with root
    ``map``, and fetching the robot's world returns the world RoboKudo holds, unless
    the test is marked to fetch it from a world server.
    """
    world = create_robot_world(root_frame=RobotWorldFrame.MAP)
    monkeypatch.setattr(robokudo_world, "world", world)
    monkeypatch.setattr(
        robokudo_world,
        "world_entity_tracker",
        WorldEntityWithIDKwargsTracker.from_world(world),
    )
    if request.node.get_closest_marker(RobotWorldMarker.SERVED_WORLD) is None:
        monkeypatch.setattr(WorldServerSource, "supply_world", supply_current_world)
    return world


def supply_current_world(source: WorldServerSource) -> World:
    """
    Return the world RoboKudo currently holds in place of a fetched one.
    """
    return robokudo_world.world_instance()


# %% Spinning shared node


@dataclass(kw_only=True)
class ExecutorShutdown:
    """Bound on waiting for the test executor's thread to stop."""

    timeout_seconds: float = 2.0
    """Longest wait for the spinning thread after the executor shut down."""


@pytest.fixture
def spinning_node(node: Node) -> Iterator[Node]:
    """
    Spin RoboKudo's registered test node, so subscriptions and services on it answer.

    Unlike ``rclpy_node``, this keeps the ROS context RoboKudo's tests share alive.
    """
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node
    executor.shutdown()
    thread.join(timeout=ExecutorShutdown().timeout_seconds)


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
