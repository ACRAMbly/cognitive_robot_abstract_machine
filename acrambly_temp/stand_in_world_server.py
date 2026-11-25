# %% Imports

"""
Stand-in for the robot's world server while no Giskard serves one.

Stands in for :mod:`acrambly` until the demo work moves it there. Run it before the
aircraft engine or its inspection, and never next to a real Giskard.
"""

from __future__ import annotations

from contextlib import ExitStack, suppress
from dataclasses import dataclass, field
from enum import StrEnum

import rclpy
from rclpy.executors import ExternalShutdownException
from typing_extensions import TYPE_CHECKING

from semantic_digital_twin.adapters.ros.world_fetcher import FetchWorldServer
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from rclpy.node import Node


# %% Stand-in world server


class StandInWorldName(StrEnum):
    """
    Names the stand-in world server uses.
    """

    ROOT = "map"
    """Root body of the served world, Giskard's generic root."""

    NODE = "stand_in_world_server"
    """ROS node serving and synchronizing the world."""


@dataclass(kw_only=True)
class StandInWorldServer:
    """
    Serve a synchronized robot world with root ``map``.

    A process that fetches later also gets every part accepted since.
    """

    node: Node
    """ROS node serving and synchronizing the world."""

    world: World = field(init=False)
    """The served world, holding only its root until parts are accepted into it."""

    synchronizer: WorldSynchronizer = field(init=False)
    """Synchronizer keeping the served world in step with the other processes."""

    server: FetchWorldServer = field(init=False)
    """Service handing the world to processes that fetch it."""

    def __post_init__(self) -> None:
        """
        Build the world, synchronize it and serve it.
        """
        self.world = World()
        with self.world.modify_world():
            self.world.add_body(Body(name=PrefixedName(StandInWorldName.ROOT.value)))
        self.synchronizer = WorldSynchronizer(_world=self.world, node=self.node)
        self.server = FetchWorldServer(node=self.node, world=self.world)

    def close(self) -> None:
        """
        Stop serving and synchronizing the world.
        """
        self.server.close()
        self.synchronizer.close()


# %% Command


def main() -> None:
    """
    Serve the stand-in world until interrupted.
    """
    with ExitStack() as resources:
        rclpy.init()
        resources.callback(rclpy.try_shutdown)
        node = rclpy.create_node(StandInWorldName.NODE)
        resources.callback(node.destroy_node)
        stand_in = StandInWorldServer(node=node)
        resources.callback(stand_in.close)
        node.get_logger().info(
            f"Serving a robot world with root '{StandInWorldName.ROOT}'."
        )
        rclpy.spin(node)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt, ExternalShutdownException):
        main()
