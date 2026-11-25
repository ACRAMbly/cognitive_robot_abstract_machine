# %% Imports

from __future__ import annotations

from abc import ABC, abstractmethod
from contextlib import ExitStack
from dataclasses import dataclass
from enum import StrEnum
from threading import Thread
from time import monotonic, sleep

from rclpy.executors import SingleThreadedExecutor
from typing_extensions import TYPE_CHECKING

from robokudo.world import init_world_entity_tracker_from_world, set_world
from robokudo_acrambly.shared_world.exceptions import WorldServerUnavailable
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

if TYPE_CHECKING:
    from rclpy.node import Node

    from semantic_digital_twin.world import World


# %% World sources


class WorldServerService(StrEnum):
    """
    The ROS service a world server offers.
    """

    TYPE = "std_srvs/srv/Trigger"
    """Service type of :class:`FetchWorldServer`."""


@dataclass(kw_only=True)
class WorldSource(ABC):
    """
    Supplies the robot's world that RoboKudo accepts beliefs into.
    """

    @abstractmethod
    def supply_world(self) -> World:
        """
        Return the robot's world, shared with the other processes that hold it.
        """

    def install(self) -> World:
        """
        Make the supplied world RoboKudo's world and publish its changes.

        A :class:`WorldSynchronizer` on the shared ROS node is attached once, and only
        when a node is registered. The world is never cleared.
        """
        world = self.supply_world()
        set_world(world)
        init_world_entity_tracker_from_world(world)
        registry = ROSNodeRegistry()
        if (
            registry.has_node()
            and not WorldSynchronizer.all_callbacks_of_this_type_from_world(world)
        ):
            WorldSynchronizer(_world=world, node=registry.get())
        return world


@dataclass(kw_only=True)
class WorldServerSource(WorldSource):
    """
    The robot's world fetched from a world server over the shared ROS node.
    """

    timeout_seconds: float = 10.0
    """
    Longest wait for a world server to answer.
    """

    poll_interval_seconds: float = 0.1
    """
    Pause between two looks for a world server.
    """

    service_suffix: str = "fetch_world"
    """
    Last part of the name of the world server's service.
    """

    def supply_world(self) -> World:
        node = ROSNodeRegistry().get()
        if node.executor is not None:
            return self.fetch_world(node=node)
        with ExitStack() as spinning:
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            thread = Thread(target=executor.spin, daemon=True)
            thread.start()
            spinning.callback(self.release_node, node=node)
            spinning.callback(thread.join)
            spinning.callback(executor.shutdown)
            return self.fetch_world(node=node)

    @staticmethod
    def release_node(*, node: Node) -> None:
        """
        Detach the node from the executor that spun it during the fetch.
        """
        node.executor = None

    def fetch_world(self, *, node: Node) -> World:
        """
        Fetch the world from a server, once one is found within the timeout.

        The node must be spinning, so the server's answer reaches it.
        """
        deadline = monotonic() + self.timeout_seconds
        while not self.is_world_server_available(node=node):
            if monotonic() >= deadline:
                raise WorldServerUnavailable(self.timeout_seconds)
            sleep(self.poll_interval_seconds)
        return fetch_world_from_service(
            node=node,
            service_suffix=self.service_suffix,
            timeout_seconds=max(deadline - monotonic(), self.poll_interval_seconds),
        )

    def is_world_server_available(self, *, node: Node) -> bool:
        """
        Return whether the ROS graph offers a world server's service.
        """
        return any(
            name.endswith(self.service_suffix) and types == [WorldServerService.TYPE]
            for name, types in node.get_service_names_and_types()
        )


@dataclass(kw_only=True)
class ProvidedWorldSource(WorldSource):
    """
    A robot's world that the caller built or fetched itself.
    """

    world: World
    """
    The world handed to RoboKudo.
    """

    def supply_world(self) -> World:
        return self.world
