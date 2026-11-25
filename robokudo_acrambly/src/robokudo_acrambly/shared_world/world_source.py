# %% Imports

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from typing_extensions import TYPE_CHECKING

from robokudo.world import init_world_entity_tracker_from_world, set_world
from semantic_digital_twin.adapters.ros.node_registry import ROSNodeRegistry
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

if TYPE_CHECKING:
    from semantic_digital_twin.world import World


# %% World sources


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

    def supply_world(self) -> World:
        return fetch_world_from_service(
            node=ROSNodeRegistry().get(), timeout_seconds=self.timeout_seconds
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
