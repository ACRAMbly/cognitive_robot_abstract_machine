# %% Imports

from __future__ import annotations

from enum import StrEnum

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

# %% Robot world stand-in


class RobotWorldFrame(StrEnum):
    """
    Root frame names of robot worlds the aircraft engine is tested against.
    """

    MAP = "map"
    """Root of the stand-in world server and of Giskard's generic world."""

    TRACY_MAP = "map2"
    """Root of Tracy's Giskard world."""


def create_robot_world(*, root_frame: RobotWorldFrame) -> World:
    """
    Build an otherwise empty robot world whose only body is its root.
    """
    world = World()
    with world.modify_world():
        world.add_body(Body(name=PrefixedName(root_frame.value)))
    return world
