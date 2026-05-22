from __future__ import annotations

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from typing_extensions import List

from semantic_digital_twin.adapters.rerun import RerunSink, RerunVisualizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection

UR_TYPE = "ur5"
"""Universal Robots model to load (e.g. ur3/ur5/ur10)."""

UR_XACRO = os.path.join(
    get_package_share_directory("ur_description"),
    "urdf",
    "ur.urdf.xacro",
)
"""Path to the UR description xacro (requires the ``ur_description`` package)."""


def load_example_world() -> World:
    """Parse the UR robot xacro into a world.

    :return: The populated world.
    """
    return URDFParser.from_xacro(
        UR_XACRO, mappings={"ur_type": UR_TYPE, "name": UR_TYPE}
    ).parse()


def movable_connections(world: World) -> List[Connection]:
    """Return the connections whose position can be driven (i.e. have a DOF).

    :param world: The world to inspect.
    :return: The list of connections with a settable position.
    """
    return [
        connection for connection in world.connections if hasattr(connection, "raw_dof")
    ]


def wiggle(world: World, duration_seconds: float = 10.0) -> None:
    """Sweep the world's movable joints with a sine wave to show live updates.

    :param world: The world whose joints are driven.
    :param duration_seconds: How long to keep moving the joints.
    """
    connections = movable_connections(world)
    if not connections:
        return
    start = time.time()
    while time.time() - start < duration_seconds:
        phase = time.time() - start
        for index, connection in enumerate(connections):
            connection.position = 0.6 * math.sin(phase + index)
        time.sleep(0.02)


def main() -> None:
    """Load the world, spawn a Rerun viewer, and animate the joints."""
    world = load_example_world()
    RerunVisualizer(
        _world=world, application_id="semdt_rerun_demo", sink=RerunSink.SPAWN
    )
    wiggle(world)


if __name__ == "__main__":
    main()
