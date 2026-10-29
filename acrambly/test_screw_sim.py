"""
Minimal simulation for the joint-driven screwing action.

Spawns a counterpart (the nut plate or the airplane assembly) and a bolt threaded onto
it through a real 1-DoF ``ScrewConnection``, then builds and performs a ``sequential``
plan with :class:`~sub_parts.shared.screwing.ScrewThreadAction`::

    plan = sequential(
        [
            ParkArmsAction(tracy.all_arms),
            ScrewThreadAction(
                arm=tracy.right_arm,
                screw_body=screw.root,
                turns=0.5,
                grasp_offset=0.030,
            ),
        ],
        context=context,
    ).plan

Run from the ``acrambly`` directory (needs a ROS 2 runtime and RViz2)::

    /home/tom/.virtualenvs/cram-env/bin/python test_screw_sim.py
    /home/tom/.virtualenvs/cram-env/bin/python test_screw_sim.py --scene airplane
"""

from __future__ import annotations

import math
import threading
from datetime import timedelta
from pathlib import Path

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.specifications.connections import (
    FixedConnectionSpecification,
)
from semantic_digital_twin.spatial_types.spatial_types import Vector3
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import kinematically_simulated_robot
from coraplex.plans.executables import GiskardExecutable
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction

from sub_parts.shared.screwing import ScrewThreadAction
from sub_parts.shared.utils import spawn_body, spawn_screw_body

# %% nut scene (default)

# ADAPT: where the screw assembly sits, in metres in the world frame.
SCREW_POSE = dict(x=0.55, y=0.0, z=1.0)

PLATE_MESH = "battat/Nut.stl"
SCREW_MESH = "battat/Bolt.stl"

# ADAPT: pitch and axis calibrated to the top-level Bolt.stl; the axis points from the
# plate towards the screw, so driving the bolt in is a negative joint rotation.
SCREW_PITCH = 0.002
SCREW_AXIS = Vector3(1.0, 0.0, 0.0)

# The nut is parked behind the bolt head, so the bolt frame sits this far along the
# thread axis from the nut frame.
BOLT_IN_PLATE = (0.022, 0.0, 0.0)

# Where the gripper takes the head, in the bolt frame, just outside the +26.5 mm face.
GRASP_OFFSET = 0.030

# ADAPT: 180 degrees (0.5 rev = 1.0 mm at the 2 mm pitch) is the safe default; 270
# degrees stalls with ``MotionMadeNoProgress`` on the wrist roll axis.
JOINT_TURNS = 0.5

# %% airplane scene

# ADAPT: each entry is (mesh, local position, local yaw/pitch/roll). The top-level
# holder and the ``plane`` fuselage share the Y long axis; ``plane/UpperBody.stl`` is
# exported long-X, so it is yawed 90 degrees to align.
AIRPLANE_PARTS = (
    ("battat/HolderUnderBody.stl", (0.0, 0.0, -0.12), (0.0, 0.0, 0.0)),
    ("battat/plane/UnderBody.stl", (0.0, -0.12, -0.07), (0.0, 3.0, 0.0)),
    ("battat/plane/UpperBody.stl", (0.0, -0.08, -0.05), (-1.5708, 0.0, 0.0)),
    ("battat/plane/RearWing.stl", (0.0, 0.0, -0.12), (0.0, 0.0, 0.0)),
)

ASSEMBLY_POSE = dict(x=0.55, y=0.0, z=1.0, yaw=0.0, pitch=0.0, roll=0.0)

# ADAPT: the screw goes vertically (world +Z) through UpperBody into RearWing, so the
# top-level Bolt.stl (authored along its local +X) is pitched -90 degrees to point its
# head up. ``axis`` stays the bolt's own +X; the rotation orients the connection.
AIRPLANE_SCREW_PARENT = "RearWing"
AIRPLANE_SCREW_POSITION = (0.0, -0.032, 0.06)
AIRPLANE_SCREW_ROTATION = (0.0, -math.pi / 2.0, 0.0)
AIRPLANE_SCREW_AXIS = Vector3(1.0, 0.0, 0.0)
AIRPLANE_TURNS = 0.5

SIMULATION_TIME_LIMIT_MINUTES = 10


def build_world() -> tuple[World, Tracy]:
    """Build the simulation world from Tracy's URDF and return it with Tracy."""
    world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
    return world, Tracy.from_world(world)


def spawn_plate(world: World) -> HasGraspCandidates:
    """Spawn the nut plate and pin it to the world."""
    plate = spawn_body(
        world,
        position=(
            SCREW_POSE["x"] - BOLT_IN_PLATE[0],
            SCREW_POSE["y"] - BOLT_IN_PLATE[1],
            SCREW_POSE["z"] - BOLT_IN_PLATE[2],
        ),
        rotation=(0.0, 0.0, 0.0),
        shape_type="mesh",
        mesh_filename=PLATE_MESH,
    )
    FixedConnectionSpecification().reconnect(world, plate.root)
    return plate


def spawn_airplane_assembly(world: World) -> dict[str, HasGraspCandidates]:
    """Spawn the airplane parts and pin each to the world."""
    parts: dict[str, HasGraspCandidates] = {}
    for mesh_filename, local_position, local_rotation in AIRPLANE_PARTS:
        part = spawn_body(
            world,
            position=(
                ASSEMBLY_POSE["x"] + local_position[0],
                ASSEMBLY_POSE["y"] + local_position[1],
                ASSEMBLY_POSE["z"] + local_position[2],
            ),
            rotation=(
                ASSEMBLY_POSE["yaw"] + local_rotation[0],
                ASSEMBLY_POSE["pitch"] + local_rotation[1],
                ASSEMBLY_POSE["roll"] + local_rotation[2],
            ),
            shape_type="mesh",
            mesh_filename=mesh_filename,
        )
        FixedConnectionSpecification().reconnect(world, part.root)
        parts[Path(mesh_filename).stem] = part
    return parts


def run(scene: str) -> bool:
    """Spawn the scene and the bolt, then perform the parking + screwing plan."""
    world, tracy = build_world()
    context = Context(world=world, robot=tracy, evaluate_conditions=False)

    if scene == "airplane":
        parts = spawn_airplane_assembly(world)
        plate = parts[AIRPLANE_SCREW_PARENT]
        screw = spawn_screw_body(
            world,
            position=AIRPLANE_SCREW_POSITION,
            rotation=AIRPLANE_SCREW_ROTATION,
            parent=plate.root,
            mesh_filename=SCREW_MESH,
            name="Bolt",
            screw_pitch=SCREW_PITCH,
            turns=AIRPLANE_TURNS,
            axis=AIRPLANE_SCREW_AXIS,
        )
        turns = AIRPLANE_TURNS
    else:
        plate = spawn_plate(world)
        screw = spawn_screw_body(
            world,
            position=BOLT_IN_PLATE,
            rotation=(0.0, 0.0, 0.0),
            parent=plate.root,
            mesh_filename=SCREW_MESH,
            name="Bolt",
            screw_pitch=SCREW_PITCH,
            turns=JOINT_TURNS,
            axis=SCREW_AXIS,
        )
        turns = JOINT_TURNS

    import rclpy

    from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
        VizMarkerPublisher,
    )

    rclpy.init()
    node = rclpy.create_node("coraplex_screw_sim")
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        VizMarkerPublisher(_world=world, node=node)

        plan = sequential(
            [
                ParkArmsAction(tracy.all_arms),
                ScrewThreadAction(
                    arm=tracy.right_arm,
                    screw_body=screw.root,
                    turns=turns,
                    grasp_offset=GRASP_OFFSET,
                ),
            ],
            context=context,
        ).plan

        GiskardExecutable.simulation_time_limit = timedelta(
            minutes=SIMULATION_TIME_LIMIT_MINUTES
        )

        with kinematically_simulated_robot:
            plan.perform()
    except Exception as error:
        print(f"screwing FAILED      : {type(error).__name__}: {error}")
        return False
    finally:
        rclpy.try_shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()

    print("screwing SUCCEEDED")
    return True


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        choices=("nut", "airplane"),
        default="nut",
        help="scene to build: the nut plate (default) or the airplane assembly",
    )
    arguments = parser.parse_args()
    raise SystemExit(0 if run(arguments.scene) else 1)
