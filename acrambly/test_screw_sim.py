"""
Simulation for joint-driven screwing actions and airplane tool assembly.

Spawns a counterpart (the nut plate or the airplane assembly with a bolt holder)
and executes a statechart. In the airplane scene, Tracy picks a bolt from the holder,
inserts it into the airplane wing (with an optional hand turn), picks up a
screwdriver from the holder, and drives the bolt to completion with the tool tip.
"""

from __future__ import annotations

import argparse
import math
from datetime import timedelta
from pathlib import Path

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import KinematicallySimulatedPlanExecutor
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.executor import NoPacing, RealTimePacer
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import Pose, Vector3
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import StaticJointState
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.specifications.connections import (
    FixedConnectionSpecification,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from sub_parts.shared.screwdriver import Screwdriver, spawn_screwdriver
from sub_parts.shared.screwing import InsertScrewAction, ScrewThreadAction
from sub_parts.shared.utils import spawn_body, spawn_screw_body

# %% nut scene

SCREW_POSE = {"x": 0.55, "y": 0.0, "z": 1.0}
PLATE_MESH = "battat/Nut.stl"
SCREW_MESH = "battat/Bolt.stl"
SCREW_PITCH = 0.002
SCREW_AXIS = Vector3(1.0, 0.0, 0.0)
BOLT_IN_PLATE = (0.022, 0.0, 0.0)
GRASP_OFFSET = 0.030
JOINT_TURNS = 0.5

# %% airplane scene

AIRPLANE_PARTS = (
    ("battat/HolderUnderBody.stl", (0.0, 0.0, -0.12), (0.0, 0.0, 0.0)),
    ("battat/plane/UnderBody.stl", (0.0, -0.12, -0.07), (0.0, 3.0, 0.0)),
    ("battat/plane/UpperBody.stl", (0.0, -0.08, -0.05), (-1.5708, 0.0, 0.0)),
    ("battat/plane/RearWing.stl", (0.0, 0.0, -0.12), (0.0, 0.0, 0.0)),
)

ASSEMBLY_POSE = {"x": 0.55, "y": 0.0, "z": 1.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}

AIRPLANE_SCREW_PARENT = "RearWing"
AIRPLANE_SCREW_POSITION = (0.0, -0.032, 0.06)
AIRPLANE_SCREW_ROTATION = (0.0, -math.pi / 2.0, 0.0)
AIRPLANE_SCREW_AXIS = Vector3(1.0, 0.0, 0.0)
AIRPLANE_TURNS = 0.5

# Bolt holder placed on the right side of the table
HOLDER_POSE = {"x": 0.55, "y": -0.25, "z": 0.88, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
HOLDER_MESH = "battat/BoltHolder.stl"
BOLT_IN_HOLDER_POSITION = (-0.025, 0.0, 0.025)
BOLT_IN_HOLDER_ROTATION = (0.0, -math.pi / 2.0, 0.0)

SCREWDRIVER_LENGTH = 0.10
SCREWDRIVER_DIAMETER = 0.03

SIMULATION_TIME_LIMIT_MINUTES = 10

# %% scene builders


def build_world() -> tuple[World, Tracy]:
    """Build the simulation world from Tracy's URDF and return it with Tracy."""
    world = URDFParser.from_file(Tracy.get_ros_file_path()).create_world()
    tracy = Tracy.from_world(world)
    for arm in tracy.all_arms:
        arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)
    return world, tracy


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


def spawn_bolt_holder(world: World) -> HasGraspCandidates:
    """Spawn the bolt holder and pin it to the world."""
    holder = spawn_body(
        world,
        position=(HOLDER_POSE["x"], HOLDER_POSE["y"], HOLDER_POSE["z"]),
        rotation=(HOLDER_POSE["yaw"], HOLDER_POSE["pitch"], HOLDER_POSE["roll"]),
        shape_type="mesh",
        mesh_filename=HOLDER_MESH,
    )
    FixedConnectionSpecification().reconnect(world, holder.root)
    return holder


# %% statechart builders


def build_nut_statechart(
    context: StatechartContext,
    tracy: Tracy,
    screw: HasGraspCandidates,
    turns: float,
) -> Statechart:
    """Build the statechart that parks Tracy's arms and screws the bolt in directly."""
    statechart = Statechart(context=context)
    statechart.add_node(
        Sequence(
            nodes=[
                ParkArmsAction(arms=tracy.all_arms),
                ScrewThreadAction(
                    arm=tracy.right_arm,
                    screw_body=screw.root,
                    turns=turns,
                    grasp_offset=GRASP_OFFSET,
                ),
                ParkArmsAction(arms=tracy.all_arms),
            ]
        )
    )
    return statechart


def build_airplane_statechart(
    context: StatechartContext,
    tracy: Tracy,
    bolt: HasGraspCandidates,
    screwdriver: Screwdriver,
    counterpart: Body,
    target_location: Pose,
    turns: float,
    *,
    turn_by_hand: bool = True,
    hand_turns: float = 0.1,
) -> Statechart:
    """Build the statechart for the complete airplane screwing workflow."""
    statechart = Statechart(context=context)
    bolt_grasp = bolt.grasp_candidates()[0]
    screwdriver_grasp = screwdriver.grasp_candidates()[0]
    nodes = [
        ParkArmsAction(arms=tracy.all_arms),
        PickUpAction(grasp=bolt_grasp, arm=tracy.right_arm),
        InsertScrewAction(
            arm=tracy.right_arm,
            grasp=bolt_grasp,
            counterpart=counterpart,
            target_location=target_location,
            turn_by_hand=turn_by_hand,
            hand_turns=hand_turns,
        ),
        PickUpAction(grasp=screwdriver_grasp, arm=tracy.right_arm),
        ScrewThreadAction(
            arm=tracy.right_arm,
            screw_body=bolt.root,
            tool_grasp=screwdriver_grasp,
            turns=turns,
        ),
        ParkArmsAction(arms=tracy.all_arms),
    ]
    statechart.add_node(Sequence(nodes=nodes))
    return statechart


# %% plan execution


def run(scene: str, turn_by_hand: bool = True, real_time: bool = False) -> bool:
    """Spawn the scene entities and execute the appropriate statechart."""
    world, tracy = build_world()
    context = StatechartContext.with_extensions(world, [RobotAccess(tracy)])

    if scene == "airplane":
        parts = spawn_airplane_assembly(world)
        plate = parts[AIRPLANE_SCREW_PARENT]
        holder = spawn_bolt_holder(world)
        bolt = spawn_screw_body(
            world,
            position=BOLT_IN_HOLDER_POSITION,
            rotation=BOLT_IN_HOLDER_ROTATION,
            parent=holder.root,
            mesh_filename=SCREW_MESH,
            name="Bolt",
            screw_pitch=SCREW_PITCH,
            turns=AIRPLANE_TURNS,
            axis=AIRPLANE_SCREW_AXIS,
        )
        screwdriver = spawn_screwdriver(
            world,
            position=(
                HOLDER_POSE["x"] + 0.025,
                HOLDER_POSE["y"],
                HOLDER_POSE["z"] + 0.025 + SCREWDRIVER_LENGTH / 2.0,
            ),
            length=SCREWDRIVER_LENGTH,
            diameter=SCREWDRIVER_DIAMETER,
        )
        target_location = Pose.from_xyz_rpy(
            *AIRPLANE_SCREW_POSITION,
            yaw=AIRPLANE_SCREW_ROTATION[0],
            pitch=AIRPLANE_SCREW_ROTATION[1],
            roll=AIRPLANE_SCREW_ROTATION[2],
            reference_frame=plate.root,
        )
        statechart = build_airplane_statechart(
            context,
            tracy,
            bolt,
            screwdriver,
            plate.root,
            target_location,
            turns=AIRPLANE_TURNS,
            turn_by_hand=turn_by_hand,
        )
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
        statechart = build_nut_statechart(context, tracy, screw, turns=JOINT_TURNS)

    import rclpy
    from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
        VizMarkerPublisher,
    )

    rclpy.init()
    node = rclpy.create_node("coraplex_screw_sim")

    try:
        TFPublisher(node=node, _world=world)
        VizMarkerPublisher(_world=world, node=node)
        world.notify_state_change()

        executor = KinematicallySimulatedPlanExecutor(
            context,
            ros_node=node,
            simulation_time_per_action=timedelta(minutes=SIMULATION_TIME_LIMIT_MINUTES),
            pacer=RealTimePacer() if real_time else NoPacing(),
        )
        try:
            executor.compile(statechart)
            executor.execute()
        finally:
            executor.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print("screwing SUCCEEDED")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        choices=("nut", "airplane"),
        default="airplane",
        help="scene to build: the nut plate or the airplane assembly (default: airplane)",
    )
    parser.add_argument(
        "--turn-by-hand",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="whether to turn the screw by hand before releasing (default: True)",
    )
    parser.add_argument(
        "--real-time",
        action="store_true",
        help="pace execution in wall-clock real time for comfortable viewing in RViz (default: fast simulation)",
    )
    arguments = parser.parse_args()
    raise SystemExit(
        0
        if run(
            arguments.scene,
            turn_by_hand=arguments.turn_by_hand,
            real_time=arguments.real_time,
        )
        else 1
    )
