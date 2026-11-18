#!/usr/bin/env python3
"""
Kinematically simulated TopWing pick-and-place for ACRAMbly / Tracy.

This version intentionally avoids the legacy PickUpAction and PlaceAction:
- the current branch has a stub GraspDescription implementation;
- PlaceAction imports removed enum/node names.

Instead, it builds the sequence from current-branch motions and ReAttachNode.
The TopWing is attached to the simulated gripper in the digital world after
the close motion, follows the TCP during transport, and is detached at the
target pose.

Run from the repository root in the cram-env environment:
    python3 acrambly/test_assemble_top_wing.py

This is for kinematic simulation/RViz only, not real hardware.
"""

from __future__ import annotations

import threading
import time
from math import pi
from typing import Dict, Tuple

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import kinematically_simulated_robot
from coraplex.plans.attachment_nodes import ReAttachNode
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import Pose

from acrambly.sub_parts.shared.utils import spawn_body


# ---------------------------------------------------------------------------
# Scene configuration
# ---------------------------------------------------------------------------

TABLE_TOP_Z = 0.88
NODE_NAME = "acrambly_top_wing_assembly"

# Fixture pose. spawn_body's rotation tuple is (yaw, pitch, roll), not
# conventional (roll, pitch, yaw).
FIXTURE_POSITION = (0.925, -0.05, TABLE_TOP_Z + 0.02)
FIXTURE_YPR = (pi / 2, 0.0, pi / 2)

# TopWing holder.
TOP_WING_HOLDER_POSITION = (0.925, 0.343, TABLE_TOP_Z + 0.02)
TOP_WING_HOLDER_YPR = (pi, 0.0, 0.0)

# Component orientations: (yaw, pitch, roll), as expected by spawn_body.
BOTTOM_WING_YPR = (0.0, 0.0, 0.0)
UNDER_BODY_YPR = (pi, pi, 0.0)
UPPER_BODY_YPR = (pi / 2, 0.0, 0.0)
TOP_WING_HOLDER_YPR = (pi, 0.0, 0.0)

# Final TopWing orientation. Pose.from_xyz_rpy uses (roll, pitch, yaw).
TOP_WING_FINAL_RPY = (0.0, pi, -pi / 2)
TOP_WING_FINAL_YPR = (-pi / 2, 0.0, 0.0)

# Mesh axis-aligned bounds (metres), measured from the STL files.
MESH_BOUNDS = {
    "BottomWing": (
        (-0.131022, -0.042403, -0.0014214169),
        (0.123255, 0.061645, 0.0276427511),
    ),
    "UnderBody": (
        (-0.037247, -0.087470, -0.0214498378),
        (0.039929, 0.099438, 0.0217984542),
    ),
    "UpperBody": (
        (-0.113834, -0.038627, -0.0319325514),
        (0.125068, 0.038553, 0.0603048913),
    ),
    "TopWing": (
        (-0.075480, -0.127092, -0.0060977931),
        (0.027930, 0.127542, 0.0290591735),
    ),
}

# Highest fixture peg in fixture-local coordinates.
FIXTURE_PEG_LOCAL = (0.04275, 0.1525, 0.0)

# Intentional visual overlap at mating planes.
ASSEMBLY_OVERLAP = 0.02

# Manipulation configuration.
ARM_SIDE = "left"
PREGRASP_DISTANCE = 0.08
LIFT_DISTANCE = 0.08
RETRACT_DISTANCE = 0.08


def fixture_peg_world_pose() -> Tuple[float, float, float]:
    """Transform the highest peg point from fixture-local to world."""
    fx, fy, fz = FIXTURE_POSITION
    lx, ly, lz = FIXTURE_PEG_LOCAL

    # For yaw=pi/2, pitch=0, roll=pi/2, world delta=(lz, lx, ly).
    return fx + lz, fy + lx, fz + ly


PEG_X, PEG_Y, PEG_TOP_Z = fixture_peg_world_pose()


# ---------------------------------------------------------------------------
# World and robot
# ---------------------------------------------------------------------------

world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy, evaluate_conditions=False)


def spawn_mesh(mesh_path, position, ypr):
    return spawn_body(
        world,
        position,
        ypr,
        "mesh",
        mesh_filename=mesh_path,
    )


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def pose_from_matrix(matrix: np.ndarray, reference_frame) -> Pose:
    """
    Convert a 4x4 homogeneous transform into the Pose API used by this branch.

    The matrix is interpreted as reference_frame_T_pose.
    """
    matrix = np.asarray(matrix, dtype=float)
    xyz = matrix[:3, 3]
    roll, pitch, yaw = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")
    return Pose.from_xyz_rpy(
        float(xyz[0]),
        float(xyz[1]),
        float(xyz[2]),
        float(roll),
        float(pitch),
        float(yaw),
        reference_frame=reference_frame,
    )


def translated_pose(pose: Pose, dx=0.0, dy=0.0, dz=0.0) -> Pose:
    """Translate a world-frame pose along world XYZ while preserving orientation."""
    matrix = np.array(pose.to_np(), dtype=float, copy=True)
    matrix[:3, 3] += np.array([dx, dy, dz], dtype=float)
    return pose_from_matrix(matrix, world.root)


def transform_pose_to_world(local_pose: Pose) -> Pose:
    """Transform a pose expressed relative to an object into the world frame."""
    return world.transform(local_pose, world.root)


# ---------------------------------------------------------------------------
# Fixture and part holders
# ---------------------------------------------------------------------------

def spawn_fixture_and_holders() -> None:
    print("\nSpawning fixture and component holders...")

    spawn_mesh(
        "fixtures/big_wooden_plate.stl",
        (0.7, 0.0, TABLE_TOP_Z + 0.01),
        (0.0, 0.0, 0.0),
    )
    spawn_mesh(
        "battat/BoltHolder.stl",
        (1.01, 0.575, TABLE_TOP_Z + 0.02),
        (pi, 0.0, 0.0),
    )
    spawn_mesh(
        "battat/HolderUpperBody.stl",
        (0.91, 0.5, TABLE_TOP_Z + 0.02),
        (pi / 2, 0.0, 0.0),
    )
    spawn_mesh(
        "battat/HolderTopWing.stl",
        TOP_WING_HOLDER_POSITION,
        TOP_WING_HOLDER_YPR,
    )
    spawn_mesh(
        "battat/HolderUnderBody.stl",
        (0.91, 0.2, TABLE_TOP_Z + 0.02),
        (pi / 2, 0.0, 0.0),
    )
    spawn_mesh(
        "battat/Plane_horizontal_holder_tall.stl",
        FIXTURE_POSITION,
        FIXTURE_YPR,
    )
    spawn_mesh(
        "battat/HolderBottomWing.stl",
        (0.925, -0.357, TABLE_TOP_Z + 0.02),
        (0.0, 0.0, 0.0),
    )


# ---------------------------------------------------------------------------
# Calculate component transforms from STL bounds and fixture peg location
# ---------------------------------------------------------------------------

def rotation_matrix_from_ypr(ypr: Tuple[float, float, float]) -> np.ndarray:
    """Rotation matrix matching spawn_body's (yaw, pitch, roll) convention."""
    yaw, pitch, roll = ypr
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)

    return np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=float,
    )


def transformed_z_bounds(mesh_name: str, ypr: Tuple[float, float, float]):
    """Compute vertical extents after rotating all eight STL bounding corners."""
    import itertools

    lower, upper = MESH_BOUNDS[mesh_name]
    corners = np.array(
        list(
            itertools.product(
                (lower[0], upper[0]),
                (lower[1], upper[1]),
                (lower[2], upper[2]),
            )
        ),
        dtype=float,
    )
    rotated = corners @ rotation_matrix_from_ypr(ypr).T
    return float(rotated[:, 2].min()), float(rotated[:, 2].max())


def calculate_assembly_positions() -> Dict[str, Tuple[float, float, float]]:
    """
    Calculate Z positions so transformed component bounds meet at the assembly
    planes. The small overlap is retained from the original simulation script.
    """
    bw_min, bw_max = transformed_z_bounds("BottomWing", BOTTOM_WING_YPR)
    ub_min, ub_max = transformed_z_bounds("UnderBody", UNDER_BODY_YPR)
    upper_min, upper_max = transformed_z_bounds("UpperBody", UPPER_BODY_YPR)
    tw_min, _ = transformed_z_bounds("TopWing", TOP_WING_FINAL_YPR)

    under_z = PEG_TOP_Z - ub_min
    under_bottom = under_z + ub_min
    under_top = under_z + ub_max

    bottom_z = under_bottom - bw_max + ASSEMBLY_OVERLAP
    upper_z = under_top - upper_min - ASSEMBLY_OVERLAP
    upper_top = upper_z + upper_max
    top_wing_z = upper_top - tw_min

    return {
        "BottomWing": (PEG_X, PEG_Y, bottom_z - 0.02),
        "UnderBody": (PEG_X, PEG_Y, under_z - 0.02),
        "UpperBody": (PEG_X, PEG_Y - 0.035, upper_z - 0.03),
        "TopWing": (PEG_X, PEG_Y, top_wing_z),
    }


def spawn_partial_plane():
    print("\nSpawning the partially assembled plane on the fixture...")
    print(f"Lower-part assembly overlap: {ASSEMBLY_OVERLAP * 1000:.1f} mm")
    positions = calculate_assembly_positions()

    return {
        "BottomWing": spawn_mesh(
            "battat/plane/BottomWing.stl",
            positions["BottomWing"],
            BOTTOM_WING_YPR,
        ),
        "UnderBody": spawn_mesh(
            "battat/plane/UnderBody.stl",
            positions["UnderBody"],
            UNDER_BODY_YPR,
        ),
        "UpperBody": spawn_mesh(
            "battat/plane/UpperBody.stl",
            positions["UpperBody"],
            UPPER_BODY_YPR,
        ),
    }


# TopWing initial pose, retained from the attached script.
TOP_WING_START_POSITION = (0.925, 0.343, 1.0)


def spawn_top_wing():
    print("\nSpawning TopWing at its independent starting position...")
    return spawn_mesh(
        "battat/plane/TopWing.stl",
        TOP_WING_START_POSITION,
        TOP_WING_HOLDER_YPR,
    )


# ---------------------------------------------------------------------------
# TopWing target and simulated pick-and-place plan
# ---------------------------------------------------------------------------

def calculate_top_wing_target() -> Pose:
    x, y, z = calculate_assembly_positions()["TopWing"]
    target = Pose.from_xyz_rpy(
        x + 0.00,
        y + 0.047,
        z - 0.07,
        *TOP_WING_FINAL_RPY,
        reference_frame=world.root,
    )
    print("\nCalculated TopWing target pose:")
    print(target)
    return target


def build_top_wing_pick_and_place_plan(top_wing):
    """
    Build a simulation-only sequence using current-branch motions.

    Grasp pose is defined in the TopWing root frame. The target TCP pose is
    calculated as world_T_object_target @ object_T_tcp, preserving the same
    grasp transform throughout transport.
    """
    arm = tracy.left_arm if ARM_SIDE == "left" else tracy.right_arm
    tool_frame = arm.end_effector.tool_frame

    # The grasp transform is relative to the TopWing object frame.
    object_T_tcp = Pose.from_xyz_rpy(
        -0.023775,
        -0.127092,
        0.01148,
        pi / 2,
        0.0,
        pi / 2,
        reference_frame=top_wing,
    )

    # Current world pose of the TCP when it is at the grasp transform.
    world_T_object = np.asarray(top_wing.global_pose.to_np(), dtype=float)
    object_T_tcp_matrix = np.asarray(object_T_tcp.to_np(), dtype=float)
    world_T_tcp_grasp_matrix = world_T_object @ object_T_tcp_matrix
    world_T_tcp_grasp = pose_from_matrix(world_T_tcp_grasp_matrix, world.root)

    # Pre-grasp is 8 cm opposite the grasp frame's local +X approach direction.
    local_pregrasp_offset = np.eye(4)
    local_pregrasp_offset[0, 3] = -PREGRASP_DISTANCE
    world_T_tcp_pregrasp = pose_from_matrix(
        world_T_tcp_grasp_matrix @ local_pregrasp_offset,
        world.root,
    )

    # Lift vertically after attaching the object to the tool frame.
    world_T_tcp_lift = translated_pose(
        world_T_tcp_grasp, dz=LIFT_DISTANCE
    )

    # Desired TCP target preserves object_T_tcp while placing the object at
    # the target object pose.
    target_object_pose = calculate_top_wing_target()
    world_T_object_target = np.asarray(target_object_pose.to_np(), dtype=float)
    world_T_tcp_target_matrix = world_T_object_target @ object_T_tcp_matrix
    world_T_tcp_target = pose_from_matrix(world_T_tcp_target_matrix, world.root)

    # Approach from above the final TCP target, then descend to the target.
    world_T_tcp_target_pre = translated_pose(
        world_T_tcp_target, dz=RETRACT_DISTANCE
    )
    world_T_tcp_retract = translated_pose(
        world_T_tcp_target, dz=RETRACT_DISTANCE
    )

    print("\nPreparing simulated TopWing pick-and-place sequence...")
    print(f"Selected arm: {ARM_SIDE}")
    print("Grasp transform (object frame):")
    print(object_T_tcp)

    actions = [
        ParkArmsAction(tracy.all_arms),
        MoveGripperMotion(
            motion=GripperState.OPEN,
            gripper=arm,
        ),
        MoveToolCenterPointMotion(
            target=world_T_tcp_pregrasp,
            arm=arm,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=world_T_tcp_grasp,
            arm=arm,
            allow_gripper_collision=False,
        ),
        MoveGripperMotion(
            motion=GripperState.CLOSE,
            gripper=arm,
        ),
        # In kinematic simulation, this reparents the object to the TCP.
        ReAttachNode(
            body=top_wing,
            new_parent=tool_frame,
        ),
        MoveToolCenterPointMotion(
            target=world_T_tcp_lift,
            arm=arm,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=world_T_tcp_target_pre,
            arm=arm,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=world_T_tcp_target,
            arm=arm,
            allow_gripper_collision=False,
        ),
        MoveGripperMotion(
            motion=GripperState.OPEN,
            gripper=arm,
        ),
        # Release the TopWing back into the world at its current target pose.
        ReAttachNode(
            body=top_wing,
            new_parent=world.root,
        ),
        MoveToolCenterPointMotion(
            target=world_T_tcp_retract,
            arm=arm,
            allow_gripper_collision=False,
        ),
        ParkArmsAction(tracy.all_arms),
    ]

    return sequential(actions, context=context).plan


# ---------------------------------------------------------------------------
# Diagnostics and main
# ---------------------------------------------------------------------------

def print_scene(plane, top_wing) -> None:
    print("\n========== FIXTURE LOCATING PEG ==========")
    print(
        f"Peg datum (x, y, z): "
        f"({PEG_X:.5f}, {PEG_Y:.5f}, {PEG_TOP_Z:.5f})"
    )

    print("\n========== PARTIALLY ASSEMBLED PLANE ==========")
    for name, part in plane.items():
        print(f"\n{name} world pose:")
        print(part.global_pose)

    print("\n========== TOPWING INITIAL POSE ==========")
    print(top_wing.global_pose)


def main() -> None:
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)

    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    try:
        TFPublisher(node=node, _world=world)
        VizMarkerPublisher(_world=world, node=node)

        spawn_fixture_and_holders()
        plane = spawn_partial_plane()
        top_wing = spawn_top_wing()

        world.notify_state_change()
        print_scene(plane, top_wing)

        print("\nInitial scene published. Allowing RViz to update...")
        time.sleep(2.0)

        plan = build_top_wing_pick_and_place_plan(top_wing)

        print("\n==============================================")
        print("STARTING KINEMATICALLY SIMULATED TOPWING PICK AND PLACE")
        print("==============================================")

        with kinematically_simulated_robot:
            plan.perform()

        world.notify_state_change()
        print("\n==============================================")
        print("TOPWING PICK-AND-PLACE PLAN FINISHED")
        print("==============================================")
        print("\nFinal TopWing world pose:")
        print(top_wing.global_pose)
        print("\nRViz remains active. Press Ctrl+C to exit.")

        while rclpy.ok():
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping scene...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
