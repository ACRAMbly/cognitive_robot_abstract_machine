"""
Airplane TopWing assembly — spiral peg-in-hole search.

Merges scene setup (fixture, peg localisation, spawn helpers)
with the Archimedean spiral approach using FollowToolCenterPointPathAction.

PlaceAction is intentionally NOT used here: on the coraplex-migration branch
it still imports the removed Arms / GraspDescription symbols and will crash.
The spiral IS the placement.

Usage:
    # Terminal 1 — run the script (keep it running, don't Ctrl+C)
source ~/ros_ws/install/setup.bash
PYTHONPATH=.:/opt/ros/jazzy/lib/python3.12/site-packages python3 acrambly/test_assemble_top_wing.py

# Terminal 2 — open RViz
source ~/ros_ws/install/setup.bash
rviz2
    cd ~/Documents/cognitive_robot_abstract_machine/acrambly
    python3 test_assemble_top_wing_spiral.py
"""

from __future__ import annotations

import threading
import time
from math import pi
from typing import Dict, List, Tuple

import numpy as np
import rclpy

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.trajectory import PoseTrajectory
from coraplex.execution_environment import ExecutionEnvironment
from coraplex.datastructures.enums import ExecutionType
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import (
    FollowToolCenterPointPathAction,
    ParkArmsAction,
)

from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from coraplex.ros_utils.joint_state_publisher import JointStatePublisher
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from coraplex.datastructures.grasp import GraspDescription
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import Pose

from acrambly.sub_parts.shared.utils import spawn_body


# ---------------------------------------------------------------------------
# Scene configuration  (unchanged from teammate's script)
# ---------------------------------------------------------------------------

TABLE_TOP_Z = 0.88
NODE_NAME = "acrambly_top_wing_spiral"

FIXTURE_POSITION = (0.925, -0.05, TABLE_TOP_Z + 0.02)
FIXTURE_YPR = (pi / 2, 0.0, pi / 2)

TOP_WING_HOLDER_POSITION = (0.925, 0.343, TABLE_TOP_Z + 0.02)
TOP_WING_HOLDER_YPR = (pi, 0.0, 0.0)

BOTTOM_WING_YPR = (0.0, 0.0, 0.0)
UNDER_BODY_YPR = (pi, pi, 0.0)
UPPER_BODY_YPR = (pi / 2, 0.0, 0.0)
TOP_WING_YPR = (0.0, 0.0, 0.0)
TOP_WING_FINAL_RPY = (0.0, pi, -pi / 2)
TOP_WING_FINAL_YPR = (-pi / 2, 0.0, 0.0)

MESH_BOUNDS = {
    "BottomWing": ((-0.131022, -0.042403, -0.0014214169),
                   ( 0.123255,  0.061645,  0.0276427511)),
    "UnderBody":  ((-0.037247, -0.087470, -0.0214498378),
                   ( 0.039929,  0.099438,  0.0217984542)),
    "UpperBody":  ((-0.113834, -0.038627, -0.0319325514),
                   ( 0.125068,  0.038553,  0.0603048913)),
    "TopWing":    ((-0.075480, -0.127092, -0.0060977931),
                   ( 0.027930,  0.127542,  0.0290591735)),
}

FIXTURE_PEG_LOCAL = (0.04275, 0.1525, 0.0)

ASSEMBLY_OVERLAP = 0.02


def _fixture_peg_world_pose() -> Tuple[float, float, float]:
    """Transform the measured highest-peg point from fixture-local to world."""
    fx, fy, fz = FIXTURE_POSITION
    lx, ly, lz = FIXTURE_PEG_LOCAL
    return (fx + lz, fy + lx, fz + ly)


PEG_X, PEG_Y, PEG_TOP_Z = _fixture_peg_world_pose()

# ---------------------------------------------------------------------------
# Spiral parameters
# ---------------------------------------------------------------------------

SPIRAL_TURNS = 1
SPIRAL_STEPS = 20
SPIRAL_PITCH = 0.002   # metres per radian (tight)
APPROACH_Z_OFFSET = 0.03   # metres above peg top to start spiral

# ---------------------------------------------------------------------------
# World and robot
# ---------------------------------------------------------------------------

world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy, evaluate_conditions=False)


def spawn_mesh(mesh_path, position, ypr):
    return spawn_body(world, position, ypr, "mesh", mesh_filename=mesh_path)


# ---------------------------------------------------------------------------
# Fixture and part holders  (unchanged from teammate's script)
# ---------------------------------------------------------------------------

def spawn_fixture_and_holders() -> None:
    print("\nSpawning fixture and component holders...")
    spawn_mesh("fixtures/big_wooden_plate.stl", (0.7, 0.0, TABLE_TOP_Z + 0.01), (0.0, 0.0, 0.0))
    spawn_mesh("battat/BoltHolder.stl", (1.01, 0.575, TABLE_TOP_Z + 0.02), (pi, 0.0, 0.0))
    spawn_mesh("battat/HolderUpperBody.stl", (0.91, 0.5, TABLE_TOP_Z + 0.02), (pi / 2, 0.0, 0.0))
    spawn_mesh("battat/HolderTopWing.stl", TOP_WING_HOLDER_POSITION, TOP_WING_HOLDER_YPR)
    spawn_mesh("battat/HolderUnderBody.stl", (0.91, 0.2, TABLE_TOP_Z + 0.02), (pi / 2, 0.0, 0.0))
    spawn_mesh("battat/Plane_horizontal_holder_tall.stl", FIXTURE_POSITION, FIXTURE_YPR)
    spawn_mesh("battat/HolderBottomWing.stl", (0.925, -0.357, TABLE_TOP_Z + 0.02), (0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Assembly positions  (unchanged from teammate's script)
# ---------------------------------------------------------------------------

def _rotation_matrix_from_ypr(ypr: Tuple[float, float, float]):
    import numpy as np
    yaw, pitch, roll = ypr
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ])


def _transformed_z_bounds(mesh_name: str, ypr: Tuple[float, float, float]):
    import itertools
    lower, upper = MESH_BOUNDS[mesh_name]
    corners = np.array(list(itertools.product(
        (lower[0], upper[0]), (lower[1], upper[1]), (lower[2], upper[2]),
    )))
    rotated = corners @ _rotation_matrix_from_ypr(ypr).T
    return float(rotated[:, 2].min()), float(rotated[:, 2].max())


def calculate_assembly_positions() -> Dict[str, Tuple[float, float, float]]:
    bw_min, bw_max = _transformed_z_bounds("BottomWing", BOTTOM_WING_YPR)
    ub_min, ub_max = _transformed_z_bounds("UnderBody", UNDER_BODY_YPR)
    upper_min, upper_max = _transformed_z_bounds("UpperBody", UPPER_BODY_YPR)
    tw_min, _ = _transformed_z_bounds("TopWing", TOP_WING_FINAL_YPR)

    under_z = PEG_TOP_Z - ub_min
    under_bottom = under_z + ub_min
    under_top = under_z + ub_max
    bottom_z = under_bottom - bw_max + ASSEMBLY_OVERLAP
    upper_z = under_top - upper_min - ASSEMBLY_OVERLAP
    upper_top = upper_z + upper_max
    top_wing_z = upper_top - tw_min

    return {
        "BottomWing": (PEG_X, PEG_Y, bottom_z - 0.02),
        "UnderBody":  (PEG_X, PEG_Y, under_z - 0.02),
        "UpperBody":  (PEG_X, PEG_Y - 0.035, upper_z - 0.03),
        "TopWing":    (PEG_X, PEG_Y, top_wing_z),
    }


def spawn_partial_plane():
    print("\nSpawning the partially assembled plane on the fixture...")
    positions = calculate_assembly_positions()
    return {
        "BottomWing": spawn_mesh("battat/plane/BottomWing.stl", positions["BottomWing"], BOTTOM_WING_YPR),
        "UnderBody":  spawn_mesh("battat/plane/UnderBody.stl",  positions["UnderBody"],  UNDER_BODY_YPR),
        "UpperBody":  spawn_mesh("battat/plane/UpperBody.stl",  positions["UpperBody"],  UPPER_BODY_YPR),
    }


TOP_WING_START_POSITION = (0.925, 0.343, 1.0)


def spawn_top_wing():
    print("\nSpawning TopWing at its independent starting position...")
    return spawn_mesh("battat/plane/TopWing.stl", TOP_WING_START_POSITION, TOP_WING_HOLDER_YPR)


# ---------------------------------------------------------------------------
# Spiral waypoints
# ---------------------------------------------------------------------------

def build_spiral_poses(
    centre_x: float,
    centre_y: float,
    z: float,
    turns: int = SPIRAL_TURNS,
    steps: int = SPIRAL_STEPS,
    pitch: float = SPIRAL_PITCH,
) -> List[Pose]:
    """
    Archimedean spiral in the XY plane at fixed z, as world-frame Poses.
    The gripper orientation is kept fixed (top-down, roll=pi/2, pitch=0, yaw=pi/2
    to match the GraspCandidate used for pickup).
    """
    poses = []
    total_angle = turns * 2 * pi
    for i in range(steps + 1):
        theta = total_angle * i / steps
        r = pitch * theta
        x = centre_x + r * np.cos(theta)
        y = centre_y + r * np.sin(theta)
        poses.append(
            Pose.from_xyz_rpy(
                x, y, z,
                pi / 2,   # roll  — same as grasp
                0.0,       # pitch
                pi / 2,   # yaw
                reference_frame=world.root,
            )
        )
    return poses


# ---------------------------------------------------------------------------
# Plan
# ---------------------------------------------------------------------------

def build_spiral_plan(top_wing):
    print("\nPreparing spiral peg-in-hole plan...")

    left_arm = tracy.right_arm

    # Let plain Body act as its own HasRootBody (manipulation/assembly branch)
    top_wing.root = top_wing

    # Grasp pose (from teammate's STL measurements)
    grasp_candidate = GraspCandidate(
        top_wing,
        Pose.from_xyz_rpy(
            -0.023775,
            -0.127092,
             0.01148,
            pi / 2, 0.0, pi / 2,
            reference_frame=top_wing,
        ),
    )

    # Spiral centred on the fixture peg, just above its top
    spiral_z = PEG_TOP_Z + APPROACH_Z_OFFSET
    spiral_poses = build_spiral_poses(PEG_X, PEG_Y, spiral_z)

    print(f"Peg datum: ({PEG_X:.4f}, {PEG_Y:.4f}, {PEG_TOP_Z:.4f})")
    print(f"Spiral z: {spiral_z:.4f}  ({len(spiral_poses)} waypoints)")

    return spiral_poses


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        TFPublisher(node=node, _world=world)
        VizMarkerPublisher(_world=world, node=node)
        JointStatePublisher(world=world, node=node)

        spawn_fixture_and_holders()
        plane = spawn_partial_plane()
        top_wing = spawn_top_wing()

        world.notify_state_change()

        print(f"\nFixture peg world position: ({PEG_X:.5f}, {PEG_Y:.5f}, {PEG_TOP_Z:.5f})")
        time.sleep(2.0)

        spiral_poses = build_spiral_plan(top_wing)

        print("\n==============================================")
        print("STARTING SPIRAL PEG-IN-HOLE")
        print("Starting in 10 seconds... set up RViz now")
        time.sleep(10)
        print("==============================================")

        left_arm = tracy.right_arm
        with ExecutionEnvironment(ExecutionType.KINEMATICALLY_SIMULATED):
            for i, pose in enumerate(spiral_poses):
                print(f"  Waypoint {i + 1}/{len(spiral_poses)}")
                sequential(
                    [
                        FollowToolCenterPointPathAction(
                            target_locations=PoseTrajectory(poses=[pose]),
                            arm=left_arm,
                        ),
                    ],
                    context=context,
                ).plan.perform()
                world.notify_state_change()
                time.sleep(0.3)

        world.notify_state_change()
        print("\n==============================================")
        print("SPIRAL PLAN FINISHED")
        print("==============================================")
        print("\nFinal TopWing world pose:")
        print(top_wing.root.global_pose)
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