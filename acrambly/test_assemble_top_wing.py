from __future__ import annotations

import threading
import time
from math import pi
from typing import Dict, Tuple

import rclpy

from coraplex.datastructures.dataclasses import Context
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from coraplex.execution_environment import kinematically_simulated_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction

from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser
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

# TopWing remains on this holder until Tracy picks it up.
TOP_WING_HOLDER_POSITION = (0.925, 0.343, TABLE_TOP_Z + 0.02)
TOP_WING_HOLDER_YPR = (pi, 0.0, 0.0)

# Component orientations: these preserve the intended orthogonal relationship
# between fuselage and wings. Values are (yaw, pitch, roll).
# Explicit Z-axis rotations (spawn_body expects (yaw, pitch, roll)).
# BottomWing: -90 degrees about Z. UnderBody: +90 degrees about Z.
BOTTOM_WING_YPR = (0.0, 0.0, 0.0)
UNDER_BODY_YPR = (pi, pi, 0.0)
UPPER_BODY_YPR = (pi / 2, 0.0, 0.0)
TOP_WING_YPR = (0.0, 0.0, 0.0)  # orientation while resting on its holder
# Final placement orientation: -90 degrees about world Z.
# Pose.from_xyz_rpy uses (roll, pitch, yaw).
TOP_WING_FINAL_RPY = (0.0 , pi , -pi / 2)
TOP_WING_FINAL_YPR = (-pi / 2, 0.0, 0.0)  # spawn_body convention: (yaw,pitch,roll)

# Mesh axis-aligned bounds (metres), measured from the STL files.
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

# The fixture's highest peg is at local STL coordinates approximately
# (0.04275, 0.1525, 0). The fixture rotation maps local +Y vertically and
# local +X to world +Y. Therefore the peg is NOT at the fixture origin.
FIXTURE_PEG_LOCAL = (0.04275, 0.1525, 0.0)


def _fixture_peg_world_pose() -> Tuple[float, float, float]:
    """Transform the measured highest-peg point from fixture-local to world."""
    fx, fy, fz = FIXTURE_POSITION
    lx, ly, lz = FIXTURE_PEG_LOCAL

    # R = Rz(yaw) * Ry(pitch) * Rx(roll), for yaw=pi/2,pitch=0,roll=pi/2:
    # world delta = (lz, lx, ly).
    return (fx + lz, fy + lx, fz + ly)


PEG_X, PEG_Y, PEG_TOP_Z = _fixture_peg_world_pose()

# Small intentional intersections at the nominal mating planes prevent thin
# visual cracks caused by mesh tessellation/rendering. This is a visual static
# assembly, not a physically constrained rigid-body joint.
ASSEMBLY_OVERLAP = 0.02  # metres (6 mm)


# ---------------------------------------------------------------------------
# World and robot
# ---------------------------------------------------------------------------

world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy, evaluate_conditions=False)

def spawn_mesh(
    mesh_path,
    position,
    ypr,
):
    return spawn_body(
        world,
        position,
        ypr,
        "mesh",
        mesh_filename=mesh_path,
    )


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
# Calculate part transforms from the STL bounds and the actual peg location
# ---------------------------------------------------------------------------

def _rotation_matrix_from_ypr(ypr: Tuple[float, float, float]):
    """Rotation matrix matching spawn_body's (yaw, pitch, roll) convention."""
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
    """Compute vertical extents after rotating all eight STL bounding corners."""
    import itertools
    import numpy as np

    lower, upper = MESH_BOUNDS[mesh_name]
    corners = np.array(list(itertools.product(
        (lower[0], upper[0]),
        (lower[1], upper[1]),
        (lower[2], upper[2]),
    )))
    rotated = corners @ _rotation_matrix_from_ypr(ypr).T
    return float(rotated[:, 2].min()), float(rotated[:, 2].max())


def calculate_assembly_positions() -> Dict[str, Tuple[float, float, float]]:
    """
    Set the three lower components at a common XY assembly datum and calculate
    their Z translations so their transformed vertical bounding planes touch.
    This removes the vertical gaps caused by applying unrotated STL Z bounds
    to parts that have been rotated in 3D.
    """
    bw_min, bw_max = _transformed_z_bounds("BottomWing", BOTTOM_WING_YPR)
    ub_min, ub_max = _transformed_z_bounds("UnderBody", UNDER_BODY_YPR)
    upper_min, upper_max = _transformed_z_bounds("UpperBody", UPPER_BODY_YPR)
    tw_min, _ = _transformed_z_bounds("TopWing", TOP_WING_FINAL_YPR)

    # UnderBody lowest transformed surface rests at the fixture peg top.
    under_z = PEG_TOP_Z - ub_min
    under_bottom = under_z + ub_min
    under_top = under_z + ub_max

    # Slightly overlap adjacent meshes so tessellation/rendering cannot show
    # a hairline crack at the nominal contact plane.
    bottom_z = under_bottom - bw_max + ASSEMBLY_OVERLAP

    # UpperBody overlaps UnderBody by the same small amount.
    upper_z = under_top - upper_min - ASSEMBLY_OVERLAP
    upper_top = upper_z + upper_max

    # TopWing lowest transformed surface is positioned at the upper surface.
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
    print("TopWing final placement rotation: -90 deg about Z")
    positions = calculate_assembly_positions()

    # Static grouped partial assembly: all three components share the fixture
    # datum and adjacent meshes overlap slightly at the calculated vertical
    # mating planes. They remain separate static mesh entities.
    plane = {
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
    return plane

# Starting position of the TopWing mesh itself
TOP_WING_START_POSITION = (0.925, 0.343, 1.0)  # above the holder, so it can be seen in RViz

def spawn_top_wing():
    print("\nSpawning TopWing at its independent starting position...")
    return spawn_mesh(
        "battat/plane/TopWing.stl",
        TOP_WING_START_POSITION,
        TOP_WING_HOLDER_YPR,
    )


# ---------------------------------------------------------------------------
# TopWing placement and manipulation
# ---------------------------------------------------------------------------

def calculate_top_wing_target() -> Pose:
    x, y, z = calculate_assembly_positions()["TopWing"]
    # Pose.from_xyz_rpy uses (roll, pitch, yaw). Rotate TopWing -90 degrees
    # around world Z in its final assembled pose.
    target = Pose.from_xyz_rpy(
        x + 0.00,
        y + 0.047,
        z - 0.07, 
        *TOP_WING_FINAL_RPY,
        reference_frame=world.root,
    )
    print("\\nCalculated TopWing target pose (180 deg about -Y):")
    print(target)
    return target


def build_top_wing_pick_and_place_plan(top_wing):
    print("\nPreparing Tracy's simulated TopWing pick-and-place plan...")

    left_arm = tracy.left_arm

    # Match the API already used by this repository's acrambly/test.py:
    # GraspCandidate receives the semantic annotation and a pose relative
    # to the object's root; PickUpAction receives that candidate and an arm.
    grasp_candidate = GraspCandidate(
        top_wing,
        Pose.from_xyz_rpy(
            -0.023775,  # Grasp point across the 10.34 cm short end.
            -0.127092,  # Negative-Y short end of the TopWing.
            0.01148,    # Centre of the mesh thickness.
            pi / 2,     # Roll: orient the gripper's local finger axis horizontally.
            0.0,        # Pitch.
            pi / 2,     # Yaw: together, these angles keep TCP +Z along local +X
                        # while rotating the gripper about that approach direction.
            reference_frame=top_wing.root,
        ),
    )

    target_pose = calculate_top_wing_target()

    return sequential(
        [
            ParkArmsAction(tracy.all_arms),
            PickUpAction(grasp_candidate, left_arm),
            # The installed PlaceAction constructor accepts the object and
            # target pose; it uses the preceding pickup to recover the arm.
            PlaceAction(top_wing, target_pose),
            ParkArmsAction(tracy.all_arms),
        ],
        context=context,
    ).plan


# ---------------------------------------------------------------------------
# Main
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
        print(part.root.global_pose)

    print("\n========== TOPWING INITIAL POSE ==========")
    print(top_wing.root.global_pose)


def main() -> None:
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)

    # Keep ROS publishing/spinning while the kinematic plan runs.
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
        print("STARTING SIMULATED TOPWING PICK AND PLACE")
        print("==============================================")

        with kinematically_simulated_robot:
            plan.perform()

        world.notify_state_change()
        print("\n==============================================")
        print("TOPWING PICK-AND-PLACE PLAN FINISHED")
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
