#!/usr/bin/env python3
"""
Real-robot TopWing peg-in-hole spiral task for ACRAMbly / Tracy.

This adapts the simulation script to the real-robot architecture on the
`manipulation/assembly` branch:

  * fetch the live world from Giskard
  * synchronise it with WorldSynchronizer
  * obtain the live AbstractRobot annotation
  * execute the motion plan under `real_robot`

IMPORTANT SAFETY NOTES
----------------------
This file is an integration scaffold, NOT a validated robot program.

The simulation's fixture/peg coordinates and TopWing start pose are NOT
assumed to be valid on the physical workstation. You must enter measured,
calibrated poses from the real robot's world frame.

The original FollowToolCenterPointPathAction maps to MoveTCPWaypointsMotion
with allow_gripper_collision=True in the current branch. That is unsuitable
for an unvalidated real insertion. This file therefore builds one-pose
MoveManipulatorAction motions using allow_gripper_collision=False.

This does NOT provide force/torque sensing, contact detection, compliant
insertion, or recovery. A position-only spiral cannot safely resolve a
mechanical jam. Do not enable physical execution until the peg/hole geometry,
grasp transform, TCP, robot workspace, collision scene, and speed/force limits
have been independently validated.

Default behaviour is a non-motion connection/preflight only. To execute:
  1. Set REAL_ROBOT_EXECUTION_ENABLED = True below.
  2. Replace ALL_MEASURED_* values with measurements in the live Giskard world.
  3. Review the path and run the operator checklist.
  4. Set operator confirmation environment variable:
       ACRAMBLY_CONFIRM_REAL_TOPWING=YES_I_HAVE_VERIFIED_THE_SETUP
  5. Run with --execute.

Example:
  source ~/ros_ws/install/setup.bash
  cd ~/Documents/cognitive_robot_abstract_machine
  python3 acrambly/test_assemble_top_wing_real.py

Dry preflight (default):
  python3 acrambly/test_assemble_top_wing_real.py

Physical motion (only after validation):
  ACRAMBLY_CONFIRM_REAL_TOPWING=YES_I_HAVE_VERIFIED_THE_SETUP \\
    python3 acrambly/test_assemble_top_wing_real.py --execute
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable

import rclpy
from rclpy.node import Node

from coraplex.alternative_motion_mappings.tracy_motion_mapping import (
    TracyRealMoveGripperMotion,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import (
    MoveManipulatorAction,
    ParkArmsAction,
)
from coraplex.datastructures.enums import Arms
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types.spatial_types import Pose


# ---------------------------------------------------------------------------
# HARD SAFETY GATE — deliberately disabled by default
# ---------------------------------------------------------------------------

REAL_ROBOT_EXECUTION_ENABLED = False

CONFIRMATION_ENV = "ACRAMBLY_CONFIRM_REAL_TOPWING"
REQUIRED_CONFIRMATION = "YES_I_HAVE_VERIFIED_THE_SETUP"

NODE_NAME = "acrambly_top_wing_real"

# Use only after checking the arm convention in your live world.
# Your simulation script uses tracy.right_arm despite naming the variable
# `left_arm`. This file makes the selected arm explicit.
ARM_SIDE = "right"

# ---------------------------------------------------------------------------
# CALIBRATION INPUTS — REPLACE WITH MEASURED LIVE-WORLD VALUES
# ---------------------------------------------------------------------------
#
# Coordinates are metres in `world.root` / the live Giskard world frame.
# Orientation values are radians, roll/pitch/yaw.
#
# None values intentionally stop the task. Do not copy simulation values
# here without physically measuring and validating them.

MEASURED_PEG_CENTER_X: float | None = None
MEASURED_PEG_CENTER_Y: float | None = None
MEASURED_PEG_TOP_Z: float | None = None

# TCP grasp pose relative to TopWing's object frame, copied from the sim code
# only as a provisional hypothesis. Confirm the grasp transform physically.
TOP_WING_TCP_GRASP_XYZ_RPY = (
    -0.023775,
    -0.127092,
    0.01148,
    math.pi / 2,
    0.0,
    math.pi / 2,
)

# The target orientation must be verified against the real gripper/TCP frame.
TARGET_TCP_RPY = (math.pi / 2, 0.0, math.pi / 2)

# The robot must already be holding the physical TopWing in the required
# grasp before insertion begins. This script intentionally does not fabricate
# a virtual TopWing or infer that it has been grasped.
REQUIRE_OPERATOR_CONFIRMED_GRASP = True

# Spiral bounds. Use only after verifying the true clearance and peg geometry.
SPIRAL_TURNS = 1
SPIRAL_STEPS = 20
SPIRAL_PITCH_METRES_PER_RADIAN = 0.002
APPROACH_Z_OFFSET_METRES = 0.030

# Slow point-to-point execution is still not force-controlled insertion.
# Review your Giskard deployment's global velocity/acceleration limits.
DWELL_BETWEEN_WAYPOINTS_SECONDS = 0.25

# Maximum radial excursion is pitch * (2*pi*turns).
MAX_SPIRAL_RADIUS_METRES = (
    SPIRAL_PITCH_METRES_PER_RADIAN * (2.0 * math.pi * SPIRAL_TURNS)
)

# Optional measured clearance envelope. A zero value disables this bound;
# set it after inspecting the real assembly geometry.
MAX_ALLOWED_SPIRAL_RADIUS_METRES: float | None = None


@dataclass(frozen=True)
class Calibration:
    peg_x: float
    peg_y: float
    peg_top_z: float


def _validate_configuration(args: argparse.Namespace) -> Calibration:
    values = (
        MEASURED_PEG_CENTER_X,
        MEASURED_PEG_CENTER_Y,
        MEASURED_PEG_TOP_Z,
    )
    if any(value is None for value in values):
        raise RuntimeError(
            "Real fixture calibration is missing. Set "
            "MEASURED_PEG_CENTER_X/Y and MEASURED_PEG_TOP_Z from measured "
            "live-world data. Simulation coordinates are not accepted."
        )

    if not REAL_ROBOT_EXECUTION_ENABLED:
        raise RuntimeError(
            "Physical execution is disabled. Review the script and explicitly "
            "set REAL_ROBOT_EXECUTION_ENABLED = True after hardware validation."
        )

    import os

    if os.environ.get(CONFIRMATION_ENV) != REQUIRED_CONFIRMATION:
        raise RuntimeError(
            f"Operator confirmation missing. Set {CONFIRMATION_ENV}="
            f"{REQUIRED_CONFIRMATION} only after the physical setup is verified."
        )

    if not args.execute:
        raise RuntimeError(
            "No --execute flag supplied. Without it this program only performs "
            "the live-world preflight."
        )

    if REQUIRE_OPERATOR_CONFIRMED_GRASP and not args.grasp_confirmed:
        raise RuntimeError(
            "The TopWing grasp is not confirmed. Confirm the actual physical "
            "grasp, then pass --grasp-confirmed."
        )

    if ARM_SIDE not in ("left", "right"):
        raise RuntimeError("ARM_SIDE must be 'left' or 'right'.")

    if SPIRAL_TURNS <= 0 or SPIRAL_STEPS < 1:
        raise RuntimeError("Invalid spiral resolution.")
    if SPIRAL_PITCH_METRES_PER_RADIAN <= 0:
        raise RuntimeError("Spiral pitch must be positive.")

    if MAX_ALLOWED_SPIRAL_RADIUS_METRES is not None:
        if MAX_SPIRAL_RADIUS_METRES > MAX_ALLOWED_SPIRAL_RADIUS_METRES:
            raise RuntimeError(
                "Configured spiral exceeds MAX_ALLOWED_SPIRAL_RADIUS_METRES."
            )

    return Calibration(
        peg_x=float(MEASURED_PEG_CENTER_X),
        peg_y=float(MEASURED_PEG_CENTER_Y),
        peg_top_z=float(MEASURED_PEG_TOP_Z),
    )


def build_spiral_tcp_poses(world, calibration: Calibration) -> list[Pose]:
    """Build fixed-orientation world-frame TCP waypoints around measured peg."""
    poses: list[Pose] = []
    total_angle = 2.0 * math.pi * SPIRAL_TURNS

    # Hold z fixed for the lateral search. The path is an XY search above
    # the measured peg top; it does not descend or force the part onto the peg.
    z = calibration.peg_top_z + APPROACH_Z_OFFSET_METRES

    for index in range(SPIRAL_STEPS + 1):
        theta = total_angle * index / SPIRAL_STEPS
        radius = SPIRAL_PITCH_METRES_PER_RADIAN * theta
        x = calibration.peg_x + radius * math.cos(theta)
        y = calibration.peg_y + radius * math.sin(theta)
        roll, pitch, yaw = TARGET_TCP_RPY

        poses.append(
            Pose.from_xyz_rpy(
                x,
                y,
                z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                reference_frame=world.root,
            )
        )
    return poses


def get_selected_arm(tracy):
    return tracy.right_arm if ARM_SIDE == "right" else tracy.left_arm


def get_selected_end_effector(tracy):
    end_effectors = list(tracy.end_effectors)
    # TracyRealMoveGripperMotion uses Arms.RIGHT/LEFT to select the gripper;
    # for TCP planning use the arm's own end effector.
    arm = get_selected_arm(tracy)
    if hasattr(arm, "end_effector"):
        return arm.end_effector
    if hasattr(arm, "get_end_effector"):
        return arm.get_end_effector()
    raise RuntimeError(
        "Could not resolve the selected arm's EndEffector from the live "
        "Tracy model. Inspect the live semantic annotation/API before moving."
    )


def build_real_spiral_plan(world, tracy, context, calibration: Calibration):
    """
    Build a real Giskard plan.

    Deliberately use MoveManipulatorAction with allow_gripper_collision=False.
    Do not use the simulation action's `allow_gripper_collision=True` setting.
    """
    arm = get_selected_arm(tracy)
    end_effector = get_selected_end_effector(tracy)
    poses = build_spiral_tcp_poses(world, calibration)

    print(f"Selected arm: {ARM_SIDE}")
    print(f"Measured peg datum: ({calibration.peg_x:.4f}, "
          f"{calibration.peg_y:.4f}, {calibration.peg_top_z:.4f})")
    print(f"Approach TCP Z: {calibration.peg_top_z + APPROACH_Z_OFFSET_METRES:.4f}")
    print(f"Waypoints: {len(poses)}")
    print(f"Maximum spiral radius: {MAX_SPIRAL_RADIUS_METRES * 1000:.2f} mm")

    actions = []
    for index, pose in enumerate(poses):
        print(f"  Prepared waypoint {index + 1:02d}/{len(poses)}")
        actions.append(
            MoveManipulatorAction(
                target_pose=pose,
                end_effector=end_effector,
                allow_gripper_collision=False,
            )
        )

    # The first action is intentionally omitted from any automatic pickup.
    # The real TopWing must already be held, and the tool must be aligned.
    return sequential(actions, context=context).plan


def fetch_live_world(node: Node):
    print("[1/4] Fetching live world from Giskard...")
    world = fetch_world_from_service(node, timeout_seconds=300)
    print(f"      Received {len(list(world.bodies))} world bodies.")

    print("[2/4] Starting world synchronisation...")
    WorldSynchronizer(_world=world, node=node, synchronous=True)
    print("      World synchronisation started.")
    return world


def create_context(world, node: Node):
    print("[3/4] Resolving Tracy robot annotation...")
    robots = world.get_semantic_annotations_by_type(AbstractRobot)
    if not robots:
        raise RuntimeError(
            "No AbstractRobot annotation found in the live Giskard world."
        )
    tracy = robots[0]

    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[TracyRealMoveGripperMotion],
        evaluate_conditions=False,
    )
    print("      Live Tracy model and real-robot Context ready.")
    return tracy, context


def run_preflight(world, tracy, calibration: Calibration | None) -> None:
    print("[4/4] Preflight only — no motion will be commanded.")
    print(f"      Robot: {tracy}")
    print(f"      Arm side configured: {ARM_SIDE}")
    if calibration is None:
        print("      Peg calibration: NOT CONFIGURED")
    else:
        print(
            "      Peg calibration: "
            f"x={calibration.peg_x:.4f}, y={calibration.peg_y:.4f}, "
            f"z={calibration.peg_top_z:.4f}"
        )
    print("")
    print("No physical motion was executed.")
    print("Before enabling execution, verify:")
    print("  - fixture and peg coordinates in the live Giskard world frame")
    print("  - TopWing grasp transform and actual gripper closure")
    print("  - TCP definition and target orientation")
    print("  - reachability, joint limits, and collision scene")
    print("  - Giskard speed/acceleration limits and E-stop")
    print("  - a validated insertion/contact strategy with force monitoring")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preflight or execute the real Tracy TopWing spiral task."
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Request physical motion (requires all independent safety gates).",
    )
    parser.add_argument(
        "--grasp-confirmed",
        action="store_true",
        help="Confirm that the real TopWing is securely grasped by the selected arm.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    try:
        world = fetch_live_world(node)
        tracy, context = create_context(world, node)

        # Dry preflight should still connect to Giskard and verify the robot.
        # Configuration errors do not prevent the operator from inspecting
        # the live world, but they always prevent plan construction/execution.
        try:
            calibration = _validate_configuration(args)
        except RuntimeError as error:
            print(f"\nEXECUTION GATE CLOSED: {error}")
            run_preflight(world, tracy, None)
            return 2

        plan = build_real_spiral_plan(world, tracy, context, calibration)

        print("\n================================================")
        print("REAL ROBOT TOPWING SPIRAL — EXECUTION REQUESTED")
        print("================================================")
        print("Keep the emergency stop reachable.")
        print("Ensure the cell is clear and the TopWing is held.")
        print("Starting in 5 seconds. Ctrl+C cancels before execution.")
        time.sleep(5.0)

        with real_robot:
            plan.perform()

        print("Motion plan returned. Verify physical seating and grasp release.")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by operator.")
        return 130
    except Exception as exc:
        print(f"\nREAL ROBOT TASK FAILED: {type(exc).__name__}: {exc}")
        raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
