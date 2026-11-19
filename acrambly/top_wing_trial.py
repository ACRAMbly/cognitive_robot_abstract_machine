#!/usr/bin/env python3
"""
Kinematically simulated TopWing pick-and-place for ACRAMbly / Tracy.

Sequence:
  1. Park arms
  2. Open gripper (animated, finger joints visibly move)
  3. Pre-grasp -> grasp
  4. Close gripper (animated); only AFTER it is closed, attach TopWing
  5. Lift, transport above insertion target
  6. XY spiral at safe height, recenter, then descend vertically
  7. Open gripper (animated); detach TopWing once the fingers have opened
  8. Retract, park

Run from the repository root in the cram-env environment:
    python3 acrambly/test_wing_spiral.py
"""

from __future__ import annotations

import itertools

import threading
import time
from math import pi
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from scipy.spatial.transform import Rotation

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import kinematically_simulated_robot
from coraplex.plans.attachment_nodes import ReAttachNode
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.composite.tool_paths import planar_spiral_xy
from coraplex.robot_plans.motions.gripper import MoveToolCenterPointMotion
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
# Mode flag
# ---------------------------------------------------------------------------

USE_REAL_ROBOT = False


# ---------------------------------------------------------------------------
# Scene configuration
# ---------------------------------------------------------------------------

TABLE_TOP_Z = 0.88
NODE_NAME = "acrambly_top_wing_assembly"

FIXTURE_POSITION = (0.925, -0.05, TABLE_TOP_Z + 0.02)
FIXTURE_YPR = (pi / 2, 0.0, pi / 2)

TOP_WING_HOLDER_POSITION = (0.925, 0.343, TABLE_TOP_Z + 0.02)
TOP_WING_HOLDER_YPR = (pi, 0.0, 0.0)

BOTTOM_WING_YPR = (0.0, 0.0, 0.0)
UNDER_BODY_YPR = (pi, pi, 0.0)
UPPER_BODY_YPR = (pi / 2, 0.0, 0.0)
TOP_WING_FINAL_RPY = (0.0, pi, -pi / 2)
TOP_WING_FINAL_YPR = (-pi / 2, 0.0, 0.0)

MESH_BOUNDS = {
    "BottomWing": (
        (-0.131022, -0.042403, -0.0014214169),
        ( 0.123255,  0.061645,  0.0276427511),
    ),
    "UnderBody": (
        (-0.037247, -0.087470, -0.0214498378),
        ( 0.039929,  0.099438,  0.0217984542),
    ),
    "UpperBody": (
        (-0.113834, -0.038627, -0.0319325514),
        ( 0.125068,  0.038553,  0.0603048913),
    ),
    "TopWing": (
        (-0.075480, -0.127092, -0.0060977931),
        ( 0.027930,  0.127542,  0.0290591735),
    ),
}

FIXTURE_PEG_LOCAL = (0.04275, 0.1525, 0.0)
ASSEMBLY_OVERLAP = 0.02

ARM_SIDE = "left"
PREGRASP_DISTANCE = 0.08
LIFT_DISTANCE = 0.08

# ---- SIDE grasp: enter through one of the long SIDE faces of TopWing ----
# This setup approaches laterally through the +X side face, 180 degrees opposite
# the previous -X side-entry approach.
# The two fingers still close along local Z, i.e. across the ~35 mm thickness.
GRASP_SIDE = "+x"            # 180-degree opposite long-side face from the previous -X approach
GRASP_DEPTH = 0.020          # metres the TCP enters past the side face
# Move the grasp point from the middle of the long side to its -Y end.
# Keep a small inset so the fingers contact the wing rather than hanging off
# the mesh boundary. Change to "+y" to use the opposite end of that same side.
GRASP_Y_END = "-y"
GRASP_END_INSET = 0.020      # metres inward from the selected end

# Actual Tracy / Robotiq tool-frame convention:
#   tool local Z = gripper approach axis
#   tool local X = finger closing axis
# Map the closing axis onto TopWing local Z (its thickness direction), so
# the two fingers are vertically opposed (up/down), not front/back.
# This sets the grasp orientation geometrically; do NOT add another wrist-3
# rotation, which was causing a 180-degree flip after the parked pose.
TOOL_APPROACH_AXIS = "z"
TOOL_CLOSING_AXIS = "x"

APPROACH_STEPS = 8
LIFT_STEPS = 6

# Joint-state signs here follow the raw DOF state used by this kinematic
# simulation. The observed close motion is left=-0.35, right=+0.35.
# The remaining finger links are coupled through the URDF mimic linkage.
GRIPPER_OPEN_POSITIONS = {
    f"{ARM_SIDE}_robotiq_85_left_knuckle_joint": 0.0,
    f"{ARM_SIDE}_robotiq_85_right_knuckle_joint": 0.0,
}
GRIPPER_CLOSED_POSITIONS = {
    f"{ARM_SIDE}_robotiq_85_left_knuckle_joint": 0.35,
    f"{ARM_SIDE}_robotiq_85_right_knuckle_joint": -0.35,
}
# A small margin leaves the pads on either side of the 35.2 mm wing thickness.
# For a Robotiq 2F-85, joint motion is not linearly equal to fingertip opening;
# therefore use the actual semantic CLOSE target, not a guessed linear fraction.
GRIPPER_CONTACT_CLOSE_FRACTION = 1.0
RETRACT_DISTANCE = 0.08

# Placement search parameters. The XY spiral stays above the plane;
# vertical insertion is a separate final phase.
SPIRAL_R_MAX = 0.04
SPIRAL_CYCLES = 3.0
SPIRAL_STEPS = 40
PLACEMENT_CLEARANCE_HEIGHT = 0.15
PLACEMENT_RECENTER_STEPS = 8
PLACEMENT_DESCENT_STEPS = 12

# Step delays (seconds)
DELAY_PARK = 1.0
DELAY_MOVE = 1.0
DELAY_SPIRAL = 0.0

# F/T thresholds (real robot only)
FT_TOPIC = "/left_ft_data"
FT_HOLE_THRESHOLD_N = 2.0
FT_INSERTION_MAX_N = 15.0


# ---------------------------------------------------------------------------
# Gripper animation configuration
# ---------------------------------------------------------------------------

GRIPPER_ANIM_FRAMES = 25
GRIPPER_ANIM_DURATION = 1.0

# ---------------------------------------------------------------------------
# Peg world position
# ---------------------------------------------------------------------------

def _fixture_peg_world_pose() -> Tuple[float, float, float]:
    fx, fy, fz = FIXTURE_POSITION
    lx, ly, lz = FIXTURE_PEG_LOCAL
    return fx + lz, fy + lx, fz + ly


PEG_X, PEG_Y, PEG_TOP_Z = _fixture_peg_world_pose()


# ---------------------------------------------------------------------------
# World and robot
# ---------------------------------------------------------------------------

world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy, evaluate_conditions=False)


def spawn_mesh(mesh_path, position, ypr):
    return spawn_body(world, position, ypr, "mesh", mesh_filename=mesh_path)


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def pose_from_matrix(matrix: np.ndarray, reference_frame) -> Pose:
    matrix = np.asarray(matrix, dtype=float)
    xyz = matrix[:3, 3]
    roll, pitch, yaw = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")
    return Pose.from_xyz_rpy(
        float(xyz[0]), float(xyz[1]), float(xyz[2]),
        float(roll), float(pitch), float(yaw),
        reference_frame=reference_frame,
    )


def translated_pose(pose: Pose, dx=0.0, dy=0.0, dz=0.0) -> Pose:
    matrix = np.array(pose.to_np(), dtype=float, copy=True)
    matrix[:3, 3] += np.array([dx, dy, dz], dtype=float)
    return pose_from_matrix(matrix, world.root)


_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def approach_offset_matrix(distance: float) -> np.ndarray:
    """Local TCP transform that moves the TCP BACK along the tool approach axis."""
    m = np.eye(4)
    m[_AXIS_INDEX[TOOL_APPROACH_AXIS], 3] = -distance
    return m


def build_object_T_tcp() -> np.ndarray:
    """Build the FINAL rotated side-entry TCP pose.

    The final wrist-3 rotation is represented in this TCP transform as well as
    being commanded at pre-grasp. This is important: otherwise the subsequent
    Cartesian grasp motion asks the IK solver for the old orientation and
    silently undoes the wrist rotation before the fingers close.
    """
    lower, upper = MESH_BOUNDS["TopWing"]
    if GRASP_Y_END == "+y":
        grasp_y = upper[1] - GRASP_END_INSET
    elif GRASP_Y_END == "-y":
        grasp_y = lower[1] + GRASP_END_INSET
    else:
        raise ValueError("GRASP_Y_END must be '-y' or '+y'")
    cz = 0.5 * (lower[2] + upper[2])

    if GRASP_SIDE == "-x":
        x_edge = lower[0]
        inward = np.array([1.0, 0.0, 0.0])
    elif GRASP_SIDE == "+x":
        x_edge = upper[0]
        inward = np.array([-1.0, 0.0, 0.0])
    else:
        raise ValueError("GRASP_SIDE must be '-x' or '+x'")

    approach = inward
    # Keep the finger gap aligned with the TopWing thickness (local Z).
    closing = np.array([0.0, 0.0, 1.0])

    ia, ic = _AXIS_INDEX[TOOL_APPROACH_AXIS], _AXIS_INDEX[TOOL_CLOSING_AXIS]
    if ia == ic:
        raise ValueError("approach and closing axes must differ")
    ik = 3 - ia - ic
    sign = 1.0 if (ic - ia) % 3 == 1 else -1.0
    R = np.zeros((3, 3))
    R[:, ia] = approach
    R[:, ic] = closing
    R[:, ik] = sign * np.cross(approach, closing)

    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = np.array([x_edge, grasp_y, cz]) + GRASP_DEPTH * inward

    return m


def wing_thickness() -> float:
    lower, upper = MESH_BOUNDS["TopWing"]
    return upper[2] - lower[2]


def linear_path(start_m: np.ndarray, end_m: np.ndarray, n: int) -> List[Pose]:
    """Straight-line Cartesian waypoints (constant orientation of end_m)."""
    poses = []
    for i in range(1, n + 1):
        a = i / n
        m = end_m.copy()
        m[:3, 3] = (1 - a) * start_m[:3, 3] + a * end_m[:3, 3]
        poses.append(pose_from_matrix(m, world.root))
    return poses


def move_through(arm, poses: List[Pose], delay: float = 0.05) -> None:
    for pose in poses:
        step(MoveToolCenterPointMotion(
            target=pose, arm=arm, allow_gripper_collision=False,
        ), delay=delay)


# ---------------------------------------------------------------------------
# Single-step execution with RViz visualization
# ---------------------------------------------------------------------------

def step(action, delay: float = 0.5) -> None:
    sequential([action], context=context).plan.perform()
    world.notify_state_change()
    time.sleep(delay)


# ---------------------------------------------------------------------------
# Gripper joint access + animation
# ---------------------------------------------------------------------------

def _find_gripper_dofs() -> List[Tuple[str, object]]:
    """Return the semantic Tracy gripper connections, not their raw DOFs.

    ActiveConnection1DOF.position applies the connection multiplier and offset
    on reads and inversely maps semantic targets onto the shared raw DOF on
    writes. This is essential for the right knuckle, whose connection may use
    a negative multiplier for the Robotiq mimic linkage.
    """
    expected = set(GRIPPER_OPEN_POSITIONS)
    found: Dict[str, object] = {}

    for conn in world.connections:
        conn_name = getattr(conn, "name", None)
        bare_name = getattr(conn_name, "name", str(conn_name))
        if bare_name in expected and hasattr(conn, "position"):
            found[bare_name] = conn

    missing = expected.difference(found)
    if missing:
        available = sorted(
            getattr(getattr(c, "name", None), "name", str(getattr(c, "name", "")))
            for c in world.connections
            if "robotiq" in str(getattr(c, "name", "")).lower()
        )
        raise RuntimeError(
            "Could not find the active Tracy Robotiq knuckle connection(s): "
            + ", ".join(sorted(missing))
            + ". Robotiq connections present in this world: "
            + (", ".join(available) if available else "NONE")
            + ". Check ARM_SIDE and the Tracy URDF."
        )

    return [(name, found[name]) for name in GRIPPER_OPEN_POSITIONS]


def _get_pos(connection) -> float:
    """Read the connection position in its semantic (multiplier-applied) frame."""
    return float(connection.position)


def _set_pos(connection, value: float) -> None:
    """Set the semantic connection position; its setter handles mimic scaling."""
    connection.position = float(value)


def _snapshot(dofs) -> Dict[str, float]:
    return {name: _get_pos(dof) for name, dof in dofs}


def _apply(dofs, values: Dict[str, float]) -> None:
    """Set both opposing knuckles and publish the updated kinematic world."""
    for name, dof in dofs:
        _set_pos(dof, values[name])
    world.notify_state_change()


def _gripper_targets(state: GripperState) -> Dict[str, float]:
    if state == GripperState.OPEN:
        return dict(GRIPPER_OPEN_POSITIONS)
    if state == GripperState.CLOSE:
        return dict(GRIPPER_CLOSED_POSITIONS)
    raise ValueError(f"Unsupported gripper state: {state}")


def animate_gripper(arm, state: GripperState,
                    close_fraction: float = 1.0) -> None:
    """Animate the actual Tracy Robotiq knuckle joints between open/closed.

    MoveGripperMotion is deliberately not used here: its perform() is a no-op
    in the kinematic simulation and the Giskard joint task is not executed by
    directly writing the world state. Tracy's semantic model provides the
    authoritative joint targets; the two knuckles are driven symmetrically in
    opposite directions and the URDF mimic joints move the finger links.
    """
    if not 0.0 <= close_fraction <= 1.0:
        raise ValueError("close_fraction must be in [0, 1]")

    dofs = _find_gripper_dofs()
    start = _snapshot(dofs)
    target = _gripper_targets(state)

    if state == GripperState.CLOSE and close_fraction < 1.0:
        target = {
            name: start[name] + close_fraction * (target[name] - start[name])
            for name in target
        }

    print(f"   [gripper] {state.name}: "
          + ", ".join(f"{name}={start[name]:.3f}->{target[name]:.3f}"
                       for name in target))

    # Interpolate from the current measured configuration to the actual
    # commanded joint target. Publish every frame so RViz updates the fingers.
    for frame in range(1, GRIPPER_ANIM_FRAMES + 1):
        alpha = frame / GRIPPER_ANIM_FRAMES
        values = {
            name: start[name] + alpha * (target[name] - start[name])
            for name in target
        }
        _apply(dofs, values)
        time.sleep(GRIPPER_ANIM_DURATION / GRIPPER_ANIM_FRAMES)

    # Verify the final state rather than silently treating a non-moving gripper
    # as a successful grasp/release.
    final = _snapshot(dofs)
    for name, expected in target.items():
        if abs(final[name] - expected) > 1e-4:
            raise RuntimeError(
                f"Gripper joint {name} did not reach target: "
                f"actual={final[name]:.5f}, expected={expected:.5f}"
            )


# ---------------------------------------------------------------------------
# F/T monitor (real robot only)
# ---------------------------------------------------------------------------

class FTMonitor:
    def __init__(self, node):
        self._fz = 0.0
        self._lock = threading.Lock()
        node.create_subscription(WrenchStamped, FT_TOPIC, self._cb, 10)

    def _cb(self, msg: WrenchStamped) -> None:
        with self._lock:
            self._fz = msg.wrench.force.z

    @property
    def fz(self) -> float:
        with self._lock:
            return self._fz


# ---------------------------------------------------------------------------
# Fixture and part spawning
# ---------------------------------------------------------------------------

def spawn_fixture_and_holders() -> None:
    print("\nSpawning fixture and component holders...")
    spawn_mesh("fixtures/big_wooden_plate.stl",
               (0.7, 0.0, TABLE_TOP_Z + 0.01), (0.0, 0.0, 0.0))
    spawn_mesh("battat/BoltHolder.stl",
               (1.01, 0.575, TABLE_TOP_Z + 0.02), (pi, 0.0, 0.0))
    spawn_mesh("battat/HolderUpperBody.stl",
               (0.91, 0.5, TABLE_TOP_Z + 0.02), (pi / 2, 0.0, 0.0))
    # Keep the dedicated TopWing fixture beneath the staged wing.  The
    # separate UnderBody staging support remains omitted.
    spawn_mesh("battat/HolderTopWing.stl",
               TOP_WING_HOLDER_POSITION, TOP_WING_HOLDER_YPR)
    spawn_mesh("battat/Plane_horizontal_holder_tall.stl",
               FIXTURE_POSITION, FIXTURE_YPR)
    spawn_mesh("battat/HolderBottomWing.stl",
               (0.925, -0.357, TABLE_TOP_Z + 0.02), (0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Assembly position calculations
# ---------------------------------------------------------------------------

def rotation_matrix_from_ypr(ypr: Tuple[float, float, float]) -> np.ndarray:
    yaw, pitch, roll = ypr
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ], dtype=float)


def transformed_z_bounds(mesh_name: str, ypr: Tuple[float, float, float]):
    import itertools
    lower, upper = MESH_BOUNDS[mesh_name]
    corners = np.array(list(itertools.product(
        (lower[0], upper[0]),
        (lower[1], upper[1]),
        (lower[2], upper[2]),
    )), dtype=float)
    rotated = corners @ rotation_matrix_from_ypr(ypr).T
    return float(rotated[:, 2].min()), float(rotated[:, 2].max())


def calculate_assembly_positions() -> Dict[str, Tuple[float, float, float]]:
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
        "UnderBody":  (PEG_X, PEG_Y, under_z - 0.02),
        "UpperBody":  (PEG_X, PEG_Y - 0.035, upper_z - 0.03),
        "TopWing":    (PEG_X, PEG_Y, top_wing_z),
    }


def spawn_partial_plane():
    print("\nSpawning the partially assembled plane on the fixture...")
    print(f"Lower-part assembly overlap: {ASSEMBLY_OVERLAP * 1000:.1f} mm")
    positions = calculate_assembly_positions()
    return {
        "BottomWing": spawn_mesh(
            "battat/plane/BottomWing.stl",
            positions["BottomWing"], BOTTOM_WING_YPR),
        "UnderBody": spawn_mesh(
            "battat/plane/UnderBody.stl",
            positions["UnderBody"], UNDER_BODY_YPR),
        "UpperBody": spawn_mesh(
            "battat/plane/UpperBody.stl",
            positions["UpperBody"], UPPER_BODY_YPR),
    }


TOP_WING_START_POSITION = (0.925, 0.343, 1.0)


def spawn_top_wing():
    print("\nSpawning TopWing at its independent starting position...")
    return spawn_mesh(
        "battat/plane/TopWing.stl",
        TOP_WING_START_POSITION, TOP_WING_HOLDER_YPR)


# ---------------------------------------------------------------------------
# Spiral insertion waypoints
# ---------------------------------------------------------------------------

def build_safe_placement_poses(
    tcp_target_matrix: np.ndarray,
    r_max: float = SPIRAL_R_MAX,
    cycles: float = SPIRAL_CYCLES,
    n_spiral_steps: int = SPIRAL_STEPS,
    clearance_height: float = PLACEMENT_CLEARANCE_HEIGHT,
    n_recenter_steps: int = PLACEMENT_RECENTER_STEPS,
    n_descent_steps: int = PLACEMENT_DESCENT_STEPS,
) -> Tuple[List[Pose], List[Pose], List[Pose]]:
    """Spiral at fixed safe Z, recenter at fixed Z, then descend vertically."""
    target = np.asarray(tcp_target_matrix, dtype=float)
    safe_z = float(target[2, 3] + clearance_height)

    spiral_poses: List[Pose] = []
    for i in range(n_spiral_steps + 1):
        tau = i / n_spiral_steps
        xy = planar_spiral_xy(tau, r0=0.0, r1=r_max, cycles=cycles)
        waypoint = target.copy()
        waypoint[0, 3] += float(xy[0])  # world X
        waypoint[1, 3] += float(xy[1])  # world Y
        waypoint[2, 3] = safe_z
        spiral_poses.append(pose_from_matrix(waypoint, world.root))

    last_xy = np.asarray(
        planar_spiral_xy(1.0, r0=0.0, r1=r_max, cycles=cycles),
        dtype=float,
    )
    recenter_poses: List[Pose] = []
    for i in range(1, n_recenter_steps + 1):
        alpha = i / n_recenter_steps
        waypoint = target.copy()
        waypoint[0, 3] += float((1.0 - alpha) * last_xy[0])
        waypoint[1, 3] += float((1.0 - alpha) * last_xy[1])
        waypoint[2, 3] = safe_z
        recenter_poses.append(pose_from_matrix(waypoint, world.root))

    descent_poses: List[Pose] = []
    for i in range(1, n_descent_steps + 1):
        alpha = i / n_descent_steps
        waypoint = target.copy()
        waypoint[2, 3] = safe_z + alpha * (target[2, 3] - safe_z)
        descent_poses.append(pose_from_matrix(waypoint, world.root))

    return spiral_poses, recenter_poses, descent_poses


# ---------------------------------------------------------------------------
# TopWing target
# ---------------------------------------------------------------------------

# The UpperBody is spawned 30 mm below its nominal calculated origin.
# Compute the TopWing's actual rotated mesh bounds from the SAME RPY orientation
# used by Pose.from_xyz_rpy below. Do not use TOP_WING_FINAL_YPR here: that
# tuple describes a different rotation and caused an approximately 35 mm air gap.
# Use a small deliberate overlap so the visible TopWing mesh seats on the
# UpperBody instead of stopping at a nominal bounding-box tangent.  The prior
# run finished with the TopWing bottom about 0.46 mm above the calculated
# UpperBody top after IK error; 6 mm gives a modest seating margin.
TOP_WING_SEATING_OVERLAP = 0.006  # 6 mm below the UpperBody mesh-bound top


def rotated_mesh_z_bounds_from_rpy(
    mesh_name: str, rpy: Tuple[float, float, float]
) -> Tuple[float, float]:
    lower, upper = MESH_BOUNDS[mesh_name]
    corners = np.array(list(itertools.product(
        (lower[0], upper[0]),
        (lower[1], upper[1]),
        (lower[2], upper[2]),
    )), dtype=float)
    rotation = Rotation.from_euler("xyz", rpy).as_matrix()
    rotated = corners @ rotation.T
    return float(rotated[:, 2].min()), float(rotated[:, 2].max())


def calculate_top_wing_target() -> Pose:
    positions = calculate_assembly_positions()
    upper_nominal_z = positions["UpperBody"][2] + 0.03
    _, upper_max = transformed_z_bounds("UpperBody", UPPER_BODY_YPR)

    # Actual UpperBody spawn origin includes the -30 mm correction in
    # calculate_assembly_positions().
    upper_body_top_z = upper_nominal_z - 0.03 + upper_max

    # Use the exact final RPY used by the target pose, not the stale YPR tuple.
    top_wing_min_z, _ = rotated_mesh_z_bounds_from_rpy(
        "TopWing", TOP_WING_FINAL_RPY
    )
    target_z = (
        upper_body_top_z - top_wing_min_z - TOP_WING_SEATING_OVERLAP
    )

    x = positions["TopWing"][0]
    y = positions["TopWing"][1] + 0.047
    target = Pose.from_xyz_rpy(
        x, y, target_z,
        *TOP_WING_FINAL_RPY,
        reference_frame=world.root,
    )
    print("\nCalculated TopWing target pose (mesh-bound contact):")
    print(target)
    predicted_wing_bottom_z = target_z + top_wing_min_z
    print(
        f"  UpperBody top Z: {upper_body_top_z:.5f} m; "
        f"TopWing rotated bottom offset: {top_wing_min_z:.5f} m"
    )
    print(
        f"  Predicted TopWing bottom Z: {predicted_wing_bottom_z:.5f} m; "
        f"target overlap below UpperBody top: "
        f"{(upper_body_top_z - predicted_wing_bottom_z)*1000:.1f} mm"
    )
    return target


# ---------------------------------------------------------------------------
# Full animated pick-transport-spiral sequence
# ---------------------------------------------------------------------------

def run_assembly(top_wing, arm, ft_monitor):
    tool_frame = arm.end_effector.tool_frame

    # --- Compute all poses up front ---
    object_T_tcp_matrix = build_object_T_tcp()

    # The grasp and pre-grasp share the corrected up/down finger orientation.
    # No extra wrist-3 joint command is applied: the TCP pose already encodes
    # the required 90-degree change relative to the previous front/back pose.
    object_T_tcp_pregrasp_matrix = object_T_tcp_matrix.copy()

    world_T_object = np.asarray(top_wing.global_pose.to_np(), dtype=float)
    world_T_tcp_grasp_matrix = world_T_object @ object_T_tcp_matrix
    world_T_tcp_grasp = pose_from_matrix(world_T_tcp_grasp_matrix, world.root)

    world_T_tcp_pregrasp_matrix = (
        world_T_object @ object_T_tcp_pregrasp_matrix
        @ approach_offset_matrix(PREGRASP_DISTANCE))
    world_T_tcp_pregrasp = pose_from_matrix(
        world_T_tcp_pregrasp_matrix, world.root)

    world_T_tcp_lift_matrix = world_T_tcp_grasp_matrix.copy()
    world_T_tcp_lift_matrix[2, 3] += LIFT_DISTANCE
    world_T_tcp_lift = pose_from_matrix(world_T_tcp_lift_matrix, world.root)

    # Diagnostic: approach direction in world frame
    approach_vec = (world_T_tcp_grasp_matrix[:3, 3]
                    - world_T_tcp_pregrasp_matrix[:3, 3])
    approach_vec /= np.linalg.norm(approach_vec)
    close_fraction = GRIPPER_CONTACT_CLOSE_FRACTION
    closing_dir_world = world_T_tcp_grasp_matrix[
        :3, _AXIS_INDEX[TOOL_CLOSING_AXIS]]
    print("\nApproach check:")
    print(f"  side-entry face: {GRASP_SIDE}X; grasp location: {GRASP_Y_END}Y end "
          f"(inset {GRASP_END_INSET * 1000:.0f} mm), depth "
          f"{GRASP_DEPTH * 1000:.0f} mm")
    print(f"  tool approach axis = +{TOOL_APPROACH_AXIS}, "
          f"closing axis = {TOOL_CLOSING_AXIS}")
    print(f"  world closing direction = {np.round(closing_dir_world, 3)}")
    print("  finger gap aligned across TopWing local Z thickness")
    print(f"  wing thickness {wing_thickness() * 1000:.1f} mm")
    print("  commanded Robotiq knuckle targets:", GRIPPER_CLOSED_POSITIONS)
    print(f"  world approach direction = {np.round(approach_vec, 3)}")
    print(f"  TCP axes in world (cols x,y,z):\n"
          f"{np.round(world_T_tcp_grasp_matrix[:3, :3], 3)}")
    print(f"  TopWing origin in world = "
          f"{np.round(world_T_object[:3, 3], 3)}")

    target_object_pose = calculate_top_wing_target()
    world_T_object_target = np.asarray(target_object_pose.to_np(), dtype=float)
    world_T_tcp_target_matrix = world_T_object_target @ object_T_tcp_matrix
    world_T_tcp_target = pose_from_matrix(world_T_tcp_target_matrix, world.root)

    world_T_tcp_above_target = translated_pose(
        world_T_tcp_target, dz=PLACEMENT_CLEARANCE_HEIGHT)
    # After release: back out along the approach axis (out of the side face),
    # then lift. Lifting straight up first would drag a finger through the wing.
    world_T_tcp_back_matrix = (
        world_T_tcp_target_matrix @ approach_offset_matrix(RETRACT_DISTANCE))
    world_T_tcp_back = pose_from_matrix(world_T_tcp_back_matrix, world.root)
    world_T_tcp_retract = translated_pose(world_T_tcp_back, dz=RETRACT_DISTANCE)

    spiral_poses, recenter_poses, descent_poses = build_safe_placement_poses(
        world_T_tcp_target_matrix
    )

    print(f"\nGrasp transform (object frame):\n"
          f"{np.round(object_T_tcp_matrix, 5)}")
    print(
        f"\nSafe placement path: {len(spiral_poses)} spiral waypoints "
        f"at +{PLACEMENT_CLEARANCE_HEIGHT*1000:.0f} mm, "
        f"{len(recenter_poses)} recenter waypoints, "
        f"{len(descent_poses)} vertical descent waypoints; "
        f"spiral radius={SPIRAL_R_MAX*1000:.0f} mm"
    )
    print("\nGripper joints found:",
          [n for n, _ in _find_gripper_dofs()] or "NONE")

    with kinematically_simulated_robot:

        print("\n[1/10] Parking arms...")
        step(ParkArmsAction(tracy.all_arms), delay=DELAY_PARK)

        print("[2/10] Opening gripper...")
        animate_gripper(arm, GripperState.OPEN)

        print("[3/10] Moving to pre-grasp with fingers oriented up/down...")
        step(MoveToolCenterPointMotion(
            target=world_T_tcp_pregrasp, arm=arm,
            allow_gripper_collision=False,
        ), delay=DELAY_MOVE)

        print("[4/10] Approaching along the gripper approach axis...")
        move_through(arm, linear_path(
            world_T_tcp_pregrasp_matrix, world_T_tcp_grasp_matrix,
            APPROACH_STEPS))

        print("[5/10] Closing gripper on TopWing...")
        animate_gripper(arm, GripperState.CLOSE, close_fraction=close_fraction)

        # In this kinematic-only scene there is no contact dynamics. The fingers
        # are genuinely moved through their URDF joints above; after that verified
        # closure, re-parent the part so it follows the gripper during transport.
        print("[6/10] Gripper closure verified -> holding TopWing in kinematic sim...")
        step(ReAttachNode(body=top_wing, new_parent=tool_frame), delay=0.2)

        print("[7/10] Lifting (straight up)...")
        move_through(arm, linear_path(
            world_T_tcp_grasp_matrix, world_T_tcp_lift_matrix, LIFT_STEPS))

        print("[8/10] Transporting to safe height above insertion target...")
        step(MoveToolCenterPointMotion(
            target=world_T_tcp_above_target, arm=arm,
            allow_gripper_collision=False,
        ), delay=DELAY_MOVE)

        print(
            f"[9/10] XY spiral at fixed clearance height "
            f"({len(spiral_poses)} waypoints)..."
        )
        for i, pose in enumerate(spiral_poses):
            print(f"       spiral {i + 1}/{len(spiral_poses)}", end="\\r")
            sequential([MoveToolCenterPointMotion(
                target=pose, arm=arm,
                allow_gripper_collision=False,
            )], context=context).plan.perform()
            world.notify_state_change()
            time.sleep(DELAY_SPIRAL)

        print("\n       Spiral complete. Recentering above the plane...")
        move_through(arm, recenter_poses, delay=0.02)

        print("       XY centred. Starting final vertical insertion...")
        for i, pose in enumerate(descent_poses):
            sequential([MoveToolCenterPointMotion(
                target=pose, arm=arm,
                allow_gripper_collision=False,
            )], context=context).plan.perform()
            world.notify_state_change()

            if USE_REAL_ROBOT and ft_monitor is not None:
                time.sleep(0.02)
                fz = ft_monitor.fz
                if abs(fz) < FT_HOLE_THRESHOLD_N and i > 0:
                    print(
                        f"\n       Low insertion force at descent "
                        f"{i + 1}/{len(descent_poses)} (Fz={fz:.2f} N)"
                    )
                    break
                if abs(fz) > FT_INSERTION_MAX_N:
                    raise RuntimeError(
                        f"Insertion force limit exceeded: Fz={fz:.2f} N. "
                        "Stopping before release."
                    )

        if not USE_REAL_ROBOT:
            print("       Simulated vertical insertion reached target pose.")
        else:
            print("       Vertical insertion phase finished.")

        print("\n[10/10] Releasing TopWing: opening gripper...")
        # Release the object first so it stays at the insertion pose
        # (re-parent to world keeps its current world pose), then open the
        # fingers visibly.
        step(ReAttachNode(body=top_wing, new_parent=world.root), delay=0.0)
        animate_gripper(arm, GripperState.OPEN)

        print("Retracting out through the side face, then lifting...")
        step(MoveToolCenterPointMotion(
            target=world_T_tcp_back, arm=arm,
            allow_gripper_collision=False,
        ), delay=DELAY_MOVE)
        step(MoveToolCenterPointMotion(
            target=world_T_tcp_retract, arm=arm,
            allow_gripper_collision=False,
        ), delay=DELAY_MOVE)

        print("Parking...")
        step(ParkArmsAction(tracy.all_arms), delay=DELAY_PARK)


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------

def print_scene(plane, top_wing) -> None:
    print("\n========== FIXTURE LOCATING PEG ==========")
    print(f"Peg datum (x, y, z): "
          f"({PEG_X:.5f}, {PEG_Y:.5f}, {PEG_TOP_Z:.5f})")
    print("\n========== PARTIALLY ASSEMBLED PLANE ==========")
    for name, part in plane.items():
        print(f"\n{name} world pose:")
        print(part.global_pose)
    print("\n========== TOPWING INITIAL POSE ==========")
    print(top_wing.global_pose)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)

    spinner = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    try:
        TFPublisher(node=node, _world=world)
        VizMarkerPublisher(_world=world, node=node)

        ft_monitor = FTMonitor(node) if USE_REAL_ROBOT else None

        spawn_fixture_and_holders()
        plane = spawn_partial_plane()
        top_wing = spawn_top_wing()

        world.notify_state_change()
        print_scene(plane, top_wing)

        print("\nInitial scene published. Open RViz now.")
        print("Add two MarkerArray displays:")
        print("  Topic: /coraplex/viz_marker  (assembly parts)")
        print("  Topic: /semworld/viz_marker  (robot)")
        time.sleep(3.0)

        arm = tracy.left_arm if ARM_SIDE == "left" else tracy.right_arm

        print("\n==============================================")
        print("STARTING: OPPOSITE-END THICKNESS GRASP → TRANSPORT → PLACE")
        print("==============================================")

        run_assembly(top_wing, arm, ft_monitor)

        world.notify_state_change()
        print("\n==============================================")
        print("ASSEMBLY COMPLETE")
        print("==============================================")
        print("\nFinal TopWing world pose:")
        print(top_wing.global_pose)
        print("\nRViz remains active. Press Ctrl+C to exit.")

        while rclpy.ok():
            world.notify_state_change()
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping scene...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()