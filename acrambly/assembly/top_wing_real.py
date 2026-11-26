#!/usr/bin/env python3
"""
TopWing pick -> transport -> force-guided peg-in-hole insertion on Tracy.

Two modes, chosen by USE_REAL_ROBOT:

  REAL (True)
    * world is fetched from the running Giskard server and kept in sync
      (same pattern as acrambly/tracy_real.py)
    * every arm motion runs inside `with real_robot:` and is executed by Giskard
    * the Robotiq gripper is commanded directly through its ROS 2 action server
      /<side>_gripper/robotiq_gripper_controller/gripper_cmd
      (TracyRealMoveGripperMotion is not used: it imports `Arms`, which no
      longer exists in coraplex.datastructures.enums on this branch)
    * insertion is force-guided: descend until contact, spiral at contact
      height until the contact force drops (hole found), then push down
    * an F/T watchdog cancels the running Giskard goal if the contact force
      exceeds FT_INSERTION_MAX_N

  SIM (False)
    * local URDF world, kinematic simulation, animated finger joints,
      visual spiral with a simulated "hole found" (unchanged behaviour)

Before a REAL run (restart Giskard every run):
    ros2 launch giskardpy_ros giskardpy_tracy_velocity.launch.py
Then, from the repository root in cram-env:
    python3 acrambly/test_wing_spiral.py

Keep the E-stop in reach.
"""

from __future__ import annotations

import threading
import time
from math import pi
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import kinematically_simulated_robot, real_robot
from coraplex.plans.attachment_nodes import ReAttachNode
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.composite.tool_paths import planar_spiral_xy
from coraplex.robot_plans.motions.gripper import MoveToolCenterPointMotion
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy
from krrood_spatial_math.spatial_types import Pose

from acrambly.sub_parts.shared.utils import spawn_body

from rclpy.executors import MultiThreadedExecutor


# ---------------------------------------------------------------------------
# Mode flag
# ---------------------------------------------------------------------------

USE_REAL_ROBOT = True


# ---------------------------------------------------------------------------
# Scene configuration
# NOTE (real robot): these are nominal positions. The fixture, holder and the
# TopWing must physically be where these numbers say, otherwise the grasp and
# insertion poses are wrong. Measure and correct them before the first run.
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
TOP_WING_START_POSITION = (0.925, 0.343, 1.0)

ARM_SIDE = "left"
PREGRASP_DISTANCE = 0.08
LIFT_DISTANCE = 0.08
SAFE_CLEAR_EXTRA = 0.25      # extra rise after lift so the wing clears the fixture
RETRACT_DISTANCE = 0.08

# ---- Side grasp: enter through the +X long face, near the -Y end ----
GRASP_SIDE = "+x"
GRASP_DEPTH = 0.020
GRASP_Y_END = "-y"
GRASP_END_INSET = 0.020

# Tracy / Robotiq tool frame: z = approach axis, x = finger closing axis
TOOL_APPROACH_AXIS = "z"
TOOL_CLOSING_AXIS = "x"

APPROACH_STEPS = 8
LIFT_STEPS = 6


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------

# Kinematic sim: semantic knuckle targets used for the finger animation.
GRIPPER_OPEN_POSITIONS = {
    f"{ARM_SIDE}_robotiq_85_left_knuckle_joint": 0.0,
    f"{ARM_SIDE}_robotiq_85_right_knuckle_joint": 0.0,
}
GRIPPER_CLOSED_POSITIONS = {
    f"{ARM_SIDE}_robotiq_85_left_knuckle_joint": 0.35,
    f"{ARM_SIDE}_robotiq_85_right_knuckle_joint": -0.35,
}
GRIPPER_CONTACT_CLOSE_FRACTION = 1.0
GRIPPER_ANIM_FRAMES = 25
GRIPPER_ANIM_DURATION = 1.0

# Real robot: ParallelGripperCommand goal on the Robotiq controller.
# The semantic CLOSE value (0.35 rad) leaves roughly a 45-50 mm gap on a
# 2F-85, which does NOT touch a 35 mm wing. Command (almost) full closure and
# let the effort limit stop the fingers on the part.
GRIPPER_ACTION_TOPIC = f"/{ARM_SIDE}_gripper/robotiq_gripper_controller/gripper_cmd"
GRIPPER_REAL_OPEN_POSITION = 0.0
GRIPPER_REAL_CLOSE_POSITION = 0.79   # knuckle joint limit is ~0.8 rad
GRIPPER_REAL_EFFORT = 10.0           # same value the team mapping uses
GRIPPER_REAL_TIMEOUT_S = 10.0


# ---------------------------------------------------------------------------
# Insertion
# ---------------------------------------------------------------------------

# Kinematic sim: visual descending spiral (big radius so it is visible in RViz)
SIM_SPIRAL_R_MAX = 0.04
SIM_SPIRAL_CYCLES = 3.0
SIM_SPIRAL_Z_DESCENT = 0.025
SIM_SPIRAL_STEPS = 40

# Real robot: descend to contact, spiral at contact height, push into hole.
REAL_DESCENT_STEP = 0.002      # m per Giskard goal while searching for contact
REAL_DESCENT_MAX = 0.040       # m below the start (above_target) before giving up
REAL_PRESS_DEPTH = 0.001       # m below contact height while spiralling
REAL_SPIRAL_R_MAX = 0.010      # m, search radius (keep small on hardware)
REAL_SPIRAL_CYCLES = 3.0
REAL_SPIRAL_STEPS = 60         # ~3 mm max between waypoints on the outer ring
REAL_INSERT_STEP = 0.002       # m per goal for the final push into the hole

# F/T: wrench_filtered from the UR force_torque_sensor_broadcaster.
# All thresholds act on |F - F_tare|, the change of the force vector since the
# tare taken in free air right above the target. This works regardless of the
# sensor frame orientation (with the side grasp the vertical direction is not
# the sensor z axis) and removes the wing weight / sensor bias.
FT_TOPIC = f"/{ARM_SIDE}_arm/force_torque_sensor_broadcaster/wrench"
FT_CONTACT_N = 3.0             # above this -> touching the surface
FT_HOLE_THRESHOLD_N = 1.5      # after contact, below this -> peg fell into hole
FT_INSERTION_MAX_N = 15.0      # watchdog cancels the Giskard goal above this
FT_TARE_SAMPLES = 25

# Delays
DELAY_PARK = 1.0
DELAY_MOVE = 1.0
DELAY_SPIRAL_SIM = 0.0
REAL_SETTLE_S = 0.15           # wait after each real motion before reading F/T


# ---------------------------------------------------------------------------
# Peg world position
# ---------------------------------------------------------------------------

def _fixture_peg_world_pose() -> Tuple[float, float, float]:
    fx, fy, fz = FIXTURE_POSITION
    lx, ly, lz = FIXTURE_PEG_LOCAL
    return fx + lz, fy + lx, fz + ly


PEG_X, PEG_Y, PEG_TOP_Z = _fixture_peg_world_pose()


# ---------------------------------------------------------------------------
# Globals filled in by setup_world()
# ---------------------------------------------------------------------------

world = None
tracy = None
context: Optional[Context] = None


def setup_world(node) -> None:
    """Build world, robot and context for the selected mode."""
    global world, tracy, context

    if USE_REAL_ROBOT:
        print("Fetching live world from Giskard (is Giskard running?)...")
        world = fetch_world_from_service(node, timeout_seconds=300)
        print(f"World received with {len(list(world.bodies))} bodies.")
        WorldSynchronizer(_world=world, node=node)
        print("World synchronized with Giskard.")
        tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]
        context = Context(
            world=world,
            robot=tracy,
            ros_node=node,
            evaluate_conditions=False,
        )
    else:
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


def translated_matrix(matrix: np.ndarray, dx=0.0, dy=0.0, dz=0.0) -> np.ndarray:
    m = np.array(matrix, dtype=float, copy=True)
    m[:3, 3] += np.array([dx, dy, dz], dtype=float)
    return m


_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def approach_offset_matrix(distance: float) -> np.ndarray:
    """Local TCP transform that moves the TCP back along the approach axis."""
    m = np.eye(4)
    m[_AXIS_INDEX[TOOL_APPROACH_AXIS], 3] = -distance
    return m


def build_object_T_tcp() -> np.ndarray:
    """Side-entry grasp in the TopWing frame, fingers closing across its thickness."""
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
    """Straight-line Cartesian waypoints (orientation of end_m)."""
    poses = []
    for i in range(1, n + 1):
        a = i / n
        m = end_m.copy()
        m[:3, 3] = (1 - a) * start_m[:3, 3] + a * end_m[:3, 3]
        poses.append(pose_from_matrix(m, world.root))
    return poses


# ---------------------------------------------------------------------------
# Execution helpers
# ---------------------------------------------------------------------------

def step(action, delay: float = 0.5) -> None:
    """
    Execute one action.

    Sim: publish the new state so RViz animates, then wait.
    Real: Giskard owns the robot state and the synchronizer keeps this world
    up to date, so this client must NOT publish state (notify_state_change
    broadcasts to the other processes). perform() blocks until Giskard is done.
    """
    sequential([action], context=context).plan.perform()
    if USE_REAL_ROBOT:
        time.sleep(REAL_SETTLE_S)
    else:
        world.notify_state_change()
        time.sleep(delay)


def move_tcp(arm, pose: Pose, delay: float = DELAY_MOVE) -> None:
    step(MoveToolCenterPointMotion(
        target=pose, arm=arm, allow_gripper_collision=False,
    ), delay=delay)


def move_through(arm, poses: List[Pose], delay: float = 0.05) -> None:
    for pose in poses:
        move_tcp(arm, pose, delay=delay)


# ---------------------------------------------------------------------------
# Real gripper: Robotiq via ROS 2 action server
# ---------------------------------------------------------------------------

class RealGripper:
    """Sends ParallelGripperCommand goals to the Robotiq controller and waits."""

    def __init__(self, node, action_topic: str):
        self._node = node
        self._topic = action_topic
        self._client = ActionClient(node, ParallelGripperCommand, action_topic)

    def wait_for_server(self, timeout_s: float = 10.0) -> None:
        print(f"   [gripper] waiting for {self._topic} ...")
        if not self._client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(
                f"Gripper action server {self._topic} not available. "
                "Check `ros2 action list | grep gripper` and that the "
                "robotiq_gripper_controller is active."
            )

    @staticmethod
    def _wait(future, timeout_s: float, what: str):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout_s:
                raise TimeoutError(f"Gripper {what} timed out after {timeout_s} s")
            time.sleep(0.01)
        return future.result()

    def command(self, position: float, effort: float = GRIPPER_REAL_EFFORT) -> None:
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(position)]
        goal.command.effort = [float(effort)]
        print(f"   [gripper] position={position:.3f} effort={effort:.1f}")
        handle = self._wait(self._client.send_goal_async(goal),
                            GRIPPER_REAL_TIMEOUT_S, "goal acceptance")
        if not handle.accepted:
            raise RuntimeError("Gripper goal rejected by the action server")
        self._wait(handle.get_result_async(), GRIPPER_REAL_TIMEOUT_S, "result")
        time.sleep(0.3)  # let the fingers settle on the part

    def open(self) -> None:
        self.command(GRIPPER_REAL_OPEN_POSITION)

    def close(self) -> None:
        self.command(GRIPPER_REAL_CLOSE_POSITION)


# ---------------------------------------------------------------------------
# Sim gripper: animate the knuckle joints directly
# ---------------------------------------------------------------------------

def _find_gripper_dofs() -> List[Tuple[str, object]]:
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
            "Could not find Robotiq knuckle connection(s): "
            + ", ".join(sorted(missing))
            + ". Present: " + (", ".join(available) if available else "NONE")
        )
    return [(name, found[name]) for name in GRIPPER_OPEN_POSITIONS]


def animate_gripper(state: GripperState, close_fraction: float = 1.0) -> None:
    dofs = _find_gripper_dofs()
    start = {name: float(dof.position) for name, dof in dofs}
    target = dict(GRIPPER_OPEN_POSITIONS if state == GripperState.OPEN
                  else GRIPPER_CLOSED_POSITIONS)
    if state == GripperState.CLOSE and close_fraction < 1.0:
        target = {n: start[n] + close_fraction * (target[n] - start[n])
                  for n in target}
    for frame in range(1, GRIPPER_ANIM_FRAMES + 1):
        a = frame / GRIPPER_ANIM_FRAMES
        for name, dof in dofs:
            dof.position = start[name] + a * (target[name] - start[name])
        world.notify_state_change()
        time.sleep(GRIPPER_ANIM_DURATION / GRIPPER_ANIM_FRAMES)


def open_gripper(gripper: Optional[RealGripper]) -> None:
    if USE_REAL_ROBOT:
        gripper.open()
    else:
        animate_gripper(GripperState.OPEN)


def close_gripper(gripper: Optional[RealGripper]) -> None:
    if USE_REAL_ROBOT:
        gripper.close()
    else:
        animate_gripper(GripperState.CLOSE, GRIPPER_CONTACT_CLOSE_FRACTION)


# ---------------------------------------------------------------------------
# F/T monitor with tare and safety watchdog
# ---------------------------------------------------------------------------

class SafetyStop(RuntimeError):
    """Raised when the contact force exceeded FT_INSERTION_MAX_N."""


class FTMonitor:
    """
    Subscribes to the wrist wrench and tracks |F - F_tare|.

    While armed, a watchdog thread cancels the running Giskard goal as soon as
    the force change exceeds FT_INSERTION_MAX_N, so a blocked position goal
    cannot keep pushing into the part.
    """

    def __init__(self, node):
        self._lock = threading.Lock()
        self._force = None
        self._tare = np.zeros(3)
        self._armed = False
        self.tripped = False
        self.peak = 0.0
        node.create_subscription(WrenchStamped, FT_TOPIC, self._cb, 10)
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _cb(self, msg: WrenchStamped) -> None:
        f = msg.wrench.force
        with self._lock:
            self._force = np.array([f.x, f.y, f.z], dtype=float)

    def wait_for_data(self, timeout_s: float = 5.0) -> None:
        t0 = time.time()
        while self.raw() is None:
            if time.time() - t0 > timeout_s:
                raise RuntimeError(
                    f"No WrenchStamped on {FT_TOPIC} within {timeout_s} s. "
                    f"Check `ros2 topic echo {FT_TOPIC} --once`."
                )
            time.sleep(0.05)

    def raw(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._force is None else self._force.copy()

    def tare(self) -> None:
        """Average the force at standstill in free air and use it as zero."""
        samples = []
        while len(samples) < FT_TARE_SAMPLES:
            samples.append(self.raw())
            time.sleep(0.01)
        with self._lock:
            self._tare = np.mean(samples, axis=0)
        print(f"   [F/T] tare = {np.round(self._tare, 2)} N")

    def delta(self) -> float:
        """Magnitude of the force change since the tare, in N."""
        f = self.raw()
        with self._lock:
            return float(np.linalg.norm(f - self._tare))

    def arm(self) -> None:
        self.tripped = False
        self.peak = 0.0
        self._armed = True

    def disarm(self) -> None:
        self._armed = False

    def _watchdog(self) -> None:
        while True:
            time.sleep(0.005)
            if not self._armed or self.raw() is None:
                continue
            d = self.delta()
            self.peak = max(self.peak, d)
            if d > FT_INSERTION_MAX_N and not self.tripped:
                self.tripped = True
                print(f"\n   [F/T] SAFETY: |dF|={d:.1f} N > {FT_INSERTION_MAX_N} N"
                      " -> cancelling Giskard goal")
                try:
                    context.giskard_wrapper.cancel_goal_async()
                except Exception as e:  # no active goal, already finished, ...
                    print(f"   [F/T] cancel: {e}")


def guarded_move(arm, pose: Pose, ft: FTMonitor) -> None:
    """Real-robot move during insertion; turns a watchdog cancel into SafetyStop."""
    try:
        move_tcp(arm, pose, delay=0.0)
    except Exception as e:
        if ft.tripped:
            raise SafetyStop(f"force limit exceeded (peak {ft.peak:.1f} N)") from e
        raise
    if ft.tripped:
        raise SafetyStop(f"force limit exceeded (peak {ft.peak:.1f} N)")


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
        (lower[0], upper[0]), (lower[1], upper[1]), (lower[2], upper[2]),
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
    positions = calculate_assembly_positions()
    return {
        "BottomWing": spawn_mesh("battat/plane/BottomWing.stl",
                                 positions["BottomWing"], BOTTOM_WING_YPR),
        "UnderBody": spawn_mesh("battat/plane/UnderBody.stl",
                                positions["UnderBody"], UNDER_BODY_YPR),
        "UpperBody": spawn_mesh("battat/plane/UpperBody.stl",
                                positions["UpperBody"], UPPER_BODY_YPR),
    }


def spawn_top_wing():
    print("\nSpawning TopWing at its starting position...")
    return spawn_mesh("battat/plane/TopWing.stl",
                      TOP_WING_START_POSITION, TOP_WING_HOLDER_YPR)


def calculate_top_wing_target() -> Pose:
    x, y, z = calculate_assembly_positions()["TopWing"]
    target = Pose.from_xyz_rpy(
        x + 0.00, y + 0.047, z - 0.07,
        *TOP_WING_FINAL_RPY,
        reference_frame=world.root,
    )
    print("\nCalculated TopWing target pose:")
    print(target)
    return target


# ---------------------------------------------------------------------------
# Insertion strategies
# ---------------------------------------------------------------------------

def world_spiral_xy(tau: float, r_max: float, cycles: float) -> Tuple[float, float]:
    xy = planar_spiral_xy(tau, r0=0.0, r1=r_max, cycles=cycles)
    return float(xy[0]), float(xy[1])


def sim_spiral_insertion(arm, target_m: np.ndarray) -> None:
    """Visual spiral for RViz: descend while spiralling in WORLD XY, then snap."""
    n = SIM_SPIRAL_STEPS
    for i in range(n + 1):
        tau = i / n
        dx, dy = world_spiral_xy(tau, SIM_SPIRAL_R_MAX, SIM_SPIRAL_CYCLES)
        m = translated_matrix(target_m, dx, dy, SIM_SPIRAL_Z_DESCENT * (1 - tau))
        print(f"        spiral {i + 1}/{n + 1}", end="\r")
        move_tcp(arm, pose_from_matrix(m, world.root), delay=DELAY_SPIRAL_SIM)
    print("\n        Simulated hole found -> final insertion pose")
    move_tcp(arm, pose_from_matrix(target_m, world.root), delay=0.3)


def real_force_guided_insertion(arm, start_m: np.ndarray, target_m: np.ndarray,
                                ft: FTMonitor) -> bool:
    """
    Force-guided peg-in-hole on hardware. All offsets are in WORLD frame,
    insertion direction is world -Z.

      A. Tare in free air at start_m (above the target).
      B. Step straight down until |dF| > FT_CONTACT_N (contact) or the
         nominal target height is reached (aligned, no search needed).
      C. Spiral in world XY at contact height - REAL_PRESS_DEPTH until the
         force drops below FT_HOLE_THRESHOLD_N (peg slipped into the hole).
      D. Push straight down at that XY to the target height.

    :return: True if the part reached the insertion height.
    :raises SafetyStop: if the watchdog cancelled a goal.
    """
    target_z = float(target_m[2, 3])

    print("        A. tare in free air...")
    time.sleep(0.5)
    ft.tare()
    ft.arm()

    try:
        # ---- B. descend to contact ----
        print("        B. descending to contact...")
        cur = start_m.copy()
        contact = False
        travelled = 0.0
        while travelled < REAL_DESCENT_MAX:
            next_z = max(cur[2, 3] - REAL_DESCENT_STEP, target_z)
            travelled += cur[2, 3] - next_z
            cur[2, 3] = next_z
            guarded_move(arm, pose_from_matrix(cur, world.root), ft)
            d = ft.delta()
            print(f"           z={cur[2, 3]:.4f}  |dF|={d:5.2f} N", end="\r")
            if d > FT_CONTACT_N:
                contact = True
                print(f"\n           contact at z={cur[2, 3]:.4f} (|dF|={d:.2f} N)")
                break
            if cur[2, 3] <= target_z + 1e-6:
                print("\n           reached insertion height without contact "
                      "(already aligned)")
                return True
        if not contact:
            print(f"\n           no contact within {REAL_DESCENT_MAX * 1000:.0f} mm "
                  "and target not reached: check target pose / fixture position")
            return False

        # ---- C. spiral at contact height ----
        print("        C. spiral search at contact height...")
        spiral_z = cur[2, 3] - REAL_PRESS_DEPTH
        centre = cur.copy()
        hole_xy = None
        for i in range(1, REAL_SPIRAL_STEPS + 1):
            tau = i / REAL_SPIRAL_STEPS
            dx, dy = world_spiral_xy(tau, REAL_SPIRAL_R_MAX, REAL_SPIRAL_CYCLES)
            m = centre.copy()
            m[0, 3] += dx
            m[1, 3] += dy
            m[2, 3] = spiral_z
            guarded_move(arm, pose_from_matrix(m, world.root), ft)
            d = ft.delta()
            print(f"           spiral {i}/{REAL_SPIRAL_STEPS}  "
                  f"r={np.hypot(dx, dy) * 1000:4.1f} mm  |dF|={d:5.2f} N", end="\r")
            if d < FT_HOLE_THRESHOLD_N:
                hole_xy = (m[0, 3], m[1, 3])
                print(f"\n           hole found at step {i} "
                      f"(offset {dx * 1000:+.1f}, {dy * 1000:+.1f} mm, |dF|={d:.2f} N)")
                break
        if hole_xy is None:
            print(f"\n           hole not found within r={REAL_SPIRAL_R_MAX * 1000:.0f} mm")
            return False

        # ---- D. push into the hole ----
        print("        D. pushing into the hole...")
        m = centre.copy()
        m[0, 3], m[1, 3] = hole_xy
        m[2, 3] = spiral_z
        while m[2, 3] > target_z + 1e-6:
            m[2, 3] = max(m[2, 3] - REAL_INSERT_STEP, target_z)
            guarded_move(arm, pose_from_matrix(m, world.root), ft)
            print(f"           z={m[2, 3]:.4f}  |dF|={ft.delta():5.2f} N", end="\r")
        print("\n           insertion height reached")
        return True
    finally:
        ft.disarm()


# ---------------------------------------------------------------------------
# Full sequence
# ---------------------------------------------------------------------------

def run_assembly(top_wing, arm, gripper: Optional[RealGripper],
                 ft: Optional[FTMonitor]) -> None:
    tool_frame = arm.end_effector.tool_frame

    # ---- Pre-compute all poses ----
    object_T_tcp = build_object_T_tcp()

    world_T_object = np.asarray(top_wing.global_pose.to_np(), dtype=float)
    grasp_m = world_T_object @ object_T_tcp
    pregrasp_m = grasp_m @ approach_offset_matrix(PREGRASP_DISTANCE)
    lift_m = translated_matrix(grasp_m, dz=LIFT_DISTANCE)
    safe_clear_m = translated_matrix(lift_m, dz=SAFE_CLEAR_EXTRA)

    world_T_object_target = np.asarray(
        calculate_top_wing_target().to_np(), dtype=float)
    target_m = world_T_object_target @ object_T_tcp
    above_target_m = translated_matrix(target_m, dz=RETRACT_DISTANCE)
    back_m = target_m @ approach_offset_matrix(RETRACT_DISTANCE)
    retract_m = translated_matrix(back_m, dz=RETRACT_DISTANCE)

    P = lambda m: pose_from_matrix(m, world.root)  # noqa: E731

    # ---- Diagnostics ----
    approach_vec = grasp_m[:3, 3] - pregrasp_m[:3, 3]
    approach_vec /= np.linalg.norm(approach_vec)
    print("\nApproach check:")
    print(f"  mode: {'REAL ROBOT' if USE_REAL_ROBOT else 'KINEMATIC SIM'}")
    print(f"  side {GRASP_SIDE}X, end {GRASP_Y_END}Y, depth {GRASP_DEPTH * 1000:.0f} mm")
    print(f"  world closing dir  = {np.round(grasp_m[:3, _AXIS_INDEX[TOOL_CLOSING_AXIS]], 3)}")
    print(f"  world approach dir = {np.round(approach_vec, 3)}")
    print(f"  wing thickness {wing_thickness() * 1000:.1f} mm")
    print(f"  grasp TCP  {np.round(grasp_m[:3, 3], 3)}")
    print(f"  target TCP {np.round(target_m[:3, 3], 3)}")
    if USE_REAL_ROBOT:
        print(f"  F/T {FT_TOPIC}: contact>{FT_CONTACT_N} N, hole<{FT_HOLE_THRESHOLD_N} N, "
              f"stop>{FT_INSERTION_MAX_N} N")
        print(f"  gripper {GRIPPER_ACTION_TOPIC}: close={GRIPPER_REAL_CLOSE_POSITION}")

    env = real_robot if USE_REAL_ROBOT else kinematically_simulated_robot

    with env:
        print("\n[1/11] Parking arms...")
        step(ParkArmsAction(tracy.all_arms), delay=DELAY_PARK)

        print("[2/11] Opening gripper...")
        open_gripper(gripper)

        print("[3/11] Moving to pre-grasp...")
        move_tcp(arm, P(pregrasp_m))

        print("[4/11] Linear approach into the side face...")
        move_through(arm, linear_path(pregrasp_m, grasp_m, APPROACH_STEPS))

        print("[5/11] Closing gripper on TopWing...")
        close_gripper(gripper)

        print("[6/11] Attaching TopWing to the tool frame...")
        step(ReAttachNode(body=top_wing, new_parent=tool_frame), delay=0.2)

        print("[7/11] Lifting straight up...")
        move_through(arm, linear_path(grasp_m, lift_m, LIFT_STEPS))

        print(f"[8/11] Rising {SAFE_CLEAR_EXTRA * 100:.0f} cm to clear the fixture...")
        move_tcp(arm, P(safe_clear_m))

        print("[9/11] Transporting to above the insertion target...")
        move_tcp(arm, P(above_target_m))

        print("[10/11] Insertion...")
        if USE_REAL_ROBOT:
            try:
                inserted = real_force_guided_insertion(arm, above_target_m, target_m, ft)
            except SafetyStop as e:
                print(f"\n        SAFETY STOP: {e}")
                print("        Holding the part, backing straight up. Not releasing.")
                ft.disarm()
                move_tcp(arm, P(translated_matrix(target_m, dz=RETRACT_DISTANCE)))
                raise
            if not inserted:
                print("        Insertion not completed. Holding the part, backing up.")
                move_tcp(arm, P(above_target_m))
                raise RuntimeError("Insertion failed, part still in the gripper")
        else:
            sim_spiral_insertion(arm, target_m)

        print("\n[11/11] Releasing TopWing...")
        step(ReAttachNode(body=top_wing, new_parent=world.root), delay=0.0)
        open_gripper(gripper)

        print("Retracting out through the side face, then up...")
        move_tcp(arm, P(back_m))
        move_tcp(arm, P(retract_m))

        print("Parking...")
        step(ParkArmsAction(tracy.all_arms), delay=DELAY_PARK)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def print_scene(plane, top_wing) -> None:
    print("\n========== FIXTURE LOCATING PEG ==========")
    print(f"Peg datum: ({PEG_X:.5f}, {PEG_Y:.5f}, {PEG_TOP_Z:.5f})")
    for name, part in plane.items():
        print(f"\n{name} world pose:")
        print(part.global_pose)
    print("\n========== TOPWING INITIAL POSE ==========")
    print(top_wing.global_pose)


def main() -> None:
    print("Init ROS")
    rclpy.init()
    node = rclpy.create_node("tracy_demo_node")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    try:
        setup_world(node)

        if USE_REAL_ROBOT:
            # No TFPublisher on hardware: robot_state_publisher already owns TF.
            # VizMarkerPublisher(_world=world, node=node)
            pass
        else:
            TFPublisher(node=node, _world=world)
            VizMarkerPublisher(_world=world, node=node)

        gripper = ft = None
        if USE_REAL_ROBOT:
            gripper = RealGripper(node, GRIPPER_ACTION_TOPIC)
            gripper.wait_for_server()
            ft = FTMonitor(node)
            ft.wait_for_data()
            print(f"   [F/T] receiving {FT_TOPIC}, raw = {np.round(ft.raw(), 2)} N")

        spawn_fixture_and_holders()
        plane = spawn_partial_plane()
        top_wing = spawn_top_wing()
        if not USE_REAL_ROBOT:
            world.notify_state_change()
        print_scene(plane, top_wing)

        print("\nRViz MarkerArray topics: /coraplex/viz_marker, /semworld/viz_marker")

        arm = tracy.left_arm if ARM_SIDE == "left" else tracy.right_arm

        if USE_REAL_ROBOT:
            print("\n*** REAL ROBOT ***")
            print("Check: workspace clear, wing in its holder, E-stop in reach,")
            print("       spawned scene in RViz matches the real table.")
            input("Press Enter to start (Ctrl+C to abort)... ")
        else:
            time.sleep(3.0)

        run_assembly(top_wing, arm, gripper, ft)

        print("\n==============================================")
        print("ASSEMBLY COMPLETE")
        print("==============================================")
        print(top_wing.global_pose)
        print("\nPress Ctrl+C to exit.")
        while rclpy.ok():
            if not USE_REAL_ROBOT:
                world.notify_state_change()
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()