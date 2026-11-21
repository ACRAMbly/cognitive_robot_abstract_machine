"""
Plan factories for ACRAMbly / Tracy.

The current branch does not provide working legacy PickUpAction,
ReachAction, PlaceAction, or GraspDescription implementations.

These plans therefore use:
- ParkArmsAction
- MoveGripperMotion
- MoveToolCenterPointMotion
- ReAttachNode

Object attachment is kinematic: after a gripper closes, the object is
reparented to that gripper's tool frame. This is a simulation mechanism,
not a physics-based grasp.
"""

from __future__ import annotations

from math import pi

import numpy as np

from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.datastructures.definitions import GripperState

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.plans.attachment_nodes import ReAttachNode
from coraplex.datastructures.enums import MovementType

from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def _pose_from_matrix(matrix: np.ndarray, reference_frame) -> Pose:
    """Convert a 4x4 homogeneous transform into a Pose."""
    matrix = np.asarray(matrix, dtype=float)

    x, y, z = matrix[:3, 3]

    # Extract roll, pitch and yaw from the rotation matrix.
    sy = np.sqrt(matrix[0, 0] ** 2 + matrix[1, 0] ** 2)

    singular = sy < 1e-8

    if not singular:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        pitch = np.arctan2(-matrix[2, 0], sy)
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = np.arctan2(-matrix[1, 2], matrix[1, 1])
        pitch = np.arctan2(-matrix[2, 0], sy)
        yaw = 0.0

    return Pose.from_xyz_rpy(
        float(x),
        float(y),
        float(z),
        float(roll),
        float(pitch),
        float(yaw),
        reference_frame=reference_frame,
    )


def _translated_pose(
    pose: Pose,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    reference_frame=None,
) -> Pose:
    """Translate a pose in world XYZ while preserving its orientation."""
    matrix = np.array(pose.to_np(), dtype=float, copy=True)

    matrix[:3, 3] += np.array([dx, dy, dz], dtype=float)

    if reference_frame is None:
        reference_frame = pose.reference_frame

    return _pose_from_matrix(matrix, reference_frame)


def _tcp_pose_for_object(
    object_pose: Pose,
    object_to_tcp: Pose,
    world: World,
) -> Pose:
    """
    Calculate world_T_tcp = world_T_object @ object_T_tcp.

    object_to_tcp must be expressed relative to the object.
    """
    world_T_object = np.asarray(object_pose.to_np(), dtype=float)
    object_T_tcp = np.asarray(object_to_tcp.to_np(), dtype=float)

    world_T_tcp = world_T_object @ object_T_tcp

    return _pose_from_matrix(world_T_tcp, world.root)


class _TranslationTarget:
    """Adapter for MoveToolCenterPointMotion's translation target API.

    This coraplex version calls target.to_position() for TRANSLATION motions,
    while semantic_digital_twin Pose.position already returns a Point3.
    """

    def __init__(self, point):
        self._point = point

    def to_position(self):
        return self._point


def _translation_target(pose: Pose) -> _TranslationTarget:
    """Wrap a Pose's Point3 position in the API expected by coraplex."""
    return _TranslationTarget(pose.position)


def _object_relative_tcp(
    body: Body,
    x: float,
    y: float,
    z: float,
    *,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> Pose:
    """Create a TCP grasp transform expressed in the object's frame."""
    return Pose.from_xyz_rpy(
        x,
        y,
        z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        reference_frame=body,
    )


# ---------------------------------------------------------------------------
# Gripper helpers
# ---------------------------------------------------------------------------

def _open_gripper(arm):
    return MoveGripperMotion(
        motion=GripperState.OPEN,
        gripper=arm,
    )


def _close_gripper(arm):
    return MoveGripperMotion(
        motion=GripperState.CLOSE,
        gripper=arm,
    )


# ---------------------------------------------------------------------------
# Park arms
# ---------------------------------------------------------------------------

def build_park_arms_plan(context: Context) -> Plan | None:
    """Park both arms and open both grippers."""
    tracy = context.robot

    return sequential(
        [
            ParkArmsAction(tracy.all_arms),
            _open_gripper(tracy.right_arm),
            _open_gripper(tracy.left_arm),
        ],
        context=context,
    ).plan


# ---------------------------------------------------------------------------
# Cube handover configuration
# ---------------------------------------------------------------------------

# The offsets below preserve the object-relative translation offsets from
# the original handover plan. They replace the old GraspDescription objects.
#
# Values are:
#   right arm: (x, y, z)
#   left arm:  (x, y, z)

GRASP_OFFSETS = {
    "cube0": {
        "right": (0.0, -0.08, 0.0),
        "left":  (0.0, 0.08, -0.03),
    },
    "cube1": {
        "right": (0.06, -0.017, -0.01),
        "left":  (-0.06, 0.033, -0.04),
    },
    "cube2": {
        "right": (0.016, -0.015, 0.04),
        "left":  (-0.09, -0.015, -0.05),
    },
}


HANDOVER_POSES = {
    "cube0": Pose.from_xyz_rpy(
        0.8, -0.08, 1.3,
    ),
    "cube1": Pose.from_xyz_rpy(
        0.8, -0.08, 1.3,
        yaw=-pi / 2,
    ),
    "cube2": Pose.from_xyz_rpy(
        0.8, -0.08, 1.3,
        yaw=-pi / 2,
    ),
}


PLACE_POSES = {
    "cube0": (0.7, 0.4, 0.92, 0.0),
    "cube1": (0.6, 0.4, 0.96, 0.0),
    "cube2": (1.0, 0.4, 0.95, 0.0),
}


# ---------------------------------------------------------------------------
# One cube: right arm -> left arm -> placement
# ---------------------------------------------------------------------------

def _build_one_cube_handover(
    world: World,
    tracy: Tracy,
    cube: Body,
    name: str,
) -> list:
    """Build the direct-motion handover sequence for one cube."""

    if name not in GRASP_OFFSETS:
        raise ValueError(
            f"No handover grasp configuration exists for {name!r}. "
            f"Expected one of: {list(GRASP_OFFSETS)}"
        )

    right_arm = tracy.right_arm
    left_arm = tracy.left_arm

    right_tool = right_arm.end_effector.tool_frame
    left_tool = left_arm.end_effector.tool_frame

    # -------------------------------------------------------
    # Grasp transforms
    # -------------------------------------------------------

    right_offset = GRASP_OFFSETS[name]["right"]
    left_offset = GRASP_OFFSETS[name]["left"]

    right_object_T_tcp = _object_relative_tcp(
        cube,
        *right_offset,
    )

    left_object_T_tcp = _object_relative_tcp(
        cube,
        *left_offset,
    )

    # The cube's current pose is read before any attachment.
    initial_object_pose = cube.global_pose

    right_grasp_tcp = _tcp_pose_for_object(
        initial_object_pose,
        right_object_T_tcp,
        world,
    )

    right_pregrasp_tcp = _translated_pose(
        right_grasp_tcp,
        dz=0.10,
        reference_frame=world.root,
    )

    # -------------------------------------------------------
    # Handover pose
    # -------------------------------------------------------

    handover_object_pose = HANDOVER_POSES[name]

    # Re-express the handover pose in the world frame.
    handover_object_pose = Pose.from_xyz_rpy(
        0.8,
        -0.08,
        1.3,
        yaw=0.0 if name == "cube0" else -pi / 2,
        reference_frame=world.root,
    )

    right_handover_tcp = _tcp_pose_for_object(
        handover_object_pose,
        right_object_T_tcp,
        world,
    )

    left_handover_tcp = _tcp_pose_for_object(
        handover_object_pose,
        left_object_T_tcp,
        world,
    )

    left_pregrasp_tcp = _translated_pose(
        left_handover_tcp,
        dz=0.10,
        reference_frame=world.root,
    )

    right_retract_tcp = _translated_pose(
        right_handover_tcp,
        dz=0.10,
        reference_frame=world.root,
    )


    # -------------------------------------------------------
    # Placement pose
    # -------------------------------------------------------

    place_x, place_y, place_z, place_yaw = PLACE_POSES[name]

    place_object_pose = Pose.from_xyz_rpy(
        place_x,
        place_y,
        place_z,
        yaw=place_yaw,
        reference_frame=world.root,
    )

    left_place_tcp = _tcp_pose_for_object(
        place_object_pose,
        left_object_T_tcp,
        world,
    )

    left_place_pregrasp_tcp = _translated_pose(
        left_place_tcp,
        dz=0.10,
        reference_frame=world.root,
    )

    left_retract_tcp = _translated_pose(
        left_place_tcp,
        dz=0.10,
        reference_frame=world.root,
    )

    # -------------------------------------------------------
    # Actions
    # -------------------------------------------------------

    actions = [
        # Park and prepare the right gripper.
        ParkArmsAction(tracy.all_arms),
        _open_gripper(right_arm),

        # RIGHT ARM: approach and pick the cube.
        # RIGHT ARM: approach and pick the cube.
        MoveToolCenterPointMotion(
            target=_translation_target(right_pregrasp_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=_translation_target(right_grasp_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        _close_gripper(right_arm),

        # Attach cube to the right gripper in the kinematic world.
        ReAttachNode(
            body=cube,
            new_parent=right_tool,
        ),

        # RIGHT ARM: carry the cube to the handover pose.
        MoveToolCenterPointMotion(
            target=_translation_target(right_handover_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # LEFT ARM: approach the cube while the right arm holds it.
        _open_gripper(left_arm),
        MoveToolCenterPointMotion(
            target=_translation_target(left_pregrasp_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=_translation_target(left_handover_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # Close the left gripper and transfer the cube to it.
        _close_gripper(left_arm),
        ReAttachNode(
            body=cube,
            new_parent=left_tool,
        ),

        # Release the right gripper after the left attachment.
        _open_gripper(right_arm),

        # Move the right arm away from the transferred cube.
        MoveToolCenterPointMotion(
            target=_translation_target(right_retract_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # LEFT ARM: move the cube to its placement position.
        MoveToolCenterPointMotion(
            target=_translation_target(left_place_pregrasp_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=_translation_target(left_place_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # Release the cube back into the world.
        _open_gripper(left_arm),
        ReAttachNode(
            body=cube,
            new_parent=world.root,
        ),

        # Retract and park.
        MoveToolCenterPointMotion(
            target=_translation_target(left_retract_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        ParkArmsAction(tracy.all_arms),
    ]

    return actions


# ---------------------------------------------------------------------------
# Complete handover plan
# ---------------------------------------------------------------------------

def build_hand_over2_plan(
    world: World,
    tracy: Tracy,
    context: Context,
    bodies: dict[str, Body],
) -> Plan | None:
    """
    Build a right-to-left handover plan for cube0, cube1 and cube2.

    Each cube is:
      1. Picked up by the right arm.
      2. Carried to the handover pose.
      3. Taken by the left arm.
      4. Released by the right arm.
      5. Placed by the left arm.
    """

    required_cubes = ("cube0", "cube1", "cube2")

    missing = [
        name for name in required_cubes
        if name not in bodies
    ]

    if missing:
        raise KeyError(
            f"Missing cube bodies: {missing}. "
            f"Received: {list(bodies)}"
        )

    actions = []

    for name in required_cubes:
        cube = bodies[name]

        print(f"Building handover actions for {name}")

        actions.extend(
            _build_one_cube_handover(
                world=world,
                tracy=tracy,
                cube=cube,
                name=name,
            )
        )

    return sequential(
        actions,
        context=context,
    ).plan


# ---------------------------------------------------------------------------
# Legacy cube stacking plan
# ---------------------------------------------------------------------------

def _build_cube_stack_move(
    world: World,
    tracy: Tracy,
    cube: Body,
    *,
    name: str,
    target_object_xyz: tuple[float, float, float],
) -> list:
    """Pick one cube and place it at a target stack position.

    The object is attached to the selected gripper after closing and detached
    after opening. This is the same kinematic grasp mechanism used by
    ``hand_over2``; it is not a contact/physics grasp.
    """
    object_pose = cube.global_pose
    object_position = object_pose.position.to_np().reshape(-1)
    object_y = float(object_position[1])

    # Use the arm on the same side as the source cube.
    if object_y > 0.0:
        arm = tracy.left_arm
    else:
        arm = tracy.right_arm

    # These spawned cubes are only 0.05 m wide.  The TCP frame is the
    # midpoint between the gripper fingers, so target the cube centre.
    # The +/-0.08 m offsets used by the separate handover task are not
    # appropriate for these small table-top cubes.
    grasp_y = 0.0

    tool_frame = arm.end_effector.tool_frame

    # Top-down grasp: rotate the TCP by pi around its local X axis so its
    # +Z approach axis points down toward the tabletop.  The same object-to-TCP
    # transform is used for placement, so the cube remains upright when carried.
    object_to_tcp = _object_relative_tcp(
        cube,
        0.0,
        grasp_y,
        0.0,
        roll=pi,
    )
    grasp_tcp = _tcp_pose_for_object(object_pose, object_to_tcp, world)
    pregrasp_tcp = _translated_pose(
        grasp_tcp, dz=0.10, reference_frame=world.root
    )

    target_object_pose = Pose.from_xyz_rpy(
        target_object_xyz[0],
        target_object_xyz[1],
        target_object_xyz[2],
        reference_frame=world.root,
    )
    target_tcp = _tcp_pose_for_object(
        target_object_pose, object_to_tcp, world
    )
    target_pregrasp_tcp = _translated_pose(
        target_tcp, dz=0.10, reference_frame=world.root
    )

    print(
        f"[Cubes] {name}: using {'left' if arm is tracy.left_arm else 'right'} "
        f"arm; source=({object_position[0]:.3f}, {object_y:.3f}, "
        f"{object_position[2]:.3f}), target={target_object_xyz}"
    )

    return [
        _open_gripper(arm),
        # Approach from above with the gripper's TCP approach axis directed
        # vertically down.  Reach the oriented pre-grasp pose first.
        MoveToolCenterPointMotion(
            target=pregrasp_tcp,
            arm=arm,
            allow_gripper_collision=False,
        ),
        # Descend vertically without changing the now-aligned orientation.
        MoveToolCenterPointMotion(
            target=_translation_target(grasp_tcp),
            arm=arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        _close_gripper(arm),
        ReAttachNode(body=cube, new_parent=tool_frame),
        MoveToolCenterPointMotion(
            target=target_pregrasp_tcp,
            arm=arm,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=target_tcp,
            arm=arm,
            allow_gripper_collision=False,
        ),
        _open_gripper(arm),
        ReAttachNode(body=cube, new_parent=world.root),
        MoveToolCenterPointMotion(
            target=_translation_target(target_pregrasp_tcp),
            arm=arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
    ]


def build_plan_cubes(
    world: World,
    tracy: Tracy,
    context: Context,
    red_box: Body,
    yellow_box: Body,
    blue_box: Body,
) -> Plan | None:
    """Stack the three simulation cubes using the current motion APIs.

    The blue cube starts at the centre and remains the base. The red cube is
    placed on blue, then the green/yellow cube is placed on red. The spawned
    boxes use 0.05 m dimensions, so each additional stack level is 0.05 m.

    Grasping is kinematic through ``ReAttachNode``, consistent with the
    working ``hand_over2`` simulation plan.
    """
    stack_x = 0.8
    stack_y = 0.0
    base_z = float(blue_box.global_pose.position.to_np().reshape(-1)[2])
    box_height = 0.05

    actions = [
        ParkArmsAction(tracy.all_arms),
        _open_gripper(tracy.left_arm),
        _open_gripper(tracy.right_arm),
    ]

    # Blue is already at the central stack location and forms the base.
    actions.extend(
        _build_cube_stack_move(
            world,
            tracy,
            red_box,
            name="red cube",
            target_object_xyz=(stack_x, stack_y, base_z + box_height),
        )
    )
    actions.extend(
        _build_cube_stack_move(
            world,
            tracy,
            yellow_box,
            name="green cube",
            target_object_xyz=(stack_x, stack_y, base_z + 2.0 * box_height),
        )
    )

    actions.append(ParkArmsAction(tracy.all_arms))

    return sequential(actions, context=context).plan


# ---------------------------------------------------------------------------
# Pose-adaptive autonomous handover (simulation ground-truth version)
# ---------------------------------------------------------------------------

AUTONOMOUS_HANDOVER_LIFT = 0.32
AUTONOMOUS_PREGRASP_HEIGHT = 0.10
AUTONOMOUS_PLACE_OFFSET_XY = (0.20, 0.12)


def _pose_yaw_from_pose(pose: Pose) -> float:
    """Read yaw from a pose rotation matrix."""
    matrix = np.asarray(pose.to_np(), dtype=float)
    return float(np.arctan2(matrix[1, 0], matrix[0, 0]))


def _build_pose_adaptive_handover_for_object(
    world: World,
    tracy: Tracy,
    cube: Body,
    name: str,
) -> list:
    """Build one right-to-left handover from the object's current world pose.

    This version uses simulation ground-truth (Body.global_pose). It does not
    require the object to be at a predefined XYZ location. The object-specific
    grasp offsets remain model calibration parameters in GRASP_OFFSETS.
    """
    if name not in GRASP_OFFSETS:
        raise ValueError(
            f"No calibrated grasp template for {name!r}; "
            f"available templates: {list(GRASP_OFFSETS)}"
        )

    right_arm = tracy.right_arm
    left_arm = tracy.left_arm
    right_tool = right_arm.end_effector.tool_frame
    left_tool = left_arm.end_effector.tool_frame

    # Read the live object pose when the plan is built.
    object_pose = cube.global_pose
    object_matrix = np.asarray(object_pose.to_np(), dtype=float)
    object_xyz = object_matrix[:3, 3].copy()
    object_yaw = _pose_yaw_from_pose(object_pose)

    # Retain the calibrated local grasp templates, but transform them using
    # the object's current pose rather than a hard-coded world pose.
    right_object_to_tcp = _object_relative_tcp(
        cube, *GRASP_OFFSETS[name]["right"]
    )
    left_object_to_tcp = _object_relative_tcp(
        cube, *GRASP_OFFSETS[name]["left"]
    )

    right_grasp_tcp = _tcp_pose_for_object(
        object_pose, right_object_to_tcp, world
    )
    right_pregrasp_tcp = _translated_pose(
        right_grasp_tcp,
        dz=AUTONOMOUS_PREGRASP_HEIGHT,
        reference_frame=world.root,
    )

    # Derive the meeting pose from the object's current position. This avoids
    # the old per-cube HANDOVER_POSES table. The meeting point is above the
    # source object; both arms approach the same object pose there.
    handover_object_pose = Pose.from_xyz_rpy(
        float(object_xyz[0]),
        float(object_xyz[1]),
        float(object_xyz[2] + AUTONOMOUS_HANDOVER_LIFT),
        yaw=object_yaw,
        reference_frame=world.root,
    )
    right_handover_tcp = _tcp_pose_for_object(
        handover_object_pose, right_object_to_tcp, world
    )
    left_handover_tcp = _tcp_pose_for_object(
        handover_object_pose, left_object_to_tcp, world
    )
    left_pregrasp_tcp = _translated_pose(
        left_handover_tcp,
        dz=AUTONOMOUS_PREGRASP_HEIGHT,
        reference_frame=world.root,
    )
    right_retract_tcp = _translated_pose(
        right_handover_tcp,
        dz=AUTONOMOUS_PREGRASP_HEIGHT,
        reference_frame=world.root,
    )

    # Choose a destination relative to the object's detected starting pose.
    # This is a simple simulation drop-zone heuristic, not collision-aware
    # placement planning.
    place_x = float(object_xyz[0] + AUTONOMOUS_PLACE_OFFSET_XY[0])
    place_y = float(object_xyz[1] + AUTONOMOUS_PLACE_OFFSET_XY[1])
    place_object_pose = Pose.from_xyz_rpy(
        place_x,
        place_y,
        float(object_xyz[2]),
        yaw=object_yaw,
        reference_frame=world.root,
    )
    left_place_tcp = _tcp_pose_for_object(
        place_object_pose, left_object_to_tcp, world
    )
    left_place_pregrasp_tcp = _translated_pose(
        left_place_tcp,
        dz=AUTONOMOUS_PREGRASP_HEIGHT,
        reference_frame=world.root,
    )
    left_retract_tcp = _translated_pose(
        left_place_tcp,
        dz=AUTONOMOUS_PREGRASP_HEIGHT,
        reference_frame=world.root,
    )

    print(
        f"[AutonomousHandover] {name}: "
        f"object=({object_xyz[0]:.3f}, {object_xyz[1]:.3f}, "
        f"{object_xyz[2]:.3f}), yaw={object_yaw:.3f}; "
        f"handover=({object_xyz[0]:.3f}, {object_xyz[1]:.3f}, "
        f"{object_xyz[2] + AUTONOMOUS_HANDOVER_LIFT:.3f}); "
        f"place=({place_x:.3f}, {place_y:.3f}, {object_xyz[2]:.3f})"
    )

    return [
        ParkArmsAction(tracy.all_arms),
        _open_gripper(right_arm),

        # Giver arm: approach and grasp using the current object pose.
        MoveToolCenterPointMotion(
            target=_translation_target(right_pregrasp_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=_translation_target(right_grasp_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        _close_gripper(right_arm),
        ReAttachNode(body=cube, new_parent=right_tool),

        # Carry to the dynamically derived meeting pose.
        MoveToolCenterPointMotion(
            target=_translation_target(right_handover_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # Receiver approaches while giver keeps the object attached.
        _open_gripper(left_arm),
        MoveToolCenterPointMotion(
            target=_translation_target(left_pregrasp_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=_translation_target(left_handover_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # Receiver closes and takes ownership before giver opens.
        _close_gripper(left_arm),
        ReAttachNode(body=cube, new_parent=left_tool),
        _open_gripper(right_arm),

        MoveToolCenterPointMotion(
            target=_translation_target(right_retract_tcp),
            arm=right_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),

        # Receiver carries to the computed drop pose and releases.
        MoveToolCenterPointMotion(
            target=_translation_target(left_place_pregrasp_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        MoveToolCenterPointMotion(
            target=_translation_target(left_place_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        _open_gripper(left_arm),
        ReAttachNode(body=cube, new_parent=world.root),
        MoveToolCenterPointMotion(
            target=_translation_target(left_retract_tcp),
            arm=left_arm,
            movement_type=MovementType.TRANSLATION,
            allow_gripper_collision=False,
        ),
        ParkArmsAction(tracy.all_arms),
    ]


def build_autonomous_handover_plan(
    world: World,
    tracy: Tracy,
    context: Context,
    bodies: dict[str, Body],
) -> Plan | None:
    """Build a pose-adaptive right-to-left handover for all supplied cubes.

    Object world poses are read at runtime from Body.global_pose. The current
    simulation implementation still uses calibrated per-object grasp offsets
    and a simple relative drop-zone heuristic; it is not camera perception or
    collision-aware autonomous planning.
    """
    required_cubes = ("cube0", "cube1", "cube2")
    missing = [key for key in required_cubes if key not in bodies]
    if missing:
        raise KeyError(
            f"Missing cube bodies: {missing}; received: {list(bodies)}"
        )

    # Execute the closest object to the nominal robot workspace first.
    # This ordering is computed from current object positions, not key order.
    def distance_from_workspace_center(key: str) -> float:
        xyz = np.asarray(
            bodies[key].global_pose.position.to_np(), dtype=float
        ).reshape(-1)
        return float(np.linalg.norm(xyz[:2] - np.array([0.8, 0.0])))

    ordered_names = sorted(
        required_cubes, key=distance_from_workspace_center
    )
    print(f"[AutonomousHandover] execution order: {ordered_names}")

    actions = []
    for name in ordered_names:
        actions.extend(
            _build_pose_adaptive_handover_for_object(
                world=world,
                tracy=tracy,
                cube=bodies[name],
                name=name,
            )
        )

    return sequential(actions, context=context).plan
