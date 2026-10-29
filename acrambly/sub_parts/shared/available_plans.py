"""
Plan factories
Pure functions that build ``coraplex.plans.plan.Plan`` objects for specific
robot tasks.  They are task-agnostic building blocks – a task module (e.g.
``task_cubes.py``) imports one of these factories and *configures* it with the
concrete graspable annotations / positions.

Contents
--------
- ``build_plan_cubes()``  – 3-step pick-and-place stacking plan
- ``build_park_arms_plan()`` – move both arms to park position
- ``build_hand_over2_plan()`` – right-to-left handover plan
"""

from math import pi

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.attachment_nodes import ReAttachNode
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    MoveManipulatorAction,
    ParkArmsAction,
)
from coraplex.robot_plans.motions.gripper import MoveGripperMotion
from sub_parts.shared.screwing import ScrewThreadAction
from sub_parts.shared.utils import select_arm


def build_plan_cubes(
    world: World,
    tracy: Tracy,
    context: Context,
    red_box: HasGraspCandidates,
    yellow_box: HasGraspCandidates,
    blue_box: HasGraspCandidates,
) -> Plan | None:
    stack_pos_x = 1
    stack_pos_y = 0

    red_arm = select_arm(red_box, tracy)
    yellow_arm = select_arm(yellow_box, tracy)
    blue_arm = select_arm(blue_box, tracy)

    return sequential(
        [
            ParkArmsAction(tracy.all_arms),
            PickUpAction(grasp=red_box.grasp_candidates()[0], arm=red_arm),
            PlaceAction(
                object_designator=red_box,
                target_location=Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    0.953,
                    reference_frame=world.root,
                ),
            ),
            ParkArmsAction(tracy.all_arms),
            PickUpAction(grasp=yellow_box.grasp_candidates()[0], arm=yellow_arm),
            PlaceAction(
                object_designator=yellow_box,
                target_location=Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.003,
                    reference_frame=world.root,
                ),
            ),
            ParkArmsAction(tracy.all_arms),
            PickUpAction(grasp=blue_box.grasp_candidates()[0], arm=blue_arm),
            PlaceAction(
                object_designator=blue_box,
                target_location=Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.053,
                    reference_frame=world.root,
                ),
            ),
            ParkArmsAction(tracy.all_arms),
        ],
        context=context,
    ).plan


def build_park_arms_plan(context: Context) -> Plan | None:
    return sequential(
        [
            ParkArmsAction(context.robot.all_arms),
            MoveGripperMotion(
                motion=GripperState.OPEN, gripper=context.robot.right_arm.end_effector
            ),
            MoveGripperMotion(
                motion=GripperState.OPEN, gripper=context.robot.left_arm.end_effector
            ),
        ],
        context=context,
    ).plan


def build_hand_over2_plan(
    world: World,
    tracy: Tracy,
    context: Context,
    bodies: dict[str, HasGraspCandidates],
) -> Plan | None:
    """Build a right-to-left handover plan.

    For each cube the right arm picks it up and presents it at the handover pose, the
    left arm takes over and then places it.
    """
    right_effector = tracy.right_arm.end_effector
    left_effector = tracy.left_arm.end_effector

    handover_pose = {
        "cube0": Pose.from_xyz_rpy(0.8, -0.08, 1.3, reference_frame=world.root),
        "cube1": Pose.from_xyz_rpy(
            0.8, -0.08, 1.3, yaw=-pi / 2, reference_frame=world.root
        ),
        "cube2": Pose.from_xyz_rpy(
            0.8, -0.08, 1.3, yaw=-pi / 2, reference_frame=world.root
        ),
    }

    place_pose = {
        "cube0": Pose.from_xyz_rpy(0.7, 0.4, 0.92, reference_frame=world.root),
        "cube1": Pose.from_xyz_rpy(0.6, 0.4, 0.96, reference_frame=world.root),
        "cube2": Pose.from_xyz_rpy(1.0, 0.4, 0.95, reference_frame=world.root),
    }

    actions: list = []
    for name, cube in bodies.items():
        yaw = 0 if name == "cube0" else -pi / 2
        actions.extend(
            [
                ParkArmsAction(tracy.all_arms),
                PickUpAction(grasp=cube.grasp_candidates()[0], arm=tracy.right_arm),
                MoveManipulatorAction(
                    target_pose=handover_pose[name],
                    end_effector=right_effector,
                    allow_gripper_collision=True,
                ),
                ReachAction(grasp=cube.grasp_candidates()[0], arm=tracy.left_arm),
                MoveGripperMotion(motion=GripperState.CLOSE, gripper=left_effector),
                ReAttachNode(body=cube.root, new_parent=left_effector.tool_frame),
                MoveGripperMotion(motion=GripperState.OPEN, gripper=right_effector),
                MoveManipulatorAction(
                    target_pose=Pose.from_xyz_rpy(
                        0.8, -0.3, 1.3, yaw=yaw, reference_frame=world.root
                    ),
                    end_effector=right_effector,
                    allow_gripper_collision=True,
                ),
                MoveManipulatorAction(
                    target_pose=Pose.from_xyz_rpy(
                        0.8, 0.3, 1.3, yaw=yaw, reference_frame=world.root
                    ),
                    end_effector=left_effector,
                    allow_gripper_collision=True,
                ),
                PlaceAction(
                    object_designator=cube,
                    target_location=place_pose[name],
                ),
                ParkArmsAction(tracy.all_arms),
            ]
        )

    return sequential(actions, context=context).plan


def build_screw_plan(
    context: Context,
    arm: Arm,
    screw_body: Body,
    turns: float = 1.0,
    grasp_offset: float | None = None,
    release: bool = True,
    allow_gripper_collision: bool = True,
) -> Plan | None:
    """
    Build a park-then-screw plan around a threaded screw body.

    The screw body must already hang from a 1-DoF ``ScrewConnection`` (spawn it with
    :func:`~sub_parts.shared.utils.spawn_screw_body`); this factory only wraps the
    parking and the :class:`~sub_parts.shared.screwing.ScrewThreadAction`.

    :param context: The plan context holding ``world`` and ``robot``.
    :param arm: The arm that grasps the screw and turns it.
    :param screw_body: The threaded body to screw in.
    :param turns: Revolutions to screw inwards.
    :param grasp_offset: Metres along the screw's local +X (towards the head) to grasp;
        ``None`` grasps at the body origin.
    :param release: Whether to open the gripper before retracting.
    :param allow_gripper_collision: Whether the gripper may touch the plate.
    :return: The plan, or ``None`` if the factory produced no plan.
    """
    screw_action = ScrewThreadAction(
        arm=arm,
        screw_body=screw_body,
        turns=turns,
        grasp_offset=grasp_offset,
        release=release,
        allow_gripper_collision=allow_gripper_collision,
    )
    return sequential(
        [ParkArmsAction(context.robot.all_arms), screw_action],
        context=context,
    ).plan
