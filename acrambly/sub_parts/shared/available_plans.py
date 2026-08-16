"""
Plan factories
Pure functions that build ``coraplex.plans.plan.Plan`` objects for specific
robot tasks.  They are task-agnostic building blocks – a task module (e.g.
``task_cubes.py``) imports one of these factories and *configures* it with the
concrete bodies / positions / grasp descriptions.

Contents
--------
- ``build_plan_cubes()``  – 3-step pick-and-place stacking plan
- ``build_park_arms_plan()`` – move both arms to park position
"""

from math import pi

from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.composite.transporting import PickAndPlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.motions.gripper import MoveGripperMotion
from semantic_digital_twin.datastructures.definitions import GripperState
from coraplex.view_manager import ViewManager
from coraplex.plans.attachment_nodes import DetachNode
from coraplex.plans.attachment_nodes import AttachNode
from sub_parts.shared.utils import select_arm

def build_plan_cubes(
    world: World,
    tracy: Tracy,
    context: Context,
    red_box: Body,
    yellow_box: Body,
    blue_box: Body,
) -> Plan | None:
    stack_pos_x = 1
    stack_pos_y = 0

    red_arm, red_end_effector = select_arm(red_box, tracy)
    yellow_arm, yellow_end_effector = select_arm(yellow_box, tracy)
    blue_arm, blue_end_effector = select_arm(blue_box, tracy)

    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                red_box,
                red_arm,
                GraspDescription(
                    ApproachDirection.RIGHT,
                    VerticalAlignment.TOP,
                    red_end_effector,
                ),
            ),
            PlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    0.953,
                    reference_frame=world.root,
                ),
                red_arm,
            ),
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                yellow_box,
                yellow_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    yellow_end_effector,
                ),
            ),
            PlaceAction(
                yellow_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.003,
                    reference_frame=world.root,
                ),
                yellow_arm,
            ),
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                blue_box,
                blue_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    blue_end_effector,
                ),
            ),
            PlaceAction(
                blue_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.053,
                    reference_frame=world.root,
                ),
                blue_arm,
            ),
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan

def build_park_arms_plan(context: Context) -> Plan | None:
    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan

def build_handover_object_plan(
    world: World,
    tracy: Tracy,
    context: Context,
    obj: Body,
    ) -> Plan | None:

    meeting_pose = Pose.from_xyz_rpy(
        0.8, 0, 1.2,
        -1.57, 0, 0,
        reference_frame=world.root,
    )
    left_away_pose = Pose.from_xyz_rpy(
        0.8, 0.3, 1.2,
        -1.57, 0, 0,
        reference_frame=world.root,
    )
    right_away_pose = Pose.from_xyz_rpy(
        0.8, -0.3, 1.2,
        -1.57, 0, 0,
        reference_frame=world.root,
    )

    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                obj,
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0],
                ),
            ),
            ReachAction(
                target_pose=meeting_pose,
                arm=Arms.RIGHT,
                grasp_description=GraspDescription(
                    ApproachDirection.BACK,
                    VerticalAlignment.BOTTOM,
                    Tracy.get_end_effectors(tracy)[1],
                ),
            ),
            ReachAction(
                target_pose=left_away_pose,
                arm=Arms.LEFT,
                grasp_description=GraspDescription(
                    ApproachDirection.RIGHT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0],
                ),
            ),
            ReachAction(
                target_pose=meeting_pose,
                arm=Arms.LEFT,
                grasp_description=GraspDescription(
                    ApproachDirection.RIGHT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0],
                ),
            ),
            MoveGripperMotion(GripperState.CLOSE, Arms.RIGHT),
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT),
            AttachNode(
                body=obj,
                new_parent=ViewManager.get_end_effector_view(
                    Arms.RIGHT, tracy
                ).tool_frame,
            ),
            ReachAction(
                target_pose=right_away_pose,
                arm=Arms.RIGHT,
                grasp_description=GraspDescription(
                    ApproachDirection.BACK,
                    VerticalAlignment.BOTTOM,
                    Tracy.get_end_effectors(tracy)[1],
                ),
            ),
            PlaceAction(
                target_location=Pose.from_xyz_rpy(
                    1,
                    -0.5,
                    0.93,
                    reference_frame=world.root,
                ),
                arm=Arms.RIGHT,
                object_designator=obj,
            ),
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan


def build_hand_over2_plan(
    world: World,
    tracy: Tracy,
    context: Context,
    cube0: Body,
    cube1: Body,
    cube2: Body,
) -> Plan | None:
    """Build a right-to-left handover plan for three cubes.

    For each cube the right arm picks it up from the top (``red_pick_grasp``),
    then reaches the handover pose ending sideways (``red_handover_grasp``,
    ``VerticalAlignment.NoAlignment``) to present the cube. The left arm takes
    over from the top (``blue_grasp``) and finally places the cube from the top
    (``blue_place_grasp``).
    """
    end_effectors = Tracy.get_end_effectors(tracy)

    grasp_descriptions = {
        "cube0": {
            "red_pick_grasp": GraspDescription(
                approach_direction=ApproachDirection.LEFT,
                vertical_alignment=VerticalAlignment.TOP,
                end_effector=end_effectors[1],
                grasp_offset=Point3(0.0, -0.08, 0.0),
            ),
            "red_handover_grasp": GraspDescription(
                approach_direction=ApproachDirection.RIGHT,
                vertical_alignment=VerticalAlignment.NoAlignment,
                rotate_gripper=True,
                end_effector=end_effectors[1],
                grasp_offset=Point3(0.0, -0.08, 0.0),
            ),
            "blue_grasp": GraspDescription(
                approach_direction=ApproachDirection.LEFT,
                vertical_alignment=VerticalAlignment.BOTTOM,
                end_effector=end_effectors[0],
                grasp_offset=Point3(0.0, 0.08, 0.0),
            ),
            "blue_place_grasp": GraspDescription(
                approach_direction=ApproachDirection.LEFT,
                vertical_alignment=VerticalAlignment.TOP,
                end_effector=end_effectors[0],
                grasp_offset=Point3(0.0, 0.08, 0.0),
            ),
        },
        "cube1": {
            "red_pick_grasp": GraspDescription(
                approach_direction=ApproachDirection.FRONT,
                vertical_alignment=VerticalAlignment.TOP,
                end_effector=end_effectors[1],
                grasp_offset=Point3(0.06, -0.017, -0.01),
            ),
            "red_handover_grasp": GraspDescription(
                approach_direction=ApproachDirection.BACK,
                vertical_alignment=VerticalAlignment.NoAlignment,
                rotate_gripper=True,
                end_effector=end_effectors[1],
                grasp_offset=Point3(0.06, -0.017, -0.01),
            ),
            "blue_grasp": GraspDescription(
                approach_direction=ApproachDirection.FRONT,
                vertical_alignment=VerticalAlignment.BOTTOM,
                end_effector=end_effectors[0],
                grasp_offset=Point3(-0.06, 0.033, -0.01),
            ),
            "blue_place_grasp": GraspDescription(
                approach_direction=ApproachDirection.FRONT,
                vertical_alignment=VerticalAlignment.TOP,
                end_effector=end_effectors[0],
                grasp_offset=Point3(-0.06, 0.033, -0.01),
            ),
        },
        "cube2": {
            "red_pick_grasp": GraspDescription(
                approach_direction=ApproachDirection.FRONT,
                vertical_alignment=VerticalAlignment.TOP,
                end_effector=end_effectors[1],
                grasp_offset=Point3(0.016, -0.015, 0.03),
            ),
            "red_handover_grasp": GraspDescription(
                approach_direction=ApproachDirection.BACK,
                vertical_alignment=VerticalAlignment.NoAlignment,
                rotate_gripper=True,
                end_effector=end_effectors[1],
                grasp_offset=Point3(0.016, -0.015, 0.03),
            ),
            "blue_grasp": GraspDescription(
                approach_direction=ApproachDirection.FRONT,
                vertical_alignment=VerticalAlignment.BOTTOM,
                end_effector=end_effectors[0],
                grasp_offset=Point3(-0.09, -0.015, -0.01),
            ),
            "blue_place_grasp": GraspDescription(
                approach_direction=ApproachDirection.FRONT,
                vertical_alignment=VerticalAlignment.TOP,
                end_effector=end_effectors[0],
                grasp_offset=Point3(-0.09, -0.015, -0.01),
            ),
        },
    }

    handover_pose = {
        "cube0": Pose.from_xyz_rpy(0.8, -0.08, 1.1, reference_frame=world.root),
        "cube1": Pose.from_xyz_rpy(
            0.8, -0.08, 1.1, yaw=-pi / 2, reference_frame=world.root
        ),
        "cube2": Pose.from_xyz_rpy(
            0.8, -0.08, 1.1, yaw=-pi / 2, reference_frame=world.root
        ),
    }

    place_pose = {
        "cube0": Pose.from_xyz_rpy(0.5, 0.4, 0.95, reference_frame=world.root),
        "cube1": Pose.from_xyz_rpy(0.6, 0.4, 0.95, reference_frame=world.root),
        "cube2": Pose.from_xyz_rpy(1.0, 0.4, 0.95, reference_frame=world.root),
    }

    actions: list = []
    for cube, name in ((cube0, "cube0"), (cube1, "cube1"), (cube2, "cube2")):
        actions.extend(
            [
                ParkArmsAction(Arms.BOTH),
                PickUpAction(
                    cube,
                    Arms.RIGHT,
                    grasp_descriptions[name]["red_pick_grasp"],
                ),
                ReachAction(
                    target_pose=handover_pose[name],
                    object_designator=cube,
                    arm=Arms.RIGHT,
                    grasp_description=grasp_descriptions[name]["red_handover_grasp"],
                ),
                ReachAction(
                    target_pose=grasp_descriptions[name][
                        "blue_grasp"
                    ].grasp_target_pose(cube),
                    object_designator=cube,
                    arm=Arms.LEFT,
                    grasp_description=grasp_descriptions[name]["blue_grasp"],
                ),
                MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.LEFT),
                AttachNode(body=cube, new_parent=end_effectors[0].tool_frame),
                MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.RIGHT),
                ParkArmsAction(Arms.BOTH),
                PlaceAction(
                    object_designator=cube,
                    target_location=place_pose[name],
                    arm=Arms.LEFT,
                    grasp_description=grasp_descriptions[name]["blue_place_grasp"],
                ),
                ParkArmsAction(Arms.BOTH),
            ]
        )

    return sequential(actions, context=context).plan