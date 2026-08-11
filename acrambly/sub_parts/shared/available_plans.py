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

from semantic_digital_twin.robots.tracy import Tracy
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

    def select_arm(cube: Body):
        cube_y = float(
            cube.global_pose.position.to_np().reshape(-1)[1]
        )

        end_effectors = Tracy.get_end_effectors(tracy)

        if cube_y > 0:
            return Arms.LEFT, end_effectors[0]
        else:
            return Arms.RIGHT, end_effectors[1]

    red_arm, red_end_effector = select_arm(red_box)
    yellow_arm, yellow_end_effector = select_arm(yellow_box)
    blue_arm, blue_end_effector = select_arm(blue_box)

    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                red_box,
                red_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    red_end_effector,
                ),
            ),
            PlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    0.955,
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
                    1.005,
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
                    1.055,
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