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

        elif cube_y < 0:
            return Arms.RIGHT, end_effectors[1]

        else:
            return Arms.LEFT, end_effectors[0]

    red_arm, red_end_effector = select_arm(red_box)
    yellow_arm, yellow_end_effector = select_arm(yellow_box)
    blue_arm, blue_end_effector = select_arm(blue_box)

    return sequential(
        [
            ParkArmsAction(Arms.BOTH),

            PickAndPlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    0.955,
                    reference_frame=world.root,
                ),
                red_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    red_end_effector,
                ),
            ),

            PickAndPlaceAction(
                yellow_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.005,
                    reference_frame=world.root,
                ),
                yellow_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    yellow_end_effector,
                ),
            ),

            PickAndPlaceAction(
                blue_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.055,
                    reference_frame=world.root,
                ),
                blue_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    blue_end_effector,
                ),
            ),
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