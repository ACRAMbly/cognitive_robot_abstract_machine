from copy import deepcopy

import numpy as np
import pytest

from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from giskardpy.motion_statechart.tasks.pointing import Pointing
from coraplex.alternative_motion_mappings.stretch_motion_mapping import (
    StretchMoveToolCenterPoint,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    ApproachDirection,
    VerticalAlignment,
    Arms,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import simulated_robot, real_robot
from coraplex.plans.factories import sequential, execute_single
from coraplex.plans.plan_node import MotionNode, ActionNode
from coraplex.robot_plans import MoveMotion, MoveToolCenterPointMotion
from coraplex.robot_plans.motions.robot_body import LookingMotion
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.spatial_types import Point3, Quaternion
from semantic_digital_twin.spatial_types.spatial_types import Pose

try:
    from coraplex.alternative_motion_mappings.hsrb_motion_mapping import *
    from giskardpy.motion_statechart.ros2_nodes.ros_tasks import (
        NavigateActionServerTask,
    )

    skip_tests = False
except (ImportError, ModuleNotFoundError, AttributeError):
    skip_tests = True


@pytest.mark.skipif(skip_tests, reason="Alternative motion mappings not available")
def test_pick_up_motion(immutable_model_world):
    world, view, context = immutable_model_world
    test_world = deepcopy(world)
    grasp_description = GraspDescription(
        ApproachDirection.FRONT,
        VerticalAlignment.NoAlignment,
        view.left_arm.end_effector,
    )
    pick_up = PickUpAction(
        test_world.get_body_by_name("milk.stl"), Arms.LEFT, grasp_description
    )

    root = sequential(
        children=[
            ActionNode(
                designator=NavigateAction(
                    Pose(
                        Point3.from_iterable([1.7, 1.5, 0]),
                        Quaternion.from_iterable([0, 0, 0, 1]),
                        test_world.root,
                    ),
                    True,
                )
            ),
            MoveTorsoAction(TorsoState.HIGH),
            pick_up,
        ],
        context=Context.from_world(test_world),
    )
    assert pick_up.plan is not None
    with simulated_robot:
        root.perform()

    pick_up_node = root.plan.get_nodes_by_designator_type(PickUpAction)[0]

    motion_nodes = list(
        filter(lambda x: isinstance(x, MotionNode), pick_up_node.descendants)
    )

    assert len(motion_nodes) == 5

    motion_charts = [type(m.designator.motion_chart) for m in motion_nodes]
    assert all(mc is not None for mc in motion_charts)
    assert CartesianPose in motion_charts
    assert JointPositionList in motion_charts


def test_move_motion_chart(immutable_model_world):
    world, view, context = immutable_model_world
    motion = MoveMotion(
        Pose(Point3.from_iterable([1, 1, 1]), reference_frame=world.root)
    )
    plan = execute_single(
        motion,
        context=context,
    )

    msc = motion.motion_chart

    assert msc
    np.testing.assert_equal(msc.goal_pose.to_position().to_np(), np.array([1, 1, 1, 1]))


@pytest.mark.skipif(skip_tests, reason="Alternative motion mappings not available")
def test_alternative_mapping(hsr_apartment_world):
    world, view, context = hsr_apartment_world
    context.alternative_motion_mappings = [HSRBMoveMotion]
    move_motion = MoveMotion(
        Pose(Point3.from_iterable([1, 1, 1]), reference_frame=world.root)
    )

    plan = execute_single(move_motion, context=context)

    with real_robot:
        assert move_motion.get_alternative_motion()
        msc = move_motion.motion_chart
        assert NavigateActionServerTask == type(msc)


# %% looking


def test_looking_motion_pointing_parameters(immutable_model_world):
    """
    The looking motion drives the camera with a wide convergence threshold and a raised
    velocity, so the head settles quickly instead of trimming towards a precise angle.
    """
    world, view, context = immutable_model_world
    camera = view.get_default_camera()
    motion = LookingMotion(
        target=Pose(Point3.from_iterable([1, 1, 1]), reference_frame=world.root),
        camera=camera,
    )
    execute_single(motion, context=context)

    pointing = motion.motion_chart

    assert isinstance(pointing, Pointing)
    assert pointing.max_velocity == 1.0
    assert pointing.threshold == 0.5
    assert pointing.pointing_axis is camera.forward_facing_axis


# %% stretch tool center point


@pytest.mark.skipif(skip_tests, reason="Alternative motion mappings not available")
def test_stretch_tool_center_point_straightens_wrist_while_turning(
    immutable_stretch_apartment_world,
):
    """
    Straightening the wrist runs alongside the base rotation, so the gripper is already
    aligned once the cartesian goal takes over.
    """
    world, robot, context = immutable_stretch_apartment_world
    context.alternative_motion_mappings = [StretchMoveToolCenterPoint]
    motion = MoveToolCenterPointMotion(
        target=Pose(Point3.from_iterable([1, 1, 1]), reference_frame=world.root),
        arm=Arms.LEFT,
    )
    execute_single(motion, context=context)

    with real_robot:
        turning_stage = motion.motion_chart.nodes[0]

    assert isinstance(turning_stage, Parallel)
    assert {type(node) for node in turning_stage.nodes} == {Pointing, JointPositionList}
    wrist_goal = next(
        node for node in turning_stage.nodes if isinstance(node, JointPositionList)
    )
    assert [
        connection.name.name for connection in wrist_goal.goal_state.connections
    ] == ["joint_wrist_yaw"]
    assert wrist_goal.goal_state.target_values == [0.0]


@pytest.mark.skipif(skip_tests, reason="Alternative motion mappings not available")
def test_stretch_tool_center_point_accepts_a_local_minimum(
    immutable_stretch_apartment_world,
):
    """
    The arm regularly settles just short of the goal pose, so converging into a local
    minimum counts as success alongside reaching the pose.
    """
    world, robot, context = immutable_stretch_apartment_world
    context.alternative_motion_mappings = [StretchMoveToolCenterPoint]
    motion = MoveToolCenterPointMotion(
        target=Pose(Point3.from_iterable([1, 1, 1]), reference_frame=world.root),
        arm=Arms.LEFT,
    )
    execute_single(motion, context=context)

    with real_robot:
        reaching_stage = motion.motion_chart.nodes[1]

    assert isinstance(reaching_stage, Parallel)
    assert reaching_stage.minimum_success == 1
    assert {type(node) for node in reaching_stage.nodes} == {
        CartesianPose,
        LocalMinimumReached,
    }
    local_minimum = next(
        node for node in reaching_stage.nodes if isinstance(node, LocalMinimumReached)
    )
    assert local_minimum.joint_convergence_threshold == 0.025
