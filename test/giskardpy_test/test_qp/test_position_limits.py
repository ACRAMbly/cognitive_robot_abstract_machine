import numpy as np

from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState


def test_joint_goal_inside_limits_reached(pr2_world_state_reset):
    connection = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
    dof = connection.dof
    lower = dof.limits.lower.position
    upper = dof.limits.upper.position
    goal = 1.0

    msc = MotionStatechart()
    joint_goal = JointPositionList(goal_state=JointState.from_mapping({connection: goal}))
    msc.add_node(joint_goal)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    assert np.isclose(connection.position, goal, atol=0.01)
    assert lower <= connection.position <= upper


def test_joint_goal_clamped_to_upper_limit(pr2_world_state_reset):
    connection = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
    dof = connection.dof
    upper = dof.limits.upper.position
    goal_beyond_limit = upper + 2.0

    msc = MotionStatechart()
    joint_goal = JointPositionList(goal_state=JointState.from_mapping({connection: goal_beyond_limit}))
    msc.add_node(joint_goal)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    assert np.isclose(connection.position, upper, atol=0.01)
    assert connection.position <= upper + 0.01


def test_joint_goal_clamped_to_lower_limit(pr2_world_state_reset):
    connection = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
    dof = connection.dof
    lower = dof.limits.lower.position
    goal_beyond_limit = lower - 2.0

    msc = MotionStatechart()
    joint_goal = JointPositionList(goal_state=JointState.from_mapping({connection: goal_beyond_limit}))
    msc.add_node(joint_goal)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    assert np.isclose(connection.position, lower, atol=0.01)
    assert connection.position >= lower - 0.01


def test_joint_above_upper_limit_recovers(pr2_world_state_reset):
    connection = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
    dof = connection.dof
    lower = dof.limits.lower.position
    upper = dof.limits.upper.position

    connection.position = upper + 0.5
    goal = 1.0

    msc = MotionStatechart()
    joint_goal = JointPositionList(goal_state=JointState.from_mapping({connection: goal}))
    msc.add_node(joint_goal)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    assert np.isclose(connection.position, goal, atol=0.01)
    assert lower <= connection.position <= upper + 0.01


def test_joint_below_lower_limit_recovers(pr2_world_state_reset):
    connection = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
    dof = connection.dof
    lower = dof.limits.lower.position
    upper = dof.limits.upper.position

    connection.position = lower - 0.5
    goal = -1.0

    msc = MotionStatechart()
    joint_goal = JointPositionList(goal_state=JointState.from_mapping({connection: goal}))
    msc.add_node(joint_goal)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    assert np.isclose(connection.position, goal, atol=0.01)
    assert lower - 0.01 <= connection.position <= upper


def test_multiple_joints_outside_limits_recover(pr2_world_state_reset):
    head_pan = pr2_world_state_reset.get_connection_by_name("head_pan_joint")
    head_tilt = pr2_world_state_reset.get_connection_by_name("head_tilt_joint")
    upper_arm_roll = pr2_world_state_reset.get_connection_by_name("r_upper_arm_roll_joint")

    head_pan.position = head_pan.dof.limits.upper.position + 0.5
    head_tilt.position = head_tilt.dof.limits.upper.position + 0.3
    upper_arm_roll.position = upper_arm_roll.dof.limits.lower.position - 0.5

    goals = {
        head_pan: 0.0,
        head_tilt: 0.5,
        upper_arm_roll: 0.0,
    }

    msc = MotionStatechart()
    joint_goal = JointPositionList(goal_state=JointState.from_mapping(goals))
    msc.add_node(joint_goal)
    end = EndMotion()
    msc.add_node(end)
    end.start_condition = joint_goal.observation_variable

    kin_sim = Executor(MotionStatechartContext(world=pr2_world_state_reset))
    kin_sim.compile(motion_statechart=msc)
    kin_sim.tick_until_end()

    for conn, goal in goals.items():
        lower = conn.dof.limits.lower.position
        upper = conn.dof.limits.upper.position
        assert np.isclose(conn.position, goal, atol=0.01), f"{conn.name} not at goal"
        assert lower - 0.01 <= conn.position <= upper + 0.01, f"{conn.name} outside limits"
