"""
Plan transformations that let particular robots carry out an action with a motion of
their own.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from cramph.composites import ControlStructureNode, Parallel
from cramph.node import StatechartNode
from giskardpy.motion_statechart.ros2_nodes.action_servers import (
    RobotiqGripperActionServerTask,
)
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPoseStraight,
    CartesianPositionTrajectory,
    HoldOrientation,
)
from giskardpy.motion_statechart.tasks.drive_tasks import DifferentialDriveBaseGoal
from giskardpy.motion_statechart.tasks.grasped_mechanism import (
    ArticulateGraspedMechanism,
)
from giskardpy.motion_statechart.tasks.gripper import MoveFingers, MoveGripper
from giskardpy.motion_statechart.weights import DefaultWeights
from krrood_spatial_math.spatial_types import Pose, Vector3
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.robots.justin import Justin
from semantic_digital_twin.robots.robotiq_85_gripper import (
    Robotiq85Gripper,
)
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.world_description.connections import DifferentialDrive
from semantic_digital_twin.world_description.kinematic_chain import KinematicChain

from coraplex.exceptions import ClosingGoalNotFound, ToolPathNotFound
from coraplex.plans.plan_transformation import (
    ConfigurationTransformation,
    ParallelInsertion,
    PlanTransformation,
)
from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.actions.composite.tool_based import ToolMotionAction
from coraplex.robot_plans.actions.core.container import (
    CloseAction,
    MovesContainerByItsHandle,
)
from coraplex.robot_plans.actions.core.navigation import DrivesBase
from coraplex.robot_plans.actions.core.pick_up import (
    GraspingAction,
    ReachAction,
)
from coraplex.robot_plans.actions.core.robot_body import MoveFingersToWidthAction
from coraplex.robot_plans.mixins import MovesToolCenterPoint, OperatesGripper

# %% keeping the torso upright while using a tool


@dataclass
class KeepTheTorsoUprightWhileUsingATool(ParallelInsertion[ToolMotionAction]):
    """
    Keeps Justin's torso upright while it moves a tool, which no other robot needs: its
    torso would otherwise lean into the motion.
    """

    def is_applicable(self, plan_node: ToolMotionAction) -> bool:
        return isinstance(plan_node.robot, Justin)

    def anchor(self, plan_node: ToolMotionAction) -> StatechartNode:
        """
        :return: The goal moving the tool along its path, alongside which the torso is
            held.
        :raises ToolPathNotFound: If the action holds no goal moving the tool along a
            path.
        """
        for node in plan_node.descendants:
            if isinstance(node, CartesianPositionTrajectory):
                return node
        raise ToolPathNotFound(plan_node)

    def nodes_to_insert(self, plan_node: ToolMotionAction) -> list[StatechartNode]:
        root = plan_node.controlled_root
        torso_tip = plan_node.robot.mobile_base.torso.tip
        return [
            AlignPlanes(
                kinematic_chain=KinematicChain(root=root, tip=torso_tip),
                tip_normal=Vector3.X(torso_tip),
                reference_normal=Vector3.Z(root),
                weight=DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE.value,
            )
        ]


# %% driving a differential drive


@dataclass
class DriveWithTheDifferentialDrive(ConfigurationTransformation[DrivesBase]):
    """
    Moves the base of a robot driven by a differential drive with a goal for that drive,
    which turns towards a pose, drives there and turns again, instead of a goal that
    asks the base to move sideways.

    Applies simulated or not.
    """

    def is_applicable(self, plan_node: DrivesBase) -> bool:
        return isinstance(plan_node.robot.drive, DifferentialDrive)

    def apply(self, plan_node: DrivesBase) -> None:
        plan_node.base_goal_type = DifferentialDriveBaseGoal


# %% keeping the base from turning while moving the tool


@dataclass
class KeepStretchFromTurningWhileMovingItsTool(ParallelInsertion[MovesToolCenterPoint]):
    """
    Holds the orientation of a Stretch's base while its tool moves, so that the base
    only drives along its heading, as the tool's sixth degree of freedom.

    The base does not turn to bring a target into reach, which would sweep it through
    wide arcs: a target out of reach fails, and where the robot stands is up to the
    plan. Left as they are: actions driving the base, actions moving a container by its
    handle, and actions below one that holds the base already.
    """

    def is_applicable(self, plan_node: MovesToolCenterPoint) -> bool:
        robot = plan_node.robot
        return (
            isinstance(robot, Stretch)
            and robot.mobile_base.full_body_controlled
            and not any(
                isinstance(node, MovesContainerByItsHandle)
                for node in [plan_node, *plan_node.path]
            )
            and not any(isinstance(node, DrivesBase) for node in plan_node.descendants)
            and not any(
                self._holds_the_base(ancestor, robot) for ancestor in plan_node.path
            )
        )

    def anchor(self, plan_node: MovesToolCenterPoint) -> StatechartNode:
        return plan_node

    def nodes_to_insert(self, plan_node: MovesToolCenterPoint) -> list[StatechartNode]:
        return [
            HoldOrientation(
                kinematic_chain=KinematicChain(
                    root=plan_node.world.root, tip=plan_node.robot.root
                )
            )
        ]

    @staticmethod
    def _holds_the_base(node: StatechartNode, robot: Stretch) -> bool:
        """
        :return: Whether `node` holds the orientation of the base of `robot` beside its
            other children, as this transformation makes an action do. A hold further
            down, such as that of an earlier action, does not cover the others.
        """
        return isinstance(node, Parallel) and any(
            type(child) is HoldOrientation and child.kinematic_chain.tip is robot.root
            for child in node.nodes
        )


# %% moving the tool center point in a straight line


@dataclass
class MoveStretchToolCenterPointInAStraightLine(PlanTransformation[CartesianPose]):
    """
    Moves the tool center point of a Stretch to each of its targets along a straight
    line, by giving every pose goal that an action moving it sets relative to its
    controlled root the tasks of a
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPoseStraight`
    reaching the same pose. Pose goals inside another goal are left as they are.
    """

    def is_applicable(self, plan_node: CartesianPose) -> bool:
        if type(plan_node) is not CartesianPose:
            return False
        parent = plan_node.parent_node
        owner = (
            None
            if parent is None
            else parent.find_upwards(
                predicate=lambda node: (
                    isinstance(node, Action)
                    or not isinstance(node, ControlStructureNode)
                )
            )
        )
        return (
            isinstance(owner, MovesToolCenterPoint)
            and isinstance(owner.robot, Stretch)
            and plan_node.kinematic_chain.root is owner.controlled_root
        )

    def apply(self, plan_node: CartesianPose) -> None:
        straight = CartesianPoseStraight(
            name=plan_node.name,
            kinematic_chain=plan_node.kinematic_chain,
            goal_pose=plan_node.goal_pose,
            cruise_linear_velocity=plan_node.cruise_linear_velocity,
            cruise_angular_velocity=plan_node.cruise_angular_velocity,
            position_threshold=plan_node.position_threshold,
            orientation_threshold=plan_node.orientation_threshold,
            goal_binding_policy=plan_node.goal_binding_policy,
        )
        # the tasks of a pose goal come before the nodes accompanying it
        for task, straight_task in zip(list(plan_node.nodes), straight.create_tasks()):
            plan_node.statechart.replace(node=task, replacement=straight_task)


# %% closing a container


@dataclass
class KeepStretchInFrontOfTheContainerItCloses(PlanTransformation[CloseAction]):
    """
    Keeps a kinematically simulated Stretch standing in front of the container it closes
    and holding on to the handle, so that it closes the container by pushing its arm
    forward.
    """

    def is_applicable(self, plan_node: CloseAction) -> bool:
        return isinstance(plan_node.robot, Stretch)

    @staticmethod
    def closing_goal(plan_node: CloseAction) -> ArticulateGraspedMechanism:
        """
        :param plan_node: The close action.
        :return: The goal closing the container.
        :raises ClosingGoalNotFound: If the action holds no goal closing a container,
            which it has none of before it is expanded.
        """
        for node in plan_node.descendants:
            if isinstance(node, ArticulateGraspedMechanism):
                return node
        raise ClosingGoalNotFound(plan_node)

    def apply(self, plan_node: CloseAction) -> None:
        closing_goal = self.closing_goal(plan_node)
        handle = closing_goal.environment_link
        tool_frame = closing_goal.tip_link
        robot_root = plan_node.robot.root
        plan_node.statechart.replace(
            node=closing_goal,
            replacement=Parallel(
                nodes=[
                    CartesianPose(
                        name="Keep holding handle",
                        kinematic_chain=KinematicChain(root=handle, tip=tool_frame),
                        goal_pose=Pose(reference_frame=tool_frame),
                    ),
                    AlignPlanes(
                        kinematic_chain=KinematicChain(
                            root=plan_node.world.root, tip=robot_root
                        ),
                        reference_normal=Vector3.X(handle),
                        tip_normal=Vector3.NEGATIVE_Y(robot_root),
                    ),
                    ArticulateGraspedMechanism(
                        tip_link=tool_frame,
                        environment_link=handle,
                        goal_position=closing_goal.goal_position,
                    ),
                ]
            ),
        )


@dataclass
class DriveRobotiqGrippersThroughTheirActionServers(
    ConfigurationTransformation[OperatesGripper]
):
    """
        Brings the grippers of a robot that have a ``control_msgs/ParallelGripperCommand``
        server into their configurations through that server, instead of with goals for
        their finger joints.

        Grippers whose servers take other action messages, such as the griplink grippers of
        DAiSy, are left as they are.
        """
    def is_applicable(self, plan_node: OperatesGripper) -> bool:
        return plan_node.gripper_goal_type is MoveFingers

    def apply(self, plan_node: OperatesGripper) -> None:
        print(f"Transforming {type(plan_node).__name__}")
        plan_node.gripper_goal_type = RobotiqGripperActionServerTask


def position_for_width(gripper, width: float) -> float:
    opened = gripper.default_configuration(GripperState.OPEN).joint_state
    closed = gripper.default_configuration(GripperState.CLOSE).joint_state

    open_width = gripper.separation_at(opened)
    closed_width = gripper.separation_at(closed)

    if width > open_width or width < 0:
        raise ValueError(f"Invalid gripper width: {width}")

    width = max(width, closed_width)

    if list(opened.connections) != list(closed.connections):
        raise ValueError("Gripper joint configurations do not match")

    low, high = 0.0, 1.0

    for _ in range(30):
        fraction = (low + high) / 2

        positions = [
            a + fraction * (b - a)
            for a, b in zip(
                opened.target_values,
                closed.target_values,
            )
        ]

        joint_state = JointState(
            connections=list(opened.connections),
            target_values=positions,
        )

        if gripper.separation_at(joint_state) > width:
            low = fraction
        else:
            high = fraction

    return float(positions[0])


@dataclass
class CloseRobotiqToObjectWidth(
    PlanTransformation[GraspingAction]
):
    squeeze: float = 0.0

    def is_applicable(self, plan_node: GraspingAction) -> bool:
        gripper = plan_node.arm.end_effector

        return (
            isinstance(gripper, Robotiq85Gripper)
            and gripper.fingers_are_modelled_like_the_real_ones
            and issubclass(
                plan_node.gripper_goal_type,
                RobotiqGripperActionServerTask,
            )
            and plan_node.grasp_candidate.graspable.root.has_collision()
        )

    def apply(self, plan_node: GraspingAction) -> None:
        gripper = plan_node.arm.end_effector
        width = plan_node.grasp_candidate.width_along_closing_axis
        width = max(0.0, width - 2 * self.squeeze)

        for node in plan_node.descendants:
            if isinstance(node, RobotiqGripperActionServerTask):
                if not any(
                    isinstance(parent, ReachAction)
                    for parent in node.path
                ):
                    node.target_position = position_for_width(
                        gripper, width
                    )
                    return

        raise RuntimeError("Robotiq closing goal not found")