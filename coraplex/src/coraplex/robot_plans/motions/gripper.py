from dataclasses import dataclass, field
from typing import Optional, List

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.templates import Parallel, Sequence
from giskardpy.motion_statechart.binding_policy import GoalBindingPolicy
from giskardpy.motion_statechart.tasks.align_planes import AlignPlanes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPosition,
    CartesianPositionTrajectory,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from semantic_digital_twin.datastructures.alignment import AlignmentPair
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.justin import Justin
from semantic_digital_twin.robots.robot_part_mixins import HasMobileBase
from semantic_digital_twin.robots.robot_parts import EndEffector
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body
from coraplex.exceptions import MissingToolFrame, MissingWaypoints
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.datastructures.enums import (
    Arms,
    MovementType,
    WaypointsMovementType,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.view_manager import ViewManager
from coraplex.utils import translate_pose_along_local_axis

TOOL_ORIENTATION_THRESHOLD = 0.02
"""
Default orientation tolerance in rad for tool-center-point poses.

.. note:: A physically simulated arm's PD-tracked joints settle with a small residual
    orientation error, so reusing the (much tighter) position tolerance as the
    rotation tolerance can leave the task perpetually unfinished, stalling the rest of
    the plan behind it.
"""

DEFAULT_TCP_POSITION_THRESHOLD = 0.005
"""
Default position tolerance in meters for tool-center-point poses, tighter than
Giskard's own task default so an approach doesn't stop short of a small object.
"""


@dataclass
class ReachMotion(BaseMotion):
    """
    Moves the tool center point through the grasp description's pre-grasp and grasp
    poses for an object.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be picked up
    """
    arm: Arms
    """
    The arm that should be used for pick up
    """
    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object
    """
    movement_type: MovementType = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """
    reverse_pose_sequence: bool = False
    """
    Reverses the sequence of poses, i.e., moves away from the object instead of towards it. Used for placing objects.
    """

    position_threshold: float = DEFAULT_TCP_POSITION_THRESHOLD
    """
    Distance threshold in meters for goal achievement.
    """

    orientation_threshold: float = TOOL_ORIENTATION_THRESHOLD
    """
    Rotation threshold in rad for goal achievement.
    """

    def _calculate_pose_sequence(self) -> List[Pose]:
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot_view)

        target_pose = GraspDescription.get_grasp_pose(
            self.grasp_description, end_effector, self.object_designator
        )
        target_pose.rotate_by_quaternion(
            GraspDescription.calculate_grasp_orientation(
                self.grasp_description,
                end_effector.front_facing_orientation.to_np(),
            )
        )
        target_pre_pose = translate_pose_along_local_axis(
            target_pose,
            end_effector.front_facing_axis.to_np()[:3],
            -0.05,  # TODO: Maybe put these values in the semantic annotates
        )

        pose = self.world.transform(target_pre_pose, self.world.root)

        sequence = [target_pre_pose, pose]
        return sequence.reverse() if self.reverse_pose_sequence else sequence

    def perform(self):
        pass

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        nodes = [
            CartesianPose(
                root_link=self.robot_view.root,
                tip_link=tip,
                goal_pose=pose,
                threshold=self.position_threshold,
                orientation_threshold=self.orientation_threshold,
                name="Reach",
            )
            for pose in self._calculate_pose_sequence()
        ]
        return Sequence(nodes=nodes)


@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    target_opening: Optional[float] = None
    """
    Explicit finger opening (in meters) to command instead of the position the
    GripperState would resolve to. ``None`` keeps the GripperState's own position.
    """

    finger_velocity: Optional[float] = None
    """
    Finger joint velocity (in m/s) to command. ``None`` keeps the default velocity.
    """

    stall_minimum_time: Optional[float] = None
    """
    Minimum stall dwell time (in seconds, see
    :attr:`~giskardpy.motion_statechart.tasks.joint_tasks.JointPositionList.stall_minimum_time`)
    to command. Only meaningful when :attr:`tolerate_stall` is True. ``None`` keeps the
    default.
    """

    tolerate_stall: bool = False
    """
    Whether the motion is considered done once the fingers' velocities settle near
    zero, even without reaching their nominal target position (see
    :attr:`~giskardpy.motion_statechart.tasks.joint_tasks.JointPositionList.tolerate_stall`).
    """

    def perform(self):
        return

    def _goal_state(self, end_effector: EndEffector) -> JointState:
        """
        The finger joint state this motion commands: the GripperState's own state, or --
        when ``target_opening`` is set -- the same finger connections remapped to that
        explicit opening.
        """
        goal_state = end_effector.get_joint_state_by_type(self.motion)
        if self.target_opening is None:
            return goal_state
        return JointState.from_mapping(
            mapping={
                connection: self.target_opening for connection in goal_state.connections
            },
        )

    @property
    def _motion_chart(self):
        arm = ViewManager().get_end_effector_view(self.gripper, self.robot)

        keyword_arguments = {}
        if self.finger_velocity is not None:
            keyword_arguments["max_velocity"] = self.finger_velocity
        if self.stall_minimum_time is not None:
            keyword_arguments["stall_minimum_time"] = self.stall_minimum_time

        return JointPositionList(
            goal_state=self._goal_state(arm),
            name=(
                "OpenGripper" if self.motion == GripperState.OPEN else "CloseGripper"
            ),
            tolerate_stall=self.tolerate_stall,
            **keyword_arguments,
        )


@dataclass
class MoveToolCenterPointMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: Optional[MovementType] = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    reference_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) to command. ``None`` keeps the default velocity.
    """

    reference_angular_velocity: Optional[float] = None
    """
    Angular reference velocity (in rad/s) to command, used only for
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPose`. ``None``
    keeps the default velocity.
    """

    position_threshold: float = DEFAULT_TCP_POSITION_THRESHOLD
    """
    Distance threshold in meters for goal achievement.
    """

    orientation_threshold: float = TOOL_ORIENTATION_THRESHOLD
    """
    Rotation threshold in rad for goal achievement, used only for
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPose`.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot).tool_frame
        root = (
            self.world.root
            if isinstance(self.robot, HasMobileBase)
            and self.robot.mobile_base.full_body_controlled
            else self.robot.root
        )
        task = None
        if self.movement_type == MovementType.TRANSLATION:
            keyword_arguments = {}
            if self.reference_linear_velocity is not None:
                keyword_arguments["reference_velocity"] = self.reference_linear_velocity
            task = CartesianPosition(
                root_link=root,
                tip_link=tip,
                goal_point=self.target.to_position(),
                name="MoveTCP",
                weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE,
                threshold=self.position_threshold,
                **keyword_arguments,
            )
        else:
            keyword_arguments = {}
            if self.reference_linear_velocity is not None:
                keyword_arguments["reference_linear_velocity"] = (
                    self.reference_linear_velocity
                )
            if self.reference_angular_velocity is not None:
                keyword_arguments["reference_angular_velocity"] = (
                    self.reference_angular_velocity
                )
            task = CartesianPose(
                root_link=root,
                tip_link=tip,
                goal_pose=self.target,
                name="MoveTCP",
                weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE,
                threshold=self.position_threshold,
                orientation_threshold=self.orientation_threshold,
                **keyword_arguments,
            )
        return task


@dataclass
class MoveTCPWaypointsMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    waypoints: List[Pose]
    """
    Waypoints the TCP should move along 
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: WaypointsMovementType = (
        WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT
    )
    """
    The type of movement that should be performed.
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot).tool_frame
        root = (
            self.world.root
            if isinstance(self.robot, HasMobileBase)
            and self.robot.mobile_base.full_body_controlled
            else self.robot.root
        )
        nodes = [
            CartesianPose(
                root_link=root,
                tip_link=tip,
                goal_pose=pose,
                # threshold=0.005,
            )
            for pose in self.waypoints
        ]
        return Sequence(nodes=nodes)


@dataclass
class MoveTCPWaypointsAlignedMotion(BaseMotion):
    """
    Moves the tool center point (TCP) of the robot along waypoints while keeping the
    given plane alignments.
    """

    waypoints: List[Point3]
    """
    Waypoints the TCP should move along.
    """
    arm: Arms
    """
    Arm with the TCP that should be moved along the waypoints.
    """
    alignment_pairs: List[AlignmentPair] = field(default_factory=list)
    """
    Normal pairs kept aligned during the motion.
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something.
    """
    tip: Optional[Body] = None
    """
    The body that should follow the waypoints. Defaults to the arm's tool frame.
    """

    def perform(self):
        return

    def _resolve_tip(self) -> Body:
        """
        :return: The body that follows the waypoints: the explicit tip if given,
            otherwise the arm's tool frame.
        :raises MissingToolFrame: If no tip is given and the arm has no tool frame.
        """
        if self.tip is not None:
            return self.tip
        tool_frame = (
            ViewManager().get_end_effector_view(self.arm, self.robot).tool_frame
        )
        if tool_frame is None:
            raise MissingToolFrame(self.arm, self.robot)
        return tool_frame

    def _upright_torso_task(self, tip_link: Body, root_link: Body) -> AlignPlanes:
        """
        :return: A task that keeps Justin's torso upright during the motion.
        """
        torso_tip = self.robot.mobile_base.torso.tip
        return AlignPlanes(
            tip_link=torso_tip,
            root_link=root_link,
            tip_normal=Vector3.X(torso_tip),
            goal_normal=Vector3.Z(root_link),
            weight=DefaultWeights.WEIGHT_ABOVE_COLLISION_AVOIDANCE.value,
        )

    @property
    def _motion_chart(self):
        if not self.waypoints:
            raise MissingWaypoints(self)

        tip_link = self._resolve_tip()
        root_link = (
            self.world.root
            if isinstance(self.robot, HasMobileBase)
            and self.robot.mobile_base.full_body_controlled
            else self.robot.root
        )
        tasks = [
            CartesianPositionTrajectory(
                root_link=root_link,
                tip_link=tip_link,
                goal_points=self.waypoints,
                maximum_skip_ahead=2,
                weight=float(DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE),
                name="MoveTCPWaypointsAligned",
            )
        ]
        tasks.extend(
            AlignPlanes(
                tip_link=tip_link,
                root_link=root_link,
                tip_normal=pair.tip_normal,
                goal_normal=pair.goal_normal,
                weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE.value,
            )
            for pair in self.alignment_pairs
        )
        if isinstance(self.robot, Justin):
            tasks.append(self._upright_torso_task(tip_link, root_link))
        motion_statechart_nodes = (
            self._only_allow_gripper_collision_rules(self.arm)
            if self.allow_gripper_collision
            else []
        )
        motion_statechart_nodes.append(Parallel(tasks))
        return Parallel(motion_statechart_nodes)


@dataclass
class MoveManipulatorMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: Pose
    """
    Target pose to which the TCP should be moved
    """

    end_effector: EndEffector
    """
    The end effector to move to the target pose
    """

    allow_gripper_collision: bool = False
    """
    If the gripper can collide with something
    """

    position_threshold: float = DEFAULT_TCP_POSITION_THRESHOLD
    """
    Distance threshold in meters for goal achievement.
    """

    orientation_threshold: float = TOOL_ORIENTATION_THRESHOLD
    """
    Rotation threshold in rad for goal achievement.
    """

    @property
    def _motion_chart(self):
        robot = self.robot
        full_body_controlled = (
            robot.mobile_base.full_body_controlled
            if isinstance(robot, HasMobileBase)
            else False
        )
        root = self.world.root if full_body_controlled else robot.root
        task = CartesianPose(
            root_link=root,
            tip_link=self.end_effector.tool_frame,
            goal_pose=self.target,
            threshold=self.position_threshold,
            orientation_threshold=self.orientation_threshold,
            binding_policy=GoalBindingPolicy.Bind_on_start,
            name=self.__class__.__name__,
        )
        return task
