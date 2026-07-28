from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar, Dict, List, Optional, Tuple, Type, Union

from nav_msgs.msg import Odometry
from rclpy.subscription import Subscription
from sensor_msgs.msg import JointState

from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.exceptions import (
    AlreadyTrackedByTfFrameError,
    ConnectionCannotBeTrackedByTfFrameError,
)
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    Connection6DoF,
    DifferentialDrive,
    OmniDrive,
)

# %% base classes


@dataclass
class InputSynchronizer(ABC):
    """
    Writes an external source of truth, e.g. a robot's joint states, into the world
    state.
    """

    world: World
    """
    The world whose state is kept in sync with the external source.
    """

    @abstractmethod
    def apply(self) -> None:
        """
        Write the most recent input into the world state.
        """

    def close(self) -> None:
        """
        Release the resources used to receive inputs.
        """


@dataclass
class WorldStateInputs:
    """
    All inputs that one loop of Giskard reads before it computes anything.
    """

    world: World
    """
    The world whose state is written and whose observers are notified.
    """

    synchronizers: List[InputSynchronizer] = field(default_factory=list)
    """
    The inputs, applied in the order they were added.
    """

    def synchronize(self) -> None:
        """
        Write all inputs into the world state and announce the change.

        The change is not announced while the world model is being modified, because the
        observers would see an inconsistent model.
        """
        for synchronizer in self.synchronizers:
            synchronizer.apply()
        if self.world.world_is_being_modified:
            return
        self.world.notify_state_change()


@dataclass
class TopicInputSynchronizer(InputSynchronizer, ABC):
    """
    Buffers the latest message of a topic and applies it on demand.
    """

    topic_name: str
    """
    Name of the topic the inputs are read from.
    """

    message_type: ClassVar[Type[Any]]
    """
    Type of the messages published on ``topic_name``.
    """

    latest_message: Optional[Any] = field(init=False, default=None)
    """
    The most recently received message, or ``None`` if nothing was received yet.
    """

    subscription: Subscription = field(init=False)
    """
    The subscription feeding ``latest_message``.
    """

    def __post_init__(self):
        if not self.topic_name.startswith("/"):
            self.topic_name = f"/{self.topic_name}"
        self.subscription = rospy.node.create_subscription(
            self.message_type, self.topic_name, self.buffer_message, 1
        )
        rospy.node.get_logger().info(f"Subscribed to {self.topic_name}")

    def buffer_message(self, message: Any) -> None:
        """
        Remember the message so that the next :meth:`apply` can use it.
        """
        self.latest_message = message

    def close(self) -> None:
        rospy.node.destroy_subscription(self.subscription)


# %% joint states


@dataclass
class JointStateInputSynchronizer(TopicInputSynchronizer, ABC):
    """
    Writes the positions of a joint state message into the world state.
    """

    message_type: ClassVar[Type[Any]] = JointState

    def write_positions(self, message: JointState) -> None:
        """
        Copy every joint position of the message into the world state.
        """
        for joint_name, position in zip(message.name, message.position):
            connection: ActiveConnection1DOF = self.world.get_connection_by_name(
                joint_name
            )
            self.world.state[connection.raw_dof.id].position = position


@dataclass
class JointStateSynchronizer(JointStateInputSynchronizer):
    """
    Applies every joint state message exactly once.
    """

    def apply(self) -> None:
        if self.latest_message is None:
            return
        self.write_positions(self.latest_message)
        self.latest_message = None


@dataclass
class JointPositionSynchronizer(JointStateInputSynchronizer):
    """
    Applies the latest joint state message in every cycle.

    Used inside the control loop, where the world state must follow the robot even if no
    new message arrived since the last cycle.
    """

    def apply(self) -> None:
        if self.latest_message is None:
            return
        self.write_positions(self.latest_message)


# %% base pose


@dataclass
class OdometrySynchronizer(TopicInputSynchronizer):
    """
    Writes the pose of an odometry message into a drive connection.
    """

    message_type: ClassVar[Type[Any]] = Odometry

    connection: Union[OmniDrive, DifferentialDrive] = field(kw_only=True)
    """
    The drive connection whose origin follows the odometry.
    """

    def apply(self) -> None:
        if self.latest_message is None:
            return
        pose = self.latest_message.pose.pose
        self.connection.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=pose.position.x,
            pos_y=pose.position.y,
            pos_z=pose.position.z,
            quat_w=pose.orientation.w,
            quat_x=pose.orientation.x,
            quat_y=pose.orientation.y,
            quat_z=pose.orientation.z,
        )


@dataclass
class TfFrameSynchronizer(InputSynchronizer):
    """
    Writes tf transforms into 6 degree of freedom connections.
    """

    connection_to_frames: Dict[Connection6DoF, Tuple[str, str]] = field(
        init=False, default_factory=dict
    )
    """
    Maps each tracked connection to its tf parent and child frame.
    """

    tf_wrapper: TFWrapper = field(init=False)
    """
    Provides the tf lookups.
    """

    def __post_init__(self):
        self.tf_wrapper = TFWrapper(node=rospy.node)

    def track(
        self, connection: Connection6DoF, tf_parent_frame: str, tf_child_frame: str
    ) -> None:
        """
        Make the origin of ``connection`` follow the transform between the two frames.

        :raises AlreadyTrackedByTfFrameError: If the connection is already tracked.
        :raises ConnectionCannotBeTrackedByTfFrameError: If the connection has not
            exactly 6 degrees of freedom.
        """
        if connection in self.connection_to_frames:
            raise AlreadyTrackedByTfFrameError(
                connection_name=str(connection.name),
                tf_parent_frame=self.connection_to_frames[connection][0],
                tf_child_frame=self.connection_to_frames[connection][1],
            )
        if not isinstance(connection, Connection6DoF):
            raise ConnectionCannotBeTrackedByTfFrameError(
                connection_name=str(connection.name),
                connection_type=type(connection).__name__,
            )
        self.connection_to_frames[connection] = (tf_parent_frame, tf_child_frame)

    def apply(self) -> None:
        for connection, (
            tf_parent_frame,
            tf_child_frame,
        ) in self.connection_to_frames.items():
            parent_T_child = self.tf_wrapper.lookup_pose(
                tf_parent_frame, tf_child_frame
            ).pose
            connection.origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
                pos_x=parent_T_child.position.x,
                pos_y=parent_T_child.position.y,
                pos_z=parent_T_child.position.z,
                quat_w=parent_T_child.orientation.w,
                quat_x=parent_T_child.orientation.x,
                quat_y=parent_T_child.orientation.y,
                quat_z=parent_T_child.orientation.z,
            )
