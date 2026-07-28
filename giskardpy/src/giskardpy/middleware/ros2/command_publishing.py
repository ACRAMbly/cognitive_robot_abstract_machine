from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Union

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
from std_msgs.msg import Float64, Float64MultiArray

from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.ros2_interface import get_parameters
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    DifferentialDrive,
    OmniDrive,
    PrismaticConnection,
)


@dataclass
class CommandPublisher(ABC):
    """
    Sends the velocities computed by the controller to the robot.
    """

    @abstractmethod
    def publish(self) -> None:
        """
        Publish the velocities currently stored in the world state.
        """

    @abstractmethod
    def stop(self) -> None:
        """
        Publish zero velocities so the robot comes to a halt.
        """


@dataclass
class JointVelocityCommandPublisher(CommandPublisher):
    """
    Publishes one velocity per joint, each on its own topic.
    """

    world: World
    """
    The world holding the commanded velocities.
    """

    namespaces: List[str]
    """
    Namespaces of the velocity controllers; each offers a ``command`` topic and a
    ``joint`` parameter.
    """

    connections: List[ActiveConnection1DOF] = field(init=False, default_factory=list)
    """
    The controlled connections, in the same order as ``namespaces``.
    """

    publishers: List[Publisher] = field(init=False, default_factory=list)
    """
    The command publishers, in the same order as ``namespaces``.
    """

    def __post_init__(self):
        for namespace in self.namespaces:
            self.publishers.append(
                rospy.node.create_publisher(Float64, f"/{namespace}/command", 10)
            )
            joint_name = (
                get_parameters(parameters=["joint"], node_name=namespace)
                .values[0]
                .string_value
            )
            connection: ActiveConnection1DOF = self.world.get_connection_by_name(
                joint_name
            )
            connection.has_hardware_interface = True
            self.connections.append(connection)

    def publish(self) -> None:
        for publisher, connection in zip(self.publishers, self.connections):
            message = Float64()
            message.data = self.world.state[connection.raw_dof.id].velocity
            publisher.publish(message)

    def stop(self) -> None:
        for publisher in self.publishers:
            publisher.publish(Float64())


@dataclass
class JointGroupVelocityCommandPublisher(CommandPublisher):
    """
    Publishes the velocities of a group of joints as a single message.
    """

    cmd_topic: str
    """
    Topic the velocity array is published on.
    """

    connections: List[ActiveConnection1DOF]
    """
    The controlled connections, in the order expected by the controller.
    """

    minimum_valid_velocity: float
    """
    Minimum magnitude that small non-prismatic, non-finger joint velocities are raised
    to so the hardware actually moves.

    A value of ``0.0`` disables clamping.
    """

    cmd_pub: Publisher = field(init=False)
    """
    The publisher for ``cmd_topic``.
    """

    def __post_init__(self):
        self.cmd_pub = rospy.node.create_publisher(
            Float64MultiArray, self.cmd_topic, 10
        )
        for connection in self.connections:
            connection.has_hardware_interface = True
        rospy.node.get_logger().info(
            f"Created publisher for {self.cmd_topic} for "
            f"{[connection.name.name for connection in self.connections]}"
        )

    def publish(self) -> None:
        message = Float64MultiArray()
        for connection in self.connections:
            message.data.append(self.clamp_velocity(connection))
        self.cmd_pub.publish(message)

    def clamp_velocity(self, connection: ActiveConnection1DOF) -> float:
        """
        Raise velocities that are too small for the hardware to the minimum magnitude.
        """
        velocity = connection.velocity
        absolute_velocity = abs(velocity)
        if isinstance(connection, PrismaticConnection):
            return velocity
        if "finger" in connection.name.name:
            return velocity
        if 0.0 < absolute_velocity < self.minimum_valid_velocity:
            return self.minimum_valid_velocity * np.sign(velocity)
        return velocity

    def stop(self) -> None:
        message = Float64MultiArray()
        for _ in self.connections:
            message.data.append(0.0)
        self.cmd_pub.publish(message)


@dataclass
class DriveVelocityCommandPublisher(CommandPublisher):
    """
    Publishes the velocity of a drive connection as a twist.
    """

    world: World
    """
    The world holding the commanded velocities.
    """

    cmd_topic: str
    """
    Topic the twist is published on.
    """

    connection: Union[OmniDrive, DifferentialDrive]
    """
    The drive connection that is commanded.
    """

    vel_pub: Publisher = field(init=False)
    """
    The publisher for ``cmd_topic``.
    """

    def __post_init__(self):
        self.vel_pub = rospy.node.create_publisher(Twist, self.cmd_topic, 10)
        self.connection.has_hardware_interface = True
        rospy.node.get_logger().info(f"Created publisher for {self.cmd_topic}.")

    def publish(self) -> None:
        command = Twist()
        command.linear.x = self.world.state[self.connection.x_velocity.id].velocity
        if isinstance(self.connection, OmniDrive):
            command.linear.y = self.world.state[self.connection.y_velocity.id].velocity
        command.angular.z = self.world.state[self.connection.yaw.id].velocity
        self.vel_pub.publish(command)

    def stop(self) -> None:
        self.vel_pub.publish(Twist())
