from dataclasses import dataclass, field

from giskardpy.model.world_config import WorldWithOmniDriveRobot
from giskardpy.middleware.ros2.giskard import RobotInterfaceConfig
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.pr2 import PR2, PR2Joint
from semantic_digital_twin.world_description.connections import (
    OmniDrive,
)


@dataclass
class WorldWithPR2Config(WorldWithOmniDriveRobot):
    odom_body_name: PrefixedName = PrefixedName("odom_combined")
    urdf_view: AbstractRobot = field(kw_only=True, default=PR2, init=False)


class PR2StandaloneInterface(RobotInterfaceConfig):
    def setup(self):
        self.register_controlled_joints(
            [
                PR2Joint.TORSO_LIFT,
                PR2Joint.HEAD_PAN,
                PR2Joint.HEAD_TILT,
                PR2Joint.RIGHT_SHOULDER_PAN,
                PR2Joint.RIGHT_SHOULDER_LIFT,
                PR2Joint.RIGHT_UPPER_ARM_ROLL,
                PR2Joint.RIGHT_FOREARM_ROLL,
                PR2Joint.RIGHT_ELBOW_FLEX,
                PR2Joint.RIGHT_WRIST_FLEX,
                PR2Joint.RIGHT_WRIST_ROLL,
                PR2Joint.LEFT_SHOULDER_PAN,
                PR2Joint.LEFT_SHOULDER_LIFT,
                PR2Joint.LEFT_UPPER_ARM_ROLL,
                PR2Joint.LEFT_FOREARM_ROLL,
                PR2Joint.LEFT_ELBOW_FLEX,
                PR2Joint.LEFT_WRIST_FLEX,
                PR2Joint.LEFT_WRIST_ROLL,
                self.world.get_connections_by_type(OmniDrive)[0].name,
            ]
        )


class PR2VelocityMujocoInterface(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(
        self,
        map_name: str = "map",
        localization_joint_name: str = "localization",
        odom_link_name: str = "odom_combined",
        drive_joint_name: str = "brumbrum",
    ):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.discover_interfaces_from_controller_manager()
        self.sync_odometry_topic("/odom", self.drive_joint_name)
        self.add_base_cmd_velocity(cmd_vel_topic="/cmd_vel")
