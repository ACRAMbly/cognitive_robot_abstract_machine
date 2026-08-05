from dataclasses import dataclass, field

from giskardpy.model.world_config import WorldWithOmniDriveRobot
from giskardpy.middleware.ros2.robot_interface_config import (
    RobotInterfaceConfig,
)
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world_description.connections import (
    OmniDrive,
    Connection6DoF,
)


@dataclass
class WorldWithHSRConfig(WorldWithOmniDriveRobot):
    urdf_view: AbstractRobot = field(kw_only=True, default=HSRB, init=False)


@dataclass
class HSRStandaloneInterface(RobotInterfaceConfig):
    """
    Simulates the arm, head and drive of the HSR without talking to hardware.
    """

    def setup(self):
        self.register_controlled_joints(
            [
                "arm_flex_joint",
                "arm_lift_joint",
                "arm_roll_joint",
                "head_pan_joint",
                "head_tilt_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
                self.world.get_connections_by_type(OmniDrive)[0].name,
            ]
        )


@dataclass
class HSRVelocityInterface(RobotInterfaceConfig):
    """
    Commands the arm, head and drive of the HSR through their velocity controllers.
    """

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(
            joint=self.world.get_connections_by_type(Connection6DoF)[0],
            tf_parent_frame="map",
            tf_child_frame="odom",
        )

        omni_drive = self.world.get_connections_by_type(OmniDrive)[0]
        self.sync_odometry_topic(
            "/laser_odom",
            omni_drive,
        )

        self.add_base_cmd_velocity(
            cmd_vel_topic="/omni_base_controller/cmd_vel", joint=omni_drive
        )

        self.sync_joint_state_topic("/joint_states")
        joints_left = [
            "arm_flex_joint",
            "arm_lift_joint",
            "arm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
            "head_pan_joint",
            "head_tilt_joint",
        ]
        self.add_joint_velocity_group_controller(
            cmd_topic="/realtime_body_controller_real/command", connections=joints_left
        )
