from dataclasses import dataclass, field
from typing import List

from giskardpy.middleware.ros2.robot_interface_config import (
    StandAloneRobotInterfaceConfig,
    RobotInterfaceConfig,
)
from giskardpy.model.world_config import (
    WorldWithOmniDriveRobot,
    WorldWithDiffDriveRobot,
)
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    DifferentialDrive,
)


@dataclass
class StretchStandaloneInterface(StandAloneRobotInterfaceConfig):
    """
    Simulates the arm, gripper, head and drive of Stretch without talking to hardware.
    """

    drive_joint_name: str = "brumbrum"
    """
    Name of the drive connection that is controlled alongside the other joints.
    """

    joint_names: List[str] = field(init=False, default_factory=list)
    """
    The drive joint plus the arm, gripper, wheel and head joints of Stretch.
    """

    def __post_init__(self) -> None:
        self.joint_names = [
            self.drive_joint_name,
            "joint_gripper_finger_left",
            "joint_gripper_finger_right",
            "joint_right_wheel",
            "joint_left_wheel",
            "joint_lift",
            "joint_arm_l3",
            "joint_arm_l2",
            "joint_arm_l1",
            "joint_arm_l0",
            "joint_wrist_yaw",
            "joint_head_pan",
            "joint_head_tilt",
        ]


@dataclass
class StretchVelocityInterface(RobotInterfaceConfig):
    """
    Commands the arm, head and drive of Stretch through their velocity controllers.
    """

    def setup(self):
        self.sync_6dof_joint_with_tf_frame(
            joint=self.world.get_connections_by_type(Connection6DoF)[0],
            tf_parent_frame="map",
            tf_child_frame="odom",
        )

        diff_drive = self.world.get_connections_by_type(DifferentialDrive)[0]
        self.sync_odometry_topic(
            "/odom",
            diff_drive,
        )

        self.add_base_cmd_velocity(cmd_vel_topic="/stretch/cmd_vel", joint=diff_drive)

        self.sync_joint_state_topic("/joint_states")
        joints = [
            "joint_arm_l0",  # 0
            "joint_lift",  # 1
            "joint_wrist_yaw",  # 2
            "joint_wrist_pitch",  # 3
            "joint_wrist_roll",  # 4
            "joint_head_pan",  # 5
            "joint_head_tilt",  # 6
            "joint_gripper_finger_left",  # 7
            "joint_right_wheel",  # 8
            "joint_left_wheel",  # 9
        ]
        self.add_joint_velocity_group_controller(
            cmd_topic="/joint_velocity_cmd",
            connections=joints,
            minimum_valid_velocity=0.03,
            minimum_velocity_overrides={
                "joint_lift": 0.0,
                "joint_arm_l0": 0.0,
                "joint_gripper_finger_left": 0.0,
            },
        )


@dataclass
class WorldWithStretchConfig(WorldWithOmniDriveRobot):
    urdf_view: AbstractRobot = field(kw_only=True, default=Stretch, init=False)

    def setup_collision_config(self):
        pass


@dataclass
class WorldWithStretchConfigDiffDrive(WorldWithDiffDriveRobot):
    urdf_view: AbstractRobot = field(kw_only=True, default=Stretch, init=False)

    def setup_collision_config(self):
        pass
