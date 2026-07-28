from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.scripts.iai_robots.hsr.configs import (
    HSRVelocityInterface,
)
from giskardpy.model.world_config import WorldFromDatabaseConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.middleware.ros2.server_config import ExecutionMode, GiskardServerConfig
from giskardpy.middleware.ros2.giskard import Giskard


def main():
    rospy.init_node("giskard")
    giskard = Giskard(
        world_config=WorldFromDatabaseConfig(primary_key=1),
        robot_interface_config=HSRVelocityInterface(),
        qp_controller_config=QPControllerConfig(
            target_frequency=40, prediction_horizon=15
        ),
        server_config=GiskardServerConfig(
            execution_mode=ExecutionMode.CLOSED_LOOP, debug_mode=False
        ),
    )
    giskard.live()


if __name__ == "__main__":
    main()
