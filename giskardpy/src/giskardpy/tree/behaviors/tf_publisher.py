from typing import Optional

from py_trees.common import Status

from giskardpy.middleware.ros2 import rospy
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.tree.blackboard_utils import (
    GiskardBlackboard,
    catch_and_raise_to_blackboard,
)


class TFPublisher(GiskardBehavior):
    """
    Published tf for attached and environment objects.
    """

    _last_logged_model_version: Optional[int] = None
    """
    Model version of the last republish, used to log a line only when the published TF
    topology actually changed, instead of on every tick.
    """

    @catch_and_raise_to_blackboard
    def update(self):
        world = GiskardBlackboard().executor.context.world
        current_version = world._model_manager.version
        if current_version != self._last_logged_model_version:
            rospy.node.get_logger().info(
                f"Publishing tf for world model version {current_version}."
            )
            self._last_logged_model_version = current_version
        GiskardBlackboard().giskard.tf_publisher.on_state_change()
        return Status.SUCCESS
