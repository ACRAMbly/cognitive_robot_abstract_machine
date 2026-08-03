import traceback
from threading import Thread

from py_trees.common import Status

from giskardpy.middleware.ros2 import rospy
from giskardpy.tree.behaviors.plugin import GiskardBehavior
from giskardpy.tree.blackboard_utils import GiskardBlackboard


class ProcessWorldUpdate(GiskardBehavior):

    def __init__(self):
        name = f"Processing world updates"
        self.worker_thread = None
        super().__init__(name)

    def update(self) -> Status:
        if self.worker_thread is None:
            pending_count = len(
                GiskardBlackboard().giskard.world_synchronizer.missed_messages
            )
            if pending_count > 0:
                rospy.node.get_logger().info(
                    f"Processing {pending_count} buffered world update(s) while idle."
                )
            self.worker_thread = Thread(target=self.process_goal, name=self.name)
            self.worker_thread.start()
        else:
            if not self.worker_thread.is_alive():
                self.worker_thread = None
                rospy.node.get_logger().info(
                    f"Finished world update, model version: {GiskardBlackboard().executor.context.world._model_manager.version}."
                )
                return Status.SUCCESS
        return Status.RUNNING

    def process_goal(self):
        try:
            GiskardBlackboard().giskard.world_synchronizer.apply_missed_messages()
        except Exception:
            rospy.node.get_logger().error(
                "Applying buffered world updates failed, the world model stays out of "
                f"sync until the next successful attempt:\n{traceback.format_exc()}"
            )
            raise
