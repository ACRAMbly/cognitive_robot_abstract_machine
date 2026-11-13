
from __future__ import annotations

import threading

import rclpy
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.robot_plans.actions.core.robot_body import (
    MoveFingersToWidthAction,
    ParkArmsAction,
)
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from rclpy.executors import MultiThreadedExecutor

from doc.source.conf import extensions
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robot_parts import AbstractRobot

table_top_z = 0.88

rclpy.init()
node = rclpy.create_node("coraplex_tracy_demo")

executor = MultiThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

world = fetch_world_from_service(node=node, timeout_seconds=300)

WorldSynchronizer(_world=world, node=node)

tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]
executor = RealRobotPlanExecutor(
    context=StatechartContext.with_extensions(world=world, extensions=[RobotAccess(robot=tracy)]),
    ros_node=node,
    collision_avoidance=False,
)

actions = [
    ParkArmsAction(arms=tracy.all_arms),
    MoveFingersToWidthAction(gripper=tracy.right_arm.end_effector, width=0.035)
]

statechart = Statechart(context=executor.context)
statechart.add_node(node=Sequence(nodes=actions))

executor.compile(statechart=statechart)

executor.execute()
executor.close()