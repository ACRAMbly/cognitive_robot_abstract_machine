
from __future__ import annotations

import threading

import rclpy
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.robot_plans.actions.core.robot_body import (
    SetGripperAction,
    ParkArmsAction,
)
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from rclpy.executors import MultiThreadedExecutor
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.datastructures.definitions import GripperState

from acrambly.sub_parts.shared.utils import spawn_body
from math import pi

table_top_z = 0.88

rclpy.init()
node = rclpy.create_node("coraplex_tracy_demo")

executor = MultiThreadedExecutor()
executor.add_node(node)

threading.Thread(target=executor.spin, daemon=True).start()

world = fetch_world_from_service(node=node, timeout_seconds=300)
WorldSynchronizer(_world=world, node=node)

UpperBody = spawn_body(world,(0.945, 0.5, table_top_z + 0.05), (pi, 0, 0), "mesh", mesh_filename="battat/plane/UpperBody.stl")


tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]
executor = RealRobotPlanExecutor(
    context=StatechartContext.with_extensions(
        world=world,
        extensions=[
            RobotAccess(robot=tracy),

        ],
    ),
    ros_node=node,
    collision_avoidance=False,
)
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from krrood_spatial_math.spatial_types import Pose

gripper = tracy.right_arm.end_effector
actions = [
    ParkArmsAction(arms=tracy.all_arms),
    PickUpAction(
            grasp_candidate=GraspCandidate(UpperBody, Pose.from_xyz_rpy(-0.1,0,0.05,0, pi/2,0,reference_frame=UpperBody.root)),
            arm=tracy.left_arm
        ),
]

statechart = Statechart(context=executor.context)
statechart.add_node(node=Sequence(nodes=actions))

executor.compile(statechart=statechart)

executor.execute()
executor.close()