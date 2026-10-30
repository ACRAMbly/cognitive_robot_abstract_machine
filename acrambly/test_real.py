
from __future__ import annotations

import threading

import mujoco
import rclpy
from rclpy.executors import MultiThreadedExecutor

from coraplex.alternative_motion_mappings.tracy_motion_mapping import TracyRealMoveGripperMotion
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ExecutionType
from coraplex.plans.factories import sequential
from coraplex.robot_plans import MoveGripperMotion
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from datetime import timedelta

from coraplex.execution_environment import PhysicallySimulatedRobot, kinematically_simulated_robot, ExecutionEnvironment
from semantic_digital_twin.datastructures.definitions import StaticJointState
from acrambly.sub_parts.shared.utils import spawn_body
from semantic_digital_twin.semantic_annotations.semantic_annotations import GelatinBox
from semantic_digital_twin.spatial_types import Pose, Point3
from math import pi

table_top_z = 0.88

world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy, evaluate_conditions=False)

def populate_world(world):
    spawn_body(world, (0.7, 0, table_top_z+0.01), (0, 0, 0), "mesh", mesh_filename="fixtures/big_wooden_plate.stl")
    spawn_body(world, (1.01, 0.575, table_top_z+0.02), (pi, 0, 0), "mesh", mesh_filename="battat/BoltHolder.stl")
    spawn_body(world, (0.91, 0.5, table_top_z+0.02), (pi/2, 0, 0), "mesh", mesh_filename="battat/HolderUpperBody.stl")
    spawn_body(world, (0.925, 0.343, table_top_z+0.02), (pi, 0, 0), "mesh", mesh_filename="battat/HolderTopWing.stl")
    spawn_body(world, (0.91, 0.2, table_top_z+0.02), (pi/2, 0, 0), "mesh", mesh_filename="battat/HolderUnderBody.stl")
    spawn_body(world, (0.925, -0.05, table_top_z+0.02), (pi/2, 0, pi/2), "mesh", mesh_filename="battat/Plane_horizontal_holder_tall.stl")
    spawn_body(world, (0.925, -0.357, table_top_z+0.02), (0, 0, 0), "mesh", mesh_filename="battat/HolderBottomWing.stl")

def spawn_plane_parts():
    plane = {}
    plane["UpperBody"] = spawn_body(world,(0.91, 0.5, table_top_z + 0.05), (0, 0, 0), "mesh", mesh_filename="battat/plane/UpperBody.stl")
    plane["TopWing"] = spawn_body(world, (0.925, 0.305, table_top_z+0.1), (-pi/2, 0, 0), "mesh", mesh_filename="battat/plane/TopWing.stl")
    plane["UnderBody"] = spawn_body(world,(0.91, 0.2, table_top_z + 0.05), (-pi/2, pi, 0), "mesh", mesh_filename="battat/plane/UnderBody.stl")
    plane["BottomWing"] = spawn_body(world, (0.925, -0.325, table_top_z+0.1), (pi, 0, 0), "mesh", mesh_filename="battat/plane/BottomWing.stl")


    return plane

populate_world(world)


plan = sequential([
    ParkArmsAction(tracy.all_arms)
    ],
    context=context).plan
#
# for arm in tracy.all_arms:
#     arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)


MUJOCO = False
RVIZ = False
REAL = True

if MUJOCO:
    simulation = MujocoSim(
        world=world,
        headless=False,
        step_size=1e-3,
        cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
        impratio=10.0,
        noslip_iterations=10,
    )

    simulation.start_stepped_simulation()

    try:
        simulation.step_simulation(timedelta(seconds=1))
        with PhysicallySimulatedRobot(simulation=simulation):
            plan.perform()
    finally:
        simulation.stop_simulation()
elif RVIZ:
    rclpy.init()
    node = rclpy.create_node("coraplex_tracy_demo")
    TFPublisher(node=node, _world=world)
    VizMarkerPublisher(_world=world, node=node)
    world.notify_state_change()

    with kinematically_simulated_robot:
        plan.perform()


elif REAL:
    rclpy.init()
    node = rclpy.create_node("coraplex_tracy_demo")

    spinner = threading.Thread(
        target=rclpy.spin, daemon=True, args=(node,)
    )
    spinner.start()

    print("fetching")
    world = fetch_world_from_service(
        node=node, timeout_seconds=300
    )

    WorldSynchronizer(_world=world, node=node)
    print("synchronized")

    populate_world(world)

    battatco = spawn_plane_parts()

    tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]
    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[TracyRealMoveGripperMotion],
        evaluate_conditions=False,
    )
    plan = sequential(
        [
            ParkArmsAction(tracy.all_arms),
            PickUpAction(
                GraspCandidate(battatco["UpperBody"],
                               Pose.from_xyz_rpy(-0.1,0,0.03,0,pi / 2,0,reference_frame=battatco["UpperBody"].root,),
                ),
                tracy.left_arm,
            ),
        ],
        context=context,
    ).plan

    with ExecutionEnvironment(
            execution_type=ExecutionType.REAL,
            collision_avoidance=False,
    ):
        plan.perform()