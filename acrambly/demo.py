
from __future__ import annotations

import mujoco
import rclpy


from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import KinematicallySimulatedPlanExecutor
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    ParkArmsAction,
    MoveManipulatorAction, MoveFingersToWidthAction,
)
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch
from krrood_spatial_math.spatial_types import Pose
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_part_mixins import HasLeftRightArm
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from datetime import timedelta

from semantic_digital_twin.datastructures.definitions import StaticJointState
from acrambly.sub_parts.shared.utils import spawn_body
from math import pi

table_top_z = 0.88


def spawn_holders(world):
    holders = {}

    spawn_body(world, (0.7, 0, table_top_z+0.01), (0, 0, 0), "mesh", mesh_filename="fixtures/big_wooden_plate.stl")
    holders["HolderBolt"] = spawn_body(world, (1.01, 0.575, table_top_z+0.02), (pi, 0, 0), "mesh", mesh_filename="battat/BoltHolder.stl")
    holders["HolderUpperBody"] = spawn_body(world, (0.91, 0.5, table_top_z+0.02), (pi/2, 0, 0), "mesh", mesh_filename="battat/HolderUpperBody.stl")
    holders["HolderTopWing"] = spawn_body(world, (0.925, 0.343, table_top_z+0.02), (pi, 0, 0), "mesh", mesh_filename="battat/HolderTopWing.stl")
    holders["HolderUnderBody"] = spawn_body(world, (0.91, 0.2, table_top_z+0.02), (pi/2, 0, 0), "mesh", mesh_filename="battat/HolderUnderBody.stl")
    holders["HolderHorizontal"] = spawn_body(world, (0.925, -0.05, table_top_z+0.02), (pi/2, 0, pi/2), "mesh", mesh_filename="battat/Plane_horizontal_holder_tall.stl")
    holders["HolderBottomWing"] = spawn_body(world, (0.925, -0.357, table_top_z+0.02), (0, 0, 0), "mesh", mesh_filename="battat/HolderBottomWing.stl")

    return holders

def spawn_plane_parts(world):
    plane = {}

    plane["UpperBody"] = spawn_body(world,(0.945, 0.5, table_top_z + 0.05), (pi, 0, 0), "mesh", mesh_filename="battat/plane/UpperBody.stl")
    plane["TopWing"] = spawn_body(world, (0.925, 0.305, table_top_z+0.1), (-pi/2, 0, 0), "mesh", mesh_filename="battat/plane/TopWing.stl")
    plane["UnderBody"] = spawn_body(world,(0.91, 0.2, table_top_z + 0.05), (-pi/2, pi, 0), "mesh", mesh_filename="battat/plane/UnderBody.stl")
    plane["BottomWing"] = spawn_body(world, (0.925, -0.325, table_top_z+0.1), (pi, 0, 0), "mesh", mesh_filename="battat/plane/BottomWing.stl")
    plane["RearWing"] = spawn_body(world, (0.923, 0.028, table_top_z+0.125), (0, 0, 0), "mesh", mesh_filename="battat/plane/RearWing.stl")

    return plane

def build_plan(context, battatco, holders):
    tracy = context.require_extension(RobotAccess).robot
    actions = [
        ParkArmsAction(arms=tracy.all_arms),
        PickUpAction(
            grasp=GraspCandidate(battatco["BottomWing"], Pose.from_xyz_rpy(0.09, 0.05,0, pi/2, 0, -pi/2,reference_frame=battatco["BottomWing"].root)),
            arm=tracy.right_arm
        ),
        MoveManipulatorAction(
            target_pose=Pose.from_xyz_rpy(-0.089, 0.13, -0.094, pi/2, 0, pi/2, reference_frame=holders["HolderHorizontal"].root),
            end_effector=tracy.right_arm.end_effector,
            allow_gripper_collision=False
        ),
        PickUpAction(
                grasp=GraspCandidate(battatco["UpperBody"], Pose.from_xyz_rpy(-0.1,0,0.03,0, pi/2,0,reference_frame=battatco["UpperBody"].root)),
                arm=tracy.left_arm
        ),
        PlaceAction(
            object_designator=battatco["UpperBody"],
            target_location=Pose.from_xyz_rpy(0, 0.037, -0.028, pi, 0, -pi/2,reference_frame=battatco["UnderBody"].root)
        ),
        MoveBranch(branch_root=battatco["UnderBody"].root, new_parent=battatco["UpperBody"].root),
        PickUpAction(
                grasp=GraspCandidate(battatco["UpperBody"], Pose.from_xyz_rpy(-0.1,0,0.03,0, pi/2,0,reference_frame=battatco["UpperBody"].root)),
                arm=tracy.left_arm
        ),
        PlaceAction(
            object_designator=battatco["UpperBody"],
            target_location=Pose.from_xyz_rpy(0, 0.187, 0, -pi/2, pi, 0,reference_frame=holders["HolderHorizontal"].root)
        ),
        MoveBranch(branch_root=battatco["BottomWing"].root, new_parent=battatco["UnderBody"].root),
        MoveBranch(branch_root=battatco["RearWing"].root, new_parent=battatco["UnderBody"].root),
        MoveFingersToWidthAction(gripper=tracy.right_arm.end_effector, width=0.075),
        MoveManipulatorAction(
            target_pose=Pose.from_xyz_rpy(-0.2, 0.13, -0.1, pi/2, 0, pi/2, reference_frame=holders["HolderHorizontal"].root),
            end_effector=tracy.right_arm.end_effector,
            allow_gripper_collision=False
        ),
        ParkArmsAction(arms=tracy.all_arms)
    ]

    statechart = Statechart(context=context)
    statechart.add_node(Sequence(nodes=actions))
    return statechart

MUJOCO = False
RVIZ = True

if MUJOCO:
    pass
    # simulation = MujocoSim(
    #     world=world,
    #     headless=False,
    #     step_size=1e-3,
    #     cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
    #     impratio=10.0,
    #     noslip_iterations=10,
    # )
    #
    # simulation.start_stepped_simulation()
    #
    # try:
    #     simulation.step_simulation(timedelta(seconds=1))
    #     with PhysicallySimulatedRobot(simulation=simulation):
    #         plan.perform()
    # finally:git
    #     simulation.stop_simulation()
elif RVIZ:
    world = URDFParser.from_file(Tracy.get_ros_file_path()).create_world()
    tracy = Tracy.from_world(world)

    for arm in tracy.all_arms:
         arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)

    holders = spawn_holders(world)
    battatco = spawn_plane_parts(world)

    rclpy.init()
    node = rclpy.create_node("coraplex_tracy_demo")
    TFPublisher(node=node, _world=world)
    VizMarkerPublisher(_world=world, node=node)
    world.notify_state_change()

    executor = KinematicallySimulatedPlanExecutor(
        StatechartContext.with_extensions(world, [RobotAccess(tracy)]),
        ros_node=node,
    )
    executor.compile(build_plan(executor.context, battatco, holders))
    executor.execute()
    executor.close()