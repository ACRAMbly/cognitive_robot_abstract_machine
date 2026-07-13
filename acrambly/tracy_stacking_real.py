import subprocess
import threading
import time
import rclpy
import typer
from typing import Literal

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.robots.robot_parts import AbstractRobot

from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body

from semantic_digital_twin.world import World
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan

from coraplex.robot_plans.actions.composite.transporting import PickAndPlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.alternative_motion_mappings.tracy_motion_mapping import TracyRealMoveGripperMotion


#### IMPORTANT: RESTART THE GISKARD SCRIPT EACH TIME YOU RUN THIS SCRIPT

# giskard_process = subprocess.Popen(
#     ["ros2", "launch", "giskardpy_ros", "giskardpy_tracy_velocity.launch.py"],
#     start_new_session=True,
# )
# print("Initializing GISKARD...")
# time.sleep(10)

def spawn_box(spawn_world: World, name: str = "box", position: tuple = (0.0, 0.0, 1.5), scale: float = 0.1, r: float = 0.0, g: float = 0.0, b: float = 0.0) -> Body:
    spawn_body = URDFParser(f"""<?xml version="1.0"?>
        <robot name="{name}_box">
          <link name="{name}_link">
            <inertial>
              <mass value="0.1"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
            </inertial>
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{scale} {scale} {scale}"/>
              </geometry>
              <material name="{name}_mat">
                <color rgba="{r} {g} {b} 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{scale} {scale} {scale}"/>
              </geometry>
            </collision>
          </link>
        </robot>
        """).parse()

    with spawn_world.modify_world():
        connection = Connection6DoF.create_with_dofs(
            parent=spawn_world.root,
            child=spawn_body.root,
            world=spawn_world,
        )
        spawn_world.merge_world(spawn_body, connection)

    time.sleep(0.5)
    # Set the initial world pose of the box via the 6-DoF DoF state.
    connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=position[0],
        y=position[1],
        z=position[2],
        reference_frame=spawn_body,
    )

    box = spawn_world.get_kinematic_structure_entity_by_name(f"{name}_link")
    return box

def setup_world(node):
    print("Getting live world from Giskard...")
    tracy_world = fetch_world_from_service(node, timeout_seconds=300)
    print(f"World received with {len(list(tracy_world.bodies))} bodies.")
    WorldSynchronizer(_world=tracy_world, node=node, synchronous=True)
    print("Synchronized.")

    print("Adding boxes to world.")

    ##### PERCEPTION CODE HERE PLS #####

    red_box_pos = (0.8, 0.5, 0.955)
    green_box_pos = (0.8, -0.5, 0.955)
    blue_box_pos = (0.8, 0, 0.955)
    SCALE = 0.1

    #####     THANK YOU            #####

    red = spawn_box(tracy_world, "red", red_box_pos, SCALE, 1.0, 0.0, 0.0)
    green = spawn_box(tracy_world, "green", green_box_pos, SCALE, 0.0, 1.0, 0.0)
    blue = spawn_box(tracy_world, "blue", blue_box_pos, SCALE, 0.0, 0.0, 1.0)

    return tracy_world, red, green, blue

def build_plan_cubes(world: World, tracy: Tracy, context: Context, red_box: Body, green_box: Body, blue_box: Body) -> Plan | None:
    stack_pos_x = 1
    stack_pos_y = 0
    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickAndPlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    0.955,
                    reference_frame=world.root
                ),
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0],
                ),
            ),
            PickAndPlaceAction(
                green_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.005,
                    reference_frame=world.root
                ),
                Arms.RIGHT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[1]
                ),
            ),
            PickAndPlaceAction(
                blue_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.055,
                    reference_frame=world.root
                ),
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0]
                ),
            ),
        ],
        context=context,
    ).plan

def build_park_arms_plan(context: Context) -> Plan | None:
    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan

def main(plan_name: Literal["park_arms", "cubes"]):
    rclpy.init()

    node = rclpy.create_node("coraplex_real_stacking")

    # Giskard action clients need the node to spin.
    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    world, red_box, green_box, blue_box = setup_world(node)

    print("Building Tracy semantic robot from giskard world...")
    tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]

    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[
            TracyRealMoveGripperMotion
        ],
        evaluate_conditions=False,
    )

    if plan_name == "cubes":
        plan = build_plan_cubes(world, tracy, context, red_box, green_box, blue_box)
    else:
        plan = build_park_arms_plan(context)

    print("Executing Plan on REAL robot through Giskard...")
    print("Keep E-stop reachable.")

    try:
        with real_robot:
            plan.perform()
            print("Plan completed.")
    except Exception as e:
        print(f"Error during plan execution: {e}")

if __name__ == "__main__":
    typer.run(main)
