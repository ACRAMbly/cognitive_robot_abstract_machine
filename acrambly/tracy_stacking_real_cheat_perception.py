# workon cram-env
# python acrambly/tracy_stacking_real_cheat_perception.py cubes

import os
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
from coraplex.robot_plans.actions.core.pick_up import ReachAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
import coraplex.alternative_motion_mappings.tracy_motion_mapping

from rclpy.action import ActionClient
from robokudo_msgs.action import Query
from geometry_msgs.msg import PoseStamped


#### IMPORTANT: RESTART THE GISKARD SCRIPT EACH TIME YOU RUN THIS SCRIPT
#### OR MAYBE ADD BASH COMMAND UP TO YOU

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

TARGET_COLORS = {"red", "yellow", "blue"}

def query_colored_block_poses_from_robokudo(node, max_attempts: int = 10) -> dict:
    """
    Query RoboKudo repeatedly until all TARGET_COLORS are found.

    Detection varies frame to frame, so if a color is missing we re-query and
    accumulate found colors across attempts, keeping the first pose per color.
    """
    poses_by_color = {}

    for attempt in range(1, max_attempts + 1):
        missing = TARGET_COLORS - set(poses_by_color)
        if not missing:
            break

        print(f"\n[attempt {attempt}/{max_attempts}] querying; still missing: {sorted(missing)}")

        # Fresh action client each attempt (avoids reusing a client whose previous
        # goal handle may not be fully released).
        action_client = ActionClient(node, Query, "/robokudo/query")
        if not action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("RoboKudo query action server is not available.")

        goal = Query.Goal()
        goal.obj.type = "block"

        send_future = action_client.send_goal_async(goal)
        # bounded wait so we never hang forever if the server wedges
        waited = 0.0
        while not send_future.done():
            time.sleep(0.05)
            waited += 0.05
            if waited > 15.0:
                raise RuntimeError(
                    "Timed out waiting for RoboKudo to accept the goal "
                    "(server may be wedged; restart the perception script)."
                )

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("RoboKudo rejected the block query.")

        result_future = goal_handle.get_result_async()
        waited = 0.0
        while not result_future.done():
            time.sleep(0.05)
            waited += 0.05
            if waited > 30.0:
                raise RuntimeError(
                    "Timed out waiting for RoboKudo result "
                    "(server may be wedged; restart the perception script)."
                )

        result = result_future.result().result

        # ===== RAW QUERY RESULT LOG (test only) =====
        print(f"  raw: {len(result.res)} objects detected this attempt")
        for i, od in enumerate(result.res):
            colors = list(od.color)
            if od.pose:
                p = od.pose[0].pose.position
                print(f"    [{i}] colors={colors}  pos=(x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f})  frame={od.pose[0].header.frame_id}")
            else:
                print(f"    [{i}] colors={colors}  pose=<none>")
        # ============================================

        # Accumulate any target colors we don't already have.
        for object_designator in result.res:
            if not object_designator.pose:
                continue
            for color in object_designator.color:
                if color in TARGET_COLORS and color not in poses_by_color:
                    poses_by_color[color] = object_designator.pose[0]
                    print(f"  -> found '{color}'")

        # Clean up this attempt's client before the next attempt.
        action_client.destroy()

        # Give the RoboKudo action server time to fully finish/close
        missing_after = TARGET_COLORS - set(poses_by_color)
        if missing_after:
            print(f"  still missing {sorted(missing_after)}; pausing before next attempt...")
            time.sleep(3.0)

    missing = TARGET_COLORS - set(poses_by_color)
    if missing:
        raise RuntimeError(
            f"RoboKudo did not detect blocks with colors {sorted(missing)} "
            f"after {max_attempts} attempts."
        )

    return poses_by_color

def pose_to_position(pose_stamped) -> tuple:
    p = pose_stamped.pose.position
    return (p.x, p.y, p.z)

def setup_world(node):
    print("Getting live world from Giskard...")
    tracy_world = fetch_world_from_service(node, timeout_seconds=300)
    print(f"World received with {len(list(tracy_world.bodies))} bodies.")
    WorldSynchronizer(_world=tracy_world, node=node, synchronous=True)
    print("Synchronized.")

    print("Adding boxes to world.")

    ##### PERCEPTION CODE HERE PLS #####

    block_poses = query_colored_block_poses_from_robokudo(node)

    red_box_pos = pose_to_position(block_poses["red"])
    green_box_pos = pose_to_position(block_poses["yellow"])
    blue_box_pos = pose_to_position(block_poses["blue"])
    SCALE = 0.1

    # ===== FILTERED POSITIONS LOG (test only) =====
    print("\n===== Positions used for spawning =====")
    print(f"  red    (from 'red')   : x={red_box_pos[0]:.3f}  y={red_box_pos[1]:.3f}  z={red_box_pos[2]:.3f}")
    print(f"  green  (from 'yellow'): x={green_box_pos[0]:.3f}  y={green_box_pos[1]:.3f}  z={green_box_pos[2]:.3f}")
    print(f"  blue   (from 'blue')  : x={blue_box_pos[0]:.3f}  y={blue_box_pos[1]:.3f}  z={blue_box_pos[2]:.3f}")
    print("=======================================\n")
    # ==============================================

    #####     THANK YOU            #####

    red = spawn_box(tracy_world, "red", red_box_pos, SCALE, 1.0, 0.0, 0.0)
    green = spawn_box(tracy_world, "green", green_box_pos, SCALE, 0.0, 1.0, 0.0)
    blue = spawn_box(tracy_world, "blue", blue_box_pos, SCALE, 0.0, 0.0, 1.0)

    return tracy_world, red, green, blue

def build_plan_cubes(world: World, tracy: Tracy, context: Context, red_box: Body, green_box: Body, blue_box: Body) -> Plan | None:
    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickAndPlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    0.6,
                    0.0,
                    0.93,
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
                    0.6,
                    0.0,
                    1.03,
                    reference_frame=world.root
                ),
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0]
                ),
            ),
            PickAndPlaceAction(
                blue_box,
                Pose.from_xyz_rpy(
                    0.6,
                    0.0,
                    1.13,
                    reference_frame=world.root
                ),
                Arms.RIGHT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[1]
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

def test_stacking_cube(z, context, tracy, body, world, arm):
    command = f"ros2 action send_goal /{arm}_gripper/robotiq_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "
    sequential([
        ParkArmsAction(Arms.BOTH),
        ReachAction(
            target_pose=body.global_pose,
            object_designator=body,
            arm=Arms.LEFT if arm=="left" else Arms.RIGHT,
            grasp_description=GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.TOP,
                Tracy.get_end_effectors(tracy)[0 if arm=="left" else 1], ))], context=context).plan.perform()
    os.system(
        command + '"{command: {position: [0.35], effort: [10.0]}}"'
    )
    print(command + '"{command: {position: [0.35], effort: [10.0]}}"')

    sequential([
        ParkArmsAction(Arms.LEFT),
        ReachAction(
            target_pose=Pose.from_xyz_rpy(
                1,
                0,
                z,
                reference_frame=world.root
            ),
            object_designator=body,
            arm=Arms.LEFT if arm=="left" else Arms.RIGHT,
            grasp_description=GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.TOP,
                Tracy.get_end_effectors(tracy)[0 if arm=="left" else 1], ))], context=context).plan.perform()
    os.system(
        command + '"{command: {position: [0], effort: [10.0]}}"'
    )
    sequential([
        ParkArmsAction(Arms.BOTH)], context=context).plan.perform()

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

    context = Context(world=world, robot=tracy, ros_node=node)
    context.evaluate_conditions = False

    if plan_name == "cubes":
        plan = build_plan_cubes(world, tracy, context, red_box, green_box, blue_box)
    else:
        plan = build_park_arms_plan(context)

    print("Executing ParkArmsAction on REAL robot through Giskard...")
    print("Keep E-stop reachable.")

    # ===== ARM MOTION COMMENTED OUT FOR QUERY-ONLY TEST =====
    # try:
    #     with real_robot:
    #         test_stacking_cube(
    #             z = 0.955,
    #             context = context,
    #             body=red_box,
    #             tracy=tracy,
    #             world=world,
    #             arm="left"
    #         )
    #         test_stacking_cube(
    #             z=1.005,
    #             context=context,
    #             body=blue_box,
    #             tracy=tracy,
    #             world=world,
    #             arm="left"
    #         )
    #         test_stacking_cube(
    #             z=1.055,
    #             context=context,
    #             body=green_box,
    #             tracy=tracy,
    #             world=world,
    #             arm="left"
    #         )
    #         #plan.perform()
    #         print("Plan completed.")
    # except Exception as e:
    #     print(f"Error during plan execution: {e}")
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    print("Query-only test done. No arm motion executed.")
    node.destroy_node()
    rclpy.shutdown()
    # ========================================================

if __name__ == "__main__":
    typer.run(main)