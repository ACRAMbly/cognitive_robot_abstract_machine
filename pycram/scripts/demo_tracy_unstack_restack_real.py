#!/usr/bin/env python3
"""
demo_tracy_unstack_restack_real.py
==============================
Tests unstacking 3 boxes from the right side and restacking them in the middle.

This script demonstrates the high-level PyCRAM pipeline:
- PickUpAction
- PlaceAction
"""

import threading
import sys
import time
import os

# Ensure the SOURCE version of pycram is used (not the older installed version
# from /install/ which colcon build puts on PYTHONPATH and which lacks Context).
_PYCRAM_SRC = os.path.join(
    os.path.dirname(__file__), "..", "src"
)
if _PYCRAM_SRC not in sys.path:
    sys.path.insert(0, os.path.abspath(_PYCRAM_SRC))

import rclpy

from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy
from tf_transformations import quaternion_from_euler

from semantic_digital_twin.world_description.connections import FixedConnection, Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
import semantic_digital_twin.spatial_types.spatial_types as cas
from semantic_digital_twin.adapters.urdf import URDFParser

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.datastructures.pose import PoseStamped
from pycram.robot_plans.actions.core.pick_up import PickUpActionDescription
from pycram.robot_plans.actions.core.placing import PlaceActionDescription
from pycram.robot_plans.motions.gripper import MoveTCPMotion
from pycram.view_manager import ViewManager
# Register real-robot gripper alternative for Tracy (calls Robotiq action servers
# instead of a Giskard joint task when inside `with real_robot:`).
import pycram.alternative_motion_mappings.tracy_motion_mapping  # noqa: F401


def create_pose(world, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = world.root
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    q = quaternion_from_euler(roll, pitch, yaw)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p


def spawn_urdf(world, name, filepath, pose_stamped):
    """Spawns an object into Giskard from a given URDF file using PyCRAM Semantic Digital Twin."""
    p = cas.Point3(
        pose_stamped.pose.position.x, 
        pose_stamped.pose.position.y, 
        pose_stamped.pose.position.z
    )
    q = cas.Quaternion(
        pose_stamped.pose.orientation.x, 
        pose_stamped.pose.orientation.y, 
        pose_stamped.pose.orientation.z, 
        pose_stamped.pose.orientation.w
    )
    parent_T_pose = cas.HomogeneousTransformationMatrix.from_point_rotation_matrix(p, q.to_rotation_matrix())
    
    parser = URDFParser.from_file(filepath, prefix=name)
    urdf_world = parser.parse()
    
    root_name = urdf_world.root.name
    other_root = urdf_world.root

    with world.modify_world():
        root_connection = Connection6DoF.create_with_dofs(
            parent=world.root, child=other_root, world=world
        )
        world.merge_world(urdf_world, root_connection)
        
    time.sleep(0.5) 
    root_connection.origin = parent_T_pose 
    
    obj_body = world.get_kinematic_structure_entity_by_name(root_name)
    print(f"Spawned URDF {name} at {p}")
    return obj_body

def generate_colored_box_urdf(color_name, r, g, b, sz_x, sz_y, sz_z):
    urdf = f"""<?xml version="1.0"?>
<robot name="{color_name}_box">
  <link name="{color_name}_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{sz_x} {sz_y} {sz_z}"/>
      </geometry>
      <material name="{color_name}_mat">
        <color rgba="{r} {g} {b} 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{sz_x} {sz_y} {sz_z}"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
    path = f"/tmp/{color_name}_box.urdf"
    with open(path, "w") as f:
        f.write(urdf)
    return path


def main():
    rclpy.init()
    node = rclpy.create_node("pycram_unstack_restack_demo")
    
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()
    
    rospy.node = node

    print("[1/3] Connecting to Giskard...")
    try:
        giskard = GiskardWrapper(node_handle=node)
    except Exception as e:
        print(f"Could not connect to Giskard: {e}")
        rclpy.shutdown()
        sys.exit(1)

    world = giskard.world
    tracy = Tracy.from_world(world)
    context = Context(world, tracy, ros_node=node)
    
    print("\n[2/3] Setting up scene (Spawning a stack of 3 boxes on the right) and RViz...")

    box_height = 0.05

    red_urdf = generate_colored_box_urdf("red", 1.0, 0.0, 0.0, box_height, box_height, box_height)
    blue_urdf = generate_colored_box_urdf("blue", 0.0, 0.0, 1.0, box_height, box_height, box_height)
    green_urdf = generate_colored_box_urdf("green", 0.0, 1.0, 0.0, box_height, box_height, box_height)

    # Spawning positions (Stacked perfectly on the right side)
    st_x = 0.80  # Closer to robot base
    st_y = -0.30 # Closer to center
    # st_z = center-height of the BOTTOM box in the MAP frame.
    # This must equal (real_table_surface_z + box_height / 2).
    # Run `ros2 topic echo /tf --once` and measure the "table" link z, then add the
    # physical table top height from the URDF to get real_table_surface_z.
    # Current value 0.93 is calibrated to our TracyBot table (surface ≈ 0.90 m in map frame).
    st_z = 0.93
    pick_pos_red =   (st_x, st_y, st_z)
    pick_pos_blue =  (st_x, st_y, st_z + box_height + 0.005)
    pick_pos_green = (st_x, st_y, st_z + (2 * box_height) + 0.010)

    # We apply yaw=1.57 so that PyCRAM computes a grasp pose that does NOT twist the robot wrist
    box_red = spawn_urdf(world, "red_box", red_urdf, create_pose(world, *pick_pos_red, yaw=1.57))
    box_blue = spawn_urdf(world, "blue_box", blue_urdf, create_pose(world, *pick_pos_blue, yaw=1.57))
    box_green = spawn_urdf(world, "green_box", green_urdf, create_pose(world, *pick_pos_green, yaw=1.57))

    time.sleep(1.0)

    # VizMarkerPublisher is started AFTER all objects are fully spawned at their correct
    # positions so its initial publish already contains correctly-placed markers.
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
    viz_pub = VizMarkerPublisher(world=world, node=node)

    # Create top-down GraspDescription (approach from above, gripper pointing down)
    arm = Arms.RIGHT
    manipulator = ViewManager.get_arm_view(arm, tracy).manipulator
    grasp = GraspDescription(
        approach_direction=ApproachDirection.FRONT,
        vertical_alignment=VerticalAlignment.TOP,
        manipulation_offset=0.15,  # Lift object 15cm up before moving sideways to clear stacks
        grasp_position_offset=-0.015, # Grasp 1cm lower to ensure solid grip without crowding the table
        manipulator=manipulator
    )

    print("\n[3/3] Executing PyCRAM Pick and Place pipeline...")
    
    with real_robot:
        # The middle stacking location (closer to robot)
        mid_x = 0.85
        mid_y = 0.0
        mid_z = 0.925
        
        # Maintain the same 1.57 yaw to prevent wrist twisting during placement
        place_pos_1 = create_pose(world, mid_x, mid_y, mid_z, yaw=1.57)
        place_pos_2 = create_pose(world, mid_x, mid_y, (mid_z + box_height)-0.003, yaw=1.57)
        place_pos_3 = create_pose(world, mid_x, mid_y, (mid_z + (2 * box_height))-0.003, yaw=1.57)
        
        # Safe overhead via-point to prevent arm from swiping through the stack.
        # roll=3.14 (not pitch) rotates around X by ~180deg which, combined with
        # the gripper's front_facing_orientation, points the gripper straight down.
        safe_pose = create_pose(world, 0.55, -0.15, 1.25, roll=3.14, yaw=1.57)
        
        print("\n--- Starting UNSTACK and RESTACK sequence ---")
        SequentialPlan(
            context,
            
            # --- 1. Unstack GREEN (Top) and place at bottom middle ---
            PickUpActionDescription(object_designator=box_green, arm=arm, grasp_description=grasp),
            PlaceActionDescription(object_designator=box_green, target_location=place_pos_1, arm=arm),
            
            # Move arm UP to safe position before crossing over stack again
            MoveTCPMotion(target=safe_pose, arm=arm),
            
            # --- 2. Unstack BLUE (Middle) and place in middle of new stack ---
            PickUpActionDescription(object_designator=box_blue, arm=arm, grasp_description=grasp),
            PlaceActionDescription(object_designator=box_blue, target_location=place_pos_2, arm=arm),
            
            # Move arm UP to safe position before crossing over stack again
            MoveTCPMotion(target=safe_pose, arm=arm),
            
            # --- 3. Unstack RED (Bottom) and place at top of new stack ---
            PickUpActionDescription(object_designator=box_red, arm=arm, grasp_description=grasp),
            PlaceActionDescription(object_designator=box_red, target_location=place_pos_3, arm=arm)
            
        ).perform()

    print("\n[DONE] Successfully unstacked and restacked 3 colored boxes!")
    
    input("\nPress Enter to shut down …")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
