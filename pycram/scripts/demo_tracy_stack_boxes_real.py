#!/usr/bin/env python3
"""
demo_tracy_stack_boxes_real.py
==============================
Tests stacking 3 boxes using PyCRAM Pick and Place Actions on the real robot.

This script demonstrates the high-level PyCRAM pipeline:
- PickUpAction
- PlaceAction
"""

import threading
import sys
import time

import rclpy

from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy
from tf_transformations import quaternion_from_euler

from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
import semantic_digital_twin.spatial_types.spatial_types as cas

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.datastructures.pose import PoseStamped
from pycram.robot_plans.actions.core.pick_up import PickUpActionDescription
from pycram.robot_plans.actions.core.placing import PlaceActionDescription
from pycram.view_manager import ViewManager
# Register real-robot gripper alternative for Tracy (calls Robotiq action servers
# instead of a Giskard joint task when inside `with real_robot:`).
import pycram.alternative_motion_mappings.tracy_motion_mapping  # noqa: F401


def create_pose(world, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    p = PoseStamped()
    # PyCRAM requires frame_id to be a Body, not a string
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


from semantic_digital_twin.adapters.urdf import URDFParser

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
    
    # 1. Parse the standalone URDF into a separate World
    parser = URDFParser.from_file(filepath, prefix=name)
    urdf_world = parser.parse()
    
    root_name = urdf_world.root.name
    other_root = urdf_world.root

    # 2. Merge it into the main live Giskard world
    # We expand `merge_world_at_pose` manually to insert a sleep.
    # This avoids a race condition in ROS where the WorldStateUpdate
    # beats the ModificationBlock to the Giskard node causing a KeyError.
    from semantic_digital_twin.world_description.connections import Connection6DoF
    with world.modify_world():
        root_connection = Connection6DoF.create_with_dofs(
            parent=world.root, child=other_root, world=world
        )
        world.merge_world(urdf_world, root_connection)
        
    time.sleep(0.5) # Wait for ModificationBlock to propagate to Giskard
    root_connection.origin = parent_T_pose # Trigger WorldStateUpdate safely
    
    # 3. Retrieve the parsed Body from the main world
    # URDFParser creates the root link under the prefix (e.g. 'box_A')
    # We retrieve the KinematicStructureEntity for the root
    obj_body = world.get_kinematic_structure_entity_by_name(root_name)
    print(f"Spawned URDF {name} at {p}")
    return obj_body

def generate_colored_box_urdf(color_name, r, g, b, size):
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
        <box size="{size} {size} {size}"/>
      </geometry>
      <material name="{color_name}_mat">
        <color rgba="{r} {g} {b} 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="{size} {size} {size}"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
    import os
    path = f"/tmp/{color_name}_box.urdf"
    with open(path, "w") as f:
        f.write(urdf)
    return path


def main():
    rclpy.init()
    node = rclpy.create_node("pycram_stacking_demo")
    
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
    
    print("\n[2/3] Setting up scene (Spawning 3 boxes on the right) and RViz...")
    
    # We will use 0.06m (6cm) cubes for stacking
    box_height = 0.06
    
    # Generate distinct colored URDF files in /tmp/
    red_urdf = generate_colored_box_urdf("red", 1.0, 0.0, 0.0, box_height)
    blue_urdf = generate_colored_box_urdf("blue", 0.0, 0.0, 1.0, box_height)
    green_urdf = generate_colored_box_urdf("green", 0.0, 1.0, 0.0, box_height)
    
    # Spawning positions (on the right side, each at different x for easy individual picking)
    pick_pos_red = (0.6, -0.4, 0.95)
    pick_pos_blue = (0.75, -0.4, 0.95)
    pick_pos_green = (0.9, -0.4, 0.95)
    
    box_red = spawn_urdf(world, "red_box", red_urdf, create_pose(world, *pick_pos_red))
    box_blue = spawn_urdf(world, "blue_box", blue_urdf, create_pose(world, *pick_pos_blue))
    box_green = spawn_urdf(world, "green_box", green_urdf, create_pose(world, *pick_pos_green))
    
    time.sleep(1.0) # Let world sync

    # VizMarkerPublisher is started AFTER all objects are fully spawned at their correct
    # positions so its initial publish already contains correctly-placed markers.
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
    viz_pub = VizMarkerPublisher(world=world, node=node)


    # Create top-down GraspDescription (approach from above, gripper pointing down)
    arm = Arms.RIGHT
    manipulator = ViewManager.get_arm_view(arm, tracy).manipulator
    # TOP approach with gripper pointing down is common.
    # We specify VerticalAlignment.TOP to align the gripper Z axis pointing down.
    grasp = GraspDescription(
        approach_direction=ApproachDirection.FRONT,
        vertical_alignment=VerticalAlignment.TOP,
        manipulation_offset=0.20,  # Lift object 20cm up before moving sideways to clear stacks
        grasp_position_offset=-0.02, # Grasp 2cm lower to ensure solid grip
        manipulator=manipulator
    )

    print("\n[3/3] Executing PyCRAM Pick and Place pipeline...")
    
    with real_robot:
        # The middle stacking location
        middle_pos = (0.6, 0.0, 0.93)
        place_pos_blue = create_pose(world, middle_pos[0], middle_pos[1], middle_pos[2] + box_height + 0.005)
        place_pos_green = create_pose(world, middle_pos[0], middle_pos[1], middle_pos[2] + (2 * box_height) + 0.010)
        
        print("\n--- Starting stacked pick and place sequence ---")
        SequentialPlan(
            context,
            # --- 1. Right arm picks RED box ---
            PickUpActionDescription(object_designator=box_red, arm=arm, grasp_description=grasp),
            
            # --- 2. Right arm places RED box in the middle ---
            PlaceActionDescription(object_designator=box_red, target_location=create_pose(world, *middle_pos), arm=arm),
            
            # --- 3. Right arm picks BLUE box ---
            PickUpActionDescription(object_designator=box_blue, arm=arm, grasp_description=grasp),
            
            # --- 4. Right arm places BLUE box on RED box ---
            PlaceActionDescription(object_designator=box_blue, target_location=place_pos_blue, arm=arm),
            
            # --- 5. Right arm picks GREEN box ---
            PickUpActionDescription(object_designator=box_green, arm=arm, grasp_description=grasp),
            
            # --- 6. Right arm places GREEN box on BLUE box ---
            PlaceActionDescription(object_designator=box_green, target_location=place_pos_green, arm=arm)
        ).perform()

    print("\n[DONE] 3 colored boxes stacked successfully using Pick and Place pipeline!")
    
    input("\nPress Enter to shut down …")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
