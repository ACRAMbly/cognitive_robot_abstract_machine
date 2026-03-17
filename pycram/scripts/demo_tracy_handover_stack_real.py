#!/usr/bin/env python3
"""
demo_tracy_handover_stack_real.py
==============================
Tests unstacking 3 boxes from the right side using the Right Arm, 
handing them over over to the Left Arm, and stacking them on the left side.

This script demonstrates the high-level PyCRAM pipeline:
- PickUpAction
- HandoverAction
- PlaceAction
"""

import threading
import sys
import time

import rclpy

from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy
from tf_transformations import quaternion_from_euler

from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body
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
from pycram.robot_plans.actions.core.handover import HandoverActionDescription
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
    path = f"/tmp/{color_name}_box.urdf"
    with open(path, "w") as f:
        f.write(urdf)
    return path


def main():
    rclpy.init()
    node = rclpy.create_node("pycram_handover_stack_demo")
    
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
    
    box_height = 0.06
    
    red_urdf = generate_colored_box_urdf("red", 1.0, 0.0, 0.0, box_height)
    blue_urdf = generate_colored_box_urdf("blue", 0.0, 0.0, 1.0, box_height)
    green_urdf = generate_colored_box_urdf("green", 0.0, 1.0, 0.0, box_height)
    
    # Spawning positions (Stacked perfectly on the right side)
    st_x = 0.60
    st_y = -0.30
    st_z = 0.95
    pick_pos_red =   (st_x, st_y, st_z)
    pick_pos_blue =  (st_x, st_y, st_z + box_height + 0.005)
    pick_pos_green = (st_x, st_y, st_z + (2 * box_height) + 0.010)
    
    # We apply yaw=1.57 so that PyCRAM computes a safe grasp pose
    box_red = spawn_urdf(world, "red_box", red_urdf, create_pose(world, *pick_pos_red, yaw=1.57))
    box_blue = spawn_urdf(world, "blue_box", blue_urdf, create_pose(world, *pick_pos_blue, yaw=1.57))
    box_green = spawn_urdf(world, "green_box", green_urdf, create_pose(world, *pick_pos_green, yaw=1.57))
    
    time.sleep(1.0)

    # VizMarkerPublisher is started AFTER all objects are fully spawned at their correct
    # positions so its initial publish already contains correctly-placed markers.
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
    viz_pub = VizMarkerPublisher(world=world, node=node)

    # Right Grasp Description
    arm_right = Arms.RIGHT
    manipulator_right = ViewManager.get_arm_view(arm_right, tracy).manipulator
    grasp_right = GraspDescription(
        approach_direction=ApproachDirection.FRONT,
        vertical_alignment=VerticalAlignment.TOP,
        manipulation_offset=0.20,  # Lift object 20cm up before moving sideways to clear stacks
        grasp_position_offset=-0.02, # Grasp 2cm lower to ensure solid grip
        manipulator=manipulator_right
    )

    print("\n[3/3] Executing PyCRAM Handover pipeline...")
    
    with real_robot:
        
        # Handover meeting point in center aerial space (Base orientation flat)
        handover_meeting_pose = create_pose(world, 0.70, 0.0, 1.30)
        
        # The stacking location for Left arm
        place_x = 0.65
        place_y = 0.35 # On the left side
        place_z = 0.93
        
        # Maintain yaw=1.57 for left arm to place safely without twisting wrists too
        place_pos_1 = create_pose(world, place_x, place_y, place_z, yaw=1.57)
        place_pos_2 = create_pose(world, place_x, place_y, place_z + box_height + 0.005, yaw=1.57)
        place_pos_3 = create_pose(world, place_x, place_y, place_z + (2 * box_height) + 0.010, yaw=1.57)
        

        print("\n--- Starting Handover Sequence (Green Top Box) ---")
        SequentialPlan(
            context,
            # Right picks
            PickUpActionDescription(object_designator=box_green, arm=Arms.RIGHT, grasp_description=grasp_right),
            
            # Handover to Left
            HandoverActionDescription(
                object_designator=box_green, 
                giver_arm=Arms.RIGHT, 
                meeting_pose=handover_meeting_pose
            ),
            
            # Left places
            PlaceActionDescription(object_designator=box_green, target_location=place_pos_1, arm=Arms.LEFT),
        ).perform()

        print("\n--- Starting Handover Sequence (Blue Middle Box) ---")
        SequentialPlan(
            context,
            PickUpActionDescription(object_designator=box_blue, arm=Arms.RIGHT, grasp_description=grasp_right),
            HandoverActionDescription(
                object_designator=box_blue, 
                giver_arm=Arms.RIGHT, 
                meeting_pose=handover_meeting_pose
            ),
            PlaceActionDescription(object_designator=box_blue, target_location=place_pos_2, arm=Arms.LEFT),
        ).perform()

        print("\n--- Starting Handover Sequence (Red Bottom Box) ---")
        SequentialPlan(
            context,
            PickUpActionDescription(object_designator=box_red, arm=Arms.RIGHT, grasp_description=grasp_right),
            HandoverActionDescription(
                object_designator=box_red, 
                giver_arm=Arms.RIGHT, 
                meeting_pose=handover_meeting_pose
            ),
            PlaceActionDescription(object_designator=box_red, target_location=place_pos_3, arm=Arms.LEFT),
        ).perform()

    print("\n[DONE] Successfully ran handover protocol on 3 boxes!")
    
    input("\nPress Enter to shut down …")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
