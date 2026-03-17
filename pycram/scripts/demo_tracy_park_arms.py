"""
demo_tracy_park_arms.py
=======================
Tracy parks both arms — visible in RViz via VizMarkerPublisher.

How to run:
  1. Terminal 1 – RViz:
       source /opt/ros/jazzy/setup.bash && rviz2
       (Fixed Frame → "map", Add → MarkerArray, Topic → /semworld/viz_marker,
        Durability: Transient Local | Add → TF)

  2. Terminal 2 / PyCharm:
       source /opt/ros/jazzy/setup.bash
       cd <repo>
       ROS_VERSION=2 .venv/bin/python pycram/scripts/demo_tracy_park_arms.py
"""

import os
import time

import rclpy

from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import StaticJointState
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.utils import tracy_installed

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import ParkArmsActionDescription

# ---------------------------------------------------------------------------
URDF_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../semantic_digital_twin/resources/urdf/tracy.urdf")
)


def main():
    if not tracy_installed():
        raise RuntimeError("Tracy URDF not found.")

    # 1. ROS node
    rclpy.init()
    node = rclpy.create_node("tracy_park_demo")

    # 2. Parse URDF → semantic world
    print("[1/3] Loading Tracy …")
    world = URDFParser.from_file(file_path=URDF_PATH).parse()
    tracy = Tracy.from_world(world)
    context = Context(world, tracy)

    # 3. Visualization: MarkerArray + TF (one call each)
    print("[2/3] Starting visualization …")
    viz = VizMarkerPublisher(world=world, node=node)
    viz.with_tf_publisher()
    time.sleep(1.0)   # give RViz a moment to connect

    # 4. Park arms
    print("[3/3] Parking arms — watch RViz …")
    with simulated_robot:
        SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()

    # 5. Print result
    print("\n✅  Done! Final joint positions:\n")
    for arm in tracy.arms:
        print(f"  {arm.name.name}")
        for connection, target in arm.get_joint_state_by_type(StaticJointState.PARK).items():
            print(f"    {connection.name.name:45s}  target={target:+.3f}  actual={connection.position:+.3f}")

    # 6. Hold 10 s so you can inspect the pose in RViz
    print("\n[RViz] Holding final pose for 10 s …")
    time.sleep(10.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
