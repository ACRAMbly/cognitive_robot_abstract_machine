"""
demo_tracy_park_arms_real.py
============================
Tests the PyCRAM → Giskard connection on the **real robot** (Tracy).

What it does
------------
Executes ``ParkArmsAction(Arms.BOTH)`` through PyCRAM's ``real_robot``
execution environment, which serialises the MotionStatechart and sends it
to the running Giskard process via a ROS 2 action.

Prerequisites
-------------
Terminal 1 — Start Giskard (same command you always use):

    source /opt/ros/jazzy/setup.bash
    ros2 launch giskardpy_ros giskardpy_tracy_standalone.launch.py

Terminal 2 — Run this script:

    source /opt/ros/jazzy/setup.bash
    cd /home/zakaria/workspace/ros/src/cognitive_robot_abstract_machine
    ROS_VERSION=2 .venv/bin/python pycram/scripts/demo_tracy_park_arms_real.py

How it works (the PyCRAM → Giskard bridge)
-------------------------------------------
1.  ``rclpy.init()``  +  ``rclpy.create_node()``   →  standard ROS 2 node.
2.  ``rospy.node = node``  →  Giskard's internal rospy shim sees our node.
3.  ``GiskardWrapper(node)``  →  fetches the live world from Giskard's world
    service, sets up Model/State synchronisers.
4.  ``Context(world, tracy, ros_node=node)``  →  the ros_node is propagated
    to every MotionExecutor call.
5.  ``with real_robot:``  →  sets ``MotionExecutor.execution_type = REAL``.
6.  ``MotionExecutor._execute_for_real()``  →  ``GiskardWrapper(node).execute(msc)``
    →  sends the serialised MSC to Giskard via the ``/giskard/command`` action.

Note: the world used in the plan is the one fetched **from Giskard**, so
joint positions already match the real robot state.
"""

import threading
import sys

import rclpy

from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy

from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.robots.tracy import Tracy

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import ParkArmsActionDescription


def main():
    # ------------------------------------------------------------------
    # 1. ROS node + background spinner
    #    The spinner is mandatory: GiskardWrapper uses ROS action clients
    #    internally which need the node to spin to process callbacks.
    # ------------------------------------------------------------------
    rclpy.init()
    node = rclpy.create_node("pycram_real_park_arms")

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    # ------------------------------------------------------------------
    # 2. Wire the Giskard ROS shim to our node.
    #    GiskardWrapper reads rospy.node for all ROS calls.
    # ------------------------------------------------------------------
    rospy.node = node

    # ------------------------------------------------------------------
    # 3. Connect to Giskard and fetch the live world model.
    #    This blocks until Giskard's world service responds (≤ 30 s).
    # ------------------------------------------------------------------
    print("[1/4] Connecting to Giskard and fetching live world …")
    print("      (Make sure `giskardpy_tracy_standalone.launch.py` is running)")
    try:
        giskard = GiskardWrapper(node_handle=node)
    except Exception as e:
        print(f"\n  Could not connect to Giskard: {e}")
        print("    Is the launch file running? Check: ros2 node list | grep giskard")
        rclpy.shutdown()
        sys.exit(1)

    world = giskard.world
    print(f"  World received — {len(list(world.bodies))} bodies")

    # ------------------------------------------------------------------
    # 4. Build the semantic robot view (Tracy) from the Giskard world.
    # ------------------------------------------------------------------
    print("[2/4] Building Tracy semantic view …")
    tracy = Tracy.from_world(world)

    # Optional: visualise in RViz (requires rviz2 running)
    try:
        viz = VizMarkerPublisher(world=world, node=node)
        viz.with_tf_publisher()
        print("       Visualization active — check RViz (topic: /semworld/viz_marker)")
    except Exception:
        print("       Visualization unavailable (RViz not required for this test).")

    # ------------------------------------------------------------------
    # 5. Build the PyCRAM context (includes ros_node for real execution).
    # ------------------------------------------------------------------
    context = Context(world, tracy, ros_node=node)

    # ------------------------------------------------------------------
    # 6. Execute ParkArms on the REAL robot via Giskard.
    # ------------------------------------------------------------------
    print("[3/4] Sending ParkArms to REAL robot via Giskard …")
    print("      Watch Tracy move!\n")

    with real_robot:
        SequentialPlan(
            context,
            ParkArmsActionDescription(Arms.BOTH),
        ).perform()

    # ------------------------------------------------------------------
    # 7. Report result
    # ------------------------------------------------------------------
    print("\n[4/4]   ParkArms completed on real robot.\n")
    from semantic_digital_twin.datastructures.definitions import StaticJointState
    for arm in tracy.arms:
        print(f"  {arm.name.name}")
        for connection, target in arm.get_joint_state_by_type(StaticJointState.PARK).items():
            print(
                f"    {connection.name.name:45s}  "
                f"target={target:+.3f}  "
                f"actual={connection.position:+.3f}"
            )

    input("\nPress Enter to shut down …")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
