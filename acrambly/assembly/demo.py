from __future__ import annotations

import rclpy
import threading

from math import pi

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    RealRobotPlanExecutor,
)

from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    ParkArmsAction,
    SetGripperAction,
    MoveManipulatorAction,
)

from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch

from krrood_spatial_math.spatial_types import Pose

from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser

from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable

from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy

from semantic_digital_twin.datastructures.definitions import (
    StaticJointState,
    GripperState,
)

from semantic_digital_twin.adapters.ros.world_fetcher import (
    fetch_world_from_service,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    WorldSynchronizer,
)

from rclpy.executors import MultiThreadedExecutor

from acrambly.sub_parts.shared.utils import spawn_body


table_top_z = 0.88


# ---------------------------------------------------------
# SEMANTIC ANNOTATIONS
# ---------------------------------------------------------

def mark_body_graspable(world, body):
    """
    Add an IsGraspable semantic annotation to a Body.

    GraspCandidate and PlaceAction use this annotation,
    rather than the Body directly.
    """

    annotation = IsGraspable(
        root=body,
        name=body.name,
    )

    with world.modify_world():
        world.add_semantic_annotation(annotation)

    return annotation


# ---------------------------------------------------------
# SPAWN HOLDERS
# ---------------------------------------------------------

def spawn_holders(world):

    holders = {}

    spawn_body(
        world,
        (0.7, 0, table_top_z + 0.01),
        (0, 0, 0),
        "mesh",
        mesh_filename="fixtures/big_wooden_plate.stl",
    )

    holders["HolderBolt"] = spawn_body(
        world,
        (1.01, 0.575, table_top_z + 0.02),
        (pi, 0, 0),
        "mesh",
        mesh_filename="battat/BoltHolder.stl",
    )

    holders["HolderUpperBody"] = spawn_body(
        world,
        (0.91, 0.5, table_top_z + 0.02),
        (pi / 2, 0, 0),
        "mesh",
        mesh_filename="battat/HolderUpperBody.stl",
    )

    holders["HolderTopWing"] = spawn_body(
        world,
        (0.925, 0.343, table_top_z + 0.02),
        (pi, 0, 0),
        "mesh",
        mesh_filename="battat/HolderTopWing.stl",
    )

    holders["HolderUnderBody"] = spawn_body(
        world,
        (0.91, 0.2, table_top_z + 0.02),
        (pi / 2, 0, 0),
        "mesh",
        mesh_filename="battat/HolderUnderBody.stl",
    )

    holders["HolderHorizontal"] = spawn_body(
        world,
        (0.925, -0.05, table_top_z + 0.02),
        (pi / 2, 0, pi / 2),
        "mesh",
        mesh_filename="battat/Plane_horizontal_holder_tall.stl",
    )

    holders["HolderBottomWing"] = spawn_body(
        world,
        (0.925, -0.357, table_top_z + 0.02),
        (0, 0, 0),
        "mesh",
        mesh_filename="battat/HolderBottomWing.stl",
    )

    return holders


# ---------------------------------------------------------
# SPAWN PLANE PARTS
# ---------------------------------------------------------

def spawn_plane_parts(world):

    plane = {}
    graspables = {}

    plane["UpperBody"] = spawn_body(
        world,
        (0.945, 0.5, table_top_z + 0.05),
        (pi, 0, 0),
        "mesh",
        mesh_filename="battat/plane/UpperBody.stl",
    )

    plane["TopWing"] = spawn_body(
        world,
        (0.925, 0.305, table_top_z + 0.1),
        (-pi / 2, 0, 0),
        "mesh",
        mesh_filename="battat/plane/TopWing.stl",
    )

    plane["UnderBody"] = spawn_body(
        world,
        (0.91, 0.2, table_top_z + 0.035),
        (-pi / 2, pi, 0),
        "mesh",
        mesh_filename="battat/plane/UnderBody.stl",
    )

    plane["BottomWing"] = spawn_body(
        world,
        (0.925, -0.325, table_top_z + 0.1),
        (pi, 0, 0),
        "mesh",
        mesh_filename="battat/plane/BottomWing.stl",
    )

    plane["RearWing"] = spawn_body(
        world,
        (0.923, 0.028, table_top_z + 0.125),
        (0, 0, 0),
        "mesh",
        mesh_filename="battat/plane/RearWing.stl",
    )

    # Only the parts directly manipulated by the grippers
    # need graspability annotations.
    graspables["BottomWing"] = mark_body_graspable(
        world, plane["BottomWing"]
    )

    graspables["UpperBody"] = mark_body_graspable(
        world, plane["UpperBody"]
    )

    graspables["TopWing"] = mark_body_graspable(
        world, plane["TopWing"]
    )

    return plane, graspables


# ---------------------------------------------------------
# BUILD MANIPULATION PLAN
# ---------------------------------------------------------

def build_plan(context, battatco, holders, graspables):

    tracy = context.require_extension(
        extension_type=RobotAccess
    ).robot

    bottom_wing = battatco["BottomWing"]
    upper_body = battatco["UpperBody"]
    under_body = battatco["UnderBody"]
    top_wing = battatco["TopWing"]
    rear_wing = battatco["RearWing"]

    bottom_wing_graspable = graspables["BottomWing"]
    upper_body_graspable = graspables["UpperBody"]
    top_wing_graspable = graspables["TopWing"]

    actions = [

        # -------------------------------------------------
        # INITIAL POSITION
        # -------------------------------------------------

        ParkArmsAction(
            arms=tracy.all_arms
        ),

        # -------------------------------------------------
        # STEP 1: PICK UP BOTTOM WING
        # -------------------------------------------------

        PickUpAction(
            grasp_candidate=GraspCandidate(
                bottom_wing_graspable,
                Pose.from_xyz_rpy(
                    0.09,
                    0.05,
                    0,
                    pi / 2,
                    0,
                    -pi / 2,
                    reference_frame=bottom_wing,
                ),
            ),
            arm=tracy.right_arm,
        ),

        MoveManipulatorAction(
            goal_pose=Pose.from_xyz_rpy(
                -0.089,
                0.13,
                -0.094,
                pi / 2,
                0,
                pi / 2,
                reference_frame=holders["HolderHorizontal"],
            ),
            arm=tracy.right_arm,
            allow_gripper_collision=False,
        ),

        # -------------------------------------------------
        # STEP 2: PICK UP UPPER BODY
        # -------------------------------------------------

        PickUpAction(
            grasp_candidate=GraspCandidate(
                upper_body_graspable,
                Pose.from_xyz_rpy(
                    -0.1,
                    0,
                    0.045,
                    0,
                    pi / 2,
                    0,
                    reference_frame=upper_body,
                ),
            ),
            arm=tracy.left_arm,
        ),

        PlaceAction(
            arm=tracy.left_arm,
            graspable=upper_body_graspable,
            goal_pose=Pose.from_xyz_rpy(
                0,
                0.05,
                -0.005,
                pi,
                -0.175,
                -pi / 2,
                reference_frame=under_body,
            ),
        ),

        MoveBranch(
            branch_root=under_body,
            new_parent=upper_body,
        ),

        # -------------------------------------------------
        # STEP 3: MOVE BODY TO HORIZONTAL HOLDER
        # -------------------------------------------------

        PickUpAction(
            grasp_candidate=GraspCandidate(
                upper_body_graspable,
                Pose.from_xyz_rpy(
                    -0.1,
                    0,
                    0.03,
                    0,
                    pi / 2,
                    0,
                    reference_frame=upper_body,
                ),
            ),
            arm=tracy.left_arm,
        ),

        PlaceAction(
            arm=tracy.left_arm,
            graspable=upper_body_graspable,
            goal_pose=Pose.from_xyz_rpy(
                0,
                0.187,
                0,
                -pi / 2,
                pi,
                0,
                reference_frame=holders["HolderHorizontal"],
            ),
        ),

        MoveBranch(
            branch_root=bottom_wing,
            new_parent=under_body,
        ),

        MoveBranch(
            branch_root=rear_wing,
            new_parent=under_body,
        ),

        # -------------------------------------------------
        # STEP 4: PICK UP AND PLACE TOP WING
        # -------------------------------------------------

        PickUpAction(
            grasp_candidate=GraspCandidate(
                top_wing_graspable,
                Pose.from_xyz_rpy(
                    -0.05,
                    -0.1,
                    0,
                    pi / 2,
                    0,
                    0,
                    reference_frame=top_wing,
                ),
            ),
            arm=tracy.left_arm,
        ),

        PlaceAction(
            arm=tracy.left_arm,
            graspable=top_wing_graspable,
            goal_pose=Pose.from_xyz_rpy(
                0.078,
                0,
                0.028,
                pi,
                0,
                0,
                reference_frame=upper_body,
            ),
        ),

        # -------------------------------------------------
        # STEP 5: RETRACT RIGHT ARM
        # -------------------------------------------------

        SetGripperAction(
            gripper_configuration=(
                tracy.right_arm.end_effector.default_configuration(
                    GripperState.OPEN
                )
            )
        ),

        MoveManipulatorAction(
            goal_pose=Pose.from_xyz_rpy(
                -0.2,
                0.13,
                -0.1,
                pi / 2,
                0,
                pi / 2,
                reference_frame=holders["HolderHorizontal"],
            ),
            arm=tracy.right_arm,
            allow_gripper_collision=False,
        ),

        ParkArmsAction(
            arms=tracy.all_arms
        ),
    ]

    statechart = Statechart(context=context)

    statechart.add_node(
        node=Sequence(nodes=actions)
    )

    return statechart


# ---------------------------------------------------------
# EXECUTION MODE
# ---------------------------------------------------------

RVIZ = True
REAL = True


# ---------------------------------------------------------
# RVIZ SIMULATION
# ---------------------------------------------------------

if RVIZ:

    world = URDFParser.from_file(
        Tracy.get_ros_file_path()
    ).create_world()

    tracy = Tracy.from_world(world)

    for arm in tracy.all_arms:
        arm.get_joint_state_by_type(
            StaticJointState.PARK
        ).apply_to(world)

    holders = spawn_holders(world)

    battatco, graspables = spawn_plane_parts(world)

    rclpy.init()

    node = rclpy.create_node(
        "coraplex_tracy_demo"
    )

    TFPublisher(
        node=node,
        _world=world,
    )

    VizMarkerPublisher(
        _world=world,
        node=node,
    )

    world.notify_state_change()

    executor = KinematicallySimulatedPlanExecutor(
        context=StatechartContext.with_extensions(
            world=world,
            extensions=[
                RobotAccess(robot=tracy)
            ],
        ),
        ros_node=node,
    )

    executor.compile(
        statechart=build_plan(
            executor.context,
            battatco,
            holders,
            graspables,
        )
    )

    executor.execute()

    executor.close()


# ---------------------------------------------------------
# REAL ROBOT EXECUTION
# ---------------------------------------------------------

elif REAL:

    rclpy.init()

    node = rclpy.create_node(
        "coraplex_tracy_demo"
    )

    ros_executor = MultiThreadedExecutor()

    ros_executor.add_node(node)

    threading.Thread(
        target=ros_executor.spin,
        daemon=True,
    ).start()

    world = fetch_world_from_service(
        node=node,
        timeout_seconds=300,
    )

    WorldSynchronizer(
        _world=world,
        node=node,
    )

    holders = spawn_holders(world)

    battatco, graspables = spawn_plane_parts(world)

    tracy = world.get_semantic_annotations_by_type(
        AbstractRobot
    )[0]

    executor = RealRobotPlanExecutor(
        context=StatechartContext.with_extensions(
            world=world,
            extensions=[
                RobotAccess(robot=tracy)
            ],
        ),
        ros_node=node,
        collision_avoidance=False,
    )

    executor.compile(
        statechart=build_plan(
            executor.context,
            battatco,
            holders,
            graspables,
        )
    )

    executor.execute()

    executor.close()