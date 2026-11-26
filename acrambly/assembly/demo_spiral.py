# from __future__ import annotations
#
# import rclpy
# import threading
# from dataclasses import dataclass, field
#
# from math import pi, cos, sin
#
# import numpy as np
# from scipy.spatial.transform import Rotation
#
# from coraplex.plans.context_extensions import RobotAccess
# from coraplex.plans.executors import (
#     KinematicallySimulatedPlanExecutor,
#     RealRobotPlanExecutor,
# )
#
# from coraplex.robot_plans.actions.core.pick_up import PickUpAction
# from coraplex.robot_plans.actions.core.placing import PlaceAction
# from coraplex.robot_plans.actions.base import Action
# from coraplex.robot_plans.mixins import GraspPoseSequence
# from coraplex.robot_plans.actions.core.robot_body import (
#     ParkArmsAction,
#     SetGripperAction,
#     MoveManipulatorAction,
# )
#
# from cramph.composites import Parallel, Sequence
# from cramph.context import StatechartContext
# from cramph.statechart import Statechart
# from cramph.world_modification_nodes import MoveBranch
#
# from krrood_spatial_math.spatial_types import Pose
#
# from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
# from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
#     VizMarkerPublisher,
# )
# from semantic_digital_twin.adapters.urdf import URDFParser
#
# from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
# from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
#
# from semantic_digital_twin.robots.robot_parts import AbstractRobot
# from semantic_digital_twin.robots.tracy import Tracy
#
# from semantic_digital_twin.datastructures.definitions import (
#     StaticJointState,
#     GripperState,
# )
#
# from semantic_digital_twin.adapters.ros.world_fetcher import (
#     fetch_world_from_service,
# )
# from semantic_digital_twin.adapters.ros.world_synchronizer import (
#     WorldSynchronizer,
# )
#
# from rclpy.executors import MultiThreadedExecutor
#
# from acrambly.sub_parts.shared.utils import spawn_body
#
#
# table_top_z = 0.88
#
#
# # ---------------------------------------------------------
# # SEMANTIC ANNOTATIONS
# # ---------------------------------------------------------
#
# def mark_body_graspable(world, body):
#     """
#     Add an IsGraspable semantic annotation to a Body.
#
#     GraspCandidate and PlaceAction use this annotation,
#     rather than the Body directly.
#     """
#
#     annotation = IsGraspable(
#         root=body,
#         name=body.name,
#     )
#
#     with world.modify_world():
#         world.add_semantic_annotation(annotation)
#
#     return annotation
#
#
# # ---------------------------------------------------------
# # SPAWN HOLDERS
# # ---------------------------------------------------------
#
# def spawn_holders(world):
#
#     holders = {}
#
#     spawn_body(
#         world,
#         (0.7, 0, table_top_z + 0.01),
#         (0, 0, 0),
#         "mesh",
#         mesh_filename="fixtures/big_wooden_plate.stl",
#     )
#
#     holders["HolderBolt"] = spawn_body(
#         world,
#         (1.01, 0.575, table_top_z + 0.02),
#         (pi, 0, 0),
#         "mesh",
#         mesh_filename="battat/BoltHolder.stl",
#     )
#
#     holders["HolderUpperBody"] = spawn_body(
#         world,
#         (0.91, 0.5, table_top_z + 0.02),
#         (pi / 2, 0, 0),
#         "mesh",
#         mesh_filename="battat/HolderUpperBody.stl",
#     )
#
#     holders["HolderTopWing"] = spawn_body(
#         world,
#         (0.925, 0.343, table_top_z + 0.02),
#         (pi, 0, 0),
#         "mesh",
#         mesh_filename="battat/HolderTopWing.stl",
#     )
#
#     holders["HolderUnderBody"] = spawn_body(
#         world,
#         (0.91, 0.2, table_top_z + 0.02),
#         (pi / 2, 0, 0),
#         "mesh",
#         mesh_filename="battat/HolderUnderBody.stl",
#     )
#
#     holders["HolderHorizontal"] = spawn_body(
#         world,
#         (0.925, -0.05, table_top_z + 0.02),
#         (pi / 2, 0, pi / 2),
#         "mesh",
#         mesh_filename="battat/Plane_horizontal_holder_tall.stl",
#     )
#
#     holders["HolderBottomWing"] = spawn_body(
#         world,
#         (0.925, -0.357, table_top_z + 0.02),
#         (0, 0, 0),
#         "mesh",
#         mesh_filename="battat/HolderBottomWing.stl",
#     )
#
#     return holders
#
#
# # ---------------------------------------------------------
# # SPAWN PLANE PARTS
# # ---------------------------------------------------------
#
# def spawn_plane_parts(world):
#
#     plane = {}
#     graspables = {}
#
#     plane["UpperBody"] = spawn_body(
#         world,
#         (0.945, 0.5, table_top_z + 0.05),
#         (pi, 0, 0),
#         "mesh",
#         mesh_filename="battat/plane/UpperBody.stl",
#     )
#
#     plane["TopWing"] = spawn_body(
#         world,
#         (0.925, 0.305, table_top_z + 0.1),
#         (-pi / 2, 0, 0),
#         "mesh",
#         mesh_filename="battat/plane/TopWing.stl",
#     )
#
#     plane["UnderBody"] = spawn_body(
#         world,
#         (0.91, 0.2, table_top_z + 0.035),
#         (-pi / 2, pi, 0),
#         "mesh",
#         mesh_filename="battat/plane/UnderBody.stl",
#     )
#
#     plane["BottomWing"] = spawn_body(
#         world,
#         (0.925, -0.325, table_top_z + 0.1),
#         (pi, 0, 0),
#         "mesh",
#         mesh_filename="battat/plane/BottomWing.stl",
#     )
#
#     plane["RearWing"] = spawn_body(
#         world,
#         (0.923, 0.028, table_top_z + 0.125),
#         (0, 0, 0),
#         "mesh",
#         mesh_filename="battat/plane/RearWing.stl",
#     )
#
#     # Only the parts directly manipulated by the grippers
#     # need graspability annotations.
#     graspables["BottomWing"] = mark_body_graspable(
#         world, plane["BottomWing"]
#     )
#
#     graspables["UpperBody"] = mark_body_graspable(
#         world, plane["UpperBody"]
#     )
#
#     graspables["TopWing"] = mark_body_graspable(
#         world, plane["TopWing"]
#     )
#
#     return plane, graspables
#
#
# # ---------------------------------------------------------
# # TOP-WING PEG-INSERTION SPIRAL
# # ---------------------------------------------------------
#
# # This is a Cartesian spiral of the REAL TCP pose calculated from the grasp
# # measured by PlaceAction at runtime. It does not assume a fixed TopWing-to-TCP
# # transform.
# TOP_WING_SPIRAL_RADIUS = 0.012          # 12 mm maximum search radius
# TOP_WING_SPIRAL_TURNS = 3.0
# TOP_WING_SPIRAL_WAYPOINTS = 120
# TOP_WING_SPIRAL_APPROACH_HEIGHT = 0.0 # Search 50 mm above the target
# TOP_WING_SPIRAL_DESCENT_WAYPOINTS = 20  # Separate straight-down insertion
# TOP_WING_SPIRAL_SPEED = 0.015           # Slow, observable Cartesian motion
#
#
# def _pose_from_matrix(matrix: np.ndarray, reference_frame) -> Pose:
#     """Convert a 4x4 reference-to-TCP matrix into a Pose."""
#     matrix = np.asarray(matrix, dtype=float)
#     xyz = matrix[:3, 3]
#     roll, pitch, yaw = Rotation.from_matrix(
#         matrix[:3, :3]
#     ).as_euler("xyz")
#     return Pose.from_xyz_rpy(
#         float(xyz[0]),
#         float(xyz[1]),
#         float(xyz[2]),
#         float(roll),
#         float(pitch),
#         float(yaw),
#         reference_frame=reference_frame,
#     )
#
#
# @dataclass(eq=False, repr=False, kw_only=True)
# class TopWingSpiralPlaceAction(PlaceAction):
#     """
#     Place the held TopWing using a runtime-calculated Cartesian spiral.
#
#     The spiral is inserted into PlaceAction's normal release sequence:
#       1. move to the normal retreat/pre-insertion pose;
#       2. follow the spiral down to the nominal grasp pose;
#       3. open the gripper while holding the TCP at the release pose;
#       4. detach the TopWing and retract.
#
#     PlaceAction's temporary end-effector collision allowance remains active for
#     the complete placement. General collision checking is not globally disabled.
#     """
#
#     _spiral_poses: list[Pose] = field(
#         default_factory=list, init=False, repr=False
#     )
#     _descent_poses: list[Pose] = field(
#         default_factory=list, init=False, repr=False
#     )
#
#     @property
#     def variable_poses(self) -> list[Pose]:
#         # Include PlaceAction's runtime poses, XY search, and vertical descent.
#         return [
#             *super().variable_poses,
#             *self._spiral_poses,
#             *self._descent_poses,
#         ]
#
#     def create_steps(self):
#         reference_frame = self.goal_pose.reference_frame
#
#         # Same variable poses and release geometry as the parent PlaceAction.
#         self._tool_goals = GraspPoseSequence(
#             pre_grasp=self.create_variable_pose(
#                 name="pre_grasp", reference_frame=reference_frame
#             ),
#             grasp=self.create_variable_pose(
#                 name="grasp", reference_frame=reference_frame
#             ),
#             retreat=self.create_variable_pose(
#                 name="retreat", reference_frame=reference_frame
#             ),
#         )
#
#         self._spiral_poses = [
#             self.create_variable_pose(
#                 name=f"spiral_{index:03d}",
#                 reference_frame=reference_frame,
#             )
#             for index in range(TOP_WING_SPIRAL_WAYPOINTS + 1)
#         ]
#         self._descent_poses = [
#             self.create_variable_pose(
#                 name=f"vertical_descent_{index:03d}",
#                 reference_frame=reference_frame,
#             )
#             for index in range(TOP_WING_SPIRAL_DESCENT_WAYPOINTS)
#         ]
#
#         steps = [
#             # First move clear of the receiver, as in the original PlaceAction.
#             self.tool_center_point_goal(
#                 target=self._tool_goals.retreat,
#                 arm=self.arm,
#                 cruise_linear_velocity=self.transport_cruise_linear_velocity,
#             )
#         ]
#
#         # First perform the complete XY search at a constant, raised height.
#         steps.extend(
#             self.tool_center_point_goal(
#                 target=waypoint,
#                 arm=self.arm,
#                 cruise_linear_velocity=TOP_WING_SPIRAL_SPEED,
#             )
#             for waypoint in self._spiral_poses
#         )
#
#         # Only after the XY search returns to its centre, descend vertically.
#         steps.extend(
#             self.tool_center_point_goal(
#                 target=waypoint,
#                 arm=self.arm,
#                 cruise_linear_velocity=TOP_WING_SPIRAL_SPEED,
#             )
#             for waypoint in self._descent_poses
#         )
#
#         # Preserve PlaceAction's release, detach and retract behaviour.
#         steps.append(
#             Parallel(
#                 nodes=[
#                     self.gripper_goal_type.create_gripper_goal(
#                         configuration=self.arm.end_effector.default_configuration(
#                             GripperState.OPEN,
#                             finger_velocity=self.max_release_opening_velocity,
#                         )
#                     ),
#                     self.tool_center_point_goal(
#                         target=self._tool_goals.grasp,
#                         arm=self.arm,
#                     ),
#                 ]
#             )
#         )
#         steps.extend(
#             self.reattach(
#                 branch_root=self.graspable.root,
#                 new_parent=self.world.root,
#             )
#         )
#         steps.append(
#             self.tool_center_point_goal(
#                 target=self._tool_goals.pre_grasp,
#                 arm=self.arm,
#                 cruise_linear_velocity=self.retract_cruise_linear_velocity,
#             )
#         )
#         return steps
#
#     def on_start(self) -> None:
#         # Register this action's temporary collision rules and run the base
#         # StatechartNode start logic. Do not call PlaceAction.on_start here:
#         # its public variable poses are symbolic expressions, not concrete poses
#         # that can be converted with Pose.to_np().
#         Action.on_start(self)
#
#         end_effector = self.arm.end_effector
#
#         # Reproduce PlaceAction's runtime grasp measurement. This is the actual
#         # relationship between the held TopWing and the TCP after PickUpAction.
#         grasp = GraspCandidate(
#             self.graspable,
#             end_effector.body_T_grasp(
#                 body=self.graspable.root,
#                 tool_P_grasp=end_effector.tool_P_grasp_when_closed,
#             ),
#         )
#
#         # Compute concrete tool-frame poses for this measured grasp at the
#         # requested object placement. These Pose objects have numeric values.
#         place_sequence = self.grasp_pose_sequence(
#             reference_T_grasp=grasp.moved_to(self.goal_pose),
#             end_effector=end_effector,
#             grasp_candidate=grasp,
#         )
#
#         # Bind PlaceAction's three symbolic variables exactly as its parent
#         # implementation does.
#         for variable_pose, concrete_pose in zip(
#             self._tool_goals.poses, place_sequence.poses
#         ):
#             self.set_variable_pose(
#                 variable_pose=variable_pose,
#                 value=concrete_pose,
#             )
#
#         reference_frame = self.goal_pose.reference_frame
#         nominal_tcp = np.asarray(
#             place_sequence.grasp.to_np(), dtype=float
#         ).copy()
#
#         # Generate a level XY spiral around the runtime-calculated TCP pose.
#         # The UpperBody frame is rotated by pi around X in this demo, so
#         # negative local Z moves upward in world coordinates. Every spiral
#         # waypoint therefore stays 50 mm above the nominal insertion pose.
#         for index, variable_pose in enumerate(self._spiral_poses):
#             progress = index / TOP_WING_SPIRAL_WAYPOINTS
#             angle = 2.0 * pi * TOP_WING_SPIRAL_TURNS * progress
#
#             # Bell-shaped radius: centre -> outer search -> centre.
#             radius = TOP_WING_SPIRAL_RADIUS * sin(pi * progress)
#             local_x = radius * cos(angle)
#             local_y = radius * sin(angle)
#             local_z = -TOP_WING_SPIRAL_APPROACH_HEIGHT
#
#             waypoint_matrix = nominal_tcp.copy()
#             waypoint_matrix[:3, 3] += np.array(
#                 [local_x, local_y, local_z], dtype=float
#             )
#             self.set_variable_pose(
#                 variable_pose=variable_pose,
#                 value=_pose_from_matrix(waypoint_matrix, reference_frame),
#             )
#
#         # Then generate a straight vertical descent at the spiral centre.
#         for index, variable_pose in enumerate(self._descent_poses, start=1):
#             progress = index / TOP_WING_SPIRAL_DESCENT_WAYPOINTS
#             waypoint_matrix = nominal_tcp.copy()
#             waypoint_matrix[:3, 3] += np.array(
#                 [0.0, 0.0,
#                  -TOP_WING_SPIRAL_APPROACH_HEIGHT * (1.0 - progress)],
#                 dtype=float,
#             )
#             self.set_variable_pose(
#                 variable_pose=variable_pose,
#                 value=_pose_from_matrix(waypoint_matrix, reference_frame),
#             )
#
#
# # ---------------------------------------------------------
# # BUILD MANIPULATION PLAN
# # ---------------------------------------------------------
#
# def build_plan(context, battatco, holders, graspables):
#
#     tracy = context.require_extension(
#         extension_type=RobotAccess
#     ).robot
#
#     bottom_wing = battatco["BottomWing"]
#     upper_body = battatco["UpperBody"]
#     under_body = battatco["UnderBody"]
#     top_wing = battatco["TopWing"]
#     rear_wing = battatco["RearWing"]
#
#     bottom_wing_graspable = graspables["BottomWing"]
#     upper_body_graspable = graspables["UpperBody"]
#     top_wing_graspable = graspables["TopWing"]
#
#     actions = [
#
#         # -------------------------------------------------
#         # INITIAL POSITION
#         # -------------------------------------------------
#
#         ParkArmsAction(
#             arms=tracy.all_arms
#         ),
#
#         # -------------------------------------------------
#         # STEP 1: PICK UP BOTTOM WING
#         # -------------------------------------------------
#
#         PickUpAction(
#             grasp_candidate=GraspCandidate(
#                 bottom_wing_graspable,
#                 Pose.from_xyz_rpy(
#                     0.09,
#                     0.05,
#                     0,
#                     pi / 2,
#                     0,
#                     -pi / 2,
#                     reference_frame=bottom_wing,
#                 ),
#             ),
#             arm=tracy.right_arm,
#         ),
#
#         MoveManipulatorAction(
#             goal_pose=Pose.from_xyz_rpy(
#                 -0.089,
#                 0.13,
#                 -0.094,
#                 pi / 2,
#                 0,
#                 pi / 2,
#                 reference_frame=holders["HolderHorizontal"],
#             ),
#             arm=tracy.right_arm,
#             allow_gripper_collision=False,
#         ),
#
#         # -------------------------------------------------
#         # STEP 2: PICK UP UPPER BODY
#         # -------------------------------------------------
#
#         PickUpAction(
#             grasp_candidate=GraspCandidate(
#                 upper_body_graspable,
#                 Pose.from_xyz_rpy(
#                     -0.1,
#                     0,
#                     0.045,
#                     0,
#                     pi / 2,
#                     0,
#                     reference_frame=upper_body,
#                 ),
#             ),
#             arm=tracy.left_arm,
#         ),
#
#         PlaceAction(
#             arm=tracy.left_arm,
#             graspable=upper_body_graspable,
#             goal_pose=Pose.from_xyz_rpy(
#                 0,
#                 0.05,
#                 -0.005,
#                 pi,
#                 -0.175,
#                 -pi / 2,
#                 reference_frame=under_body,
#             ),
#         ),
#
#         MoveBranch(
#             branch_root=under_body,
#             new_parent=upper_body,
#         ),
#
#         # -------------------------------------------------
#         # STEP 3: MOVE BODY TO HORIZONTAL HOLDER
#         # -------------------------------------------------
#
#         PickUpAction(
#             grasp_candidate=GraspCandidate(
#                 upper_body_graspable,
#                 Pose.from_xyz_rpy(
#                     -0.1,
#                     0,
#                     0.03,
#                     0,
#                     pi / 2,
#                     0,
#                     reference_frame=upper_body,
#                 ),
#             ),
#             arm=tracy.left_arm,
#         ),
#
#         PlaceAction(
#             arm=tracy.left_arm,
#             graspable=upper_body_graspable,
#             goal_pose=Pose.from_xyz_rpy(
#                 0,
#                 0.187,
#                 0,
#                 -pi / 2,
#                 pi,
#                 0,
#                 reference_frame=holders["HolderHorizontal"],
#             ),
#         ),
#
#         MoveBranch(
#             branch_root=bottom_wing,
#             new_parent=under_body,
#         ),
#
#         MoveBranch(
#             branch_root=rear_wing,
#             new_parent=under_body,
#         ),
#
#         # -------------------------------------------------
#         # STEP 4: PICK UP AND PLACE TOP WING
#         # -------------------------------------------------
#
#         PickUpAction(
#             grasp_candidate=GraspCandidate(
#                 top_wing_graspable,
#                 Pose.from_xyz_rpy(
#                     -0.05,
#                     -0.1,
#                     0,
#                     pi / 2,
#                     0,
#                     0,
#                     reference_frame=top_wing,
#                 ),
#             ),
#             arm=tracy.left_arm,
#         ),
#
#         # Runtime-calculated spiral placement. The action measures the actual
#         # held-object grasp when it starts, then uses that transform for every
#         # TCP waypoint and performs the release/retraction itself.
#         TopWingSpiralPlaceAction(
#             arm=tracy.left_arm,
#             graspable=top_wing_graspable,
#             goal_pose=Pose.from_xyz_rpy(
#                 0.078,
#                 0,
#                 0.028,
#                 pi,
#                 0,
#                 0,
#                 reference_frame=upper_body,
#             ),
#             position_threshold=0.001,
#             orientation_threshold=0.02,
#         ),
#
#         # -------------------------------------------------
#         # STEP 5: RETRACT RIGHT ARM
#         # -------------------------------------------------
#
#         SetGripperAction(
#             gripper_configuration=(
#                 tracy.right_arm.end_effector.default_configuration(
#                     GripperState.OPEN
#                 )
#             )
#         ),
#
#         MoveManipulatorAction(
#             goal_pose=Pose.from_xyz_rpy(
#                 -0.2,
#                 0.13,
#                 -0.1,
#                 pi / 2,
#                 0,
#                 pi / 2,
#                 reference_frame=holders["HolderHorizontal"],
#             ),
#             arm=tracy.right_arm,
#             allow_gripper_collision=False,
#         ),
#
#         ParkArmsAction(
#             arms=tracy.all_arms
#         ),
#     ]
#
#     statechart = Statechart(context=context)
#
#     statechart.add_node(
#         node=Sequence(nodes=actions)
#     )
#
#     return statechart
#
#
# # ---------------------------------------------------------
# # EXECUTION MODE
# # ---------------------------------------------------------
#
# RVIZ = True
# REAL = True
#
#
# # ---------------------------------------------------------
# # RVIZ SIMULATION
# # ---------------------------------------------------------
#
# if RVIZ:
#
#     world = URDFParser.from_file(
#         Tracy.get_ros_file_path()
#     ).create_world()
#
#     tracy = Tracy.from_world(world)
#
#     for arm in tracy.all_arms:
#         arm.get_joint_state_by_type(
#             StaticJointState.PARK
#         ).apply_to(world)
#
#     holders = spawn_holders(world)
#
#     battatco, graspables = spawn_plane_parts(world)
#
#     rclpy.init()
#
#     node = rclpy.create_node(
#         "coraplex_tracy_demo"
#     )
#
#     TFPublisher(
#         node=node,
#         _world=world,
#     )
#
#     VizMarkerPublisher(
#         _world=world,
#         node=node,
#     )
#
#     world.notify_state_change()
#
#     executor = KinematicallySimulatedPlanExecutor(
#         context=StatechartContext.with_extensions(
#             world=world,
#             extensions=[
#                 RobotAccess(robot=tracy)
#             ],
#         ),
#         ros_node=node,
#     )
#
#     executor.compile(
#         statechart=build_plan(
#             executor.context,
#             battatco,
#             holders,
#             graspables,
#         )
#     )
#
#     executor.execute()
#
#     executor.close()
#
#
# # ---------------------------------------------------------
# # REAL ROBOT EXECUTION
# # ---------------------------------------------------------
#
# elif REAL:
#
#     rclpy.init()
#
#     node = rclpy.create_node(
#         "coraplex_tracy_demo"
#     )
#
#     ros_executor = MultiThreadedExecutor()
#
#     ros_executor.add_node(node)
#
#     threading.Thread(
#         target=ros_executor.spin,
#         daemon=True,
#     ).start()
#
#     world = fetch_world_from_service(
#         node=node,
#         timeout_seconds=300,
#     )
#
#     WorldSynchronizer(
#         _world=world,
#         node=node,
#     )
#
#     holders = spawn_holders(world)
#
#     battatco, graspables = spawn_plane_parts(world)
#
#     tracy = world.get_semantic_annotations_by_type(
#         AbstractRobot
#     )[0]
#
#     executor = RealRobotPlanExecutor(
#         context=StatechartContext.with_extensions(
#             world=world,
#             extensions=[
#                 RobotAccess(robot=tracy)
#             ],
#         ),
#         ros_node=node,
#         collision_avoidance=False,
#     )
#
#     executor.compile(
#         statechart=build_plan(
#             executor.context,
#             battatco,
#             holders,
#             graspables,
#         )
#     )
#
#     executor.execute()
#
#     executor.close()


from __future__ import annotations

import rclpy
import threading
from dataclasses import dataclass, field

from math import pi, cos, sin

import numpy as np
from scipy.spatial.transform import Rotation

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    RealRobotPlanExecutor,
)

from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.mixins import GraspPoseSequence
from coraplex.robot_plans.actions.core.robot_body import (
    ParkArmsAction,
    SetGripperAction,
    MoveManipulatorAction,
)

from cramph.composites import Parallel, Sequence
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
# TOP-WING PEG-INSERTION SPIRAL
# ---------------------------------------------------------

# This is a Cartesian spiral of the REAL TCP pose calculated from the grasp
# measured by PlaceAction at runtime. It does not assume a fixed TopWing-to-TCP
# transform.
TOP_WING_SPIRAL_RADIUS = 0.012          # 12 mm maximum search radius
TOP_WING_SPIRAL_TURNS = 3.0
TOP_WING_SPIRAL_WAYPOINTS = 120
TOP_WING_SPIRAL_APPROACH_HEIGHT = 0.0 # Search 50 mm above the target
TOP_WING_SPIRAL_DESCENT_WAYPOINTS = 20  # Separate straight-down insertion
TOP_WING_SPIRAL_SPEED = 0.015           # Slow, observable Cartesian motion


def _pose_from_matrix(matrix: np.ndarray, reference_frame) -> Pose:
    """Convert a 4x4 reference-to-TCP matrix into a Pose."""
    matrix = np.asarray(matrix, dtype=float)
    xyz = matrix[:3, 3]
    roll, pitch, yaw = Rotation.from_matrix(
        matrix[:3, :3]
    ).as_euler("xyz")
    return Pose.from_xyz_rpy(
        float(xyz[0]),
        float(xyz[1]),
        float(xyz[2]),
        float(roll),
        float(pitch),
        float(yaw),
        reference_frame=reference_frame,
    )


@dataclass(eq=False, repr=False, kw_only=True)
class TopWingSpiralPlaceAction(PlaceAction):
    SPIRAL_RADIUS = TOP_WING_SPIRAL_RADIUS
    SPIRAL_TURNS = TOP_WING_SPIRAL_TURNS
    SPIRAL_WAYPOINTS = TOP_WING_SPIRAL_WAYPOINTS
    APPROACH_HEIGHT = TOP_WING_SPIRAL_APPROACH_HEIGHT
    DESCENT_WAYPOINTS = TOP_WING_SPIRAL_DESCENT_WAYPOINTS
    SPIRAL_SPEED = TOP_WING_SPIRAL_SPEED

    """
    Place the held TopWing using a runtime-calculated Cartesian spiral.

    The spiral is inserted into PlaceAction's normal release sequence:
      1. move to the normal retreat/pre-insertion pose;
      2. follow the spiral down to the nominal grasp pose;
      3. open the gripper while holding the TCP at the release pose;
      4. detach the TopWing and retract.

    PlaceAction's temporary end-effector collision allowance remains active for
    the complete placement. General collision checking is not globally disabled.
    """

    _spiral_poses: list[Pose] = field(
        default_factory=list, init=False, repr=False
    )
    _descent_poses: list[Pose] = field(
        default_factory=list, init=False, repr=False
    )

    @property
    def variable_poses(self) -> list[Pose]:
        # Include PlaceAction's runtime poses, XY search, and vertical descent.
        return [
            *super().variable_poses,
            *self._spiral_poses,
            *self._descent_poses,
        ]

    def create_steps(self):
        reference_frame = self.goal_pose.reference_frame

        # Same variable poses and release geometry as the parent PlaceAction.
        self._tool_goals = GraspPoseSequence(
            pre_grasp=self.create_variable_pose(
                name="pre_grasp", reference_frame=reference_frame
            ),
            grasp=self.create_variable_pose(
                name="grasp", reference_frame=reference_frame
            ),
            retreat=self.create_variable_pose(
                name="retreat", reference_frame=reference_frame
            ),
        )

        self._spiral_poses = [
            self.create_variable_pose(
                name=f"spiral_{index:03d}",
                reference_frame=reference_frame,
            )
            for index in range(self.SPIRAL_WAYPOINTS + 1)
        ]
        self._descent_poses = [
            self.create_variable_pose(
                name=f"vertical_descent_{index:03d}",
                reference_frame=reference_frame,
            )
            for index in range(self.DESCENT_WAYPOINTS)
        ]

        steps = [
            # First move clear of the receiver, as in the original PlaceAction.
            self.tool_center_point_goal(
                target=self._tool_goals.retreat,
                arm=self.arm,
                cruise_linear_velocity=self.transport_cruise_linear_velocity,
            )
        ]

        # First perform the complete XY search at a constant, raised height.
        steps.extend(
            self.tool_center_point_goal(
                target=waypoint,
                arm=self.arm,
                cruise_linear_velocity=self.SPIRAL_SPEED,
            )
            for waypoint in self._spiral_poses
        )

        # Only after the XY search returns to its centre, descend vertically.
        steps.extend(
            self.tool_center_point_goal(
                target=waypoint,
                arm=self.arm,
                cruise_linear_velocity=self.SPIRAL_SPEED,
            )
            for waypoint in self._descent_poses
        )

        # Preserve PlaceAction's release, detach and retract behaviour.
        steps.append(
            Parallel(
                nodes=[
                    self.gripper_goal_type.create_gripper_goal(
                        configuration=self.arm.end_effector.default_configuration(
                            GripperState.OPEN,
                            finger_velocity=self.max_release_opening_velocity,
                        )
                    ),
                    self.tool_center_point_goal(
                        target=self._tool_goals.grasp,
                        arm=self.arm,
                    ),
                ]
            )
        )
        steps.extend(
            self.reattach(
                branch_root=self.graspable.root,
                new_parent=self.world.root,
            )
        )
        steps.append(
            self.tool_center_point_goal(
                target=self._tool_goals.pre_grasp,
                arm=self.arm,
                cruise_linear_velocity=self.retract_cruise_linear_velocity,
            )
        )
        return steps

    def on_start(self) -> None:
        # Register this action's temporary collision rules and run the base
        # StatechartNode start logic. Do not call PlaceAction.on_start here:
        # its public variable poses are symbolic expressions, not concrete poses
        # that can be converted with Pose.to_np().
        Action.on_start(self)

        end_effector = self.arm.end_effector

        # Reproduce PlaceAction's runtime grasp measurement. This is the actual
        # relationship between the held TopWing and the TCP after PickUpAction.
        grasp = GraspCandidate(
            self.graspable,
            end_effector.body_T_grasp(
                body=self.graspable.root,
                tool_P_grasp=end_effector.tool_P_grasp_when_closed,
            ),
        )

        # Compute concrete tool-frame poses for this measured grasp at the
        # requested object placement. These Pose objects have numeric values.
        place_sequence = self.grasp_pose_sequence(
            reference_T_grasp=grasp.moved_to(self.goal_pose),
            end_effector=end_effector,
            grasp_candidate=grasp,
        )

        # Bind PlaceAction's three symbolic variables exactly as its parent
        # implementation does.
        for variable_pose, concrete_pose in zip(
            self._tool_goals.poses, place_sequence.poses
        ):
            self.set_variable_pose(
                variable_pose=variable_pose,
                value=concrete_pose,
            )

        reference_frame = self.goal_pose.reference_frame
        nominal_tcp = np.asarray(
            place_sequence.grasp.to_np(), dtype=float
        ).copy()

        # Generate a level XY spiral around the runtime-calculated TCP pose.
        # The UpperBody frame is rotated by pi around X in this demo, so
        # negative local Z moves upward in world coordinates. Every spiral
        # waypoint therefore stays 50 mm above the nominal insertion pose.
        for index, variable_pose in enumerate(self._spiral_poses):
            progress = index / self.SPIRAL_WAYPOINTS
            angle = 2.0 * pi * self.SPIRAL_TURNS * progress

            # Bell-shaped radius: centre -> outer search -> centre.
            radius = self.SPIRAL_RADIUS * sin(pi * progress)
            local_x = radius * cos(angle)
            local_y = radius * sin(angle)
            local_z = -self.APPROACH_HEIGHT

            waypoint_matrix = nominal_tcp.copy()
            waypoint_matrix[:3, 3] += np.array(
                [local_x, local_y, local_z], dtype=float
            )
            self.set_variable_pose(
                variable_pose=variable_pose,
                value=_pose_from_matrix(waypoint_matrix, reference_frame),
            )

        # Then generate a straight vertical descent at the spiral centre.
        for index, variable_pose in enumerate(self._descent_poses, start=1):
            progress = index / self.DESCENT_WAYPOINTS
            waypoint_matrix = nominal_tcp.copy()
            waypoint_matrix[:3, 3] += np.array(
                [0.0, 0.0,
                 -self.APPROACH_HEIGHT * (1.0 - progress)],
                dtype=float,
            )
            self.set_variable_pose(
                variable_pose=variable_pose,
                value=_pose_from_matrix(waypoint_matrix, reference_frame),
            )



@dataclass(eq=False, repr=False, kw_only=True)
class UpperBodySpiralPlaceAction(TopWingSpiralPlaceAction):
    """
    Place UpperBody onto UnderBody with a level XY search spiral, then
    a separate straight insertion along the UnderBody reference-frame Z axis.
    """

    SPIRAL_RADIUS = 0.010       # 10 mm search radius
    SPIRAL_TURNS = 3.0
    SPIRAL_WAYPOINTS = 100
    APPROACH_HEIGHT = 0.025     # 25 mm clear of the nominal insertion pose
    DESCENT_WAYPOINTS = 20
    SPIRAL_SPEED = 0.012        # Slower than transport for visible search


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

        UpperBodySpiralPlaceAction(
            arm=tracy.left_arm,
            graspable=upper_body_graspable,
            goal_pose=Pose.from_xyz_rpy(
                0.002,
                0.05,
                -0.035,
                pi,
                -0.175,
                -pi / 2,
                reference_frame=under_body,
            ),
            position_threshold=0.001,
            orientation_threshold=0.02,
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

        # Runtime-calculated spiral placement. The action measures the actual
        # held-object grasp when it starts, then uses that transform for every
        # TCP waypoint and performs the release/retraction itself.
        TopWingSpiralPlaceAction(
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
            position_threshold=0.001,
            orientation_threshold=0.02,
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