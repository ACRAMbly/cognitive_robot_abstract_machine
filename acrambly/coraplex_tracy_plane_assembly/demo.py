"""ACRAMBLY airplane assembly with Tracy.

Run without arguments for RViz simulation. Use --real for Real Robot.
"""

from __future__ import annotations

import argparse
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from math import pi
from pathlib import Path
from typing import ClassVar

import rclpy
from rclpy.executors import MultiThreadedExecutor

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    RealRobotPlanExecutor,
)
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    MoveManipulatorAction,
    ParkArmsAction,
    SetGripperAction,
)
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import GripperState, StaticJointState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    Connection6DoF,
)
from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.semantic_annotations.semantic_annotations import GelatinBox


ASSETS_DIR = Path(__file__).resolve().parent / "assets"

XYZ = tuple[float, float, float]
XYZRPY = tuple[float, float, float, float, float, float]
PartSpec = tuple[str, str, XYZ, XYZ]


# %% Scene geometry


@dataclass
class HolderParts:
    """A fixture body and its fixed pose relative to the board."""

    body: Body
    position: XYZ
    rotation: XYZ


@dataclass
class PlaneBoard:
    """Constructs the wooden board and its bolted fixture bodies."""

    assets_dir: Path = ASSETS_DIR
    CALIBRATION_ORIGIN: ClassVar[XYZ] = (0.7, 0.0, 0.01)
    position: XYZ = CALIBRATION_ORIGIN

    HOLDER_SPECS: ClassVar[tuple[PartSpec, ...]] = (
        ("HolderBolt", "battat/BoltHolder.stl", (1.01, 0.575, 0.02), (0, 0, pi)),
        ("HolderUpperBody", "battat/HolderUpperBody.stl", (0.91, 0.5, 0.02), (0, 0, pi/2)),
        ("HolderTopWing", "battat/HolderTopWing.stl", (0.925, 0.343, 0.02), (0, 0, pi)),
        ("HolderUnderBody", "battat/HolderUnderBody.stl", (0.91, 0.2, 0.02), (0, 0, pi/2)),
        ("HolderHorizontal", "battat/Plane_horizontal_holder_tall.stl", (0.925, -0.05, 0.02), (pi/2, 0, pi/2)),
        ("HolderBottomWing", "battat/HolderBottomWing.stl", (0.925, -0.357, 0.02), (0, 0, 0)),
    )

    def mesh_body(
        self,
        mesh_file: str,
        prefix: str | None = None,
    ) -> Body:
        mesh_path = self.assets_dir / mesh_file

        if not mesh_path.exists():
            raise FileNotFoundError(mesh_path)

        mesh = Mesh(
            origin=HomogeneousTransformationMatrix(),
            filename=str(mesh_path),
        )

        shapes = ShapeCollection([mesh])

        name = (
            PrefixedName(prefix=prefix, name=mesh_path.stem)
            if prefix is not None
            else PrefixedName(name=mesh_path.stem)
        )

        return Body(
            name=name,
            collision=shapes,
            visual=shapes,
        )

    def board_floor(self) -> Body:
        return self.mesh_body("fixtures/big_wooden_plate.stl")

    def plane_parts_holder(self) -> dict[str, HolderParts]:
        bx, by, bz = self.CALIBRATION_ORIGIN
        return {
            name: HolderParts(
                body=self.mesh_body(mesh_file),
                position=(position[0] - bx, position[1] - by, position[2] - bz),
                rotation=rotation,
            )
            for name, mesh_file, position, rotation in self.HOLDER_SPECS
        }


@dataclass
class PlaneScene:
    """Owns the board, bolted holders and graspable airplane parts."""

    table_top_z: float = 0.88
    plane_holder_board: PlaneBoard = field(default_factory=PlaneBoard)

    board_floor: Body = field(init=False, repr=False)
    holders: dict[str, HolderParts] = field(init=False, default_factory=dict)
    parts: dict[str, IsGraspable] = field(init=False, default_factory=dict)

    PART_SPECS: ClassVar[tuple[PartSpec, ...]] = (
        ("UpperBody", "battat/plane/UpperBody.stl", (0.945, 0.5, 0.05), (0, 0, pi)),
        ("TopWing", "battat/plane/TopWing.stl", (0.925, 0.305, 0.1), (0, 0, -pi/2)),
        ("UnderBody", "battat/plane/UnderBody.stl", (0.91, 0.2, 0.035), (0, pi, -pi/2)),
        ("BottomWing", "battat/plane/BottomWing.stl", (0.925, -0.325, 0.1), (0, 0, pi)),
        ("RearWing", "battat/plane/RearWing.stl", (0.923, 0.028, 0.125), (0, 0, 0)),
    )

    def build_world(self) -> World:
        world = URDFParser.from_file(Tracy.get_ros_file_path()).create_world()
        tracy = Tracy.from_world(world)

        for arm in tracy.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)

        self.populate(world)
        return world

    def populate(self, world: World) -> None:
        with world.modify_world():
            self._stand_the_board(world)
            self._stand_the_holders(world)
        self._spawn_plane_parts(world)

    @staticmethod
    def _add_fixed_body(
        world: World,
        parent: Body,
        body: Body,
        name: str,
        position: XYZ,
        rotation: XYZ = (0.0, 0.0, 0.0),
    ) -> None:
        world.add_kinematic_structure_entity(body)
        world.add_connection(
            FixedConnection.create_with_dofs(
                world=world,
                parent=parent,
                child=body,
                name=PrefixedName(name=name),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=position[0], y=position[1], z=position[2],
                    roll=rotation[0], pitch=rotation[1], yaw=rotation[2],
                ),
            )
        )

    def _stand_the_board(self, world: World) -> None:
        self.board_floor = self.plane_holder_board.board_floor()
        x, y, z = self.plane_holder_board.position

        self._add_fixed_body(
            world=world,
            parent=world.root,
            body=self.board_floor,
            name="plane_base_floor_connection",
            position=(x, y, self.table_top_z + z),
        )

    def _stand_the_holders(self, world: World) -> None:
        self.holders = self.plane_holder_board.plane_parts_holder()
        for name, holder in self.holders.items():
            self._add_fixed_body(
                world=world,
                parent=self.board_floor,
                body=holder.body,
                name=f"{name}_connection",
                position=holder.position,
                rotation=holder.rotation,
            )

    def _spawn_plane_parts(self, world: World) -> None:
        bx, by, bz = self.plane_holder_board.position
        cx, cy, cz = self.plane_holder_board.CALIBRATION_ORIGIN

        dx, dy, dz = bx - cx, by - cy, bz - cz

        self.parts = {}

        with world.modify_world():
            for name, mesh_file, position, rotation in self.PART_SPECS:

                body = self.plane_holder_board.mesh_body(
                    mesh_file,
                    prefix="objects",
                )

                world.add_kinematic_structure_entity(body)

                world.add_connection(
                    Connection6DoF.create_with_dofs(
                        world=world,
                        parent=world.root,
                        child=body,
                        parent_T_connection_expression=(
                            HomogeneousTransformationMatrix.from_xyz_rpy(
                                x=position[0] + dx,
                                y=position[1] + dy,
                                z=self.table_top_z + position[2] + dz,
                                roll=rotation[0],
                                pitch=rotation[1],
                                yaw=rotation[2],
                            )
                        ),
                    )
                )

                annotation = GelatinBox(root=body)
                world.add_semantic_annotations([annotation])

                self.parts[name] = annotation


# %% Assembly plan


@dataclass
class PlaneAssemblyPlan:

    scene: PlaneScene

    def _part_pose(self, name: str, pose: XYZRPY) -> Pose:
        return Pose.from_xyz_rpy(*pose, reference_frame=self.scene.parts[name].root)

    def _holder_pose(self, name: str, pose: XYZRPY) -> Pose:
        # Fixtures are bare Body instances in PlaneScene, not annotations.
        return Pose.from_xyz_rpy(*pose, reference_frame=self.scene.holders[name].body)

    def _grasp(self, name: str, pose: XYZRPY) -> GraspCandidate:
        return GraspCandidate(self.scene.parts[name], self._part_pose(name, pose))

    def hold_bottom_wing(self, tracy: Tracy) -> list[StatechartNode]:
        """Step 1: right arm transports the wing and stays at the holder."""
        return [
            PickUpAction(
                grasp_candidate=self._grasp("BottomWing", (0.09, 0.05, 0.015, pi/2, 0, -pi/2)),
                arm=tracy.right_arm,
            ),
            MoveManipulatorAction(
                goal_pose=self._holder_pose("HolderHorizontal", (-0.1, 0.2, -0.094, pi/2, 0, pi/2)),
                arm=tracy.right_arm,
                allow_gripper_collision=False,
            ),
            MoveManipulatorAction(
                goal_pose=self._holder_pose("HolderHorizontal", (-0.105, 0.14, -0.084, pi/2, 0, pi/2)),
                arm=tracy.right_arm,
                allow_gripper_collision=False,
            ),
        ]

    def assemble_body(self, tracy: Tracy) -> list[StatechartNode]:
        """Step 2: left arm places the upper body onto the underbody."""
        return [
            PickUpAction(
                grasp_candidate=self._grasp("UpperBody", (0, 0, -0.005, 0, pi/2, 0)),
                arm=tracy.left_arm,
            ),
            PlaceAction(
                arm=tracy.left_arm,
                graspable=self.scene.parts["UpperBody"],
                goal_pose=self._part_pose("UnderBody", (0, 0.05, -0.02, pi, 0, -pi/2)),
            ),
            MoveBranch(
                branch_root=self.scene.parts["UnderBody"].root,
                new_parent=self.scene.parts["UpperBody"].root,
            ),
        ]

    def mount_body_on_wing(self, tracy: Tracy) -> list[StatechartNode]:
        """Step 3: left arm puts the assembled body on the bottom wing."""
        return [
            PickUpAction(
                grasp_candidate=self._grasp("UpperBody", (-0.1, 0, 0.06, 0, pi/2, 0)),
                arm=tracy.left_arm,
            ),
            PlaceAction(
                arm=tracy.left_arm,
                graspable=self.scene.parts["UpperBody"],
                goal_pose=self._part_pose("BottomWing", (0, -0.038, 0.025, 0, 0.1, pi/2)),
            ),
            MoveBranch(
                branch_root=self.scene.parts["BottomWing"].root,
                new_parent=self.scene.parts["UnderBody"].root,
            ),
            MoveBranch(
                branch_root=self.scene.parts["RearWing"].root,
                new_parent=self.scene.parts["UnderBody"].root,
            ),
        ]

    def mount_top_wing(self, tracy: Tracy) -> list[StatechartNode]:
        """Step 4: left arm places the top wing on the body."""
        return [
            PickUpAction(
                grasp_candidate=self._grasp("TopWing", (-0.05, -0.1, 0, pi/2, 0, 0)),
                arm=tracy.left_arm,
            ),
            PlaceAction(
                arm=tracy.left_arm,
                graspable=self.scene.parts["TopWing"],
                goal_pose=self._part_pose("UpperBody", (0.078, 0, 0.028, pi, 0, 0)),
            ),
            MoveBranch(
                branch_root=self.scene.parts["TopWing"].root,
                new_parent=self.scene.parts["UpperBody"].root,
            ),
        ]

    def release_and_retract(self, tracy: Tracy) -> list[StatechartNode]:
        """Open the right gripper only after all assembly stages finish."""
        return [
            SetGripperAction(
                gripper_configuration=tracy.right_arm.end_effector.default_configuration(
                    GripperState.OPEN
                ),
            ),
            MoveManipulatorAction(
                goal_pose=self._holder_pose("HolderHorizontal", (-0.2, 0.13, -0.1, pi/2, 0, pi/2)),
                arm=tracy.right_arm,
                allow_gripper_collision=False,
            ),
        ]

    def lift_off_partial_plane(self, tracy: Tracy) -> list[StatechartNode]:
        """Lift the whole body holding the plane together with on arm."""
        return [
            PickUpAction(
                grasp_candidate=self._grasp("UpperBody", (0.1, 0, 0, pi/2, 0, pi)),
                arm=tracy.right_arm,
            ),
            MoveManipulatorAction(
                goal_pose=self._holder_pose("HolderHorizontal", (0, 0.25, -0.2, pi/2, pi, pi/2)),
                arm=tracy.right_arm,
                allow_gripper_collision=False,
            ),
        ]

    def build(self, context: StatechartContext) -> Statechart:
        tracy = context.require_extension(extension_type=RobotAccess).robot
        actions = [
            ParkArmsAction(arms=tracy.all_arms),
            *self.hold_bottom_wing(tracy),          # works
            # *self.assemble_body(tracy),             # Precision required
            *self.mount_body_on_wing(tracy),
            # *self.mount_top_wing(tracy),
            # *self.release_and_retract(tracy),
            # *self.lift_off_partial_plane(tracy),
            # ParkArmsAction(arms=[tracy.left_arm]),
        ]

        statechart = Statechart(context=context)
        statechart.add_node(node=Sequence(nodes=actions))
        return statechart


# %% Execution


@dataclass
class DemoRunner(ABC):
    """Shares the plan execution logic between simulation and hardware."""

    scene: PlaneScene = field(default_factory=PlaneScene)

    @abstractmethod
    def create_executor(self, node):
        """Prepare a populated world and return the plan executor."""

    def run(self) -> None:
        rclpy.init()
        node = rclpy.create_node("coraplex_plane_assembly")
        executor = self.create_executor(node)

        try:
            plan = PlaneAssemblyPlan(scene=self.scene).build(executor.context)
            executor.compile(statechart=plan)
            executor.execute()
        finally:
            executor.close()


@dataclass
class SimulationDemoRunner(DemoRunner):
    def create_executor(self, node) -> KinematicallySimulatedPlanExecutor:
        world = self.scene.build_world()
        [tracy] = world.get_semantic_annotations_by_type(Tracy)

        TFPublisher(node=node, _world=world)
        VizMarkerPublisher(_world=world, node=node)
        world.notify_state_change()

        return KinematicallySimulatedPlanExecutor(
            context=StatechartContext.with_extensions(
                world=world,
                extensions=[RobotAccess(robot=tracy)],
            ),
            ros_node=node,
        )


@dataclass
class RealRobotDemoRunner(DemoRunner):
    collision_avoidance: bool = False

    def create_executor(self, node) -> RealRobotPlanExecutor:
        self.ros_executor = MultiThreadedExecutor()
        self.ros_executor.add_node(node)
        threading.Thread(target=self.ros_executor.spin, daemon=True).start()

        world = fetch_world_from_service(node=node, timeout_seconds=300)
        self.world_synchronizer = WorldSynchronizer(_world=world, node=node)
        self.scene.populate(world)

        tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]
        return RealRobotPlanExecutor(
            context=StatechartContext.with_extensions(
                world=world,
                extensions=[RobotAccess(robot=tracy)],
            ),
            ros_node=node,
            collision_avoidance=self.collision_avoidance,
        )

    def run(self) -> None:
        try:
            super().run()
        finally:
            if hasattr(self, "ros_executor"):
                self.ros_executor.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--real",
        action="store_true",
        help="Run on real Tracy.",
    )
    args = parser.parse_args()

    runner = RealRobotDemoRunner() if args.real else SimulationDemoRunner()
    runner.run()
