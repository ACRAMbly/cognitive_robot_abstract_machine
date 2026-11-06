import os
from abc import ABC
from dataclasses import field, dataclass
from pathlib import Path

import rclpy

from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import KinematicallySimulatedPlanExecutor
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from robokudo.types import scene
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.definitions import StaticJointState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")

class HolderParts:
    body: Body
    position: tuple
    rotation: tuple

@dataclass
class PlaneBoard:
    def board_floor(self):
        return self.spawn_body("fixtures/big_wooden_plate.stl")

    def spawn_body(self, mesh_file):
        mesh_path = Path(ASSETS_DIR, mesh_file)
        return Body(
            name=PrefixedName(name=mesh_path.stem),
            collision=ShapeCollection([Mesh(origin=HomogeneousTransformationMatrix(), filename=str(mesh_path))]),
            visual=ShapeCollection([Mesh(origin=HomogeneousTransformationMatrix(), filename=str(mesh_path))])
        )

    def plane_parts_holder(self):
        holders = {}
        holders["HolderBolt"] = HolderParts(body=self.spawn_body("battat/BoltHolder.stl"))
        holders["HolderUpperBody"] = self.spawn_body("battat/HolderUpperBody.stl")
        holders["HolderTopWing"] = self.spawn_body("battat/HolderTopWing.stl")
        holders["HolderUnderBody"] = self.spawn_body("battat/HolderUnderBody.stl")
        holders["HolderHorizontal"] = self.spawn_body("battat/Plane_horizontal_holder_tall.stl")
        holders["HolderBottomWing"] = self.spawn_body("battat/HolderBottomWing.stl")

        return holders

@dataclass
class PlaneScene:
    table_top_z: float = 0.88

    plane_holder_board: PlaneBoard = field(default_factory=PlaneBoard)

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

    def _stand_the_board(self, world: World) -> None:
        self.board_floor = self.plane_holder_board.board_floor()
        world.add_kinematic_structure_entity(self.board_floor)
        world.add_connection(
            FixedConnection.create_with_dofs(
                world=world,
                parent=world.root,
                child=self.board_floor,
                name=PrefixedName("plane_base_floor_connection"),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=0.7,
                    z=self.table_top_z+0.01
                )
            )
        )

        self.holders = {}

@dataclass
class PlaneAssemblyPlan:
    scene: PlaneScene = field(default_factory=PlaneScene)

    def build(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(RobotAccess).robot
        actions = [ParkArmsAction(arms=robot.all_arms)]

        statechart = Statechart(context=context)
        statechart.add_node(Sequence(nodes=actions))
        return statechart

@dataclass
class DemoRunner(ABC):
    scene: PlaneScene = field(default_factory=PlaneScene)

    def run(self) -> None:
        world = self.scene.build_world()
        [tracy] = world.get_semantic_annotations_by_type(Tracy)

        rclpy.init()
        node = rclpy.create_node("coraplex_plan_assembly")
        TFPublisher(node=node, _world=world)
        VizMarkerPublisher(_world=world, node=node)

        world.notify_state_change()

        executor = KinematicallySimulatedPlanExecutor(
            StatechartContext.with_extensions(world, [RobotAccess(tracy)]),
            ros_node=node
        )

        executor.compile(PlaneAssemblyPlan(scene=self.scene).build(executor.context))
        executor.execute()
        executor.close()

if __name__ == "__main__":
    runner = DemoRunner()
    runner.run()