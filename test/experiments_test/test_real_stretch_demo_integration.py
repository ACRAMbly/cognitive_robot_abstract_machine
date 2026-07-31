"""
Coverage for the machinery the real-Stretch demo only exercises on the robot.

The demo's simulated run builds its world from the URDF and drives the simulated motion
mappings, so the fetch-from-service path and the real mappings are otherwise untested
until the robot is in front of you.
"""

import numpy as np
import pytest

from coraplex.alternative_motion_mappings.stretch_motion_mapping import (
    StretchMoveReal,
    StretchMoveSim,
    StretchMoveToolCenterPoint,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import Arms
from coraplex.execution_environment import real_robot, simulated_robot
from coraplex.plans.factories import execute_single
from coraplex.robot_plans import MoveMotion, MoveToolCenterPointMotion
from experiments.real_stretch_apartment_demo import demo
from giskardpy.middleware.ros2 import rospy
from giskardpy.middleware.ros2.utils.utils_for_tests import StretchTester
from giskardpy.motion_statechart.goals.cartesian_goals import DifferentialDriveBaseGoal
from semantic_digital_twin.adapters.ros.world_fetcher import (
    FetchWorldServer,
    fetch_world_from_service,
)
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Shelf,
    ShelfLayer,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import Scale

SHELF_SCALE = Scale(0.305, 0.85, 1.9)
"""
Shelf dimensions, matching the ones the demo spawns.
"""


def stretch_context(world, alternative_motion_mappings) -> Context:
    """
    Build a plan context around the Stretch in ``world``, as the demo does.
    """
    return Context(
        world=world,
        robot=world.get_semantic_annotations_by_type(Stretch)[0],
        evaluate_conditions=False,
        alternative_motion_mappings=alternative_motion_mappings,
    )


# %% world acquisition


def test_fetched_stretch_world_carries_the_robot_the_demo_needs(
    rclpy_node, stretch_world_copy
):
    """
    On the robot the world arrives over the fetch service rather than from the URDF, so
    the annotation and the joint states the plan drives must survive that trip.
    """
    served_stretch = stretch_world_copy.get_semantic_annotations_by_type(Stretch)[0]
    FetchWorldServer(node=rclpy_node, world=stretch_world_copy)

    fetched_world = fetch_world_from_service(rclpy_node, timeout_seconds=30)

    fetched_stretch = fetched_world.get_semantic_annotations_by_type(Stretch)[0]
    assert fetched_stretch.root.name.name == served_stretch.root.name.name
    assert [state.name for state in fetched_stretch.get_torso().joint_states] == [
        state.name for state in served_stretch.get_torso().joint_states
    ]


def test_apartment_furniture_spawns_into_a_fetched_world(
    rclpy_node, stretch_world_copy
):
    """
    The demo spawns its furniture into whatever world it was handed.

    A fetched world is replayed from modification blocks rather than parsed, so spawning
    has to work there exactly as it does in the URDF-built world.
    """
    FetchWorldServer(node=rclpy_node, world=stretch_world_copy)
    fetched_world = fetch_world_from_service(rclpy_node, timeout_seconds=30)

    with fetched_world.modify_world():
        shelf = Shelf.get_specification(
            "shelf",
            Shelf.get_default_root_specification(
                scale=SHELF_SCALE, wall_thickness=0.035
            ),
        ).spawn(
            fetched_world,
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                1, 0, SHELF_SCALE.z / 2, reference_frame=fetched_world.root
            ),
        )
        shelf.add(
            ShelfLayer.create_with_new_body_in_world(
                world=fetched_world,
                name="shelf_layer1",
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    1, 0, 0.283, reference_frame=fetched_world.root
                ),
                scale=Scale(0.305, 0.85, 0.018),
            )
        )

    assert fetched_world.is_kinematic_structure_entity_in_world_by_name("shelf_layer1")
    assert [layer.root.name.name for layer in shelf.shelf_layers] == ["shelf_layer1"]


# %% real execution path


def test_real_execution_selects_the_real_base_motion(stretch_world_copy):
    """
    Driving the real base goes through :class:`StretchMoveReal`; without the execution
    type being honoured the simulated mapping would silently be used instead.
    """
    context = stretch_context(stretch_world_copy, [StretchMoveSim, StretchMoveReal])
    motion = MoveMotion(
        Pose.from_xyz_rpy(1, 1, 0, reference_frame=stretch_world_copy.root)
    )
    execute_single(motion, context=context)

    with real_robot:
        assert motion.get_alternative_motion() is StretchMoveReal
        assert isinstance(motion.motion_chart, DifferentialDriveBaseGoal)

    with simulated_robot:
        assert motion.get_alternative_motion() is StretchMoveSim


def test_real_tool_center_point_motion_builds_against_the_world(stretch_world_copy):
    """
    The real tool-center-point motion resolves the arm and both of its stages against
    the world, so a mismatch between mapping and robot surfaces here rather than mid-
    motion on the robot.
    """
    context = stretch_context(stretch_world_copy, [StretchMoveToolCenterPoint])
    motion = MoveToolCenterPointMotion(
        target=Pose.from_xyz_rpy(
            0.5, 0.5, 0.8, reference_frame=stretch_world_copy.root
        ),
        arm=Arms.LEFT,
    )
    execute_single(motion, context=context)

    with real_robot:
        chart = motion.motion_chart

    assert len(chart.nodes) == 2
    assert chart.nodes[1].minimum_success == 1


# %% the whole demo against a standalone controller

PLACEMENT_TOLERANCE = 0.2
"""
How far from the bedside table's centre the cereal box may land.

The measured placement error is about 0.1m, so this discriminates a successful place
from one that missed the furniture without being tight enough to chase controller noise.
"""


@pytest.fixture()
def standalone_stretch_controller():
    """
    A standalone Giskard serving and synchronizing a Stretch world, standing in for the
    robot's own controller.
    """
    rospy.init_node("giskard")
    tester = StretchTester()
    try:
        yield tester
    finally:
        tester.print_stats()
        rospy.shutdown()


def test_demo_runs_against_a_standalone_controller(
    standalone_stretch_controller, monkeypatch
):
    """
    The whole demo, on the path it takes on the robot: it fetches the world from the
    running controller, spawns its furniture into it, and executes every action through
    the controller's action interface.

    Only the hardware is missing, so this covers the wiring between demo, world server
    and controller that the simulated run never touches.
    """
    monkeypatch.setenv("STRETCH_DEMO_EXECUTION", "REAL")

    demo.main()

    world = standalone_stretch_controller.api.world
    cereal = world.get_body_by_name("cheeze_it.obj")
    bedside_table = world.get_body_by_name("bedside_table.dae")
    assert cereal.parent_connection.parent is not world.get_body_by_name("shelf_layer2")
    np.testing.assert_allclose(
        world.compute_forward_kinematics(world.root, cereal).to_position().to_np()[:2],
        world.compute_forward_kinematics(world.root, bedside_table)
        .to_position()
        .to_np()[:2],
        atol=PLACEMENT_TOLERANCE,
    )
