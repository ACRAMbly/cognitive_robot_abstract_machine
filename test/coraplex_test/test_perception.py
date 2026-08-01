"""
Coverage for the perception sources and for writing their detections into the world.

The sim and the real robot run the same plan; only the source of the detections differs.
These tests pin that seam: that the right source is chosen for an execution type, that
each source reports the same :class:`~coraplex.perception.Detection` shape, and that
applying a detection moves the annotated body.
"""

from __future__ import annotations

import threading

import numpy as np
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor

from coraplex.datastructures.enums import (
    ApproachDirection,
    Arms,
    ExecutionType,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.exceptions import AmbiguousDetection, PerceivedObjectNotInWorld
from coraplex.perception import (
    Detection,
    PerceptionInterface,
    PerceptionQuery,
    RoboKudoPerception,
    WorldPerception,
)
from coraplex.plans.factories import execute_single
from coraplex.plans.plan_node import MotionNode
from coraplex.robot_plans import MoveToolCenterPointMotion
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.geometry import BoundingBox

PERCEIVED_MILK_POSITION = (2.6, 2.2, 1.05)
"""
Where the perception sources in these tests claim to have seen the milk.

Offset from where the fixture spawns it (2.37, 2.0, 1.05) so that a body which did not
move fails the assertions.
"""


def whole_scene_region(world) -> BoundingBox:
    """
    A region large enough to contain the whole fixture apartment.
    """
    return BoundingBox(
        origin=HomogeneousTransformationMatrix(reference_frame=world.root),
        min_x=-10,
        min_y=-10,
        min_z=-10,
        max_x=10,
        max_y=10,
        max_z=10,
    )


# %% choosing a source


@pytest.mark.parametrize(
    "execution_type, expected_source",
    [
        (ExecutionType.SIMULATED, WorldPerception),
        (ExecutionType.NO_EXECUTION, WorldPerception),
        (ExecutionType.REAL, RoboKudoPerception),
    ],
)
def test_source_is_chosen_by_execution_type(execution_type, expected_source):
    """
    The plan does not change between sim and real; the execution type alone decides
    where detections come from.
    """
    source = PerceptionInterface.for_execution_type(execution_type, ros_node=None)

    assert type(source) is expected_source


# %% applying detections


def test_detection_moves_the_annotated_body_to_the_perceived_pose(
    immutable_model_world,
):
    """
    Applying a detection is what makes perception load-bearing: the body ends up where
    the source saw it, not where it was spawned.
    """
    world, view, context = immutable_model_world
    milk_body = world.get_body_by_name("milk.stl")
    perceived_pose = Pose.from_xyz_rpy(
        *PERCEIVED_MILK_POSITION, reference_frame=world.root
    )

    annotations = Detection(class_label="Milk", pose=perceived_pose).apply_to(world)

    assert annotations == world.get_semantic_annotations_by_type(Milk)
    assert [annotation.class_label for annotation in annotations] == ["Milk"]
    np.testing.assert_allclose(
        milk_body.global_pose.to_position().to_np().flatten()[:3],
        PERCEIVED_MILK_POSITION,
        atol=1e-9,
    )


def test_detection_of_an_object_the_world_does_not_hold_is_rejected(
    immutable_model_world,
):
    """
    A label with nothing behind it in the world has no body to write a pose to, so it
    must not pass silently.
    """
    world, view, context = immutable_model_world

    with pytest.raises(PerceivedObjectNotInWorld):
        Detection(
            class_label="definitely_not_an_annotation",
            pose=Pose(reference_frame=world.root),
        ).apply_to(world)


def test_detection_matching_several_bodies_is_rejected(mutable_model_world):
    """
    With the label on two different bodies there is no way to tell which one was seen,
    so the ambiguity is reported instead of guessed away.

    Uses the mutable world because adding an annotation is a model change, which the
    immutable fixture does not roll back.
    """
    world, view, context = mutable_model_world
    with world.modify_world():
        world.add_semantic_annotation(Milk(root=world.get_body_by_name("spoon.stl")))

    with pytest.raises(AmbiguousDetection):
        Detection(class_label="Milk", pose=Pose(reference_frame=world.root)).apply_to(
            world
        )


def test_several_annotations_on_one_body_are_not_ambiguous(mutable_model_world):
    """
    Two annotations describing the same body name one object, so the detection applies
    to both rather than being rejected.
    """
    world, view, context = mutable_model_world
    milk_body = world.get_body_by_name("milk.stl")
    with world.modify_world():
        world.add_semantic_annotation(Milk(root=milk_body))
    perceived_pose = Pose.from_xyz_rpy(
        *PERCEIVED_MILK_POSITION, reference_frame=world.root
    )

    annotations = Detection(class_label="Milk", pose=perceived_pose).apply_to(world)

    assert len(annotations) == 2
    assert {annotation.root for annotation in annotations} == {milk_body}
    np.testing.assert_allclose(
        milk_body.global_pose.to_position().to_np().flatten()[:3],
        PERCEIVED_MILK_POSITION,
        atol=1e-9,
    )


# %% reading detections out of the world


def test_world_perception_reports_the_pose_the_world_holds(immutable_model_world):
    """
    The simulated source stands in for a perfect sensor, so its detection must match the
    body's current pose rather than any stored or spawned value.
    """
    world, view, context = immutable_model_world
    milk_body = world.get_body_by_name("milk.stl")
    milk_body.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        *PERCEIVED_MILK_POSITION, reference_frame=world.root
    )
    query = PerceptionQuery(Milk, whole_scene_region(world), view, world)

    detections = WorldPerception().detect(query)

    assert [detection.class_label for detection in detections] == ["Milk"]
    np.testing.assert_allclose(
        detections[0].pose.to_position().to_np().flatten()[:3],
        milk_body.global_pose.to_position().to_np().flatten()[:3],
        atol=1e-9,
    )


def test_world_perception_reports_nothing_outside_the_queried_region(
    immutable_model_world,
):
    """
    The region is part of the question, so a body outside it is not an answer.
    """
    world, view, context = immutable_model_world
    empty_region = BoundingBox(
        origin=HomogeneousTransformationMatrix(reference_frame=world.root),
        min_x=-10,
        min_y=-10,
        min_z=-10,
        max_x=-9,
        max_y=-9,
        max_z=-9,
    )
    query = PerceptionQuery(Milk, empty_region, view, world)

    assert WorldPerception().detect(query) == []


# %% perception correcting a grasp


def test_detection_corrects_a_grasp_planned_before_it(immutable_model_world):
    """
    What the whole seam is for: the plan is expanded (and the grasp planned) against
    whatever pose the world happened to hold, and the detection that runs afterwards
    moves the grasp with the object.

    Without this, a wrong prior in the world silently aims the reach at empty space.
    """
    world, view, context = immutable_model_world
    milk_body = world.get_body_by_name("milk.stl")
    wrong_prior = (1.0, 1.0, 1.0)
    milk_body.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        *wrong_prior, reference_frame=world.root
    )

    plan = execute_single(
        PickUpAction(
            milk_body,
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
        ),
        context=context,
    )
    plan.notify()
    targets = [
        node.designator.target
        for node in plan.descendants
        if isinstance(node, MotionNode)
        and isinstance(node.designator, MoveToolCenterPointMotion)
    ]

    def distances_to(position) -> list[float]:
        return [
            float(
                np.linalg.norm(
                    world.transform(target, world.root)
                    .to_position()
                    .to_np()
                    .flatten()[:3]
                    - np.array(position)
                )
            )
            for target in targets
        ]

    # Planned against the wrong prior, one motion sits exactly on the believed object.
    assert min(distances_to(wrong_prior)) < 1e-9
    assert min(distances_to(PERCEIVED_MILK_POSITION)) > 1.0

    Detection(
        class_label="Milk",
        pose=Pose.from_xyz_rpy(*PERCEIVED_MILK_POSITION, reference_frame=world.root),
    ).apply_to(world)

    assert min(distances_to(PERCEIVED_MILK_POSITION)) < 1e-9


# %% reading detections off the robokudo action


class RecordedQueryServer:
    """
    Action server that answers every perception query with one canned object designator.

    Stands in for a running perception pipeline so the conversion from its message
    format into a :class:`~coraplex.perception.Detection` is exercised without one.
    """

    def __init__(self, node_name: str, action_name: str, class_label: str, position):
        from robokudo_msgs.action import Query

        self.class_label = class_label
        self.position = position
        self.received_types = []
        self.node = rclpy.create_node(node_name)
        self.server = ActionServer(self.node, Query, action_name, self.execute_callback)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self.executor.spin, daemon=True, name=node_name
        )
        self.thread.start()

    def execute_callback(self, goal_handle):
        from geometry_msgs.msg import PoseStamped
        from robokudo_msgs.action import Query
        from robokudo_msgs.msg import ObjectDesignator
        from std_msgs.msg import Header

        self.received_types.append(goal_handle.request.obj.type)
        goal_handle.succeed()

        pose_stamped = PoseStamped(header=Header(frame_id="map"))
        (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
        ) = self.position
        pose_stamped.pose.orientation.w = 1.0
        return Query.Result(
            res=[ObjectDesignator(type=self.class_label, pose=[pose_stamped])]
        )

    def stop(self):
        self.server.destroy()
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


@pytest.fixture
def robokudo_query_server(rclpy_node):
    """
    A stand-in perception pipeline serving the query action on its own node.
    """
    server = RecordedQueryServer(
        node_name="robokudo_stand_in",
        action_name="robokudo/query",
        class_label="Milk",
        position=PERCEIVED_MILK_POSITION,
    )
    yield server
    server.stop()


def test_robokudo_detection_is_named_and_placed_by_the_pipeline(
    immutable_model_world, rclpy_node, robokudo_query_server
):
    """
    The real source contributes the label and the pose; everything downstream treats it
    the same as a simulated one.
    """
    world, view, context = immutable_model_world
    query = PerceptionQuery(Milk, whole_scene_region(world), view, world)

    detections = RoboKudoPerception(ros_node=rclpy_node).detect(query)

    assert robokudo_query_server.received_types == ["milk"]
    assert [detection.class_label for detection in detections] == ["Milk"]
    np.testing.assert_allclose(
        detections[0].pose.to_position().to_np().flatten()[:3],
        PERCEIVED_MILK_POSITION,
        atol=1e-9,
    )


def test_robokudo_detection_moves_the_body_in_the_world(
    immutable_model_world, rclpy_node, robokudo_query_server
):
    """
    End to end for the real path: what the pipeline reports is what the world ends up
    holding, which is what the grasp is later planned against.
    """
    world, view, context = immutable_model_world
    query = PerceptionQuery(Milk, whole_scene_region(world), view, world)

    for detection in RoboKudoPerception(ros_node=rclpy_node).detect(query):
        detection.apply_to(world)

    np.testing.assert_allclose(
        world.get_body_by_name("milk.stl")
        .global_pose.to_position()
        .to_np()
        .flatten()[:3],
        PERCEIVED_MILK_POSITION,
        atol=1e-9,
    )
