"""
Verify smooth observations and their synchronized object beliefs.
"""

from __future__ import annotations

from math import pi
from collections.abc import Iterator
from dataclasses import dataclass
from datetime import timedelta
from time import monotonic, sleep
from uuid import uuid4

import numpy as np
import pytest
import rclpy
from py_trees.blackboard import Blackboard
from py_trees.common import Status
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

from robokudo import world as perception
from robokudo.annotators.semantic_world_connector import SemanticDigitalTwinConnector
from robokudo.cas import CAS
from robokudo.descriptors.analysis_engines.scripted_tetris_sync import (
    AnalysisEngine,
    ReceiverUnavailableError,
)
from robokudo.descriptors.scripted_tetris_observations import create_tetris_scenario
from robokudo.io.scripted_observations import (
    ModelGeometry,
    ObservationStep,
    ScriptedCamera,
    ScriptedResource,
    TetrisBlock,
    ScriptedScenario,
)
from robokudo.pipeline import Pipeline
from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
from robokudo.io.scripted_observations import ScriptedIdentifier, PlaybackTiming
from robokudo.scripts.scripted_tetris_sync import (
    SynchronizedTetrisApplication,
    ProcessRole,
)
from robokudo.synchronized_tetris import SynchronizedTetrisReceiver
from semantic_digital_twin.adapters.ros.tf_publisher import (
    TFPublisher,
    TfPublisherModelCallback,
)
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Mesh, Box
from semantic_digital_twin.world_description.world_entity import Body

# %% smooth motion


def test_intermediate_observations_translate_and_rotate_between_keyframes() -> None:
    """
    Include intermediate CAD poses along each visible motion segment.
    """
    scenario = create_tetris_scenario().with_intermediate_poses(
        motion_samples=4, pause_samples=2
    )
    translated = [
        observation
        for observation in scenario.observations
        if observation.step is ObservationStep.TRANSLATED
    ]
    rotated = [
        observation
        for observation in scenario.observations
        if observation.step is ObservationStep.ROTATED
    ]

    np.testing.assert_allclose(
        translated[1].detections[0].pose.translation, [-0.275, -0.175, 0.95]
    )
    np.testing.assert_allclose(
        Rotation.from_quat(rotated[1].detections[0].pose.rotation).as_rotvec(),
        [0.0, 0.0, pi / 4],
    )


# %% CAD belief updates


@pytest.fixture
def perception_world() -> Iterator[World]:
    """
    Isolate perception bodies and beliefs from other pipeline tests.
    """
    previous_world = perception.world_instance()
    previous_beliefs = dict(perception.get_object_belief_states())
    perception.get_object_belief_states().clear()
    world = World()
    perception.set_world(world)
    yield world
    perception.set_world(previous_world)
    perception.get_object_belief_states().clear()
    perception.get_object_belief_states().update(previous_beliefs)


def test_connector_updates_cad_pose_without_replacing_geometry(
    perception_world: World,
) -> None:
    """
    Apply camera-relative CAD poses while retaining mesh and collision shapes.
    """
    scenario = create_tetris_scenario()
    camera = ScriptedCamera()
    perception.setup_world_for_camera_frame(
        world_frame="map", camera_frame=camera.frame
    )
    observation = CAS()
    observation.world_frame = "map"
    observation.camera_frame = camera.frame
    observation.camera_to_world_transform = (
        HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0)
    )
    geometry = ModelGeometry.load(
        TetrisBlock.FIRST, ScriptedResource.MESH_DIRECTORY.resolve()
    )
    connector = SemanticDigitalTwinConnector()
    pipeline = Pipeline(children=[connector])
    pipeline.cas = observation
    first_detection = scenario.observations[0].detections[0]
    first_hypothesis = first_detection.create_hypothesis(
        geometry, camera, connector.name
    )
    belief = connector.associate_hypotheses_with_beliefs(
        [first_hypothesis], [], observation
    )[0][1]

    assert isinstance(belief.body.visual.shapes[0], Mesh)
    assert isinstance(belief.body.collision.shapes[0], Box)
    visual = belief.body.visual
    collision = belief.body.collision
    target = scenario.observations[2].detections[0]
    hypothesis = target.create_hypothesis(geometry, camera, connector.name)
    connector.associate_hypotheses_with_beliefs([hypothesis], [belief], observation)

    assert belief.body.visual is visual
    assert belief.body.collision is collision
    expected = HomogeneousTransformationMatrix.from_xyz_quaternion(
        pos_x=1.0 + target.pose.translation[0],
        pos_y=target.pose.translation[1],
        pos_z=target.pose.translation[2],
        quat_x=target.pose.rotation[0],
        quat_y=target.pose.rotation[1],
        quat_z=target.pose.rotation[2],
        quat_w=target.pose.rotation[3],
    )
    np.testing.assert_allclose(
        belief.body.parent_connection.origin.to_np(), expected.to_np(), atol=1e-12
    )


# %% receiving world


@dataclass
class ObservationClock:
    """
    Advance synthetic capture time without real-time playback waits.
    """

    seconds: float = 0.0
    """
    Elapsed time used by the observation source.
    """

    def __call__(self) -> float:
        """
        Return the configured playback time.
        """
        return self.seconds


def test_receiving_world_preserves_meshes_poses_and_identity_through_absence(
    perception_world: World,
    node: Node,
) -> None:
    """
    Replicate a complete observation cycle into an independent SDT world.
    """
    receiving_world = World()
    topic = f"/scripted_twin_test/twin_{uuid4().hex}"
    receiver = WorldSynchronizer(_world=receiving_world, node=node, topic_name=topic)
    clock = ObservationClock()
    scenario = create_tetris_scenario().with_intermediate_poses(2, 1)
    engine = AnalysisEngine(
        node=node,
        synchronization_topic=topic,
        clock=clock,
        scenario=scenario,
        timing=PlaybackTiming(frame_interval=timedelta(milliseconds=50)),
        repeat=False,
    )
    Blackboard().set(ScriptedIdentifier.OUTPUT_BUFFER, AnnotatorOutputPerPipelineMap())
    pipeline = engine.implementation()
    pipeline.setup()
    identities = None
    try:
        for observation in scenario.observations:
            pipeline.tick_once()
            deadline = monotonic() + 10.0
            while pipeline.status is Status.RUNNING and monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.001)
                pipeline.tick_once()
                sleep(0.001)
            assert pipeline.status is Status.SUCCESS
            position = engine.synchronizer.latest_published_position
            while not receiver.has_applied(position) and monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            assert receiver.has_applied(position)
            beliefs = perception.get_object_belief_states()
            if identities is None:
                identities = set(beliefs)
            assert set(beliefs) == identities
            for identifier, belief in beliefs.items():
                body = receiving_world.get_kinematic_structure_entity_by_id(identifier)
                assert body is not belief.body
                assert isinstance(body.visual.shapes[0], Mesh)
                assert isinstance(body.collision.shapes[0], Box)
                np.testing.assert_allclose(
                    body.parent_connection.origin.to_np(),
                    belief.body.parent_connection.origin.to_np(),
                    atol=1e-12,
                )
                np.testing.assert_allclose(
                    body.visual.shapes[0].mesh.bounds,
                    belief.body.visual.shapes[0].mesh.bounds,
                )
            clock.seconds += engine.timing.frame_interval.total_seconds()
    finally:
        engine.shutdown()
        receiver.close()


def test_source_rejects_its_own_subscription_as_receiver(
    perception_world: World,
    node: Node,
) -> None:
    """
    Fail before recording structure when no receiving world is listening.
    """
    publishers = list(node.publishers)
    subscriptions = list(node.subscriptions)
    engine = AnalysisEngine(
        node=node,
        synchronization_topic=f"/scripted_twin_test/twin_{uuid4().hex}",
        receiver_timeout=timedelta(milliseconds=10),
    )
    with pytest.raises(ReceiverUnavailableError):
        engine.implementation()
    assert perception_world.bodies == []
    assert (
        WorldSynchronizer.all_callbacks_of_this_type_from_world(perception_world) == []
    )
    assert list(node.publishers) == publishers
    assert list(node.subscriptions) == subscriptions
    engine.shutdown()


def test_source_rejects_an_unrelated_topic_listener(
    perception_world: World,
    node: Node,
) -> None:
    """
    A diagnostic subscription does not establish native receiving-world readiness.
    """
    topic = f"/scripted_twin_test/twin_{uuid4().hex}"
    subscription = node.create_subscription(String, topic, lambda message: None, 10)
    engine = AnalysisEngine(
        node=node,
        synchronization_topic=topic,
        receiver_timeout=timedelta(milliseconds=10),
    )
    try:
        with pytest.raises(ReceiverUnavailableError):
            engine.implementation()
        assert perception_world.bodies == []
    finally:
        engine.shutdown()
        node.destroy_subscription(subscription)


def test_receiver_shutdown_releases_callbacks_and_borrowed_node_entities(
    node: Node,
) -> None:
    """
    Closing a receiver leaves its borrowed ROS node usable and empty.
    """
    publishers = list(node.publishers)
    subscriptions = list(node.subscriptions)
    timers = list(node.timers)
    receiver = SynchronizedTetrisReceiver(
        node=node,
        synchronization_topic=f"/scripted_twin_test/twin_{uuid4().hex}",
    )
    with receiver.world.modify_world(publish_changes=False):
        receiver.world.add_body(Body(name=PrefixedName(ScriptedIdentifier.CAMERA)))
    receiver.publish_visualization()
    receiver.close()
    receiver.close()
    for callback_type in (
        WorldSynchronizer,
        TFPublisher,
        TfPublisherModelCallback,
        VizMarkerPublisher,
    ):
        assert callback_type.all_callbacks_of_this_type_from_world(receiver.world) == []
    assert list(node.publishers) == publishers
    assert list(node.subscriptions) == subscriptions
    assert list(node.timers) == timers


def test_failed_source_startup_closes_owned_ros_resources(
    perception_world: World,
) -> None:
    """
    An unavailable receiver does not leave an application node or executor alive.
    """
    application = SynchronizedTetrisApplication(
        role=ProcessRole.SOURCE,
        synchronization_topic=f"/scripted_twin_test/twin_{uuid4().hex}",
        receiver_timeout=timedelta(milliseconds=10),
    )
    with pytest.raises(ReceiverUnavailableError):
        with application:
            application.run()
    assert not application.context.ok()
    assert application.executor.get_nodes() == []
    assert application.closed


def test_receiver_publishes_initial_metric_cad_and_collision_boxes(
    perception_world: World,
    node: Node,
) -> None:
    """
    Display independently reconstructed shapes at accepted world poses.
    """
    topic = f"/scripted_twin_test/twin_{uuid4().hex}"
    receiver = SynchronizedTetrisReceiver(node=node, synchronization_topic=topic)
    scenario = ScriptedScenario(create_tetris_scenario().observations[:1])
    world_T_camera = HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, yaw=pi / 2)
    engine = AnalysisEngine(
        node=node,
        synchronization_topic=topic,
        scenario=scenario,
        world_T_camera=world_T_camera,
    )
    Blackboard().set(ScriptedIdentifier.OUTPUT_BUFFER, AnnotatorOutputPerPipelineMap())
    try:
        pipeline = engine.implementation()
        pipeline.setup()
        deadline = monotonic() + 10.0
        pipeline.tick_once()
        while pipeline.status is Status.RUNNING and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
            pipeline.tick_once()
            sleep(0.001)
        assert pipeline.status is Status.SUCCESS
        position = engine.synchronizer.latest_published_position
        while (
            not receiver.synchronizer.has_applied(position) and monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.01)
        assert receiver.synchronizer.has_applied(position)
        assert receiver.world is not perception_world
        receiver.publish_visualization()

        for detection, belief in zip(
            scenario.observations[0].detections,
            perception.get_object_belief_states().values(),
        ):
            body = receiver.world.get_kinematic_structure_entity_by_id(belief.uuid)
            geometry = ModelGeometry.load(
                detection.block, ScriptedResource.MESH_DIRECTORY.resolve()
            )
            mesh = body.visual.shapes[0]
            collision = body.collision.shapes[0]
            assert isinstance(mesh, Mesh)
            assert isinstance(collision, Box)
            lower = geometry.vertices.min(axis=0)
            upper = geometry.vertices.max(axis=0)
            np.testing.assert_allclose(mesh.mesh.bounds, [lower, upper])
            np.testing.assert_allclose(
                collision.origin.to_np()[:3, 3], (lower + upper) / 2
            )
            np.testing.assert_allclose(
                [collision.scale.x, collision.scale.y, collision.scale.z],
                upper - lower,
            )
            camera_T_model = HomogeneousTransformationMatrix.from_xyz_quaternion(
                *detection.pose.translation,
                *detection.pose.rotation,
            )
            np.testing.assert_allclose(
                body.parent_connection.origin.to_np(),
                world_T_camera.to_np() @ camera_T_model.to_np(),
                atol=1e-12,
            )
        assert TFPublisher.all_callbacks_of_this_type_from_world(perception_world) == []
        assert (
            VizMarkerPublisher.all_callbacks_of_this_type_from_world(perception_world)
            == []
        )
        assert (
            len(
                VizMarkerPublisher.all_callbacks_of_this_type_from_world(receiver.world)
            )
            == 2
        )
    finally:
        engine.shutdown()
        receiver.close()
