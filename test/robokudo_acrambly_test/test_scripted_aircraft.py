# %% Imports

from __future__ import annotations

from enum import StrEnum

import pytest
import numpy
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from .dataset.aircraft_source import AircraftObservationSequence
from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.annotators.exceptions import (
    InvalidObservationFrame,
    InvalidObservationPose,
)
from robokudo.cas import CASViews
from robokudo import world as robokudo_world
from robokudo.types.cv import ImageROI
from robokudo.types.scene import ObjectHypothesis
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.world_entity import Body

from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from semantic_digital_twin.world import World


# %% Explicit observation frames


class AircraftFrameLabel(StrEnum):
    """Frame identifiers for detached aircraft source calibration."""

    WORLD = "aircraft_world"
    """Declared world frame of the isolated source fixture."""

    CAMERA = "aircraft_camera"
    """Declared optical camera frame of the isolated source fixture."""


@pytest.fixture
def aircraft_frames() -> AircraftFrames:
    """
    Supply detached frame references without initializing a belief world.
    """
    world_body = Body(name=PrefixedName(AircraftFrameLabel.WORLD))
    camera_body = Body(name=PrefixedName(AircraftFrameLabel.CAMERA))
    return AircraftFrames(
        world_frame=str(world_body.name),
        camera_frame=str(camera_body.name),
        camera_to_world_transform=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=10.0,
            y=20.0,
            z=30.0,
            yaw=1.0,
            reference_frame=world_body,
            child_frame=camera_body,
        ),
    )


# %% Minimal observations


def test_aircraft_source_emits_configured_model_labels_and_origin_poses(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Every configured aircraft kind appears with its camera-relative origin.
    """
    engine = AircraftObservationSequence(
        frames=aircraft_frames, clock=ControlledClock()
    )
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert pipeline.status is Status.SUCCESS
    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    assert [part.model for part in engine.scenario.parts] == list(AircraftModel)
    assert [hypothesis.annotations[0].classname for hypothesis in hypotheses] == [
        part.model.value.label for part in engine.scenario.parts
    ]
    for part, hypothesis in zip(engine.scenario.parts, hypotheses, strict=True):
        assert [type(annotation) for annotation in hypothesis.annotations] == [
            Classification,
            PoseAnnotation,
        ]
        assert hypothesis.annotations[1].translation == part.pose.translation
        assert hypothesis.annotations[1].rotation == part.pose.rotation


@pytest.mark.parametrize("empty_scene", [False, True])
def test_aircraft_capture_contains_only_explicit_frame_and_capture_metadata(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    empty_scene: bool,
) -> None:
    """
    Each fresh CAS retains exact calibration without synthetic sensor evidence.
    """
    clock = ControlledClock()
    engine = AircraftObservationSequence(frames=aircraft_frames, clock=clock)
    if empty_scene:
        engine.scenario.parts.clear()
    pipeline = engine.implementation()
    pipeline.setup()
    initial_observation = pipeline.cas
    pipeline.tick_once()
    observation = pipeline.cas

    assert observation is not initial_observation
    assert set(observation.views) == {
        CASViews.CAS_ID,
        CASViews.DATA_TIMESTAMP,
        CASViews.WORLD_FRAME,
        CASViews.CAMERA_FRAME,
        CASViews.CAMERA_TO_WORLD_TRANSFORM,
    }
    assert type(observation.data_timestamp) is int
    assert observation.data_timestamp == clock.timestamp_nanoseconds
    assert observation.world_frame == aircraft_frames.world_frame
    assert observation.camera_frame == aircraft_frames.camera_frame
    numpy.testing.assert_array_equal(
        observation.camera_to_world_transform.to_np(),
        aircraft_frames.camera_to_world_transform.to_np(),
    )
    assert (
        observation.camera_to_world_transform.reference_frame
        is aircraft_frames.camera_to_world_transform.reference_frame
    )
    assert (
        observation.camera_to_world_transform.child_frame
        is aircraft_frames.camera_to_world_transform.child_frame
    )
    assert len(observation.annotations) == len(engine.scenario.parts)
    for hypothesis in observation.annotations:
        assert hypothesis.roi == ImageROI()
        assert hypothesis.points is None
        assert hypothesis.point_indices == []
        assert hypothesis.object_knowledge is None


def test_aircraft_capture_leaves_world_and_beliefs_unchanged(
    aircraft_frames: AircraftFrames,
    mini_world: World,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Captures preserve existing bodies, connections, annotations, and beliefs.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    initial_beliefs = dict(robokudo_world.get_object_belief_states())
    engine = AircraftObservationSequence(
        frames=aircraft_frames, clock=ControlledClock()
    )
    pipeline = engine.implementation()
    initial_bodies = tuple(mini_world.bodies)
    initial_connections = tuple(mini_world.connections)
    initial_annotations = tuple(mini_world.semantic_annotations)
    pipeline.setup()
    pipeline.tick_once()

    assert tuple(mini_world.bodies) == initial_bodies
    assert tuple(mini_world.connections) == initial_connections
    assert tuple(mini_world.semantic_annotations) == initial_annotations
    assert robokudo_world.get_object_belief_states() == initial_beliefs


def test_aircraft_detection_identifiers_are_reproducible_and_distinct(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Independent sequences assign deterministic identifiers to their detections.
    """
    engine = AircraftObservationSequence(
        frames=aircraft_frames, clock=ControlledClock()
    )
    identifiers = []
    for _ in range(2):
        pipeline = engine.implementation()
        pipeline.setup()
        pipeline.tick_once()
        identifiers.append([hypothesis.id for hypothesis in pipeline.cas.annotations])

    assert identifiers[0] == identifiers[1]
    assert len(set(identifiers[0])) == len(engine.scenario.parts)


# %% Invalid frame configuration


def test_aircraft_source_rejects_conflicting_transform_frames(
    aircraft_frames: AircraftFrames,
) -> None:
    """
    A transform must explicitly map between the configured frame identifiers.
    """
    aircraft_frames.camera_frame = aircraft_frames.world_frame
    engine = AircraftObservationSequence(
        frames=aircraft_frames, clock=ControlledClock()
    )

    with pytest.raises(InvalidObservationFrame):
        engine.implementation()


@pytest.mark.parametrize(
    "rotation", [[0.0, 0.0, 0.0, 0.0], [0.0, float("nan"), 0.0, 1.0]]
)
def test_aircraft_source_rejects_invalid_pose_quaternions(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    rotation: list[float],
) -> None:
    """
    Completed captures require finite nonzero pose quaternions.
    """
    engine = AircraftObservationSequence(
        frames=aircraft_frames, clock=ControlledClock()
    )
    engine.scenario.parts[0].pose.rotation = rotation
    pipeline = engine.implementation()
    pipeline.setup()

    with pytest.raises(InvalidObservationPose):
        pipeline.tick_once()
