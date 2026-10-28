# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import numpy
import pytest
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from robokudo.types.tf import Pose
from robokudo_acrambly.aircraft_observations.scenario import AircraftLayout
from robokudo_acrambly.playback.exceptions import (
    InvalidAbsenceInterval,
    InvalidPlaybackTiming,
    InvalidPoseKeyframes,
)
from robokudo_acrambly.playback.motion import (
    AbsenceInterval,
    MotionBoundary,
    PoseKeyframe,
    VisibilityBoundary,
)
from robokudo_acrambly.playback.schedule import PlaybackTiming

from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock
from .test_scripted_tetris_motion import InvalidKeyframeExample, ShortestRotationExample

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.cas import CAS
    from robokudo.pipeline import Pipeline
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames


# %% Waiting captures


def test_early_aircraft_tick_retains_the_completed_capture(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    An early tick waits without replacing the preceding completed CAS.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    completed_observation = pipeline.cas

    pipeline.tick_once()

    assert pipeline.status is Status.RUNNING
    assert pipeline.cas is completed_observation


def test_aircraft_capture_completes_exactly_at_the_next_due_time(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    A waiting capture becomes due on its precise nanosecond boundary.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    previous_observation = pipeline.cas
    clock.timestamp_nanoseconds += engine.scenario.timing.sample_period_nanoseconds - 1
    pipeline.tick_once()

    assert pipeline.status is Status.RUNNING
    assert pipeline.cas is previous_observation
    clock.timestamp_nanoseconds += 1
    pipeline.tick_once()
    assert pipeline.status is Status.SUCCESS
    assert pipeline.cas is not previous_observation
    assert pipeline.cas.data_timestamp == clock.timestamp_nanoseconds


# %% Complete captures


class AircraftSequenceExpectation(IntEnum):
    """
    Sample count required by the approved default aircraft sequence.
    """

    CAPTURE_COUNT = 301


@dataclass
class AircraftCaptures:
    """
    Completed aircraft observations indexed by scheduled capture offset.
    """

    engine: AnalysisEngine
    """
    Public engine supplying the consumed scenario definition.
    """

    pipeline: Pipeline
    """
    Pipeline retaining the most recently completed observation.
    """

    observations: dict[int, CAS]
    """
    Independent CAS captures indexed by elapsed nanoseconds.
    """


@pytest.fixture
def aircraft_captures(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> AircraftCaptures:
    """
    Collect the default observation sequence without real-time sleeping.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    timing = engine.scenario.timing
    sequence_start = clock.timestamp_nanoseconds
    observations = {}
    for offset in range(
        0, timing.duration_nanoseconds + 1, timing.sample_period_nanoseconds
    ):
        clock.timestamp_nanoseconds = sequence_start + offset
        pipeline.tick_once()
        observations[offset] = pipeline.cas
    return AircraftCaptures(engine, pipeline, observations)


def test_aircraft_sequence_emits_both_endpoints_before_stable_exhaustion(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Finite playback emits every scheduled capture exactly once and then stops.
    """
    timing = aircraft_captures.engine.scenario.timing
    observations = list(aircraft_captures.observations.values())
    initial_timestamp = observations[0].data_timestamp

    assert [observation.data_timestamp for observation in observations] == list(
        range(
            initial_timestamp,
            initial_timestamp + timing.duration_nanoseconds + 1,
            timing.sample_period_nanoseconds,
        )
    )
    assert len({id(observation) for observation in observations}) == len(observations)
    assert aircraft_captures.pipeline.status is Status.SUCCESS
    for _ in range(2):
        aircraft_captures.pipeline.tick_once()
        assert aircraft_captures.pipeline.status is Status.FAILURE
        assert aircraft_captures.pipeline.cas is observations[-1]


def test_aircraft_visibility_follows_configured_half_open_absence_intervals(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Partial absence, empty captures, and reappearance preserve exact boundaries.
    """
    parts = aircraft_captures.engine.scenario.parts
    for offset, observation in aircraft_captures.observations.items():
        expected_labels = [
            part.model.value.label
            for part in parts
            if not any(
                interval.start_nanoseconds <= offset < interval.end_nanoseconds
                for interval in part.absence_intervals
            )
        ]
        assert [
            hypothesis.annotations[0].classname
            for hypothesis in observation.annotations
        ] == expected_labels


def test_default_aircraft_sequence_reaches_every_presence_stage(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    The default sequence contains every part, partial absence, and an empty scene.
    """
    parts = aircraft_captures.engine.scenario.parts
    absent_models = AircraftLayout().partial_absence_models
    all_labels = [part.model.value.label for part in parts]
    expected_stages = {
        0: all_labels,
        MotionBoundary.TRANSLATION_FINISHED: all_labels,
        MotionBoundary.ROTATION_FINISHED: all_labels,
        VisibilityBoundary.PARTIAL_ABSENCE_START: [
            part.model.value.label for part in parts if part.model not in absent_models
        ],
        VisibilityBoundary.TOTAL_ABSENCE_START: [],
        VisibilityBoundary.REAPPEARANCE: all_labels,
        aircraft_captures.engine.scenario.timing.duration_nanoseconds: all_labels,
    }

    assert (
        len(aircraft_captures.observations) == AircraftSequenceExpectation.CAPTURE_COUNT
    )
    for offset, labels in expected_stages.items():
        assert [
            hypothesis.annotations[0].classname
            for hypothesis in aircraft_captures.observations[offset].annotations
        ] == labels


# %% Motion and holds


def test_aircraft_translation_reaches_each_configured_midpoint(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Every independent part translates linearly between its configured endpoints.
    """
    parts = aircraft_captures.engine.scenario.parts
    movement_start, movement_end = parts[0].keyframes[:2]
    midpoint = (
        movement_start.elapsed_nanoseconds + movement_end.elapsed_nanoseconds
    ) // 2
    observation = aircraft_captures.observations[midpoint]
    for part, hypothesis in zip(parts, observation.annotations, strict=True):
        first_pose, final_pose = [keyframe.pose for keyframe in part.keyframes[:2]]
        observed_pose = hypothesis.annotations[1]
        assert observed_pose.translation == pytest.approx(
            (numpy.asarray(first_pose.translation) + final_pose.translation) / 2.0
        )
        assert observed_pose.rotation == first_pose.rotation


def test_aircraft_rotation_reaches_each_configured_midpoint(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    The default positive camera-Z turn reaches half its angle at the midpoint.
    """
    parts = aircraft_captures.engine.scenario.parts
    rotation_start, rotation_end = parts[0].keyframes[2:]
    midpoint = (
        rotation_start.elapsed_nanoseconds + rotation_end.elapsed_nanoseconds
    ) // 2
    for part, hypothesis in zip(
        parts, aircraft_captures.observations[midpoint].annotations, strict=True
    ):
        observed_pose = hypothesis.annotations[1]
        start_pose, final_pose = [keyframe.pose for keyframe in part.keyframes[2:]]
        expected_rotation = Rotation.from_rotvec(
            Rotation.from_quat(final_pose.rotation).as_rotvec() / 2.0
        )
        assert observed_pose.translation == start_pose.translation
        numpy.testing.assert_allclose(
            Rotation.from_quat(observed_pose.rotation).as_matrix(),
            expected_rotation.as_matrix(),
        )


def test_aircraft_keyframes_are_emitted_with_exact_endpoint_values(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Visible keyframe captures retain the configured translations and quaternions.
    """
    for index, part in enumerate(aircraft_captures.engine.scenario.parts):
        for keyframe in part.keyframes:
            observed_pose = (
                aircraft_captures.observations[keyframe.elapsed_nanoseconds]
                .annotations[index]
                .annotations[1]
            )
            assert observed_pose.translation == keyframe.pose.translation
            assert observed_pose.rotation == keyframe.pose.rotation


@pytest.mark.parametrize("keyframe_index", [None, 1, 3])
def test_aircraft_holds_emit_fresh_detections_at_unchanged_poses(
    aircraft_captures: AircraftCaptures, keyframe_index: int | None
) -> None:
    """
    Initial, translated, and rotated holds continue independent observations.
    """
    timing = aircraft_captures.engine.scenario.timing
    parts = aircraft_captures.engine.scenario.parts
    hold_start = (
        0
        if keyframe_index is None
        else parts[0].keyframes[keyframe_index].elapsed_nanoseconds
    )
    first_capture = aircraft_captures.observations[hold_start]
    second_capture = aircraft_captures.observations[
        hold_start + timing.sample_period_nanoseconds
    ]
    for part, first, second in zip(
        parts, first_capture.annotations, second_capture.annotations, strict=True
    ):
        expected_pose = (
            part.pose if keyframe_index is None else part.keyframes[keyframe_index].pose
        )
        assert first.id != second.id
        assert first.annotations[1].translation == expected_pose.translation
        assert second.annotations[1].translation == expected_pose.translation
        assert first.annotations[1].rotation == expected_pose.rotation
        assert second.annotations[1].rotation == expected_pose.rotation


@pytest.mark.parametrize("quaternion_sign", [1.0, -1.0])
def test_aircraft_rotation_uses_the_short_path_across_quaternion_signs(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    quaternion_sign: float,
) -> None:
    """
    A turn from 170 to minus 170 degrees passes through the known 180-degree pose.
    """
    example = ShortestRotationExample()
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    part = engine.scenario.parts[0]
    engine.scenario.parts = [part]
    part.pose.rotation = (
        Rotation.from_rotvec([0.0, 0.0, numpy.deg2rad(example.initial_angle_degrees)])
        .as_quat()
        .tolist()
    )
    period = engine.scenario.timing.sample_period_nanoseconds
    target_pose = Pose(
        translation=list(part.pose.translation),
        rotation=(
            quaternion_sign
            * Rotation.from_rotvec(
                [0.0, 0.0, numpy.deg2rad(example.final_angle_degrees)]
            ).as_quat()
        ).tolist(),
    )
    part.keyframes = [PoseKeyframe(2 * period, target_pose)]
    engine.scenario.timing.duration_nanoseconds = 2 * period
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    clock.timestamp_nanoseconds += period
    pipeline.tick_once()
    observed_pose = pipeline.cas.annotations[0].annotations[1]
    expected_rotation = Rotation.from_rotvec(
        [0.0, 0.0, numpy.deg2rad(example.midpoint_angle_degrees)]
    )
    numpy.testing.assert_allclose(
        Rotation.from_quat(observed_pose.rotation).as_matrix(),
        expected_rotation.as_matrix(),
        atol=1e-12,
    )
    clock.timestamp_nanoseconds += period
    pipeline.tick_once()
    assert pipeline.cas.annotations[0].annotations[1].rotation == target_pose.rotation


def test_aircraft_motion_continues_while_a_part_is_absent(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Reappearance uses the scheduled pose reached during an invisible interval.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    part = engine.scenario.parts[0]
    engine.scenario.parts = [part]
    period = engine.scenario.timing.sample_period_nanoseconds
    target_pose = part.keyframes[-1].pose
    part.keyframes = [PoseKeyframe(2 * period, target_pose)]
    part.absence_intervals = [AbsenceInterval(period, 3 * period)]
    engine.scenario.timing.duration_nanoseconds = 3 * period
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    initial_identifier = pipeline.cas.annotations[0].id
    for _ in range(2):
        clock.timestamp_nanoseconds += period
        pipeline.tick_once()
        assert pipeline.cas.annotations == []
    clock.timestamp_nanoseconds += period
    pipeline.tick_once()
    reappeared = pipeline.cas.annotations[0]

    assert reappeared.annotations[1].translation == target_pose.translation
    assert reappeared.annotations[1].rotation == target_pose.rotation
    assert reappeared.id != initial_identifier


def test_aircraft_detection_identifiers_remain_unique_through_reappearance(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Every visible detection has a fresh identifier across the complete sequence.
    """
    identifiers = [
        hypothesis.id
        for observation in aircraft_captures.observations.values()
        for hypothesis in observation.annotations
    ]
    assert len(set(identifiers)) == len(identifiers)


# %% Metadata and delayed delivery


def test_every_aircraft_capture_rewrites_explicit_frame_configuration(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Completed CAS replacement retains frame meaning through total absence.
    """
    frames = aircraft_captures.engine.frames
    for observation in aircraft_captures.observations.values():
        assert observation.world_frame == frames.world_frame
        assert observation.camera_frame == frames.camera_frame
        assert observation.camera_to_world_transform.reference_frame is (
            frames.camera_to_world_transform.reference_frame
        )
        assert observation.camera_to_world_transform.child_frame is (
            frames.camera_to_world_transform.child_frame
        )
        numpy.testing.assert_array_equal(
            observation.camera_to_world_transform.to_np(),
            frames.camera_to_world_transform.to_np(),
        )


def test_late_aircraft_delivery_preserves_every_scheduled_pose_and_timestamp(
    aircraft_captures: AircraftCaptures,
) -> None:
    """
    Overdue captures retain the chronological observations of regular delivery.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(
        frames=aircraft_captures.engine.frames,
        scenario=aircraft_captures.engine.scenario,
        clock=clock,
    )
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    clock.timestamp_nanoseconds += engine.scenario.timing.duration_nanoseconds
    for offset, expected in aircraft_captures.observations.items():
        if offset:
            pipeline.tick_once()
        assert pipeline.status is Status.SUCCESS
        assert pipeline.cas.data_timestamp == expected.data_timestamp
        assert [hypothesis.id for hypothesis in pipeline.cas.annotations] == [
            hypothesis.id for hypothesis in expected.annotations
        ]
        assert [
            hypothesis.annotations[1] for hypothesis in pipeline.cas.annotations
        ] == [hypothesis.annotations[1] for hypothesis in expected.annotations]
    pipeline.tick_once()
    assert pipeline.status is Status.FAILURE


def test_zero_duration_aircraft_sequence_emits_one_capture(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Coincident initial and final sequence endpoints produce one observation.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    engine.scenario.timing.duration_nanoseconds = 0
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    observation = pipeline.cas

    assert pipeline.status is Status.SUCCESS
    assert observation.data_timestamp == clock.timestamp_nanoseconds
    pipeline.tick_once()
    assert pipeline.status is Status.FAILURE
    assert pipeline.cas is observation


# %% Invalid configuration


@pytest.mark.parametrize(
    "timing",
    [
        PlaybackTiming(duration_nanoseconds=-1),
        PlaybackTiming(sample_period_nanoseconds=0),
        PlaybackTiming(sample_period_nanoseconds=-1),
        PlaybackTiming(duration_nanoseconds=1, sample_period_nanoseconds=2),
    ],
)
def test_aircraft_engine_rejects_timing_without_both_endpoints(
    aircraft_frames: AircraftFrames, timing: PlaybackTiming
) -> None:
    """
    Finite playback requires a positive cadence dividing a nonnegative duration.
    """
    engine = AnalysisEngine(frames=aircraft_frames)
    engine.scenario.timing = timing
    with pytest.raises(InvalidPlaybackTiming):
        engine.implementation()


@pytest.mark.parametrize(
    "example",
    [
        InvalidKeyframeExample([-1]),
        InvalidKeyframeExample([0]),
        InvalidKeyframeExample([2, 1]),
        InvalidKeyframeExample([1, 1]),
        InvalidKeyframeExample([1], alignment_error_nanoseconds=1),
    ],
)
def test_aircraft_engine_rejects_unordered_or_unsampled_keyframes(
    aircraft_frames: AircraftFrames, example: InvalidKeyframeExample
) -> None:
    """
    Edited keyframes must increase strictly and align with scheduled captures.
    """
    engine = AnalysisEngine(frames=aircraft_frames)
    part = engine.scenario.parts[0]
    period = engine.scenario.timing.sample_period_nanoseconds
    part.keyframes = [
        PoseKeyframe(offset * period + example.alignment_error_nanoseconds, part.pose)
        for offset in example.sample_offsets
    ]
    with pytest.raises(InvalidPoseKeyframes):
        engine.implementation()


@pytest.mark.parametrize(
    "interval",
    [
        AbsenceInterval(-1, 0),
        AbsenceInterval(0, 0),
        AbsenceInterval(1, 0),
        AbsenceInterval(0, 1),
    ],
)
def test_aircraft_engine_rejects_invalid_absence_intervals(
    aircraft_frames: AircraftFrames, interval: AbsenceInterval
) -> None:
    """
    Visibility intervals require ordered nonnegative boundaries on the cadence.
    """
    engine = AnalysisEngine(frames=aircraft_frames)
    engine.scenario.parts[0].absence_intervals = [interval]
    with pytest.raises(InvalidAbsenceInterval):
        engine.implementation()
