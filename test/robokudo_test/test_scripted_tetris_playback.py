# %% Imports

from __future__ import annotations

import pytest
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo.descriptors.analysis_engines.scripted_tetris import AnalysisEngine
from robokudo.garden import grow_tree
from robokudo.scripted_observations.clock import TimeUnit
from robokudo.scripted_observations.exceptions import InvalidPlaybackTiming
from robokudo.scripted_observations.scenario import PlaybackTiming
from robokudo.types.scene import ObjectHypothesis

from .test_scripted_tetris import (
    CameraMetadata,
    CaptureExample,
    ControlledClock,
    tetris_meshes,
)
from .test_scripted_tetris_motion import CapturedSequence

if TYPE_CHECKING:
    from .test_scripted_tetris import LocalMeshes
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap


# %% Scheduled observations


def test_early_tick_waits_without_replacing_the_completed_observation(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    A tick before the next capture preserves the last completed CAS.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    completed_observation = pipeline.get_cas()

    clock.timestamp_nanoseconds += engine.scenario.timing.sample_period_nanoseconds - 1
    pipeline.tick_once()

    assert pipeline.status == Status.RUNNING
    assert pipeline.get_cas() is completed_observation
    clock.timestamp_nanoseconds += 1
    pipeline.tick_once()
    assert pipeline.status == Status.SUCCESS
    assert pipeline.get_cas().data_timestamp == clock.timestamp_nanoseconds


def test_finite_headless_playback_includes_both_endpoints_and_then_stops(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    The headless runner observes exhaustion after the final capture once.
    """
    clock = ControlledClock()
    sequence_start = clock.timestamp_nanoseconds
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    engine.scenario.timing = PlaybackTiming(
        duration_nanoseconds=CaptureExample.FINAL_SAMPLE_INDEX
        * CaptureExample.SAMPLE_PERIOD_NANOSECONDS,
        sample_period_nanoseconds=CaptureExample.SAMPLE_PERIOD_NANOSECONDS,
    )
    timing = engine.scenario.timing
    pipeline = engine.implementation()
    tree = grow_tree(pipeline, node=node, include_gui=False)
    observations = list(
        CapturedSequence.capture(
            engine, clock, pipeline=pipeline, tick=tree.tick
        ).observations.values()
    )

    expected_capture_times = list(
        range(
            sequence_start,
            sequence_start + timing.duration_nanoseconds + 1,
            timing.sample_period_nanoseconds,
        )
    )
    assert [observation.data_timestamp for observation in observations] == (
        expected_capture_times
    )
    assert len({id(observation) for observation in observations}) == len(
        expected_capture_times
    )

    for _ in range(2):
        clock.timestamp_nanoseconds += timing.duration_nanoseconds
        tree.tick()
        assert tree.root.children[0].status == Status.FAILURE
        assert pipeline.get_cas() is observations[-1]


def test_zero_duration_completes_one_initial_observation(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Coincident initial and final endpoints produce one completed capture.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    engine.scenario.timing.duration_nanoseconds = 0
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert pipeline.status == Status.SUCCESS
    observation = pipeline.get_cas()
    assert observation.data_timestamp == clock.timestamp_nanoseconds
    pipeline.tick_once()
    assert pipeline.status == Status.FAILURE
    assert pipeline.get_cas() is observation


def test_stationary_captures_have_new_reproducible_detection_identifiers(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Each capture distinguishes its detections while repeating deterministically.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    identifiers_by_sequence = []

    for _ in range(2):
        pipeline = engine.implementation()
        pipeline.setup()
        sequence_identifiers = []
        for _ in range(2):
            pipeline.tick_once()
            assert pipeline.status == Status.SUCCESS
            sequence_identifiers.extend(
                hypothesis.id
                for hypothesis in pipeline.get_cas().filter_annotations_by_type(
                    ObjectHypothesis
                )
            )
            clock.timestamp_nanoseconds += (
                engine.scenario.timing.sample_period_nanoseconds
            )
        identifiers_by_sequence.append(sequence_identifiers)

    assert identifiers_by_sequence[0] == identifiers_by_sequence[1]
    assert len(set(identifiers_by_sequence[0])) == 2 * len(engine.scenario.blocks)


def test_late_ticks_preserve_every_scheduled_capture_and_camera_timestamp(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Delivery delays preserve chronological captures and their camera metadata.
    """
    clock = ControlledClock()
    sequence_start = clock.timestamp_nanoseconds
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    engine.scenario.timing = PlaybackTiming(
        duration_nanoseconds=CaptureExample.FINAL_SAMPLE_INDEX
        * TimeUnit.NANOSECONDS_PER_SECOND,
        sample_period_nanoseconds=TimeUnit.NANOSECONDS_PER_SECOND,
    )
    timing = engine.scenario.timing
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    initial_observation = pipeline.get_cas()
    clock.timestamp_nanoseconds += 2 * timing.duration_nanoseconds

    observations = list(
        CapturedSequence.capture(
            engine,
            clock,
            pipeline=pipeline,
            advance_clock=False,
            previous_observations={0: initial_observation},
        ).observations.values()
    )

    expected_capture_times = list(
        range(
            sequence_start,
            sequence_start + timing.duration_nanoseconds + 1,
            timing.sample_period_nanoseconds,
        )
    )
    assert [observation.data_timestamp for observation in observations] == (
        expected_capture_times
    )
    for observation, capture_timestamp in zip(
        observations, expected_capture_times, strict=True
    ):
        CameraMetadata(engine.scenario.camera, capture_timestamp).assert_matches(
            observation
        )
    pipeline.tick_once()
    assert pipeline.status == Status.FAILURE
    assert pipeline.get_cas() is observations[-1]


# %% Invalid schedules


@pytest.mark.parametrize(
    "timing",
    [
        PlaybackTiming(duration_nanoseconds=-1),
        PlaybackTiming(sample_period_nanoseconds=0),
        PlaybackTiming(sample_period_nanoseconds=-1),
        PlaybackTiming(duration_nanoseconds=1, sample_period_nanoseconds=2),
    ],
)
def test_engine_rejects_a_schedule_that_cannot_include_both_endpoints(
    tetris_meshes: LocalMeshes,
    timing: PlaybackTiming,
) -> None:
    """
    Invalid finite timing is rejected before any capture can be emitted.
    """
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    engine.scenario.timing = timing

    with pytest.raises(InvalidPlaybackTiming):
        engine.implementation()
