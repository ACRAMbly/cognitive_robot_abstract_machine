# %% Imports

from __future__ import annotations

import time
from dataclasses import dataclass

import pytest
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

from robokudo import world as robokudo_world
from robokudo.descriptors.analysis_engines.scripted_tetris import AnalysisEngine
from robokudo.scripted_observations.clock import TimeUnit
from robokudo.scripted_observations.exceptions import InvalidAbsenceInterval
from robokudo.scripted_observations.scenario import AbsenceInterval, AppearanceBoundary
from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.types.scene import ObjectHypothesis

from .test_scripted_tetris import CameraMetadata, ControlledClock, tetris_meshes
from .test_scripted_tetris_motion import (
    CapturedSequence,
    motion_sequence,
)

if TYPE_CHECKING:
    from .test_scripted_tetris import LocalMeshes
    from rclpy.node import Node
    from semantic_digital_twin.world import World

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap


# %% Default appearance boundaries


def test_first_disappearance_replaces_the_preceding_two_block_scene(
    motion_sequence: CapturedSequence,
) -> None:
    """
    The boundary emits only the stationary reference without a stale first block.
    """
    blocks = motion_sequence.engine.scenario.blocks
    period = motion_sequence.engine.scenario.timing.sample_period_nanoseconds
    before = motion_sequence.observations[AppearanceBoundary.FIRST_DISAPPEARS - period]
    after = motion_sequence.observations[AppearanceBoundary.FIRST_DISAPPEARS]

    assert [
        before.filter_by_type(Classification, hypothesis.annotations)[0].classname
        for hypothesis in before.filter_annotations_by_type(ObjectHypothesis)
    ] == [block.model.value.identifier for block in blocks]
    assert [
        after.filter_by_type(Classification, hypothesis.annotations)[0].classname
        for hypothesis in after.filter_annotations_by_type(ObjectHypothesis)
    ] == [blocks[1].model.value.identifier]
    assert after is not before


def test_empty_interval_and_reappearance_emit_the_new_boundary_states(
    motion_sequence: CapturedSequence,
) -> None:
    """
    An empty capture replaces the reference before both blocks reappear.
    """
    blocks = motion_sequence.engine.scenario.blocks
    period = motion_sequence.engine.scenario.timing.sample_period_nanoseconds
    expected_models = {
        AppearanceBoundary.SECOND_DISAPPEARS
        - period: [blocks[1].model.value.identifier],
        AppearanceBoundary.SECOND_DISAPPEARS: [],
        AppearanceBoundary.BOTH_REAPPEAR - period: [],
        AppearanceBoundary.BOTH_REAPPEAR: [
            block.model.value.identifier for block in blocks
        ],
        motion_sequence.engine.scenario.timing.duration_nanoseconds: [
            block.model.value.identifier for block in blocks
        ],
    }
    for offset, identifiers in expected_models.items():
        observation = motion_sequence.observations[offset]
        assert [
            observation.filter_by_type(Classification, hypothesis.annotations)[
                0
            ].classname
            for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis)
        ] == identifiers


# %% Complete scheduled sequence


@dataclass
class SequenceAgreement:
    """
    Agreed default schedule checked once against the public engine descriptor.
    """

    duration_nanoseconds: int = 30 * TimeUnit.NANOSECONDS_PER_SECOND
    """
    Capture duration required by the approved specification.
    """

    sample_period_nanoseconds: int = TimeUnit.NANOSECONDS_PER_SECOND // 10
    """
    Capture cadence required by the approved specification.
    """

    capture_count: int = 301
    """
    Number of observations including both endpoints.
    """

    first_disappearance_nanoseconds: int = 21 * TimeUnit.NANOSECONDS_PER_SECOND
    """
    Agreed boundary at which the moving block disappears.
    """

    second_disappearance_nanoseconds: int = 24 * TimeUnit.NANOSECONDS_PER_SECOND
    """
    Agreed boundary at which the stationary reference disappears.
    """

    reappearance_nanoseconds: int = 27 * TimeUnit.NANOSECONDS_PER_SECOND
    """
    Agreed boundary at which both blocks return.
    """


def test_default_sequence_contains_every_scheduled_visibility_state(
    motion_sequence: CapturedSequence,
) -> None:
    """
    All 301 captures reflect the configured half-open absence intervals.
    """
    timing = motion_sequence.engine.scenario.timing
    agreement = SequenceAgreement()
    blocks = motion_sequence.engine.scenario.blocks
    first_absence = blocks[0].absence_intervals[0]
    second_absence = blocks[1].absence_intervals[0]
    assert first_absence == AbsenceInterval(
        AppearanceBoundary.FIRST_DISAPPEARS, AppearanceBoundary.BOTH_REAPPEAR
    )
    assert second_absence == AbsenceInterval(
        AppearanceBoundary.SECOND_DISAPPEARS, AppearanceBoundary.BOTH_REAPPEAR
    )
    assert timing.duration_nanoseconds == agreement.duration_nanoseconds
    assert timing.sample_period_nanoseconds == agreement.sample_period_nanoseconds
    assert (
        AppearanceBoundary.FIRST_DISAPPEARS == agreement.first_disappearance_nanoseconds
    )
    assert (
        AppearanceBoundary.SECOND_DISAPPEARS
        == agreement.second_disappearance_nanoseconds
    )
    assert AppearanceBoundary.BOTH_REAPPEAR == agreement.reappearance_nanoseconds
    expected_offsets = list(
        range(0, timing.duration_nanoseconds + 1, timing.sample_period_nanoseconds)
    )
    assert len(expected_offsets) == agreement.capture_count
    assert list(motion_sequence.observations) == expected_offsets
    observations = list(motion_sequence.observations.values())
    assert len({id(observation) for observation in observations}) == len(
        expected_offsets
    )
    assert len({observation.cas_id for observation in observations}) == len(
        expected_offsets
    )
    sequence_start = observations[0].data_timestamp
    assert [observation.data_timestamp for observation in observations] == [
        sequence_start + offset for offset in expected_offsets
    ]
    absent_offsets = {
        block.model: set().union(
            *[
                range(
                    interval.start_nanoseconds,
                    interval.end_nanoseconds,
                    timing.sample_period_nanoseconds,
                )
                for interval in block.absence_intervals
            ]
        )
        for block in blocks
    }
    identifiers = []
    for offset, observation in motion_sequence.observations.items():
        hypotheses = observation.filter_annotations_by_type(ObjectHypothesis)
        assert [
            observation.filter_by_type(Classification, hypothesis.annotations)[
                0
            ].classname
            for hypothesis in hypotheses
        ] == [
            block.model.value.identifier
            for block in blocks
            if offset not in absent_offsets[block.model]
        ]
        identifiers.extend(hypothesis.id for hypothesis in hypotheses)
    assert len(set(identifiers)) == len(identifiers)


def test_reappearing_blocks_hold_their_last_visible_poses(
    motion_sequence: CapturedSequence,
) -> None:
    """
    Reappearance retains the last pose and creates a new detection identifier.
    """
    period = motion_sequence.engine.scenario.timing.sample_period_nanoseconds
    for block in motion_sequence.engine.scenario.blocks:
        absence = block.absence_intervals[0]
        expected_pose = block.keyframes[-1].pose if block.keyframes else block.pose
        before = motion_sequence.find_hypothesis(
            absence.start_nanoseconds - period, block.model
        )
        after = motion_sequence.find_hypothesis(absence.end_nanoseconds, block.model)
        assert after.id != before.id
        for offset in [
            absence.start_nanoseconds - period,
            absence.end_nanoseconds,
            motion_sequence.engine.scenario.timing.duration_nanoseconds,
        ]:
            observation = motion_sequence.observations[offset]
            hypothesis = motion_sequence.find_hypothesis(offset, block.model)
            pose = observation.filter_by_type(PoseAnnotation, hypothesis.annotations)[0]
            assert pose.translation == expected_pose.translation
            assert pose.rotation == expected_pose.rotation


# %% Camera metadata throughout absence


def test_all_captures_retain_camera_metadata_and_construction_times(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Empty and visible observations have exact capture metadata and real creation times.
    """
    clock = ControlledClock()
    sequence_start = clock.timestamp_nanoseconds
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    construction_start = time.time_ns()
    sequence = CapturedSequence.capture(engine, clock)
    construction_end = time.time_ns()
    for offset, observation in sequence.observations.items():
        CameraMetadata(engine.scenario.camera, sequence_start + offset).assert_matches(
            observation
        )
        assert construction_start <= observation.timestamp <= construction_end


# %% Delayed delivery and completion


def test_late_complete_sequence_matches_regular_captures_and_stays_exhausted(
    motion_sequence: CapturedSequence,
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Waiting adds no capture and delayed ticks preserve the complete finite sequence.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    assert pipeline.status == Status.SUCCESS
    observations = [pipeline.get_cas()]
    for _ in range(2):
        pipeline.tick_once()
        assert pipeline.status == Status.RUNNING
        assert pipeline.get_cas() is observations[0]
    clock.timestamp_nanoseconds += 2 * engine.scenario.timing.duration_nanoseconds
    observations = list(
        CapturedSequence.capture(
            engine,
            clock,
            pipeline=pipeline,
            advance_clock=False,
            previous_observations={0: observations[0]},
        ).observations.values()
    )
    assert len(observations) == len(motion_sequence.observations)
    for observed, expected in zip(
        observations, motion_sequence.observations.values(), strict=True
    ):
        assert observed.data_timestamp == expected.data_timestamp
        observed_hypotheses = observed.filter_annotations_by_type(ObjectHypothesis)
        expected_hypotheses = expected.filter_annotations_by_type(ObjectHypothesis)
        assert [hypothesis.id for hypothesis in observed_hypotheses] == [
            hypothesis.id for hypothesis in expected_hypotheses
        ]
        assert [
            observed.filter_by_type(Classification, hypothesis.annotations)
            for hypothesis in observed_hypotheses
        ] == [
            expected.filter_by_type(Classification, hypothesis.annotations)
            for hypothesis in expected_hypotheses
        ]
        assert [
            observed.filter_by_type(PoseAnnotation, hypothesis.annotations)
            for hypothesis in observed_hypotheses
        ] == [
            expected.filter_by_type(PoseAnnotation, hypothesis.annotations)
            for hypothesis in expected_hypotheses
        ]
    for _ in range(2):
        clock.timestamp_nanoseconds += engine.scenario.timing.duration_nanoseconds
        pipeline.tick_once()
        assert pipeline.status == Status.FAILURE
        assert pipeline.get_cas() is observations[-1]


# %% Persistent world isolation


def test_complete_sequence_preserves_the_world_and_belief_baseline(
    tetris_meshes: LocalMeshes,
    mini_world: World,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Motion, disappearance, and reappearance leave persistent world state unchanged.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    initial_bodies = tuple(mini_world.bodies)
    initial_beliefs = dict(robokudo_world.get_object_belief_states())
    clock = ControlledClock()
    sequence = CapturedSequence.capture(
        AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock), clock
    )

    assert tuple(robokudo_world.world_instance().bodies) == initial_bodies
    assert robokudo_world.get_object_belief_states() == initial_beliefs
    for observation in sequence.observations.values():
        hypotheses = observation.filter_annotations_by_type(ObjectHypothesis)
        assert [hypothesis.object_knowledge for hypothesis in hypotheses] == [
            None
        ] * len(hypotheses)


# %% Editable visibility


def test_edited_absence_intervals_control_each_scheduled_capture(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Multiple short configured intervals control visibility independently of motion.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    first, second = engine.scenario.blocks
    period = engine.scenario.timing.sample_period_nanoseconds
    engine.scenario.timing.duration_nanoseconds = 4 * period
    first.absence_intervals = [
        AbsenceInterval(period, 2 * period),
        AbsenceInterval(3 * period, 4 * period),
    ]
    second.absence_intervals = [AbsenceInterval(0, period)]
    sequence = CapturedSequence.capture(engine, clock)
    expected_models = [
        [first.model.value.identifier],
        [second.model.value.identifier],
        [first.model.value.identifier, second.model.value.identifier],
        [second.model.value.identifier],
        [first.model.value.identifier, second.model.value.identifier],
    ]
    for observation, identifiers in zip(
        sequence.observations.values(), expected_models, strict=True
    ):
        assert [
            observation.filter_by_type(Classification, hypothesis.annotations)[
                0
            ].classname
            for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis)
        ] == identifiers


@pytest.mark.parametrize(
    "absence",
    [
        AbsenceInterval(-2, -1),
        AbsenceInterval(0, 0),
        AbsenceInterval(2, 1),
        AbsenceInterval(1, TimeUnit.NANOSECONDS_PER_SECOND),
        AbsenceInterval(0, 1),
    ],
)
def test_engine_rejects_reversed_negative_or_unsampled_absence_boundaries(
    tetris_meshes: LocalMeshes, absence: AbsenceInterval
) -> None:
    """
    Edited visibility boundaries must form nonnegative sampled intervals.
    """
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    engine.scenario.blocks[0].absence_intervals = [absence]

    with pytest.raises(InvalidAbsenceInterval):
        engine.implementation()
