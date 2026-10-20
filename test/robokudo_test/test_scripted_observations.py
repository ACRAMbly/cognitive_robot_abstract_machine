"""
Verify scripted perception through the analysis engine's CAS output.
"""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass
from datetime import timedelta
from enum import IntEnum, StrEnum

import numpy as np
import open3d
import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
from robokudo.descriptors.analysis_engines.scripted_tetris import AnalysisEngine
from robokudo.descriptors.scripted_tetris_observations import create_tetris_scenario
from robokudo.io.scripted_observations import (
    InvalidScriptedObservationError,
    PlaybackTiming,
    PreviewColor,
    ScriptedCamera,
    ScriptedIdentifier,
    ScriptedResource,
    ScriptedScenario,
    TetrisBlock,
)
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
)
from robokudo.types.scene import ObjectHypothesis
from robokudo.vis.scripted_observations import ObservationPreview

# %% deterministic playback


@dataclass
class PlaybackClock:
    """
    Provide a controllable elapsed time without waiting during playback.
    """

    seconds: float = 0.0
    """
    Elapsed playback time.
    """

    def __call__(self) -> float:
        """
        Return the current elapsed playback time.
        """
        return self.seconds


@pytest.fixture
def playback_clock() -> PlaybackClock:
    """
    Provide a fresh playback clock for each scenario execution.
    """
    return PlaybackClock()


@pytest.fixture
def scripted_pipeline(playback_clock: PlaybackClock) -> Pipeline:
    """
    Build a finite scenario using the existing isolated blackboard fixture.
    """
    Blackboard().set(ScriptedIdentifier.OUTPUT_BUFFER, AnnotatorOutputPerPipelineMap())
    pipeline = AnalysisEngine(repeat=False, clock=playback_clock).implementation()
    pipeline.setup()
    return pipeline


# %% visible blocks


def test_first_observation_classifies_both_target_blocks(
    scripted_pipeline: Pipeline,
) -> None:
    """
    Expose both configured model identities in the first observation.
    """
    scripted_pipeline.tick_once()

    hypotheses = scripted_pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    classifications = [
        annotation
        for hypothesis in hypotheses
        for annotation in hypothesis.annotations
        if isinstance(annotation, Classification)
    ]
    assert scripted_pipeline.status is Status.SUCCESS
    assert [classification.class_id for classification in classifications] == [
        block.value.class_identifier for block in TetrisBlock
    ]


# %% visibility sequence


def test_absent_blocks_do_not_leak_from_previous_observations(
    scripted_pipeline: Pipeline, playback_clock: PlaybackClock
) -> None:
    """
    Remove absent detections and restore both blocks on reappearance.
    """
    first_identifier = TetrisBlock.FIRST.value.class_identifier
    second_identifier = TetrisBlock.SECOND.value.class_identifier
    expected_visible_models = [
        [first_identifier, second_identifier],
        [first_identifier, second_identifier],
        [first_identifier, second_identifier],
        [second_identifier],
        [],
        [first_identifier, second_identifier],
    ]
    observations = []
    for expected_identifiers in expected_visible_models:
        scripted_pipeline.tick_once()
        observation = scripted_pipeline.cas
        observations.append(observation)
        classifications = [
            annotation.class_id
            for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis)
            for annotation in hypothesis.annotations
            if isinstance(annotation, Classification)
        ]
        assert classifications == expected_identifiers
        playback_clock.seconds += 1.0

    assert len({id(observation) for observation in observations}) == len(observations)


# %% mesh bounding boxes


@pytest.mark.parametrize("block", list(TetrisBlock))
def test_rotated_bounding_box_uses_mesh_center_and_metric_dimensions(
    scripted_pipeline: Pipeline, playback_clock: PlaybackClock, block: TetrisBlock
) -> None:
    """
    Keep the mesh's offset CAD origin distinct from the rotated box center.
    """
    for _ in range(3):
        scripted_pipeline.tick_once()
        playback_clock.seconds += 1.0
    hypothesis = next(
        hypothesis
        for hypothesis in scripted_pipeline.cas.filter_annotations_by_type(
            ObjectHypothesis
        )
        if any(
            isinstance(annotation, Classification)
            and annotation.class_id == block.value.class_identifier
            for annotation in hypothesis.annotations
        )
    )
    box = next(
        annotation
        for annotation in hypothesis.annotations
        if isinstance(annotation, BoundingBox3DAnnotation)
    )
    pose = next(
        annotation
        for annotation in hypothesis.annotations
        if type(annotation) is PoseAnnotation
    )
    mesh = open3d.io.read_triangle_mesh(
        str(ScriptedResource.MESH_DIRECTORY.resolve() / block.value.mesh_filename)
    )
    bounds = mesh.get_axis_aligned_bounding_box()
    center_x, center_y, center_z = (
        bounds.get_center() * block.value.metres_per_mesh_unit
    )
    expected_center = np.asarray(pose.translation) + [-center_y, center_x, center_z]

    assert box.pose.translation == pytest.approx(expected_center)
    assert box.pose.rotation == pose.rotation
    assert [box.x_length, box.y_length, box.z_length] == pytest.approx(
        bounds.get_extent() * block.value.metres_per_mesh_unit
    )


# %% CAD poses


def test_detection_poses_preserve_scripted_cad_frames(
    scripted_pipeline: Pipeline, playback_clock: PlaybackClock
) -> None:
    """
    Publish the specified translation and xyzw rotation without recentering.
    """
    for observation in create_tetris_scenario().observations:
        scripted_pipeline.tick_once()
        hypotheses = scripted_pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
        poses = [
            annotation
            for hypothesis in hypotheses
            for annotation in hypothesis.annotations
            if type(annotation) is PoseAnnotation
        ]
        assert len(poses) == len(observation.detections)
        for pose, detection in zip(poses, observation.detections):
            assert pose.translation == detection.pose.translation
            assert pose.rotation == detection.pose.rotation
        playback_clock.seconds += 1.0


# %% observation metadata


def test_observation_metadata_retains_camera_frame_and_capture_time(
    scripted_pipeline: Pipeline, playback_clock: PlaybackClock
) -> None:
    """
    Use one capture timestamp and camera frame throughout each observation.
    """
    scripted_pipeline.tick_once()
    first_capture_time = scripted_pipeline.cas.data_timestamp
    playback_clock.seconds += 1.0
    scripted_pipeline.tick_once()
    observation = scripted_pipeline.cas
    camera = ScriptedCamera()

    assert observation.camera_frame == camera.frame
    assert observation.camera_info.header.frame_id == observation.camera_frame
    capture_stamp = observation.camera_info.header.stamp
    assert (
        capture_stamp.sec * camera.nanoseconds_per_second + capture_stamp.nanosec
    ) == observation.data_timestamp
    assert (
        observation.data_timestamp - first_capture_time == camera.nanoseconds_per_second
    )


# %% playback scheduling


@pytest.mark.parametrize(
    "frame_interval", [timedelta(milliseconds=250), timedelta(seconds=2)]
)
def test_configured_interval_waits_without_publishing_duplicates(
    scripted_pipeline: Pipeline,
    playback_clock: PlaybackClock,
    frame_interval: timedelta,
) -> None:
    """
    Wait between frames without replacing the last completed observation.
    """
    pipeline = AnalysisEngine(
        clock=playback_clock, timing=PlaybackTiming(frame_interval=frame_interval)
    ).implementation()
    pipeline.setup()
    pipeline.tick_once()
    first_observation = pipeline.cas

    playback_clock.seconds = frame_interval.total_seconds() / 2
    pipeline.tick_once()
    assert pipeline.status is Status.RUNNING
    assert pipeline.cas is first_observation

    playback_clock.seconds = frame_interval.total_seconds()
    pipeline.tick_once()
    assert pipeline.status is Status.SUCCESS
    assert pipeline.cas is not first_observation


@pytest.mark.parametrize("repeat", [False, True])
def test_playback_end_respects_repeat_setting(
    scripted_pipeline: Pipeline, playback_clock: PlaybackClock, repeat: bool
) -> None:
    """
    Stop finite playback or repeat the first scene with a later capture time.
    """
    pipeline = AnalysisEngine(repeat=repeat, clock=playback_clock).implementation()
    pipeline.setup()
    pipeline.tick_once()
    first_observation = pipeline.cas
    observation_count = len(create_tetris_scenario().observations)
    for _ in range(observation_count - 1):
        playback_clock.seconds += 1.0
        pipeline.tick_once()
    last_observation = pipeline.cas
    playback_clock.seconds += 1.0
    pipeline.tick_once()

    if not repeat:
        assert pipeline.status is Status.FAILURE
        assert pipeline.cas is last_observation
        return

    assert pipeline.status is Status.SUCCESS
    assert pipeline.cas.data_timestamp == (
        first_observation.data_timestamp
        + observation_count * ScriptedCamera.nanoseconds_per_second
    )
    assert [
        annotation.translation
        for hypothesis in pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
        for annotation in hypothesis.annotations
        if type(annotation) is PoseAnnotation
    ] == [
        annotation.translation
        for hypothesis in first_observation.filter_annotations_by_type(ObjectHypothesis)
        for annotation in hypothesis.annotations
        if type(annotation) is PoseAnnotation
    ]


# %% image regions


def test_image_regions_contain_cropped_binary_masks(
    scripted_pipeline: Pipeline, playback_clock: PlaybackClock
) -> None:
    """
    Provide nonempty, image-bounded rectangles with cropped foreground masks.
    """
    for _ in create_tetris_scenario().observations:
        scripted_pipeline.tick_once()
        information = scripted_pipeline.cas.camera_info
        for hypothesis in scripted_pipeline.cas.filter_annotations_by_type(
            ObjectHypothesis
        ):
            region = hypothesis.roi.roi
            assert 0 <= region.pos.x < region.pos.x + region.width <= information.width
            assert (
                0 <= region.pos.y < region.pos.y + region.height <= information.height
            )
            np.testing.assert_array_equal(
                hypothesis.roi.mask,
                np.ones((region.height, region.width), dtype=np.uint8),
            )
            assert hypothesis.roi.mask.dtype == np.uint8
        playback_clock.seconds += 1.0


def test_preview_displays_current_regions_in_cas_and_gui(
    scripted_pipeline: Pipeline,
) -> None:
    """
    Expose the same diagnostic image through the CAS and annotator output.
    """
    scripted_pipeline.tick_once()
    observation = scripted_pipeline.cas
    image = observation.color_image
    output_map = Blackboard().get(ScriptedIdentifier.OUTPUT_BUFFER)
    output = output_map.map[scripted_pipeline.name].outputs[
        ScriptedIdentifier.ANNOTATOR
    ]

    assert image.shape == (
        observation.camera_info.height,
        observation.camera_info.width,
        3,
    )
    assert image.dtype == np.uint8
    np.testing.assert_array_equal(output.image, image)
    for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis):
        region = hypothesis.roi.roi
        np.testing.assert_array_equal(
            image[
                region.pos.y + region.height // 2,
                region.pos.x + region.width // 2,
            ],
            PreviewColor.FOREGROUND.value,
        )


# %% classification metadata


def test_classification_retains_model_label_and_detection_confidence(
    scripted_pipeline: Pipeline,
) -> None:
    """
    Carry the declared label and classification confidence into each detection.
    """
    scripted_pipeline.tick_once()
    hypotheses = scripted_pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    detections = create_tetris_scenario().observations[0].detections
    for hypothesis, detection in zip(hypotheses, detections):
        classification = next(
            annotation
            for annotation in hypothesis.annotations
            if isinstance(annotation, Classification)
        )
        assert classification.classname == detection.block.value.mesh_filename.stem
        assert classification.confidence == detection.confidence


# %% invalid playback settings


@pytest.mark.parametrize("frame_interval", [timedelta(0), timedelta(seconds=-1)])
def test_nonpositive_playback_interval_is_rejected(frame_interval: timedelta) -> None:
    """
    Reject timelines that cannot advance capture timestamps.
    """
    with pytest.raises(InvalidScriptedObservationError):
        AnalysisEngine(
            timing=PlaybackTiming(frame_interval=frame_interval)
        ).implementation()


def test_empty_scenario_is_rejected() -> None:
    """
    Distinguish a missing playback sequence from a valid empty observation.
    """
    with pytest.raises(InvalidScriptedObservationError):
        AnalysisEngine(scenario=ScriptedScenario(observations=())).implementation()


# %% preview labels


def test_model_labels_have_separate_legend_rows(scripted_pipeline: Pipeline) -> None:
    """
    Keep model labels legible independently of projected object spacing.
    """
    scripted_pipeline.tick_once()
    style = ObservationPreview(ScriptedCamera())
    image = scripted_pipeline.cas.color_image
    hypotheses = scripted_pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    for row, _ in enumerate(hypotheses, start=1):
        baseline = style.heading_height + row * style.legend_line_height
        legend_row = image[baseline - style.legend_line_height + 1 : baseline + 1]
        assert np.any(np.all(legend_row == PreviewColor.TEXT.value, axis=2))


# %% command-line startup


class RunnerArgument(StrEnum):
    """Arguments of a bounded headless engine invocation."""

    TIMEOUT = "timeout"
    """GNU command that interrupts the repeating demonstration."""

    INTERRUPT = "--signal=INT"
    """Signal allowing the runner to clean up its ROS nodes."""

    KILL_DELAY = "--kill-after=3s"
    """Fallback limit if shutdown does not complete."""

    MODULE = "-m"
    """Python module execution flag."""

    ENTRYPOINT = "robokudo.scripts.main"
    """Public RoboKudo runner module."""

    ENGINE = "_ae"
    """Analysis engine selection flag."""

    HEADLESS = "_headless"
    """Disable GUI startup."""

    WITHOUT_3D = "_no3d"
    """Disable the Open3D viewer."""


class RunnerLimit(IntEnum):
    """Limits of the command-line startup check."""

    STARTUP_SECONDS = 12
    """Time allowed for startup and repeated observation processing."""

    PROCESS_SECONDS = 20
    """Outer limit including graceful shutdown."""

    TIMED_OUT = 124
    """GNU timeout result for a process interrupted at its configured deadline."""


def test_headless_runner_processes_without_a_belief_world() -> None:
    """Keep the actual runner alive until interrupted, with no world initialization."""
    result = subprocess.run(
        [
            RunnerArgument.TIMEOUT,
            RunnerArgument.INTERRUPT,
            RunnerArgument.KILL_DELAY,
            f"{RunnerLimit.STARTUP_SECONDS}s",
            sys.executable,
            RunnerArgument.MODULE,
            RunnerArgument.ENTRYPOINT,
            RunnerArgument.ENGINE,
            ScriptedIdentifier.ENGINE,
            RunnerArgument.HEADLESS,
            RunnerArgument.WITHOUT_3D,
        ],
        capture_output=True,
        text=True,
        timeout=RunnerLimit.PROCESS_SECONDS,
    )
    assert result.returncode == RunnerLimit.TIMED_OUT, result.stdout + result.stderr
