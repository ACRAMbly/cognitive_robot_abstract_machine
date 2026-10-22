# %% Imports

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest
import trimesh
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo.cas import CAS
from robokudo.descriptors.analysis_engines.scripted_tetris import AnalysisEngine
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
    Shape,
)
from robokudo.scripted_observations.projection import MaskPixel
from robokudo.types.scene import ObjectHypothesis
from robokudo.types.tf import Pose
from robokudo.scripted_observations.scenario import PoseKeyframe
from robokudo.scripted_observations.exceptions import InvalidPoseKeyframes

from .test_scripted_tetris import ControlledClock, LocalMeshes, tetris_meshes
from .test_scripted_tetris_masks import projection_mesh_directory

if TYPE_CHECKING:
    from pathlib import Path
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.scripted_observations.models import TetrisModel


# %% Completed observations


@dataclass
class CapturedSequence:
    """Completed CAS observations indexed by their scheduled capture offset."""

    engine: AnalysisEngine
    """Public engine whose scenario defines the expected observation sequence."""

    observations: dict[int, CAS]
    """Completed observations indexed by elapsed capture nanoseconds."""

    def find_hypothesis(
        self, elapsed_nanoseconds: int, model: TetrisModel
    ) -> ObjectHypothesis:
        """Find the detection classified as the requested model in one capture."""
        observation = self.observations[elapsed_nanoseconds]
        return next(
            hypothesis
            for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis)
            if observation.filter_by_type(Classification, hypothesis.annotations)[
                0
            ].classname
            == model.value.identifier
        )


def capture_sequence(
    engine: AnalysisEngine, clock: ControlledClock
) -> CapturedSequence:
    """Drive the public engine through every configured scheduled capture."""
    pipeline = engine.implementation()
    pipeline.setup()
    sequence_start = clock.timestamp_nanoseconds
    timing = engine.scenario.timing
    observations = {}
    for offset in range(
        0, timing.duration_nanoseconds + 1, timing.sample_period_nanoseconds
    ):
        clock.timestamp_nanoseconds = sequence_start + offset
        pipeline.tick_once()
        assert pipeline.status == Status.SUCCESS
        observations[offset] = pipeline.get_cas()
    return CapturedSequence(engine, observations)


@pytest.fixture
def motion_sequence(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> CapturedSequence:
    """Capture the default motion with deterministic local CAD resources."""
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    return capture_sequence(engine, clock)


# %% Translation


def test_translation_reaches_the_configured_midpoint(
    motion_sequence: CapturedSequence,
) -> None:
    """The moving block translates linearly while the reference stays stationary."""
    moving_block, stationary_block = motion_sequence.engine.scenario.blocks
    movement_start, movement_end = moving_block.keyframes[:2]
    midpoint = (
        movement_start.elapsed_nanoseconds + movement_end.elapsed_nanoseconds
    ) // 2
    observation = motion_sequence.observations[midpoint]
    moving_hypothesis = motion_sequence.find_hypothesis(midpoint, moving_block.model)
    stationary_hypothesis = motion_sequence.find_hypothesis(
        midpoint, stationary_block.model
    )
    moving_pose = observation.filter_by_type(
        PoseAnnotation, moving_hypothesis.annotations
    )[0]
    stationary_pose = observation.filter_by_type(
        PoseAnnotation, stationary_hypothesis.annotations
    )[0]

    assert moving_pose.translation == pytest.approx(
        (np.asarray(movement_start.pose.translation) + movement_end.pose.translation)
        / 2.0
    )
    assert moving_pose.rotation == movement_start.pose.rotation
    assert stationary_pose.translation == stationary_block.pose.translation
    assert stationary_pose.rotation == stationary_block.pose.rotation


# %% Rotation


def test_rotation_reaches_the_midpoint_between_keyframes(
    motion_sequence: CapturedSequence,
) -> None:
    """The first block reaches half its configured positive turn at the midpoint."""
    moving_block = motion_sequence.engine.scenario.blocks[0]
    rotation_start, rotation_end = moving_block.keyframes[2:]
    midpoint = (
        rotation_start.elapsed_nanoseconds + rotation_end.elapsed_nanoseconds
    ) // 2
    hypothesis = motion_sequence.find_hypothesis(midpoint, moving_block.model)
    pose = motion_sequence.observations[midpoint].filter_by_type(
        PoseAnnotation, hypothesis.annotations
    )[0]
    endpoint_rotation = Rotation.from_quat(rotation_end.pose.rotation)
    expected_rotation = Rotation.from_rotvec(endpoint_rotation.as_rotvec() / 2.0)

    assert pose.translation == rotation_start.pose.translation
    assert Rotation.from_quat(pose.rotation).as_matrix() == pytest.approx(
        expected_rotation.as_matrix()
    )


def test_configured_keyframes_are_emitted_exactly(
    motion_sequence: CapturedSequence,
) -> None:
    """Every visible keyframe retains its configured translation and quaternion."""
    block = motion_sequence.engine.scenario.blocks[0]
    for keyframe in block.keyframes:
        hypothesis = motion_sequence.find_hypothesis(
            keyframe.elapsed_nanoseconds, block.model
        )
        observation = motion_sequence.observations[keyframe.elapsed_nanoseconds]
        pose = observation.filter_by_type(PoseAnnotation, hypothesis.annotations)[0]
        assert pose.translation == keyframe.pose.translation
        assert pose.rotation == keyframe.pose.rotation


@pytest.mark.parametrize("quaternion_sign", [1.0, -1.0])
def test_rotation_follows_the_short_path_across_quaternion_signs(
    quaternion_sign: float,
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """A 170 to minus-170 degree turn passes through 180 for either quaternion sign."""
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    block = engine.scenario.blocks[0]
    block.pose.rotation = (
        Rotation.from_rotvec([0.0, 0.0, np.deg2rad(170.0)]).as_quat().tolist()
    )
    target_pose = Pose(
        translation=list(block.pose.translation),
        rotation=(
            quaternion_sign
            * Rotation.from_rotvec([0.0, 0.0, np.deg2rad(-170.0)]).as_quat()
        ).tolist(),
    )
    period = engine.scenario.timing.sample_period_nanoseconds
    block.keyframes = [PoseKeyframe(2 * period, target_pose)]
    engine.scenario.timing.duration_nanoseconds = block.keyframes[
        -1
    ].elapsed_nanoseconds
    sequence = capture_sequence(engine, clock)
    hypothesis = sequence.find_hypothesis(period, block.model)
    midpoint_pose = sequence.observations[period].filter_by_type(
        PoseAnnotation, hypothesis.annotations
    )[0]
    expected_rotation = Rotation.from_rotvec([0.0, 0.0, np.pi])

    assert Rotation.from_quat(midpoint_pose.rotation).as_matrix() == pytest.approx(
        expected_rotation.as_matrix()
    )
    endpoint = sequence.find_hypothesis(
        block.keyframes[-1].elapsed_nanoseconds, block.model
    )
    endpoint_pose = sequence.observations[
        block.keyframes[-1].elapsed_nanoseconds
    ].filter_by_type(PoseAnnotation, endpoint.annotations)[0]
    assert endpoint_pose.rotation == target_pose.rotation


# %% Motion pauses


@pytest.mark.parametrize("keyframe_index", [None, 1, 3])
def test_motion_pauses_keep_sampling_stationary_poses(
    motion_sequence: CapturedSequence, keyframe_index: int | None
) -> None:
    """A hold preserves poses while capture times and detection identifiers advance."""
    moving_block, reference_block = motion_sequence.engine.scenario.blocks
    period = motion_sequence.engine.scenario.timing.sample_period_nanoseconds
    hold_start = (
        0
        if keyframe_index is None
        else moving_block.keyframes[keyframe_index].elapsed_nanoseconds
    )
    expected_pose = (
        moving_block.pose
        if keyframe_index is None
        else moving_block.keyframes[keyframe_index].pose
    )
    first_offset = hold_start + period
    second_offset = first_offset + period
    first_observation = motion_sequence.observations[first_offset]
    second_observation = motion_sequence.observations[second_offset]
    first_hypothesis = motion_sequence.find_hypothesis(first_offset, moving_block.model)
    second_hypothesis = motion_sequence.find_hypothesis(
        second_offset, moving_block.model
    )

    assert second_observation is not first_observation
    assert (
        second_observation.data_timestamp - first_observation.data_timestamp == period
    )
    assert first_hypothesis.id != second_hypothesis.id
    for elapsed in [first_offset, second_offset]:
        observation = motion_sequence.observations[elapsed]
        moving_hypothesis = motion_sequence.find_hypothesis(elapsed, moving_block.model)
        reference_hypothesis = motion_sequence.find_hypothesis(
            elapsed, reference_block.model
        )
        moving_pose = observation.filter_by_type(
            PoseAnnotation, moving_hypothesis.annotations
        )[0]
        reference_pose = observation.filter_by_type(
            PoseAnnotation, reference_hypothesis.annotations
        )[0]
        assert moving_pose.translation == expected_pose.translation
        assert moving_pose.rotation == expected_pose.rotation
        assert reference_pose.translation == reference_block.pose.translation
        assert reference_pose.rotation == reference_block.pose.rotation


# %% Invalid keyframes


@dataclass
class InvalidKeyframeExample:
    """Keyframe offsets that cannot form a sampled motion sequence."""

    sample_offsets: list[int]
    """Offsets expressed as multiples of the configured sample period."""

    alignment_error_nanoseconds: int = 0
    """Additional time making a keyframe fall between scheduled captures."""


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
def test_engine_rejects_unordered_or_unsampled_keyframes(
    tetris_meshes: LocalMeshes, example: InvalidKeyframeExample
) -> None:
    """Edited keyframes must increase strictly and align with emitted samples."""
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    period = engine.scenario.timing.sample_period_nanoseconds
    block = engine.scenario.blocks[0]
    block.keyframes = [
        PoseKeyframe(offset * period + example.alignment_error_nanoseconds, block.pose)
        for offset in example.sample_offsets
    ]

    with pytest.raises(InvalidPoseKeyframes):
        engine.implementation()


# %% Geometry throughout motion


def test_rotating_box_center_follows_its_offset_cad_geometry(
    motion_sequence: CapturedSequence, tetris_meshes: LocalMeshes
) -> None:
    """The halfway-rotated enclosing box moves its nonzero model-local center."""
    block = motion_sequence.engine.scenario.blocks[0]
    rotation_start, rotation_end = block.keyframes[2:]
    midpoint = (
        rotation_start.elapsed_nanoseconds + rotation_end.elapsed_nanoseconds
    ) // 2
    observation = motion_sequence.observations[midpoint]
    hypothesis = motion_sequence.find_hypothesis(midpoint, block.model)
    box = observation.filter_by_type(BoundingBox3DAnnotation, hypothesis.annotations)[0]
    shape = observation.filter_by_type(Shape, hypothesis.annotations)[0]
    reference_mesh = tetris_meshes.geometries[block.model].copy()
    reference_mesh.apply_scale(block.model.value.metres_per_mesh_unit)
    reference_box = reference_mesh.bounding_box.copy()
    rotation_angle = Rotation.from_quat(rotation_end.pose.rotation).magnitude() / 2.0
    reference_box.apply_transform(
        trimesh.transformations.rotation_matrix(rotation_angle, [0.0, 0.0, 1.0])
    )
    reference_box.apply_translation(rotation_start.pose.translation)

    assert box.pose.translation == pytest.approx(reference_box.centroid)
    assert [box.x_length, box.y_length, box.z_length] == pytest.approx(
        reference_mesh.extents
    )
    assert shape.geometry.mesh.bounds == pytest.approx(reference_mesh.bounds)


def test_all_motion_samples_stay_fully_in_frame_and_separate(
    projection_mesh_directory: Path,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """Every default intermediate pose has full and disjoint image evidence."""
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=projection_mesh_directory, clock=clock)
    sequence = capture_sequence(engine, clock)
    camera = engine.scenario.camera

    for observation in sequence.observations.values():
        occupancy = np.zeros((camera.image_height, camera.image_width), dtype=np.uint8)
        for hypothesis in observation.filter_annotations_by_type(ObjectHypothesis):
            region = hypothesis.roi.roi
            left, top, right, bottom = region.get_corner_points()
            assert 0 <= left < right <= camera.image_width
            assert 0 <= top < bottom <= camera.image_height
            assert hypothesis.roi.mask.shape == (region.height, region.width)
            occupancy[top:bottom, left:right] += (
                hypothesis.roi.mask == MaskPixel.FOREGROUND
            )
        assert np.count_nonzero(occupancy > 1) == 0


# %% Delivery delays


def test_late_delivery_preserves_the_scheduled_motion_poses(
    motion_sequence: CapturedSequence,
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """Overdue samples retain the poses of their scheduled captures."""
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    clock.timestamp_nanoseconds += engine.scenario.timing.duration_nanoseconds

    for elapsed in list(motion_sequence.observations)[1:]:
        pipeline.tick_once()
        assert pipeline.status == Status.SUCCESS
        expected = motion_sequence.observations[elapsed]
        observed = pipeline.get_cas()
        expected_hypotheses = expected.filter_annotations_by_type(ObjectHypothesis)
        observed_hypotheses = observed.filter_annotations_by_type(ObjectHypothesis)
        assert [
            observed.filter_by_type(PoseAnnotation, hypothesis.annotations)[0]
            for hypothesis in observed_hypotheses
        ] == [
            expected.filter_by_type(PoseAnnotation, hypothesis.annotations)[0]
            for hypothesis in expected_hypotheses
        ]
