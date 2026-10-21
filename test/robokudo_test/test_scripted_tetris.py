# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
import trimesh
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo import world as robokudo_world

from robokudo.descriptors.analysis_engines.scripted_tetris import (
    AnalysisEngine,
    MeshEnvironment,
)
from robokudo.scripted_observations.clock import Clock, TimeUnit
from robokudo.scripted_observations.exceptions import MissingMeshResource
from robokudo.scripted_observations.models import TetrisModel
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
    Shape,
)
from robokudo.types.scene import ObjectHypothesis
from robokudo.utils.camera_model import (
    pinhole_camera_parameters_from_horizontal_field_of_view,
)


if TYPE_CHECKING:
    from rclpy.node import Node
    from semantic_digital_twin.world import World

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap


# %% Local CAD resources


@dataclass
class LocalMeshes:
    """
    Mesh resources supplied to the public analysis engine.
    """

    directory: Path
    """Directory containing the configured PLY resources."""

    geometries: dict[TetrisModel, trimesh.Trimesh]
    """
    Unscaled CAD geometry indexed by its model definition.
    """


@pytest.fixture
def tetris_meshes(tmp_path: Path) -> LocalMeshes:
    """
    Create deterministic meshes with an offset CAD origin.
    """
    horizontal_arm = trimesh.creation.box(extents=[100.0, 30.0, 30.0])
    horizontal_arm.apply_translation([50.0, 15.0, 15.0])
    vertical_arm = trimesh.creation.box(extents=[30.0, 70.0, 30.0])
    vertical_arm.apply_translation([15.0, 65.0, 15.0])
    first_mesh = trimesh.util.concatenate([horizontal_arm, vertical_arm])
    second_mesh = trimesh.creation.box(extents=[60.0, 40.0, 20.0])
    second_mesh.apply_translation([80.0, 60.0, 20.0])
    geometries = {
        TetrisModel.FIRST: first_mesh,
        TetrisModel.SECOND: second_mesh,
    }
    for model, mesh in geometries.items():
        mesh.export(tmp_path / model.value.mesh_filename)
    return LocalMeshes(directory=tmp_path, geometries=geometries)


# %% Static observations


def test_static_observation_classifies_both_models(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    The public engine emits one hypothesis for each configured model.
    """
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert pipeline.status == Status.SUCCESS
    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    classifications = [
        pipeline.cas.filter_by_type(Classification, hypothesis.annotations)[0].classname
        for hypothesis in hypotheses
    ]
    assert classifications == [
        block.model.value.identifier for block in engine.scenario.blocks
    ]


@pytest.mark.parametrize("quarter_turns", [0, 1])
def test_metric_geometry_preserves_the_cad_origin(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    quarter_turns: int,
) -> None:
    """
    CAD geometry and its offset enclosing box describe one metric shape.
    """
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    rotation_angle = quarter_turns * np.pi / 2.0
    engine.scenario.blocks[0].pose.rotation = (
        Rotation.from_euler("z", rotation_angle).as_quat().tolist()
    )
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    for block, hypothesis in zip(engine.scenario.blocks, hypotheses, strict=True):
        object_pose = pipeline.cas.filter_by_type(
            PoseAnnotation, hypothesis.annotations
        )[0]
        shape = pipeline.cas.filter_by_type(Shape, hypothesis.annotations)[0]
        box = pipeline.cas.filter_by_type(
            BoundingBox3DAnnotation, hypothesis.annotations
        )[0]
        reference_mesh = tetris_meshes.geometries[block.model].copy()
        reference_mesh.apply_scale(block.model.value.metres_per_mesh_unit)

        assert type(object_pose) is PoseAnnotation
        assert object_pose.translation == block.pose.translation
        assert object_pose.rotation == block.pose.rotation
        assert shape.geometry.mesh.bounds == pytest.approx(reference_mesh.bounds)
        assert shape.geometry.origin.to_np() == pytest.approx(np.eye(4))
        assert [box.x_length, box.y_length, box.z_length] == pytest.approx(
            reference_mesh.extents
        )

        if block.model == TetrisModel.FIRST:
            reference_mesh.apply_transform(
                trimesh.transformations.rotation_matrix(rotation_angle, [0.0, 0.0, 1.0])
            )
        reference_mesh.apply_translation(block.pose.translation)
        assert box.pose.translation == pytest.approx(reference_mesh.bounds.mean(axis=0))
        assert box.pose.rotation == block.pose.rotation


def test_missing_mesh_reports_the_configured_resource(tmp_path: Path) -> None:
    """
    A missing CAD resource identifies the path the developer must supply.
    """
    engine = AnalysisEngine(mesh_directory=tmp_path)

    with pytest.raises(MissingMeshResource) as raised:
        engine.implementation()

    assert (
        raised.value.mesh_path
        == tmp_path / engine.scenario.blocks[0].model.value.mesh_filename
    )


# %% Capture metadata


@dataclass
class ControlledClock(Clock):
    """
    Clock whose current Unix time is controlled by the test.
    """

    timestamp_nanoseconds: int = 1_700_000_000_123_456_789
    """
    Unix time reported to the observation source.
    """

    def now_nanoseconds(self) -> int:
        return self.timestamp_nanoseconds


@pytest.mark.parametrize("empty_scene", [False, True])
def test_observation_preserves_camera_and_capture_metadata(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    empty_scene: bool,
) -> None:
    """
    Camera calibration and capture time remain available in an empty scene.
    """
    clock = ControlledClock()
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory, clock=clock)
    if empty_scene:
        engine.scenario.blocks.clear()
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    camera = engine.scenario.camera
    expected_intrinsics = pinhole_camera_parameters_from_horizontal_field_of_view(
        camera.image_width, camera.image_height, camera.horizontal_field_of_view_degrees
    )
    observation = pipeline.get_cas()
    assert observation.camera_frame == camera.frame
    assert observation.data_timestamp == clock.timestamp_nanoseconds
    assert observation.timestamp != observation.data_timestamp
    assert observation.camera_info.width == camera.image_width
    assert observation.camera_info.height == camera.image_height
    assert observation.camera_info.header.frame_id == camera.frame
    assert (
        observation.camera_info.header.stamp.sec
        == clock.timestamp_nanoseconds // TimeUnit.NANOSECONDS_PER_SECOND
    )
    assert (
        observation.camera_info.header.stamp.nanosec
        == clock.timestamp_nanoseconds % TimeUnit.NANOSECONDS_PER_SECOND
    )
    assert list(observation.camera_info.k) == [
        expected_intrinsics.focal_length_x,
        0.0,
        expected_intrinsics.center_x,
        0.0,
        expected_intrinsics.focal_length_y,
        expected_intrinsics.center_y,
        0.0,
        0.0,
        1.0,
    ]
    assert observation.camera_intrinsic.width == camera.image_width
    assert observation.camera_intrinsic.height == camera.image_height
    assert observation.camera_intrinsic.intrinsic_matrix == pytest.approx(
        np.asarray(observation.camera_info.k).reshape(3, 3)
    )


def test_mesh_directory_can_be_configured_for_the_default_engine(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Launcher construction obtains the configured mesh directory.
    """
    monkeypatch.setenv(MeshEnvironment.DIRECTORY, str(tetris_meshes.directory))
    engine = AnalysisEngine()
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    hypotheses = pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    assert [
        Path(
            pipeline.cas.filter_by_type(Shape, hypothesis.annotations)[
                0
            ].geometry.filename
        )
        for hypothesis in hypotheses
    ] == [
        tetris_meshes.directory / block.model.value.mesh_filename
        for block in engine.scenario.blocks
    ]


# %% Observation isolation


def test_detection_identifiers_are_deterministic_and_distinct(
    tetris_meshes: LocalMeshes,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Repeated sequences reproduce distinct detection identifiers for their models.
    """
    engine = AnalysisEngine(
        mesh_directory=tetris_meshes.directory, clock=ControlledClock()
    )
    first_pipeline = engine.implementation()
    first_pipeline.setup()
    first_pipeline.tick_once()
    second_pipeline = engine.implementation()
    second_pipeline.setup()
    second_pipeline.tick_once()

    first_identifiers = [
        hypothesis.id
        for hypothesis in first_pipeline.cas.filter_annotations_by_type(
            ObjectHypothesis
        )
    ]
    second_identifiers = [
        hypothesis.id
        for hypothesis in second_pipeline.cas.filter_annotations_by_type(
            ObjectHypothesis
        )
    ]
    assert first_identifiers == second_identifiers
    assert len(set(first_identifiers)) == len(engine.scenario.blocks)


def test_observation_leaves_beliefs_and_world_contents_unchanged(
    tetris_meshes: LocalMeshes,
    mini_world: World,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The engine supplies hypotheses without adding persistent world objects.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    initial_bodies = tuple(mini_world.bodies)
    initial_beliefs = dict(robokudo_world.get_object_belief_states())
    engine = AnalysisEngine(mesh_directory=tetris_meshes.directory)
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert tuple(robokudo_world.world_instance().bodies) == initial_bodies
    assert robokudo_world.get_object_belief_states() == initial_beliefs
    assert [
        hypothesis.object_knowledge
        for hypothesis in pipeline.cas.filter_annotations_by_type(ObjectHypothesis)
    ] == [None] * len(engine.scenario.blocks)
