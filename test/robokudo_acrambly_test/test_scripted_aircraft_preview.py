# %% Imports

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import numpy
import pytest
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.aircraft.models import AircraftModel, AircraftModelCatalog
from robokudo_acrambly.aircraft_observations.preview import AircraftPreview
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from robokudo import world as robokudo_world
from robokudo.cas import CASViews
from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo_acrambly.playback.motion import AbsenceInterval, PoseKeyframe

from ..robokudo_test.dataset.semantic_observations import BeliefWorldSnapshot
from .test_aircraft_models import AircraftFixtureResource, replacement_model
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo_acrambly.aircraft.models import AircraftModelDefinition
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames
    from semantic_digital_twin.world import World


# %% Catalog preview configuration


@pytest.fixture
def preview_engine(
    tmp_path: Path,
    replacement_model: AircraftModelDefinition,
    aircraft_frames: AircraftFrames,
) -> AnalysisEngine:
    """
    Supply an offset metric mesh and one independently posed aircraft part.
    """
    catalog = AircraftModelCatalog(tmp_path, definitions=(replacement_model,))
    engine = AnalysisEngine(
        frames=aircraft_frames,
        clock=ControlledClock(),
        preview=AircraftPreview(catalog),
    )
    engine.scenario.parts = [
        part
        for part in engine.scenario.parts
        if part.model.value.label == replacement_model.label
    ]
    engine.scenario.parts[0].pose.rotation = (
        Rotation.from_rotvec([0.0, 0.0, numpy.pi / 2.0]).as_quat().tolist()
    )
    return engine


# %% Independent display geometry


def test_aircraft_preview_places_catalog_mesh_at_the_emitted_origin_pose(
    preview_engine: AnalysisEngine,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Native geometry follows the captured mesh origin without mutating source data.
    """
    part = preview_engine.scenario.parts[0]
    template = preview_engine.preview.catalog.models[
        part.model.value.label
    ].geometry.mesh
    original_vertices = template.vertices.copy()
    original_pose = deepcopy(part.pose)
    pipeline = preview_engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    source = pipeline.get_annotators()[0]
    geometries = (
        annotator_output_buffer.map[pipeline.name].outputs[source.name].geometries
    )
    observed_pose = pipeline.cas.annotations[0].annotations[1]

    assert pipeline.status is Status.SUCCESS
    assert len(geometries) == len(preview_engine.scenario.parts)
    expected_vertices = (
        Rotation.from_quat(observed_pose.rotation).apply(original_vertices)
        + observed_pose.translation
    )
    numpy.testing.assert_allclose(
        numpy.asarray(geometries[0].vertices), expected_vertices
    )
    numpy.testing.assert_array_equal(
        numpy.asarray(geometries[0].triangles), template.faces
    )
    numpy.testing.assert_array_equal(template.vertices, original_vertices)
    assert part.pose == original_pose
    assert observed_pose.translation == original_pose.translation
    assert observed_pose.rotation == original_pose.rotation


def test_aircraft_preview_uses_loaded_templates_after_resources_are_removed(
    preview_engine: AnalysisEngine,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Later captures reuse preview resources loaded before source construction.
    """
    catalog = preview_engine.preview.catalog
    for definition in catalog.definitions:
        (catalog.mesh_directory / definition.resource).unlink()
    pipeline = preview_engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    source = pipeline.get_annotators()[0]
    original_geometries = source.get_annotator_output_struct().geometries
    preview_engine.clock.timestamp_nanoseconds += (
        preview_engine.scenario.timing.sample_period_nanoseconds
    )
    pipeline.tick_once()
    geometries = source.get_annotator_output_struct().geometries

    assert pipeline.status is Status.SUCCESS
    assert len(geometries) == len(catalog.models)
    assert geometries is not original_geometries
    assert geometries[0] is not original_geometries[0]


def test_aircraft_preview_replaces_moving_and_absent_part_geometry(
    preview_engine: AnalysisEngine,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Completed captures replace moved meshes and clear wholly absent scenes.
    """
    part = preview_engine.scenario.parts[0]
    period = preview_engine.scenario.timing.sample_period_nanoseconds
    final_pose = part.keyframes[-1].pose
    part.keyframes = [PoseKeyframe(period, final_pose)]
    part.absence_intervals = [AbsenceInterval(2 * period, 3 * period)]
    preview_engine.scenario.timing.duration_nanoseconds = 3 * period
    pipeline = preview_engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    output = pipeline.get_annotators()[0].get_annotator_output_struct()
    template = preview_engine.preview.catalog.models[
        part.model.value.label
    ].geometry.mesh
    preview_engine.clock.timestamp_nanoseconds += period
    pipeline.tick_once()

    expected_vertices = (
        Rotation.from_quat(final_pose.rotation).apply(template.vertices)
        + final_pose.translation
    )
    numpy.testing.assert_allclose(
        numpy.asarray(output.geometries[0].vertices), expected_vertices
    )
    preview_engine.clock.timestamp_nanoseconds += period
    pipeline.tick_once()
    assert output.geometries == []
    preview_engine.clock.timestamp_nanoseconds += period
    pipeline.tick_once()
    numpy.testing.assert_allclose(
        numpy.asarray(output.geometries[0].vertices), expected_vertices
    )


@pytest.mark.parametrize("exhausted", [False, True])
def test_aircraft_preview_retains_geometry_without_a_completed_capture(
    preview_engine: AnalysisEngine,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    exhausted: bool,
) -> None:
    """
    Waiting and exhausted ticks retain the preceding display output and CAS.
    """
    if exhausted:
        preview_engine.scenario.timing.duration_nanoseconds = 0
    pipeline = preview_engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    completed_observation = pipeline.cas
    output = pipeline.get_annotators()[0].get_annotator_output_struct()
    completed_geometry = output.geometries
    pipeline.tick_once()

    assert pipeline.status is (Status.FAILURE if exhausted else Status.RUNNING)
    assert pipeline.cas is completed_observation
    assert output.geometries is completed_geometry


def test_aircraft_preview_leaves_the_world_unchanged(
    preview_engine: AnalysisEngine,
    mini_world: World,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Source construction and mesh display preserve all persistent world state.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    baseline = BeliefWorldSnapshot.capture()
    pipeline = preview_engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert BeliefWorldSnapshot.capture() == baseline


def test_aircraft_preview_adds_no_sensor_or_geometry_evidence_to_the_cas(
    preview_engine: AnalysisEngine,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Mesh verification retains the source's minimal observation payload.
    """
    pipeline = preview_engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert set(pipeline.cas.views) == {
        CASViews.CAS_ID,
        CASViews.DATA_TIMESTAMP,
        CASViews.WORLD_FRAME,
        CASViews.CAMERA_FRAME,
        CASViews.CAMERA_TO_WORLD_TRANSFORM,
    }
    assert [
        type(annotation) for annotation in pipeline.cas.annotations[0].annotations
    ] == [
        Classification,
        PoseAnnotation,
    ]


# %% Checkout aircraft resources


def test_aircraft_preview_places_all_available_checkout_catalog_meshes(
    aircraft_frames: AircraftFrames,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Actual aircraft resources retain their metric coordinates at observed origins.
    """
    directory = (
        Path(__file__).resolve().parents[2] / AircraftFixtureResource.SOURCE_DIRECTORY
    )
    if not all((directory / model.value.resource).is_file() for model in AircraftModel):
        pytest.skip("The checkout aircraft resources are unavailable.")
    catalog = AircraftModelCatalog(directory)
    engine = AnalysisEngine(
        frames=aircraft_frames,
        clock=ControlledClock(),
        preview=AircraftPreview(catalog),
    )
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()
    geometries = pipeline.get_annotators()[0].get_annotator_output_struct().geometries

    assert len(geometries) == len(catalog.models)
    for hypothesis, geometry in zip(pipeline.cas.annotations, geometries, strict=True):
        label = hypothesis.annotations[0].classname
        observed_pose = hypothesis.annotations[1]
        expected_vertices = (
            Rotation.from_quat(observed_pose.rotation).apply(
                catalog.models[label].geometry.mesh.vertices
            )
            + observed_pose.translation
        )
        numpy.testing.assert_allclose(
            numpy.asarray(geometry.vertices), expected_vertices
        )
