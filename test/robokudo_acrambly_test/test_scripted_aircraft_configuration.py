# %% Imports

from __future__ import annotations

import json
from enum import Enum, StrEnum
from pathlib import Path

import numpy
import pytest
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo.utils.module_loader import ModuleLoader
from robokudo.annotators.exceptions import InvalidObservationPose
from robokudo.types.tf import Pose
from robokudo import world as robokudo_world
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.exceptions import MissingAircraftResource
from robokudo_acrambly.aircraft.types import AircraftPart
from robokudo_acrambly.aircraft_observations.configuration import (
    AircraftConfigurationField,
    AircraftEnvironment,
)
from robokudo_acrambly.annotators.aircraft_observation import AircraftObservationLabel
from robokudo_acrambly.aircraft_observations.exceptions import (
    MissingAircraftFrameConfiguration,
    MissingAircraftMeshConfiguration,
)

from .test_aircraft_models import AircraftFixtureResource, replacement_model
from .test_aircraft_belief_creation import semdt_cas
from .dataset.aircraft_beliefs import AircraftBeliefCapture
from ..robokudo_test.dataset.semantic_observations import BeliefWorldSnapshot

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelDefinition
    from semantic_digital_twin.world import World


# %% Documented launcher configuration


@pytest.fixture(autouse=True)
def isolated_aircraft_beliefs(semdt_cas: CAS) -> None:
    """Reset local beliefs and runtime world around each launcher test."""


class AircraftConfigurationExample(StrEnum):
    """
    Package exposing the aircraft analysis engine.
    """

    PACKAGE = "robokudo_acrambly"
    """Application import namespace for the ordinary module loader."""


class AircraftConfigurationFile(Enum):
    """Frame calibration documents relative to the repository root."""

    DOCUMENTED = Path("robokudo_acrambly/doc/aircraft_frames.json")
    """Frame calibration shipped with the usage guide."""

    ROTATED = Path("test/robokudo_acrambly_test/dataset/aircraft_frames_rotated.json")
    """Independent translation and quarter-turn camera calibration."""


class CameraPoseComponent(StrEnum):
    """Required components of the configured camera world pose."""

    TRANSLATION = "translation"
    """Camera-origin coordinates in metres."""

    ROTATION = "rotation"
    """Camera orientation as an xyzw quaternion."""


@pytest.fixture(
    params=[
        AircraftConfigurationFile.DOCUMENTED,
        AircraftConfigurationFile.ROTATED,
    ]
)
def aircraft_configuration(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest
) -> Path:
    """
    Select explicit camera calibration and the checked-in aircraft meshes.
    """
    configuration = Path(__file__).resolve().parents[2] / request.param.value
    monkeypatch.setenv(AircraftEnvironment.FRAME_CONFIGURATION, str(configuration))
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY,
        str(
            Path(__file__).resolve().parents[2]
            / AircraftFixtureResource.SOURCE_DIRECTORY
        ),
    )
    return configuration


def test_launcher_emits_the_documented_camera_calibration(
    aircraft_configuration: Path,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    The ordinary launcher preserves explicitly configured camera frame meaning.
    """
    configuration = json.loads(aircraft_configuration.read_text())
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )
    pipeline = engine.implementation()
    pipeline.setup()
    AircraftBeliefCapture(pipeline).complete()

    assert pipeline.status is Status.SUCCESS
    assert (
        pipeline.cas.world_frame
        == configuration[AircraftConfigurationField.WORLD_FRAME]
    )
    assert (
        pipeline.cas.camera_frame
        == configuration[AircraftConfigurationField.CAMERA_FRAME]
    )
    camera_pose = Pose(**configuration[AircraftConfigurationField.CAMERA_POSE])
    numpy.testing.assert_array_equal(
        pipeline.cas.camera_to_world_transform.to_np()[:3, 3],
        camera_pose.translation,
    )
    numpy.testing.assert_allclose(
        pipeline.cas.camera_to_world_transform.to_np()[:3, :3],
        Rotation.from_quat(camera_pose.rotation).as_matrix(),
        atol=numpy.finfo(float).eps,
    )


def test_launcher_shares_catalog_between_preview_and_belief_geometry(
    aircraft_configuration: Path,
) -> None:
    """Display and belief creation reuse the same loaded aircraft geometry."""
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )
    pipeline = engine.implementation()

    source, connector = pipeline.get_annotators()
    assert (
        connector.descriptor.parameters.model_geometry is source.preview.catalog.models
    )
    assert connector.descriptor.parameters.semantic_types == {
        label: model.definition.semantic_type
        for label, model in source.preview.catalog.models.items()
    }


def test_launcher_enables_preview_from_an_explicit_mesh_directory(
    aircraft_configuration: Path,
    replacement_model: AircraftModelDefinition,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    An explicit mesh directory enables one native mesh per observed aircraft kind.
    """
    resource = (tmp_path / replacement_model.resource).read_bytes()
    for model in AircraftModel:
        (tmp_path / model.value.resource).write_bytes(resource)
    monkeypatch.setenv(AircraftEnvironment.MESH_DIRECTORY, str(tmp_path))
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )
    pipeline = engine.implementation()
    pipeline.setup()
    AircraftBeliefCapture(pipeline).complete()
    output = pipeline.get_annotators()[0].get_annotator_output_struct()

    assert len(output.geometries) == len(pipeline.cas.annotations)
    assert len(output.geometries) == len(AircraftModel)


def test_launcher_requires_explicit_frame_configuration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Missing frame configuration cannot silently introduce identity calibration.
    """
    monkeypatch.delenv(AircraftEnvironment.FRAME_CONFIGURATION, raising=False)
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )

    with pytest.raises(MissingAircraftFrameConfiguration):
        engine.implementation()


@pytest.mark.parametrize("camera_pose_component", list(CameraPoseComponent))
def test_launcher_rejects_incomplete_camera_calibration(
    aircraft_configuration: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    camera_pose_component: CameraPoseComponent,
) -> None:
    """
    Missing camera pose components cannot acquire spatial constructor defaults.
    """
    configuration = json.loads(aircraft_configuration.read_text())
    configuration[AircraftConfigurationField.CAMERA_POSE].pop(camera_pose_component)
    configuration_path = tmp_path / aircraft_configuration.name
    configuration_path.write_text(json.dumps(configuration))
    monkeypatch.setenv(AircraftEnvironment.FRAME_CONFIGURATION, str(configuration_path))
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )

    with pytest.raises(InvalidObservationPose):
        engine.implementation()


def test_launcher_adds_aircraft_beliefs_to_the_existing_world(
    aircraft_configuration: Path,
    mini_world: World,
    monkeypatch: pytest.MonkeyPatch,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Aircraft acceptance preserves existing bodies while adding typed beliefs.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    initial_bodies = tuple(mini_world.bodies)
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )
    pipeline = engine.implementation()
    assert robokudo_world.get_object_belief_states() == {}
    pipeline.setup()
    AircraftBeliefCapture(pipeline).complete()

    assert len(robokudo_world.get_object_belief_states()) == len(AircraftModel)
    assert len(mini_world.get_semantic_annotations_by_type(AircraftPart)) == len(
        AircraftModel
    )
    assert tuple(mini_world.bodies[: len(initial_bodies)]) == initial_bodies


# %% Required belief geometry


def test_launcher_requires_explicit_mesh_configuration(
    aircraft_configuration: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A missing mesh directory rejects construction before world changes."""
    monkeypatch.delenv(AircraftEnvironment.MESH_DIRECTORY, raising=False)
    baseline = BeliefWorldSnapshot.capture()
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )

    with pytest.raises(MissingAircraftMeshConfiguration):
        engine.implementation()

    assert BeliefWorldSnapshot.capture() == baseline


def test_launcher_rejects_unavailable_meshes_before_world_changes(
    aircraft_configuration: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An incomplete catalog cannot leave a partially initialized belief world."""
    monkeypatch.setenv(AircraftEnvironment.MESH_DIRECTORY, str(tmp_path))
    baseline = BeliefWorldSnapshot.capture()
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )

    with pytest.raises(MissingAircraftResource):
        engine.implementation()

    assert BeliefWorldSnapshot.capture() == baseline
