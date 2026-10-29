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
from robokudo_acrambly.aircraft_observations.configuration import (
    AircraftConfigurationField,
    AircraftEnvironment,
)
from robokudo_acrambly.annotators.aircraft_observation import AircraftObservationLabel
from robokudo_acrambly.aircraft_observations.exceptions import (
    MissingAircraftFrameConfiguration,
)

from .test_aircraft_models import replacement_model
from ..robokudo_test.dataset.semantic_observations import BeliefWorldSnapshot

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo_acrambly.aircraft.models import AircraftModelDefinition
    from semantic_digital_twin.world import World


# %% Documented launcher configuration


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
    Select the documented frame configuration without enabling mesh loading.
    """
    configuration = Path(__file__).resolve().parents[2] / request.param.value
    monkeypatch.setenv(AircraftEnvironment.FRAME_CONFIGURATION, str(configuration))
    monkeypatch.delenv(AircraftEnvironment.MESH_DIRECTORY, raising=False)
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
    pipeline.tick_once()

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


def test_launcher_leaves_mesh_preview_disabled_by_default(
    aircraft_configuration: Path,
) -> None:
    """Frame configuration alone emits observations without loading catalog meshes."""
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )
    pipeline = engine.implementation()

    assert pipeline.get_annotators()[0].preview is None


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
    pipeline.tick_once()
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


def test_launcher_configuration_preserves_world_and_beliefs(
    aircraft_configuration: Path,
    mini_world: World,
    monkeypatch: pytest.MonkeyPatch,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
) -> None:
    """
    Loading frame references and emitting a capture preserves persistent state.
    """
    monkeypatch.setattr(robokudo_world, "world", mini_world)
    baseline = BeliefWorldSnapshot.capture()
    engine = ModuleLoader().load_ae(
        AircraftConfigurationExample.PACKAGE, AircraftObservationLabel.SOURCE
    )
    pipeline = engine.implementation()
    pipeline.setup()
    pipeline.tick_once()

    assert BeliefWorldSnapshot.capture() == baseline
