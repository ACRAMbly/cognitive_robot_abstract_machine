# %% Imports

from __future__ import annotations

from copy import deepcopy

import numpy
import pytest
from rclpy.constants import S_TO_NS
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.semantic_observations import (
    ObservationCapture,
    SemanticObservationAcceptance,
)
from robokudo.types.annotation import PoseAnnotation
from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames

from ..robokudo_test.dataset.semantic_observations import BeliefWorldSnapshot
from .test_aircraft_belief_creation import (
    aircraft_catalog,
    aircraft_connector,
    captured_cas,
    observation_example,
    semdt_cas,
)
from .test_scripted_aircraft_playback import aircraft_captures
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from robokudo.annotators.semantic_world_connector import (
        SemanticDigitalTwinConnector,
    )
    from robokudo.cas import CAS

    from ..robokudo_test.dataset.semantic_observations import ObservationExample
    from .test_scripted_aircraft_playback import AircraftCaptures


# %% Registered observation frames


@pytest.fixture
def compatibility_baseline(captured_cas: CAS) -> BeliefWorldSnapshot:
    """
    Retain the local state after the hosting fixture initializes both frames.
    """
    return BeliefWorldSnapshot.capture()


@pytest.fixture
def aircraft_frames(
    captured_cas: CAS, compatibility_baseline: BeliefWorldSnapshot
) -> AircraftFrames:
    """
    Supply registered frame references before source construction begins.
    """
    return AircraftFrames(
        world_frame=captured_cas.world_frame,
        camera_frame=captured_cas.camera_frame,
        camera_to_world_transform=captured_cas.camera_to_world_transform,
    )


@pytest.fixture
def aircraft_acceptance(
    aircraft_connector: SemanticDigitalTwinConnector,
) -> SemanticObservationAcceptance:
    """
    Apply the existing application's configured aircraft semantic mapping.
    """
    return SemanticObservationAcceptance(
        aircraft_connector.descriptor.parameters.semantic_types
    )


# %% Configured model acceptance


def test_aircraft_source_resolves_every_configured_semantic_kind(
    aircraft_captures: AircraftCaptures,
    aircraft_acceptance: SemanticObservationAcceptance,
) -> None:
    """
    Public aircraft detections resolve through the configured acceptance API.
    """
    observation = aircraft_captures.observations[0]
    prepared = [
        aircraft_acceptance.prepare(
            hypothesis, observation, aircraft_captures.engine.name()
        )
        for hypothesis in observation.annotations
    ]

    assert [accepted.hypothesis for accepted in prepared] == observation.annotations
    assert [accepted.semantic_type for accepted in prepared] == [
        part.model.value.semantic_type
        for part in aircraft_captures.engine.scenario.parts
    ]


def test_aircraft_captures_prepare_independently_calculated_world_poses(
    aircraft_captures: AircraftCaptures,
    aircraft_acceptance: SemanticObservationAcceptance,
    observation_example: ObservationExample,
) -> None:
    """
    All movement and hold captures preserve the configured camera conversion.
    """
    camera_orientation_in_world = (
        Rotation.from_quat(observation_example.world_pose.rotation)
        * Rotation.from_quat(observation_example.camera_pose.rotation).inv()
    )
    for observation in aircraft_captures.observations.values():
        for hypothesis in observation.annotations:
            (camera_pose,) = observation.filter_by_type(
                PoseAnnotation, hypothesis.annotations
            )
            camera_displacement = (
                numpy.asarray(camera_pose.translation)
                - observation_example.camera_pose.translation
            )
            expected_translation = numpy.asarray(
                observation_example.world_pose.translation
            ) + [
                -camera_displacement[1],
                camera_displacement[0],
                camera_displacement[2],
            ]
            expected_orientation = camera_orientation_in_world * Rotation.from_quat(
                camera_pose.rotation
            )

            prepared = aircraft_acceptance.prepare(
                hypothesis, observation, aircraft_captures.engine.name()
            )

            numpy.testing.assert_allclose(
                prepared.world_pose.translation, expected_translation, atol=1e-12
            )
            numpy.testing.assert_allclose(
                Rotation.from_quat(prepared.world_pose.rotation).as_matrix(),
                expected_orientation.as_matrix(),
                atol=1e-12,
            )


# %% Capture metadata acceptance


def test_aircraft_capture_metadata_is_accepted_through_total_absence(
    aircraft_captures: AircraftCaptures,
    aircraft_acceptance: SemanticObservationAcceptance,
) -> None:
    """
    Every capture retains exact accepted time and frames, including empty scenes.
    """
    initial_timestamp = ControlledClock().timestamp_nanoseconds
    frames = aircraft_captures.engine.frames
    for elapsed_nanoseconds, observation in aircraft_captures.observations.items():
        capture = ObservationCapture.from_cas(observation)
        capture.validate_camera_transform(observation)
        expected_timestamp = initial_timestamp + elapsed_nanoseconds

        assert observation.data_timestamp == expected_timestamp
        assert capture.timestamp.sec * S_TO_NS + capture.timestamp.nanosec == (
            expected_timestamp
        )
        assert capture.world_frame == frames.world_frame
        assert capture.camera_frame == frames.camera_frame
        for hypothesis in observation.annotations:
            prepared = aircraft_acceptance.prepare(
                hypothesis, observation, aircraft_captures.engine.name()
            )
            assert prepared.world_pose.frame == capture.world_frame
            assert prepared.world_pose.timestamp == capture.timestamp


# %% Observation and world isolation


def test_aircraft_acceptance_preserves_observations_and_initialized_world(
    aircraft_captures: AircraftCaptures,
    aircraft_acceptance: SemanticObservationAcceptance,
    compatibility_baseline: BeliefWorldSnapshot,
) -> None:
    """
    Source execution and acceptance preparation preserve all persistent state.
    """
    assert BeliefWorldSnapshot.capture() == compatibility_baseline
    for observation in aircraft_captures.observations.values():
        original_annotations = deepcopy(observation.annotations)
        original_views = set(observation.views)
        world_T_camera = observation.camera_to_world_transform.to_np().copy()
        original_capture = ObservationCapture.from_cas(observation)

        for hypothesis in observation.annotations:
            aircraft_acceptance.prepare(
                hypothesis, observation, aircraft_captures.engine.name()
            )

        assert observation.annotations == original_annotations
        assert set(observation.views) == original_views
        numpy.testing.assert_array_equal(
            observation.camera_to_world_transform.to_np(), world_T_camera
        )
        assert ObservationCapture.from_cas(observation) == original_capture
        assert BeliefWorldSnapshot.capture() == compatibility_baseline
