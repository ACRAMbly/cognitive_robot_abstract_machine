# %% Imports

from __future__ import annotations

from copy import deepcopy

import pytest
from builtin_interfaces.msg import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import CameraInfo
from typing_extensions import TYPE_CHECKING, cast

from robokudo.types.annotation import StampedPoseAnnotation
from robokudo.annotators.semantic_world_connector import SemanticDigitalTwinConnector
from robokudo.annotators.exceptions import (
    MissingModelLabel,
    UnknownModelLabel,
    ConflictingModelLabels,
    InvalidObservationPose,
    InvalidObservationFrame,
    InvalidCaptureMetadata,
    InvalidCameraTransform,
)
from robokudo import world
from robokudo.cas import CASViews
from robokudo.types.annotation import (
    BoundingBox3DAnnotation,
    Classification,
    PoseAnnotation,
)
from robokudo.types.scene import ObjectHypothesis

from .dataset.semantic_observations import (
    ModelLabel,
    ObservedBlock,
    ObservedPanel,
    ObservationExample,
    InvalidPose,
    BeliefWorldSnapshot,
    InvalidCapture,
    CaptureRejection,
    ClassificationRejection,
)
from .test_semantic_world_connector import semdt_cas, _attach_connector_to_pipeline

if TYPE_CHECKING:
    from robokudo.cas import CAS

# %% Configured acceptance


@pytest.fixture
def configured_connector() -> SemanticDigitalTwinConnector:
    """
    Require local semantic kinds without application-package imports.
    """
    connector = SemanticDigitalTwinConnector()
    connector.descriptor.parameters.semantic_types = {
        ModelLabel.BLOCK: ObservedBlock,
        ModelLabel.PANEL: ObservedPanel,
    }
    return connector


@pytest.fixture
def observation_example() -> ObservationExample:
    """
    Supply a known camera-to-world conversion and a precise capture time.
    """
    return ObservationExample()


@pytest.fixture
def captured_cas(semdt_cas: CAS, observation_example: ObservationExample) -> CAS:
    """
    Extend the existing connector world fixture with capture metadata.
    """
    observation_example.configure_capture(semdt_cas)
    return semdt_cas


@pytest.fixture(params=[False, True], ids=["empty_world", "populated_world"])
def belief_world_snapshot(
    request: pytest.FixtureRequest,
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> BeliefWorldSnapshot:
    """
    Capture either an empty world or a belief with accepted pose and geometry.
    """
    _attach_connector_to_pipeline(configured_connector, captured_cas)
    if request.param:
        hypothesis = observation_example.create_hypothesis()
        hypothesis.annotations.append(
            BoundingBox3DAnnotation(
                pose=deepcopy(observation_example.camera_pose),
                x_length=0.1,
                y_length=0.2,
                z_length=0.3,
            )
        )
        configured_connector.associate_hypotheses_with_beliefs(
            [hypothesis], [], captured_cas
        )
    return BeliefWorldSnapshot.capture()


def test_missing_model_label_leaves_belief_world_empty(
    configured_connector: SemanticDigitalTwinConnector, semdt_cas: CAS
) -> None:
    """
    Configured observations require a model before creating any belief.
    """
    hypothesis = ObjectHypothesis()
    hypothesis.annotations.append(
        PoseAnnotation(translation=[0.0, 0.0, 1.0], rotation=[0.0, 0.0, 0.0, 1.0])
    )

    with pytest.raises(MissingModelLabel):
        configured_connector.associate_hypotheses_with_beliefs(
            [hypothesis], [], semdt_cas
        )

    assert world.get_object_belief_states() == {}


# %% Validation before mutation


@pytest.mark.parametrize("invalid_pose", list(InvalidPose))
def test_invalid_pose_after_valid_hypothesis_preserves_all_persistent_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    invalid_pose: InvalidPose,
    belief_world_snapshot: BeliefWorldSnapshot,
) -> None:
    """
    A later invalid pose cannot create or update an earlier valid belief.
    """
    valid_hypothesis = observation_example.create_hypothesis()
    valid_hypothesis.annotations[-1].translation[0] += 1.0
    invalid_hypothesis = observation_example.create_hypothesis()
    invalid_pose.invalidate(invalid_hypothesis)

    with pytest.raises(InvalidObservationPose):
        configured_connector.associate_hypotheses_with_beliefs(
            [valid_hypothesis, invalid_hypothesis],
            list(world.get_object_belief_states().values()),
            captured_cas,
        )

    assert BeliefWorldSnapshot.capture() == belief_world_snapshot


@pytest.mark.parametrize(
    "capture_rejection",
    [
        CaptureRejection(InvalidCapture.MISSING_WORLD_FRAME, InvalidObservationFrame),
        CaptureRejection(InvalidCapture.MISSING_CAMERA_FRAME, InvalidObservationFrame),
        CaptureRejection(InvalidCapture.AMBIGUOUS_FRAMES, InvalidObservationFrame),
        CaptureRejection(InvalidCapture.UNKNOWN_WORLD_FRAME, InvalidObservationFrame),
        CaptureRejection(InvalidCapture.MISSING_CAPTURE_TIME, InvalidCaptureMetadata),
        CaptureRejection(InvalidCapture.NEGATIVE_CAPTURE_TIME, InvalidCaptureMetadata),
        CaptureRejection(
            InvalidCapture.CONFLICTING_CAPTURE_TIME, InvalidCaptureMetadata
        ),
        CaptureRejection(
            InvalidCapture.CONFLICTING_CAMERA_FRAME, InvalidObservationFrame
        ),
        CaptureRejection(InvalidCapture.MISSING_TRANSFORM, InvalidCameraTransform),
        CaptureRejection(InvalidCapture.REVERSED_TRANSFORM, InvalidCameraTransform),
        CaptureRejection(InvalidCapture.NONRIGID_TRANSFORM, InvalidCameraTransform),
        CaptureRejection(InvalidCapture.NONFINITE_TRANSFORM, InvalidCameraTransform),
    ],
)
def test_invalid_capture_preserves_all_persistent_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    capture_rejection: CaptureRejection,
    belief_world_snapshot: BeliefWorldSnapshot,
) -> None:
    """
    Missing or contradictory capture metadata cannot partially change beliefs.
    """
    capture_rejection.invalid_capture.invalidate(captured_cas, observation_example)

    with pytest.raises(capture_rejection.exception_type):
        configured_connector.associate_hypotheses_with_beliefs(
            [
                observation_example.create_hypothesis(),
                observation_example.create_hypothesis(),
            ],
            list(world.get_object_belief_states().values()),
            captured_cas,
        )

    assert BeliefWorldSnapshot.capture() == belief_world_snapshot


@pytest.mark.parametrize(
    "classification_rejection",
    [
        ClassificationRejection([], MissingModelLabel),
        ClassificationRejection([ModelLabel.UNKNOWN], UnknownModelLabel),
        ClassificationRejection(
            [ModelLabel.BLOCK, ModelLabel.PANEL], ConflictingModelLabels
        ),
    ],
)
def test_invalid_labels_after_valid_hypothesis_preserve_all_persistent_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    classification_rejection: ClassificationRejection,
    belief_world_snapshot: BeliefWorldSnapshot,
) -> None:
    """
    Semantic validation rejects a later hypothesis before applying earlier ones.
    """
    invalid_hypothesis = ObjectHypothesis(
        annotations=[
            *(
                Classification(classname=label)
                for label in classification_rejection.model_labels
            ),
            deepcopy(observation_example.camera_pose),
        ]
    )

    with pytest.raises(classification_rejection.exception_type):
        configured_connector.associate_hypotheses_with_beliefs(
            [observation_example.create_hypothesis(), invalid_hypothesis],
            list(world.get_object_belief_states().values()),
            captured_cas,
        )

    assert BeliefWorldSnapshot.capture() == belief_world_snapshot


@pytest.mark.parametrize("declared_frame", [None, ModelLabel.UNKNOWN, ""])
def test_unsupported_stamped_frame_after_valid_hypothesis_preserves_all_persistent_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    declared_frame: str | None,
    belief_world_snapshot: BeliefWorldSnapshot,
) -> None:
    """
    Stamped poses require an explicit supported frame before any accepted changes.
    """
    pose = StampedPoseAnnotation(
        translation=list(observation_example.camera_pose.translation),
        rotation=list(observation_example.camera_pose.rotation),
        frame=cast(str, declared_frame),
        timestamp=deepcopy(observation_example.capture_time),
    )

    with pytest.raises(InvalidObservationFrame):
        configured_connector.associate_hypotheses_with_beliefs(
            [
                observation_example.create_hypothesis(),
                observation_example.create_hypothesis(pose),
            ],
            list(world.get_object_belief_states().values()),
            captured_cas,
        )

    assert BeliefWorldSnapshot.capture() == belief_world_snapshot


@pytest.mark.parametrize(
    "timestamp", [None, Time(sec=-1), Time(nanosec=S_TO_NS), Time()]
)
def test_invalid_stamped_capture_after_valid_hypothesis_preserves_all_persistent_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    timestamp: Time | None,
    belief_world_snapshot: BeliefWorldSnapshot,
) -> None:
    """
    Missing, invalid, or contradictory stamped capture times reject the observation.
    """
    pose = StampedPoseAnnotation(
        translation=list(observation_example.camera_pose.translation),
        rotation=list(observation_example.camera_pose.rotation),
        frame=cast(str, captured_cas.camera_frame),
        timestamp=cast(Time, timestamp),
    )

    with pytest.raises(InvalidCaptureMetadata):
        configured_connector.associate_hypotheses_with_beliefs(
            [
                observation_example.create_hypothesis(),
                observation_example.create_hypothesis(pose),
            ],
            list(world.get_object_belief_states().values()),
            captured_cas,
        )

    assert BeliefWorldSnapshot.capture() == belief_world_snapshot


# %% Camera poses


def test_plain_camera_pose_preserves_capture_time_and_mesh_origin(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    World annotations describe the mesh origin at capture, with nanosecond precision.
    """
    hypothesis = observation_example.create_hypothesis()

    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)

    accepted_pose = hypothesis.annotations[-1]
    assert isinstance(accepted_pose, StampedPoseAnnotation)
    assert accepted_pose.translation == pytest.approx(
        observation_example.world_pose.translation
    )
    assert accepted_pose.rotation == pytest.approx(
        observation_example.world_pose.rotation
    )
    assert accepted_pose.frame == captured_cas.world_frame
    assert accepted_pose.timestamp == observation_example.capture_time


@pytest.mark.parametrize("pose_frame", [CASViews.CAMERA_FRAME, CASViews.WORLD_FRAME])
def test_stamped_pose_is_resolved_once_without_changing_capture_metadata(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    pose_frame: CASViews,
) -> None:
    """
    Stamped world poses need no camera transform; camera poses are converted once.
    """
    source_pose = (
        observation_example.camera_pose
        if pose_frame == CASViews.CAMERA_FRAME
        else observation_example.world_pose
    )
    pose = StampedPoseAnnotation(
        translation=list(source_pose.translation),
        rotation=list(source_pose.rotation),
        frame=captured_cas.get(pose_frame),
        timestamp=deepcopy(observation_example.capture_time),
    )
    if pose_frame == CASViews.WORLD_FRAME:
        captured_cas.views.pop(CASViews.CAMERA_TO_WORLD_TRANSFORM)
        captured_cas.views.pop(CASViews.CAMERA_FRAME)
    hypothesis = observation_example.create_hypothesis(pose)
    original_annotations = deepcopy(hypothesis.annotations)

    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)
    first_annotations = deepcopy(hypothesis.annotations)
    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)

    accepted_pose = hypothesis.annotations[-1]
    assert isinstance(accepted_pose, StampedPoseAnnotation)
    assert accepted_pose.translation == pytest.approx(
        observation_example.world_pose.translation
    )
    assert accepted_pose.rotation == pytest.approx(
        observation_example.world_pose.rotation
    )
    assert accepted_pose.timestamp == observation_example.capture_time
    assert accepted_pose.frame == captured_cas.world_frame
    assert hypothesis.annotations == first_annotations
    assert hypothesis.annotations[: len(original_annotations)] == original_annotations


@pytest.mark.parametrize(
    "classification_rejection",
    [
        ClassificationRejection([ModelLabel.UNKNOWN], UnknownModelLabel),
        ClassificationRejection(
            [ModelLabel.BLOCK, ModelLabel.PANEL], ConflictingModelLabels
        ),
        ClassificationRejection(
            [ModelLabel.BLOCK, ModelLabel.UNKNOWN], UnknownModelLabel
        ),
    ],
)
def test_invalid_model_labels_are_rejected_before_creating_beliefs(
    configured_connector: SemanticDigitalTwinConnector,
    semdt_cas: CAS,
    classification_rejection: ClassificationRejection,
) -> None:
    """
    Unknown or conflicting kinds cannot become persistent beliefs.
    """
    hypothesis = ObjectHypothesis()
    hypothesis.annotations.extend(
        Classification(classname=label)
        for label in classification_rejection.model_labels
    )
    hypothesis.annotations.append(
        PoseAnnotation(translation=[0.0, 0.0, 1.0], rotation=[0.0, 0.0, 0.0, 1.0])
    )

    with pytest.raises(classification_rejection.exception_type):
        configured_connector.associate_hypotheses_with_beliefs(
            [hypothesis], [], semdt_cas
        )

    assert world.get_object_belief_states() == {}


# %% Accepted metadata


def test_same_model_classifications_are_accepted_independently_of_confidence(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    Association thresholds do not reject an unambiguous classification.
    """
    hypothesis = observation_example.create_hypothesis()
    hypothesis.annotations.append(
        Classification(classname=ModelLabel.BLOCK, confidence=0.0)
    )
    configured_connector.descriptor.parameters.confidence_threshold = 1.0

    accepted = configured_connector.associate_hypotheses_with_beliefs(
        [hypothesis], [], captured_cas
    )

    assert [belief.latest_hypothesis for _, belief in accepted] == [hypothesis]


def test_reprocessing_plain_pose_preserves_source_and_derived_annotations(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    A plain camera pose and its derived world pose coexist without repeated conversion.
    """
    hypothesis = observation_example.create_hypothesis()
    source_annotations = deepcopy(hypothesis.annotations)
    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)
    prepared_annotations = deepcopy(hypothesis.annotations)

    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)

    assert hypothesis.annotations == prepared_annotations
    assert hypothesis.annotations[: len(source_annotations)] == source_annotations


def test_camera_info_capture_time_is_used_when_data_timestamp_is_absent(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    Camera-info capture metadata is a supported precise CAS timestamp source.
    """
    captured_cas.views.pop(CASViews.DATA_TIMESTAMP)
    camera_info = CameraInfo()
    camera_info.header.frame_id = captured_cas.camera_frame
    camera_info.header.stamp = deepcopy(observation_example.capture_time)
    captured_cas.set(CASViews.CAMERA_INFO, camera_info)
    hypothesis = observation_example.create_hypothesis()

    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)

    assert hypothesis.annotations[-1].timestamp == camera_info.header.stamp


def test_newest_supported_pose_supersedes_an_older_invalid_pose(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    Only the newest supported pose determines the accepted observation pose.
    """
    hypothesis = observation_example.create_hypothesis()
    hypothesis.annotations.insert(0, PoseAnnotation())

    configured_connector.add_world_pose_annotations([hypothesis], captured_cas)

    assert hypothesis.annotations[-1].translation == pytest.approx(
        observation_example.world_pose.translation
    )
