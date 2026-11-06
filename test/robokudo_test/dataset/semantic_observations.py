# %% Imports

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from enum import StrEnum, auto

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo
import numpy as np
from rclpy.time import Time as ROSTime
from typing_extensions import TYPE_CHECKING, cast
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody, IsPerceivable
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

from robokudo import world
from robokudo.cas import CASViews
from robokudo.types.annotation import (
    Classification,
    PoseAnnotation,
    StampedPoseAnnotation,
)
from robokudo.types.scene import ObjectHypothesis

if TYPE_CHECKING:
    from uuid import UUID

    from robokudo.cas import CAS
    from robokudo.types.core import Annotation
    from robokudo.types.belief_state import ObjectBeliefState


# %% Local semantic kinds


class ModelLabel(StrEnum):
    """
    Model kinds for generic connector observations.
    """

    BLOCK = "block"
    PANEL = "panel"
    UNKNOWN = "unknown"


@dataclass(eq=False)
class ObservedBlock(HasRootBody, IsPerceivable):
    """
    A locally defined perceivable block with a root body.
    """


@dataclass(eq=False)
class ObservedPanel(HasRootBody, IsPerceivable):
    """
    A locally defined perceivable panel with a root body.
    """


# %% Deterministic capture


@dataclass
class ObservationExample:
    """
    A camera rotated a quarter turn about z and translated from the world.
    """

    capture_time: Time = field(
        default_factory=lambda: Time(sec=1_700_000_000, nanosec=987_654_321)
    )
    """
    Capture timestamp whose precision would be lost in a float conversion.
    """

    camera_pose: PoseAnnotation = field(
        default_factory=lambda: PoseAnnotation(
            translation=[1.0, 2.0, 3.0], rotation=[0.0, 0.0, 0.0, 1.0]
        )
    )
    """
    Mesh origin expressed in camera coordinates.
    """

    world_pose: PoseAnnotation = field(
        default_factory=lambda: PoseAnnotation(
            translation=[8.0, 21.0, 33.0], rotation=[0.0, 0.0, 0.5**0.5, 0.5**0.5]
        )
    )
    """
    Independently worked world pose of the same mesh origin.
    """

    def configure_capture(self, cas: CAS) -> None:
        """
        Set precise capture metadata and the nonidentity camera transform.
        """
        cas.data_timestamp = ROSTime.from_msg(self.capture_time).nanoseconds
        cas.camera_to_world_transform = (
            HomogeneousTransformationMatrix.from_xyz_quaternion(
                pos_x=10.0,
                pos_y=20.0,
                pos_z=30.0,
                quat_x=0.0,
                quat_y=0.0,
                quat_z=0.5**0.5,
                quat_w=0.5**0.5,
                reference_frame=world.world_instance().get_body_by_name(
                    cas.world_frame
                ),
                child_frame=world.world_instance().get_body_by_name(cas.camera_frame),
            )
        )

    def create_hypothesis(
        self, pose: PoseAnnotation | StampedPoseAnnotation | None = None
    ) -> ObjectHypothesis:
        """
        Create a configured observation without optional bounding-box evidence.
        """
        return ObjectHypothesis(
            annotations=[
                Classification(classname=ModelLabel.BLOCK),
                deepcopy(self.camera_pose if pose is None else pose),
            ]
        )


# %% Persistent state comparisons


@dataclass
class BeliefSnapshot:
    """
    Identity, accepted observations, pose, and geometry of one persistent belief.
    """

    belief_identity: int
    """
    Identity of the persistent belief instance.
    """

    body_identity: int
    """
    Identity of the persistent body instance.
    """

    hypothesis_identities: list[int]
    """
    Identity of every retained observation.
    """

    accepted_annotations: list[list[Annotation]]
    """
    Copies of accepted poses, capture metadata, and classifications.
    """

    body_pose: list[list[float]]
    """
    Accepted body origin in the world.
    """

    visual_shape_identities: list[int]
    """
    Identities and count of the body's visual shapes.
    """

    collision_shape_identities: list[int]
    """
    Identities and count of the body's collision shapes.
    """

    @classmethod
    def from_belief(cls, belief: ObjectBeliefState) -> BeliefSnapshot:
        """
        Capture observable accepted state without retaining mutable values.
        """
        return cls(
            id(belief),
            id(belief.body),
            [id(hypothesis) for hypothesis in belief.hypotheses],
            [deepcopy(hypothesis.annotations) for hypothesis in belief.hypotheses],
            belief.body.global_transform.to_np().tolist(),
            [id(shape) for shape in belief.body.visual.shapes],
            [id(shape) for shape in belief.body.collision.shapes],
        )


@dataclass
class BeliefWorldSnapshot:
    """
    Exact persistent belief state and recorded world changes.
    """

    beliefs: dict[UUID, BeliefSnapshot]
    """
    Accepted object state keyed by persistent UUID.
    """

    entity_identities: list[int]
    """
    Identities of all kinematic entities.
    """

    connection_identities: list[int]
    """
    Identities of all world connections.
    """

    semantic_identities: list[int]
    """
    Identities of all semantic annotations.
    """

    degree_of_freedom_values: list[float]
    """
    Full world state, including derivatives.
    """

    modification_block_identities: list[int]
    """
    Recorded model changes before and after rejection.
    """

    @classmethod
    def capture(cls) -> BeliefWorldSnapshot:
        """
        Snapshot the local world through its public interfaces.
        """
        belief_world = world.world_instance()
        return cls(
            {
                identifier: BeliefSnapshot.from_belief(belief)
                for identifier, belief in world.get_object_belief_states().items()
            },
            [id(entity) for entity in belief_world.kinematic_structure_entities],
            [id(connection) for connection in belief_world.connections],
            [id(annotation) for annotation in belief_world.semantic_annotations],
            list(belief_world.state.data),
            [id(block) for block in belief_world.modification_history],
        )


# %% Invalid pose examples


class InvalidPose(StrEnum):
    """
    Unsupported or malformed observation poses.
    """

    MISSING = auto()
    MISSING_TRANSLATION = auto()
    MISSING_QUATERNION = auto()
    SHORT_TRANSLATION = auto()
    SHORT_QUATERNION = auto()
    ZERO_QUATERNION = auto()
    NONFINITE_TRANSLATION = auto()
    NONFINITE_QUATERNION = auto()

    def invalidate(self, hypothesis: ObjectHypothesis) -> None:
        """
        Replace the otherwise valid pose with this invalid input.
        """
        pose = hypothesis.annotations[-1]
        match self:
            case InvalidPose.MISSING:
                hypothesis.annotations.pop()
            case InvalidPose.MISSING_TRANSLATION:
                pose.translation = None
            case InvalidPose.MISSING_QUATERNION:
                pose.rotation = None
            case InvalidPose.SHORT_TRANSLATION:
                pose.translation.pop()
            case InvalidPose.SHORT_QUATERNION:
                pose.rotation.pop()
            case InvalidPose.ZERO_QUATERNION:
                pose.rotation = [0.0] * len(pose.rotation)
            case InvalidPose.NONFINITE_TRANSLATION:
                pose.translation[0] = float("nan")
            case InvalidPose.NONFINITE_QUATERNION:
                pose.rotation[0] = float("inf")


# %% Invalid capture metadata


class InvalidCapture(StrEnum):
    """
    Missing or contradictory capture and coordinate metadata.
    """

    MISSING_WORLD_FRAME = auto()
    MISSING_CAMERA_FRAME = auto()
    AMBIGUOUS_FRAMES = auto()
    UNKNOWN_WORLD_FRAME = auto()
    MISSING_CAPTURE_TIME = auto()
    NEGATIVE_CAPTURE_TIME = auto()
    CONFLICTING_CAPTURE_TIME = auto()
    CONFLICTING_CAMERA_FRAME = auto()
    MISSING_TRANSFORM = auto()
    REVERSED_TRANSFORM = auto()
    NONRIGID_TRANSFORM = auto()
    NONFINITE_TRANSFORM = auto()

    def invalidate(self, cas: CAS, example: ObservationExample) -> None:
        """
        Change one required part of otherwise coherent CAS metadata.
        """
        match self:
            case InvalidCapture.MISSING_WORLD_FRAME:
                cas.views.pop(CASViews.WORLD_FRAME)
            case InvalidCapture.MISSING_CAMERA_FRAME:
                cas.views.pop(CASViews.CAMERA_FRAME)
            case InvalidCapture.AMBIGUOUS_FRAMES:
                cas.camera_frame = cast(str, cas.world_frame)
            case InvalidCapture.UNKNOWN_WORLD_FRAME:
                cas.world_frame = ModelLabel.UNKNOWN
            case InvalidCapture.MISSING_CAPTURE_TIME:
                cas.views.pop(CASViews.DATA_TIMESTAMP)
            case InvalidCapture.NEGATIVE_CAPTURE_TIME:
                cas.data_timestamp = -1
            case InvalidCapture.CONFLICTING_CAPTURE_TIME:
                camera_info = CameraInfo()
                camera_info.header.frame_id = cas.camera_frame
                camera_info.header.stamp = deepcopy(example.capture_time)
                camera_info.header.stamp.nanosec -= 1
                cas.set(CASViews.CAMERA_INFO, camera_info)
            case InvalidCapture.CONFLICTING_CAMERA_FRAME:
                camera_info = CameraInfo()
                camera_info.header.frame_id = cas.world_frame
                camera_info.header.stamp = deepcopy(example.capture_time)
                cas.set(CASViews.CAMERA_INFO, camera_info)
            case InvalidCapture.MISSING_TRANSFORM:
                cas.views.pop(CASViews.CAMERA_TO_WORLD_TRANSFORM)
            case InvalidCapture.REVERSED_TRANSFORM:
                transform = cast(
                    HomogeneousTransformationMatrix, cas.camera_to_world_transform
                )
                cas.camera_to_world_transform = HomogeneousTransformationMatrix(
                    transform.to_np(),
                    reference_frame=transform.child_frame,
                    child_frame=transform.reference_frame,
                )
            case InvalidCapture.NONRIGID_TRANSFORM | InvalidCapture.NONFINITE_TRANSFORM:
                transform = cast(
                    HomogeneousTransformationMatrix, cas.camera_to_world_transform
                )
                matrix = transform.to_np()
                matrix[0, 0] = (
                    2.0 if self == InvalidCapture.NONRIGID_TRANSFORM else np.nan
                )
                cas.camera_to_world_transform = HomogeneousTransformationMatrix(
                    matrix,
                    reference_frame=transform.reference_frame,
                    child_frame=transform.child_frame,
                )


# %% Validation expectations


@dataclass
class CaptureRejection:
    """
    Invalid capture metadata and its required validation exception.
    """

    invalid_capture: InvalidCapture
    """
    Single inconsistency introduced into a captured observation.
    """

    exception_type: type[Exception]
    """
    Exception identifying the invalid metadata category.
    """


@dataclass
class ClassificationRejection:
    """
    Model classifications and their required acceptance exception.
    """

    model_labels: list[ModelLabel]
    """
    Classifications assigned to one object hypothesis.
    """

    exception_type: type[Exception]
    """
    Exception identifying the invalid classification category.
    """
