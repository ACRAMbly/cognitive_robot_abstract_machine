# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from math import isfinite
from numbers import Integral, Real

import numpy as np
from builtin_interfaces.msg import Time
from rclpy.constants import S_TO_NS
from rclpy.time import Time as ROSTime
from sensor_msgs.msg import CameraInfo
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.exceptions import (
    ConflictingModelLabels,
    MissingModelLabel,
    UnknownModelLabel,
    InvalidObservationPose,
    InvalidObservationFrame,
    InvalidCaptureMetadata,
    InvalidCameraTransform,
    ObservationProblem,
)
from robokudo import world
from robokudo.cas import CASViews
from robokudo.types.annotation import (
    Classification,
    PoseAnnotation,
    StampedPoseAnnotation,
)
from robokudo.utils.annotator_helper import transform_pose_from_camera_to_world

if TYPE_CHECKING:
    from robokudo.types.scene import ObjectHypothesis
    from robokudo.cas import CAS
    from semantic_digital_twin.semantic_annotations.mixins import HasRootBody


# %% Configured semantic acceptance


@dataclass
class SemanticObservationAcceptance:
    """
    Resolve observation model kinds through application-supplied semantics.
    """

    semantic_types: dict[str, type[HasRootBody]]
    """Configured classification labels and their concrete semantic types."""

    def prepare(
        self, hypothesis: ObjectHypothesis, cas: CAS, source: str
    ) -> PreparedObservation:
        """
        Validate one configured hypothesis and resolve its captured world pose.
        """
        semantic_type = self.resolve_semantic_type(hypothesis)
        observation_pose = ObservationPose.from_hypothesis(hypothesis)
        return PreparedObservation(
            hypothesis, semantic_type, observation_pose.to_world(cas, source)
        )

    def resolve_semantic_type(self, hypothesis: ObjectHypothesis) -> type[HasRootBody]:
        """
        Accept one unambiguous model kind independently of confidence.
        """
        model_labels = frozenset(
            annotation.classname
            for annotation in hypothesis.annotations
            if isinstance(annotation, Classification)
        )
        if not model_labels or "" in model_labels:
            raise MissingModelLabel()
        for model_label in model_labels:
            if model_label not in self.semantic_types:
                raise UnknownModelLabel(model_label)
        if len(model_labels) != 1:
            raise ConflictingModelLabels(model_labels)
        return self.semantic_types[next(iter(model_labels))]


@dataclass
class PreparedObservation:
    """
    A semantically accepted observation ready for persistent belief changes.
    """

    hypothesis: ObjectHypothesis
    """
    Original detection whose identity is local to this observation.
    """

    semantic_type: type[HasRootBody]
    """
    Application-supplied semantic kind resolved from the model label.
    """

    world_pose: StampedPoseAnnotation
    """
    Mesh-origin pose in the declared world frame at capture time.
    """


# %% Capture metadata


@dataclass
class ObservationCapture:
    """
    Consistent source frames and precise capture time for one observation.
    """

    world_frame: str
    """
    Declared world coordinate frame.
    """

    camera_frame: str | None
    """
    Declared camera coordinate frame when available.
    """

    timestamp: Time
    """
    Original capture time with integer nanosecond precision.
    """

    @staticmethod
    def read_timestamp(timestamp: Time) -> int:
        """
        Validate a ROS timestamp and retain its integer nanosecond precision.
        """
        if (
            not isinstance(timestamp, Time)
            or timestamp.sec < 0
            or not 0 <= timestamp.nanosec < S_TO_NS
        ):
            raise InvalidCaptureMetadata(ObservationProblem.INVALID_TIME)
        return ROSTime.from_msg(timestamp).nanoseconds

    @classmethod
    def from_cas(cls, cas: CAS) -> ObservationCapture:
        """
        Require declared world coordinates and coherent capture timestamps.
        """
        if (
            not cas.world_frame
            or len(world.world_instance().get_bodies_by_name(cas.world_frame)) != 1
        ):
            raise InvalidObservationFrame(ObservationProblem.WORLD_FRAME)
        if cas.world_frame == cas.camera_frame:
            raise InvalidObservationFrame(ObservationProblem.AMBIGUOUS_FRAME)

        timestamp = cas.data_timestamp
        if timestamp is not None and (
            not isinstance(timestamp, Integral)
            or isinstance(timestamp, bool)
            or timestamp < 0
            or timestamp // S_TO_NS > np.iinfo(np.int32).max
        ):
            raise InvalidCaptureMetadata(ObservationProblem.INVALID_TIME)
        if cas.contains(CASViews.CAMERA_INFO):
            camera_info = cas.get(CASViews.CAMERA_INFO)
            if not isinstance(camera_info, CameraInfo):
                raise InvalidCaptureMetadata(ObservationProblem.INVALID_TIME)
            camera_timestamp = cls.read_timestamp(camera_info.header.stamp)
            if timestamp is not None and timestamp != camera_timestamp:
                raise InvalidCaptureMetadata(ObservationProblem.CONFLICTING_TIME)
            timestamp = camera_timestamp
            if (
                camera_info.header.frame_id
                and cas.camera_frame
                and camera_info.header.frame_id != cas.camera_frame
            ):
                raise InvalidObservationFrame(ObservationProblem.CONFLICTING_FRAME)
        if timestamp is None:
            raise InvalidCaptureMetadata(ObservationProblem.MISSING_TIME)
        return cls(
            cas.world_frame,
            cas.camera_frame,
            ROSTime(nanoseconds=int(timestamp)).to_msg(),
        )

    def validate_camera_transform(self, cas: CAS) -> None:
        """
        Require a finite rigid transform between the declared camera and world.
        """
        if not self.camera_frame:
            raise InvalidObservationFrame(ObservationProblem.CAMERA_FRAME)
        transform = cas.camera_to_world_transform
        if transform is None:
            raise InvalidCameraTransform(ObservationProblem.MISSING_TRANSFORM)
        if (
            transform.reference_frame is None
            or transform.child_frame is None
            or str(transform.reference_frame.name) != self.world_frame
            or str(transform.child_frame.name) != self.camera_frame
        ):
            raise InvalidCameraTransform(ObservationProblem.TRANSFORM_FRAMES)
        matrix = transform.to_np()
        rotation = matrix[
            : PoseComponentCount.TRANSLATION, : PoseComponentCount.TRANSLATION
        ]
        if (
            not np.isfinite(matrix).all()
            or not np.allclose(matrix[-1], np.eye(matrix.shape[0])[-1])
            or not np.allclose(
                rotation.T @ rotation, np.eye(PoseComponentCount.TRANSLATION)
            )
            or not np.isclose(np.linalg.det(rotation), 1.0)
        ):
            raise InvalidCameraTransform(ObservationProblem.INVALID_TRANSFORM)


# %% Supported poses


class PoseComponentCount(IntEnum):
    """
    Coordinate counts in the supported translation and quaternion formats.
    """

    TRANSLATION = 3
    QUATERNION = 4


@dataclass
class ObservationPose:
    """
    The newest supported, finite mesh-origin pose in one hypothesis.
    """

    pose: PoseAnnotation | StampedPoseAnnotation
    """
    Original observation pose with its declared capture metadata.
    """

    @staticmethod
    def get_latest_pose_annotation(
        object_hypothesis: ObjectHypothesis,
    ) -> PoseAnnotation | StampedPoseAnnotation | None:
        """
        Return the newest supported plain or stamped observation pose.
        """
        return next(
            (
                annotation
                for annotation in reversed(object_hypothesis.annotations)
                if type(annotation) in (PoseAnnotation, StampedPoseAnnotation)
            ),
            None,
        )

    @classmethod
    def from_hypothesis(cls, hypothesis: ObjectHypothesis) -> ObservationPose:
        """
        Select and validate the newest plain or stamped pose.
        """
        pose = cls.get_latest_pose_annotation(hypothesis)
        if pose is None or pose.translation is None or pose.rotation is None:
            raise InvalidObservationPose()
        if (
            len(pose.translation) != PoseComponentCount.TRANSLATION
            or len(pose.rotation) != PoseComponentCount.QUATERNION
        ):
            raise InvalidObservationPose()
        if not all(
            isinstance(coordinate, Real) and isfinite(coordinate)
            for coordinate in (*pose.translation, *pose.rotation)
        ):
            raise InvalidObservationPose()
        quaternion_norm = np.linalg.norm(pose.rotation)
        if not isfinite(quaternion_norm) or quaternion_norm == 0:
            raise InvalidObservationPose()
        return cls(pose)

    def to_world(self, cas: CAS, source: str) -> StampedPoseAnnotation:
        """
        Resolve a mesh-origin pose exactly once while retaining capture metadata.
        """
        capture = ObservationCapture.from_cas(cas)
        if isinstance(self.pose, StampedPoseAnnotation):
            if not self.pose.frame or self.pose.frame not in (
                capture.camera_frame,
                capture.world_frame,
            ):
                raise InvalidObservationFrame(ObservationProblem.UNSUPPORTED_FRAME)
            if capture.read_timestamp(self.pose.timestamp) != capture.read_timestamp(
                capture.timestamp
            ):
                raise InvalidCaptureMetadata(ObservationProblem.CONFLICTING_TIME)
            if self.pose.frame == capture.world_frame:
                return self.pose

        capture.validate_camera_transform(cas)
        world_pose = transform_pose_from_camera_to_world(
            cas,
            PoseAnnotation(
                translation=self.pose.translation, rotation=self.pose.rotation
            ),
        )
        return StampedPoseAnnotation(
            source=source,
            translation=world_pose.translation,
            rotation=world_pose.rotation,
            frame=capture.world_frame,
            timestamp=capture.timestamp,
        )
