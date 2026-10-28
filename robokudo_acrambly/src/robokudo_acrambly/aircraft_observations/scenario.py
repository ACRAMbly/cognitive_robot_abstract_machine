# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field

import numpy
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time
from typing_extensions import TYPE_CHECKING

from robokudo.annotators.exceptions import InvalidObservationFrame, ObservationProblem
from robokudo.annotators.semantic_observations import ObservationCapture
from robokudo.cas import CAS
from robokudo.types.tf import Pose
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.playback.schedule import PlaybackTiming
from robokudo_acrambly.playback.motion import (
    AbsenceInterval,
    MotionBoundary,
    MotionDefinition,
    ObjectMotion,
    PoseKeyframe,
    VisibilityBoundary,
)

if TYPE_CHECKING:
    from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix


# %% Frame configuration


@dataclass
class AircraftFrames:
    """
    Explicit camera calibration retained across independent captures.
    """

    world_frame: str
    """Declared world coordinate frame supplied by the application."""

    camera_frame: str
    """
    Optical camera frame with X right, Y down, and Z forward.
    """

    camera_to_world_transform: HomogeneousTransformationMatrix
    """Rigid transform from the camera frame into the declared world frame."""

    def validate(self) -> None:
        """
        Require distinct frames and explicit finite rigid calibration.
        """
        if not self.world_frame:
            raise InvalidObservationFrame(ObservationProblem.WORLD_FRAME)
        if self.world_frame == self.camera_frame:
            raise InvalidObservationFrame(ObservationProblem.AMBIGUOUS_FRAME)
        observation = CAS()
        observation.camera_to_world_transform = self.camera_to_world_transform
        ObservationCapture(
            self.world_frame, self.camera_frame, Time()
        ).validate_camera_transform(observation)

    def write_metadata(self, observation: CAS, capture_timestamp: int) -> None:
        """
        Write the configured frames and precise scheduled time into one capture.
        """
        observation.data_timestamp = capture_timestamp
        observation.world_frame = self.world_frame
        observation.camera_frame = self.camera_frame
        observation.camera_to_world_transform = self.camera_to_world_transform


# %% Independent part poses


@dataclass
class AircraftPartState(ObjectMotion):
    """
    One independently observed aircraft kind at its mesh-origin pose.
    """

    model: AircraftModel
    """
    Existing aircraft kind identifying the observed component.
    """

    pose: Pose
    """
    Camera-relative source mesh origin in metres and xyzw quaternion order.
    """

    keyframes: list[PoseKeyframe] = field(default_factory=list)
    """
    Scheduled mesh-origin poses after the initial capture.
    """

    absence_intervals: list[AbsenceInterval] = field(default_factory=list)
    """
    Half-open intervals omitting this part's detections.
    """

    def describe_motion(self) -> MotionDefinition:
        return MotionDefinition(
            self.model.value.label, self.pose, self.keyframes, self.absence_intervals
        )


@dataclass
class AircraftLayout:
    """
    Editable spacing of independent aircraft parts for mesh inspection.
    """

    columns: int = 4
    """
    Number of parts arranged along each camera-X row.
    """

    horizontal_spacing: float = 0.4
    """
    Metres between neighbouring mesh origins along camera X.
    """

    vertical_spacing: float = 0.3
    """
    Metres between neighbouring rows along camera Y.
    """

    first_origin: Pose = field(
        default_factory=lambda: Pose(
            translation=[-0.6, -0.3, 1.0], rotation=[0.0, 0.0, 0.0, 1.0]
        )
    )
    """
    Camera-relative mesh origin of the first configured part.
    """

    translation_distance: float = 0.1
    """
    Metres travelled along camera X between translation endpoints.
    """

    rotation_angle_radians: float = numpy.pi / 2.0
    """
    Positive camera-Z rotation following the translation hold.
    """

    partial_absence_models: frozenset[AircraftModel] = field(
        default_factory=lambda: frozenset(list(AircraftModel)[::2])
    )
    """
    Configured kinds omitted during the partial-absence interval.
    """

    def create_parts(self) -> list[AircraftPartState]:
        """
        Place each catalog kind at an independent camera-relative origin.
        """
        parts = []
        for index, model in enumerate(AircraftModel):
            initial_pose = Pose(
                translation=[
                    self.first_origin.translation[0]
                    + index % self.columns * self.horizontal_spacing,
                    self.first_origin.translation[1]
                    + index // self.columns * self.vertical_spacing,
                    self.first_origin.translation[2],
                ],
                rotation=list(self.first_origin.rotation),
            )
            translated_pose = Pose(
                translation=[
                    initial_pose.translation[0] + self.translation_distance,
                    *initial_pose.translation[1:],
                ],
                rotation=list(initial_pose.rotation),
            )
            rotated_pose = Pose(
                translation=list(translated_pose.translation),
                rotation=(
                    Rotation.from_rotvec([0.0, 0.0, self.rotation_angle_radians])
                    * Rotation.from_quat(translated_pose.rotation)
                )
                .as_quat()
                .tolist(),
            )
            parts.append(
                AircraftPartState(
                    model=model,
                    pose=initial_pose,
                    keyframes=[
                        PoseKeyframe(
                            MotionBoundary.INITIAL_HOLD_FINISHED, initial_pose
                        ),
                        PoseKeyframe(
                            MotionBoundary.TRANSLATION_FINISHED, translated_pose
                        ),
                        PoseKeyframe(
                            MotionBoundary.TRANSLATION_HOLD_FINISHED, translated_pose
                        ),
                        PoseKeyframe(MotionBoundary.ROTATION_FINISHED, rotated_pose),
                    ],
                    absence_intervals=[
                        AbsenceInterval(
                            (
                                VisibilityBoundary.PARTIAL_ABSENCE_START
                                if model in self.partial_absence_models
                                else VisibilityBoundary.TOTAL_ABSENCE_START
                            ),
                            VisibilityBoundary.REAPPEARANCE,
                        )
                    ],
                )
            )
        return parts


# %% Complete scenario


@dataclass
class AircraftScenario:
    """
    Editable independent aircraft parts in optical camera coordinates.
    """

    parts: list[AircraftPartState] = field(
        default_factory=lambda: AircraftLayout().create_parts()
    )
    """
    Configured aircraft kinds and their mesh-origin poses.
    """

    timing: PlaybackTiming = field(default_factory=PlaybackTiming)
    """
    Finite duration and cadence of aircraft observations.
    """
