# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, StrEnum

import numpy
from scipy.spatial.transform import Rotation

from robokudo_acrambly.playback.clock import TimeUnit
from robokudo_acrambly.playback.schedule import PlaybackTiming
from robokudo_acrambly.playback.motion import (
    AbsenceInterval,
    MotionBoundary,
    MotionDefinition,
    ObjectMotion,
    PoseKeyframe,
    VisibilityBoundary,
)
from robokudo_acrambly.scripted_observations.models import TetrisModel
from robokudo.types.tf import Pose


# %% Camera descriptor


class CameraFrame(StrEnum):
    """Optical reference frame with X right, Y down, and Z forward."""

    OPTICAL = "scripted_camera_optical_frame"


@dataclass
class CameraSettings:
    """Editable calibration of the fixed optical camera."""

    image_width: int = 640
    """Horizontal image extent in pixels."""

    image_height: int = 480
    """Vertical image extent in pixels."""

    horizontal_field_of_view_degrees: float = 60.0
    """Horizontal angular extent of the camera image."""

    camera_frame: str = CameraFrame.OPTICAL
    """Reference frame locating every object pose in an observation."""


# %% Scene descriptor


class AppearanceBoundary(IntEnum):
    """Default offsets separating visible and absent observations."""

    FIRST_DISAPPEARS = VisibilityBoundary.PARTIAL_ABSENCE_START
    """Capture offset at which the first block becomes absent."""

    SECOND_DISAPPEARS = VisibilityBoundary.TOTAL_ABSENCE_START
    """Capture offset at which the second block also becomes absent."""

    BOTH_REAPPEAR = VisibilityBoundary.REAPPEARANCE
    """Capture offset at which both blocks become visible again."""


@dataclass
class BlockState(ObjectMotion):
    """One block's camera-relative motion and discrete visibility."""

    model: TetrisModel
    """CAD model describing this block's object kind."""

    pose: Pose
    """Camera-relative CAD-origin pose in metres and xyzw quaternion order."""

    keyframes: list[PoseKeyframe] = field(default_factory=list)
    """Scheduled poses after the initial pose, with equal poses defining holds."""

    absence_intervals: list[AbsenceInterval] = field(default_factory=list)
    """Half-open capture intervals in which this block has no hypothesis."""

    def describe_motion(self) -> MotionDefinition:
        return MotionDefinition(
            self.model.value.identifier,
            self.pose,
            self.keyframes,
            self.absence_intervals,
        )


# %% Default CAD-origin poses


@dataclass
class ScenarioPoses:
    """Editable endpoints for the default moving and stationary blocks."""

    initial_pose: Pose = field(
        default_factory=lambda: Pose(
            translation=[-0.30, -0.15, 1.00],
            rotation=Rotation.identity().as_quat().tolist(),
        )
    )
    """Initial camera-relative CAD origin of the moving block."""

    translated_pose: Pose = field(
        default_factory=lambda: Pose(
            translation=[-0.20, -0.15, 1.00],
            rotation=Rotation.identity().as_quat().tolist(),
        )
    )
    """Camera-relative CAD origin reached by translation."""

    rotation_angle_radians: float = numpy.pi / 2.0
    """Positive rotation about camera Z after translation."""

    stationary_pose: Pose = field(
        default_factory=lambda: Pose(
            translation=[0.05, -0.15, 1.00],
            rotation=Rotation.identity().as_quat().tolist(),
        )
    )
    """Camera-relative CAD origin of the stationary reference."""


# %% Complete scenario


@dataclass
class TetrisScenario:
    """Editable scene for synthetic Tetris observations."""

    camera: CameraSettings = field(default_factory=CameraSettings)
    """Calibration and reference frame of the synthetic camera."""

    timing: PlaybackTiming = field(default_factory=PlaybackTiming)
    """Duration and capture cadence of the observation sequence."""

    blocks: list[BlockState] = field(
        default_factory=lambda: TetrisScenario.create_blocks()
    )
    """Blocks with editable camera-relative keyframes and absence intervals."""

    @staticmethod
    def create_blocks(poses: ScenarioPoses | None = None) -> list[BlockState]:
        """Create the initial moving block and its stationary reference."""
        if poses is None:
            poses = ScenarioPoses()
        initial_pose = poses.initial_pose
        translated_pose = poses.translated_pose
        rotated_pose = Pose(
            translation=list(translated_pose.translation),
            rotation=Rotation.from_rotvec([0.0, 0.0, poses.rotation_angle_radians])
            .as_quat()
            .tolist(),
        )
        return [
            BlockState(
                TetrisModel.FIRST_BLOCK,
                initial_pose,
                keyframes=[
                    PoseKeyframe(MotionBoundary.INITIAL_HOLD_FINISHED, initial_pose),
                    PoseKeyframe(MotionBoundary.TRANSLATION_FINISHED, translated_pose),
                    PoseKeyframe(
                        MotionBoundary.TRANSLATION_HOLD_FINISHED, translated_pose
                    ),
                    PoseKeyframe(MotionBoundary.ROTATION_FINISHED, rotated_pose),
                ],
                absence_intervals=[
                    AbsenceInterval(
                        AppearanceBoundary.FIRST_DISAPPEARS,
                        AppearanceBoundary.BOTH_REAPPEAR,
                    )
                ],
            ),
            BlockState(
                TetrisModel.SECOND_BLOCK,
                poses.stationary_pose,
                absence_intervals=[
                    AbsenceInterval(
                        AppearanceBoundary.SECOND_DISAPPEARS,
                        AppearanceBoundary.BOTH_REAPPEAR,
                    )
                ],
            ),
        ]
