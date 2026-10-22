# %% Imports

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum, StrEnum

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from robokudo.scripted_observations.clock import TimeUnit
from robokudo.scripted_observations.exceptions import InvalidPoseKeyframes
from robokudo.scripted_observations.models import TetrisModel
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

    frame: str = CameraFrame.OPTICAL
    """Reference frame locating every object pose in an observation."""


# %% Scene descriptor


@dataclass
class PlaybackTiming:
    """Capture cadence of the finite observation sequence."""

    duration_nanoseconds: int = 30 * TimeUnit.NANOSECONDS_PER_SECOND
    """Elapsed capture time from the first observation through the final one."""

    sample_period_nanoseconds: int = TimeUnit.NANOSECONDS_PER_SECOND // 10
    """Scheduled interval between consecutive observations."""


class MotionBoundary(IntEnum):
    """Default capture offsets separating motion from stationary holds."""

    INITIAL_HOLD_FINISHED = 3 * TimeUnit.NANOSECONDS_PER_SECOND
    TRANSLATION_FINISHED = 9 * TimeUnit.NANOSECONDS_PER_SECOND
    TRANSLATION_HOLD_FINISHED = 12 * TimeUnit.NANOSECONDS_PER_SECOND
    ROTATION_FINISHED = 18 * TimeUnit.NANOSECONDS_PER_SECOND


@dataclass
class PoseKeyframe:
    """An editable CAD-origin pose at a scheduled capture offset."""

    elapsed_nanoseconds: int
    """Elapsed capture time at which this pose is reached."""

    pose: Pose
    """Camera-relative endpoint pose in metres and xyzw quaternion order."""


@dataclass
class BlockState:
    """One visible block located by its original CAD origin."""

    model: TetrisModel
    """CAD model describing this block's object kind."""

    pose: Pose
    """Camera-relative CAD-origin pose in metres and xyzw quaternion order."""

    keyframes: list[PoseKeyframe] = field(default_factory=list)
    """Scheduled poses after the initial pose, with equal poses defining holds."""

    def validate_keyframes(self, timing: PlaybackTiming) -> None:
        """Reject unordered endpoints or endpoints absent from the capture schedule."""
        offsets = [0] + [keyframe.elapsed_nanoseconds for keyframe in self.keyframes]
        unordered = any(end <= start for start, end in zip(offsets, offsets[1:]))
        unsampled = any(
            offset <= timing.duration_nanoseconds
            and offset % timing.sample_period_nanoseconds != 0
            for offset in offsets
        )
        if unordered or unsampled:
            raise InvalidPoseKeyframes(
                self.model.value.identifier,
                offsets[1:],
                timing.sample_period_nanoseconds,
            )

    def interpolate_pose(self, elapsed_nanoseconds: int) -> Pose:
        """Return the camera-relative pose at an elapsed capture time."""
        previous = PoseKeyframe(0, self.pose)
        for keyframe in self.keyframes:
            if elapsed_nanoseconds >= keyframe.elapsed_nanoseconds:
                previous = keyframe
                continue
            if elapsed_nanoseconds == previous.elapsed_nanoseconds:
                return previous.pose
            fraction = (elapsed_nanoseconds - previous.elapsed_nanoseconds) / (
                keyframe.elapsed_nanoseconds - previous.elapsed_nanoseconds
            )
            translation = np.asarray(previous.pose.translation) + fraction * (
                np.asarray(keyframe.pose.translation) - previous.pose.translation
            )
            orientation = Slerp(
                [previous.elapsed_nanoseconds, keyframe.elapsed_nanoseconds],
                Rotation.from_quat([previous.pose.rotation, keyframe.pose.rotation]),
            )(elapsed_nanoseconds)
            return Pose(
                translation=translation.tolist(),
                rotation=orientation.as_quat().tolist(),
            )
        return previous.pose


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
    """Visible blocks and their editable camera-relative keyframes."""

    @staticmethod
    def create_blocks() -> list[BlockState]:
        """Create the initial moving block and its stationary reference."""
        initial_pose = Pose(
            translation=[-0.30, -0.15, 1.00], rotation=[0.0, 0.0, 0.0, 1.0]
        )
        translated_pose = Pose(
            translation=[-0.20, -0.15, 1.00], rotation=list(initial_pose.rotation)
        )
        rotated_pose = Pose(
            translation=list(translated_pose.translation),
            rotation=Rotation.from_rotvec([0.0, 0.0, np.pi / 2.0]).as_quat().tolist(),
        )
        return [
            BlockState(
                TetrisModel.FIRST,
                initial_pose,
                keyframes=[
                    PoseKeyframe(MotionBoundary.INITIAL_HOLD_FINISHED, initial_pose),
                    PoseKeyframe(MotionBoundary.TRANSLATION_FINISHED, translated_pose),
                    PoseKeyframe(
                        MotionBoundary.TRANSLATION_HOLD_FINISHED, translated_pose
                    ),
                    PoseKeyframe(MotionBoundary.ROTATION_FINISHED, rotated_pose),
                ],
            ),
            BlockState(
                TetrisModel.SECOND,
                Pose(translation=[0.05, -0.15, 1.00], rotation=[0.0, 0.0, 0.0, 1.0]),
            ),
        ]
