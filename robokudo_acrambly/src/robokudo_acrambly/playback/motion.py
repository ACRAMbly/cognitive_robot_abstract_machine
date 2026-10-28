# %% Imports

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum

import numpy
from scipy.spatial.transform import Rotation, Slerp

from robokudo.types.tf import Pose
from robokudo_acrambly.playback.clock import TimeUnit
from robokudo_acrambly.playback.exceptions import (
    InvalidAbsenceInterval,
    InvalidPoseKeyframes,
)
from robokudo_acrambly.playback.schedule import PlaybackTiming


# %% Motion boundaries


class MotionBoundary(IntEnum):
    """
    Default capture offsets separating motion from stationary holds.
    """

    INITIAL_HOLD_FINISHED = 3 * TimeUnit.NANOSECONDS_PER_SECOND
    TRANSLATION_FINISHED = 9 * TimeUnit.NANOSECONDS_PER_SECOND
    TRANSLATION_HOLD_FINISHED = 12 * TimeUnit.NANOSECONDS_PER_SECOND
    ROTATION_FINISHED = 18 * TimeUnit.NANOSECONDS_PER_SECOND


class VisibilityBoundary(IntEnum):
    """
    Default capture offsets separating presence, absence, and reappearance.
    """

    PARTIAL_ABSENCE_START = 21 * TimeUnit.NANOSECONDS_PER_SECOND
    TOTAL_ABSENCE_START = 24 * TimeUnit.NANOSECONDS_PER_SECOND
    REAPPEARANCE = 27 * TimeUnit.NANOSECONDS_PER_SECOND


# %% Pose and visibility descriptors


@dataclass
class PoseKeyframe:
    """
    An editable CAD-origin pose at a scheduled capture offset.
    """

    elapsed_nanoseconds: int
    """
    Elapsed capture time at which this pose is reached.
    """

    pose: Pose
    """
    Camera-relative endpoint pose in metres and xyzw quaternion order.
    """


@dataclass
class AbsenceInterval:
    """
    An editable half-open interval in which an object has no detection.
    """

    start_nanoseconds: int
    """
    First elapsed capture time at which the object is absent.
    """

    end_nanoseconds: int
    """
    Elapsed capture time at which the object becomes visible again.
    """


# %% Configured object motion


@dataclass
class MotionDefinition:
    """
    An object kind, initial pose, sampled keyframes, and visibility intervals.
    """

    model_identifier: str
    """
    Classification kind identifying invalid motion configuration.
    """

    pose: Pose
    """
    Camera-relative mesh-origin pose at the initial capture.
    """

    keyframes: list[PoseKeyframe]
    """
    Scheduled endpoints after the initial pose.
    """

    absence_intervals: list[AbsenceInterval]
    """
    Half-open intervals omitting detections for this object.
    """


@dataclass
class ObjectMotion(ABC):
    """
    Provide sampled pose interpolation and discrete object visibility.
    """

    @abstractmethod
    def describe_motion(self) -> MotionDefinition:
        """
        Describe the object kind and its current motion configuration.
        """

    def is_visible(self, elapsed_nanoseconds: int) -> bool:
        """
        Return whether this object appears in the scheduled observation.
        """
        motion = self.describe_motion()
        return not any(
            interval.start_nanoseconds <= elapsed_nanoseconds < interval.end_nanoseconds
            for interval in motion.absence_intervals
        )

    def validate_absence_intervals(self, timing: PlaybackTiming) -> None:
        """
        Reject negative, reversed, or unsampled visibility boundaries.
        """
        motion = self.describe_motion()
        for interval in motion.absence_intervals:
            unsampled = timing.contains_unsampled_offsets(
                [interval.start_nanoseconds, interval.end_nanoseconds]
            )
            if (
                interval.start_nanoseconds < 0
                or interval.end_nanoseconds <= interval.start_nanoseconds
                or unsampled
            ):
                raise InvalidAbsenceInterval(
                    motion.model_identifier,
                    interval,
                    timing.sample_period_nanoseconds,
                )

    def validate_keyframes(self, timing: PlaybackTiming) -> None:
        """
        Reject unordered endpoints or endpoints absent from the capture schedule.
        """
        motion = self.describe_motion()
        offsets = [0] + [keyframe.elapsed_nanoseconds for keyframe in motion.keyframes]
        unordered = any(end <= start for start, end in zip(offsets, offsets[1:]))
        unsampled = timing.contains_unsampled_offsets(offsets)
        if unordered or unsampled:
            raise InvalidPoseKeyframes(
                motion.model_identifier,
                offsets[1:],
                timing.sample_period_nanoseconds,
            )

    def interpolate_pose(self, elapsed_nanoseconds: int) -> Pose:
        """
        Return the camera-relative pose at an elapsed capture time.
        """
        motion = self.describe_motion()
        previous = PoseKeyframe(0, motion.pose)
        for keyframe in motion.keyframes:
            if elapsed_nanoseconds >= keyframe.elapsed_nanoseconds:
                previous = keyframe
                continue
            if elapsed_nanoseconds == previous.elapsed_nanoseconds:
                return previous.pose
            fraction = (elapsed_nanoseconds - previous.elapsed_nanoseconds) / (
                keyframe.elapsed_nanoseconds - previous.elapsed_nanoseconds
            )
            translation = numpy.asarray(previous.pose.translation) + fraction * (
                numpy.asarray(keyframe.pose.translation) - previous.pose.translation
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
