"""
Exceptions raised by RoboKudo annotators.
"""

# %% Imports

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from robokudo.exceptions import RoboKudoError

# %% model configuration


@dataclass
class NotAPoseModelException(RoboKudoError, ValueError):
    """
    Raised when an annotator that reads body keypoints loaded a model reporting none.
    """

    annotator_name: str
    """
    Name of the annotator whose model reports no keypoints.
    """

    model: str
    """
    The model the annotator loaded.
    """

    def error_message(self) -> str:
        return f"{self.annotator_name} loaded {self.model}, which reports no body keypoints."

    def suggest_correction(self) -> str:
        return "set 'pretrained_model' to a YOLO pose model, e.g. 'yolov8n-pose.pt'."


# %% Semantic observation validation


@dataclass
class MissingModelLabel(RoboKudoError, ValueError):
    """
    A configured hypothesis has no model classification.
    """

    def error_message(self) -> str:
        return "The configured object hypothesis has no model label."

    def suggest_correction(self) -> str:
        return "provide an unambiguous Classification with a configured classname."


@dataclass
class UnknownModelLabel(RoboKudoError, ValueError):
    """
    A model classification is absent from the semantic configuration.
    """

    model_label: str
    """Classification label that could not be resolved."""

    def error_message(self) -> str:
        return f"The model label {self.model_label!r} is not configured."

    def suggest_correction(self) -> str:
        return "configure this model's semantic type or correct its classification."


@dataclass
class ConflictingModelLabels(RoboKudoError, ValueError):
    """
    One hypothesis declares different model kinds.
    """

    model_labels: frozenset[str]
    """Incompatible classification labels on the same hypothesis."""

    def error_message(self) -> str:
        return (
            f"The hypothesis has conflicting model labels: {sorted(self.model_labels)}."
        )

    def suggest_correction(self) -> str:
        return "provide classifications agreeing on a single configured model label."


@dataclass
class InvalidObservationPose(RoboKudoError, ValueError):
    """
    An observation has no supported finite pose with a usable quaternion.
    """

    def error_message(self) -> str:
        return "The configured hypothesis has a missing or invalid pose."

    def suggest_correction(self) -> str:
        return "provide a plain or stamped pose with three finite coordinates and a nonzero finite xyzw quaternion."


class ObservationProblem(StrEnum):
    """
    Reasons a configured observation cannot be accepted.
    """

    WORLD_FRAME = "the declared world frame must identify one existing world body"
    CAMERA_FRAME = "a camera-relative pose requires an unambiguous camera frame"
    AMBIGUOUS_FRAME = "camera and world frame identifiers must differ"
    UNSUPPORTED_FRAME = "the stamped pose must declare the camera or world frame"
    CONFLICTING_FRAME = "camera-info and CAS camera frames must agree"
    MISSING_TIME = "the CAS requires a capture timestamp"
    INVALID_TIME = "capture timestamps must be valid nonnegative ROS times"
    CONFLICTING_TIME = "all declared capture timestamps must agree"
    MISSING_TRANSFORM = "a camera-relative pose requires a camera-to-world transform"
    TRANSFORM_FRAMES = (
        "the transform must reference the declared world and camera bodies"
    )
    INVALID_TRANSFORM = "the camera-to-world transform must be finite and rigid"


@dataclass
class InvalidObservationFrame(RoboKudoError, ValueError):
    """
    Observation coordinate frames are absent, unsupported, or contradictory.
    """

    problem: ObservationProblem
    """
    Frame requirement the observation violates.
    """

    def error_message(self) -> str:
        return f"Invalid observation frame: {self.problem.value}."

    def suggest_correction(self) -> str:
        return "declare consistent camera and world frames matching the source pose."


@dataclass
class InvalidCaptureMetadata(RoboKudoError, ValueError):
    """
    Required observation capture times are absent or contradictory.
    """

    problem: ObservationProblem
    """
    Capture-time requirement the observation violates.
    """

    def error_message(self) -> str:
        return f"Invalid capture metadata: {self.problem.value}."

    def suggest_correction(self) -> str:
        return "retain the observation's capture time consistently in CAS and stamped annotations."


@dataclass
class InvalidCameraTransform(RoboKudoError, ValueError):
    """
    A required camera-to-world transform is absent or unusable.
    """

    problem: ObservationProblem
    """
    Camera-transform requirement the observation violates.
    """

    def error_message(self) -> str:
        return f"Invalid camera transform: {self.problem.value}."

    def suggest_correction(self) -> str:
        return "supply a rigid transform from the declared camera body to the declared world body."
