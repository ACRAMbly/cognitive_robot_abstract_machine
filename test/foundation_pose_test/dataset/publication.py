"""Minimal framework data contracts for CPU annotation publication checks."""

from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Any

# %% annotation contracts


@dataclass(kw_only=True)
class PublishedAnnotation:
    """Supply the common framework annotation fields."""

    source: str = ""
    """Publishing stage."""


@dataclass
class TargetClassification(PublishedAnnotation):
    """Associate a detection with its object model."""

    class_id: int
    """Object model identity."""

    confidence: float = 1.0
    """Detection confidence."""


@dataclass
class PublishedPose(PublishedAnnotation):
    """Store the pose fields published by an annotator."""

    translation: list[float] = field(default_factory=list)
    """Metric translation."""

    rotation: list[float] = field(default_factory=list)
    """Quaternion in xyzw order."""


@dataclass
class MaskedTarget:
    """Hold a target's framework region and published annotations."""

    roi: Any
    """Framework region and cropped mask."""

    annotations: list[PublishedAnnotation]
    """Detection classification and subsequent estimation outcomes."""


@dataclass
class ThreadedStage:
    """Supply the framework descriptor inheritance boundary."""

    @dataclass
    class Descriptor:
        """Describe a stage independently of ROS execution."""
