"""
CPU replacements for FoundationPose's model and rendering dependencies.
"""

from __future__ import annotations

import importlib
import math
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock

import numpy as np
import pytest
import torch
from typing_extensions import Any, Iterator

from .dataset.publication import (
    MaskedTarget,
    PublishedAnnotation,
    PublishedPose,
    TargetClassification,
    ThreadedStage,
)

# %% dependency boundaries


class Dependency(StrEnum):
    """
    Imports replaced when exercising registration without model assets.
    """

    UTILITIES = "robokudo_foundation_pose.Utils"
    """GPU depth processing and geometry helpers."""
    RENDERER = "nvdiffrast"
    """Rendering package."""
    TORCH_RENDERER = "nvdiffrast.torch"
    """Tensor rendering interface."""
    SCORER = "robokudo_foundation_pose.learning.training.predict_score"
    """Learned pose scoring."""
    REFINER = "robokudo_foundation_pose.learning.training.predict_pose_refine"
    """Learned pose refinement."""
    FRAMEWORK = "robokudo"
    """Perception framework package."""
    DEFINITIONS = "robokudo.defs"
    """Framework package metadata."""


class FoundationPoseModule(StrEnum):
    """Production import boundaries under test."""

    PACKAGE = "robokudo_foundation_pose"
    """Root package and its source directory name."""

    ESTIMATOR = "robokudo_foundation_pose.estimater"
    """Public registration implementation."""

    ANNOTATOR = "robokudo_foundation_pose.annotator.foundation_pose_annotator"
    """Target annotation publication implementation."""


class Device(StrEnum):
    """Accelerator request replaced by CPU computation."""

    CUDA = "cuda"
    """Device used by production rendering and refinement."""


class FrameworkDependency(StrEnum):
    """
    Framework imports unavailable in the isolated model test environment.
    """

    GEOMETRY = "open3d"
    """Geometry visualization dependency."""
    BEHAVIOR = "py_trees"
    """Behavior tree execution dependency."""
    ROS = "rclpy"
    """ROS client dependency."""
    ANNOTATORS = "robokudo.annotators"
    """Framework stage package."""
    CORE = "robokudo.annotators.core"
    """Framework stage base classes."""
    SCENE = "robokudo.cas"
    """Scene storage interface."""
    TYPES = "robokudo.types"
    """Framework data package."""
    ANNOTATION_BASE = "robokudo.types.core"
    """Shared annotation fields."""
    ANNOTATION = "robokudo.types.annotation"
    """Classification and pose annotations."""
    TARGET = "robokudo.types.scene"
    """Detected target representation."""
    UTILITIES = "robokudo.utils"
    """Framework image utilities."""
    DECORATORS = "robokudo.utils.decorators"
    """Framework timing decorators."""


@pytest.fixture
def estimator_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """
    Import real registration with unavailable GPU imports replaced.
    """
    source = (
        Path(__file__).resolve().parents[2]
        / FoundationPoseModule.PACKAGE
        / FoundationPoseModule.PACKAGE
    )
    monkeypatch.syspath_prepend(str(source))
    for name in tuple(sys.modules):
        if name == FoundationPoseModule.PACKAGE or name.startswith(
            FoundationPoseModule.PACKAGE + "."
        ):
            monkeypatch.delitem(sys.modules, name)
    for dependency in Dependency:
        monkeypatch.setitem(sys.modules, dependency, Mock())
    module = importlib.import_module(FoundationPoseModule.ESTIMATOR)
    yield module
    for name in tuple(sys.modules):
        if name == FoundationPoseModule.PACKAGE or name.startswith(
            FoundationPoseModule.PACKAGE + "."
        ):
            sys.modules.pop(name)


@pytest.fixture
def annotator_module(
    estimator_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> Iterator[ModuleType]:
    """
    Import the real annotator around lightweight framework contracts.
    """
    for dependency in FrameworkDependency:
        monkeypatch.setitem(sys.modules, dependency, Mock())
    core: Any = sys.modules[FrameworkDependency.CORE]
    core.ThreadedAnnotator = ThreadedStage
    sys.modules[Dependency.FRAMEWORK].annotators.core = sys.modules[
        FrameworkDependency.CORE
    ]
    annotations: Any = sys.modules[FrameworkDependency.ANNOTATION]
    annotations.Classification = TargetClassification
    annotations.PoseAnnotation = PublishedPose
    targets: Any = sys.modules[FrameworkDependency.TARGET]
    targets.ObjectHypothesis = MaskedTarget
    base: Any = sys.modules[FrameworkDependency.ANNOTATION_BASE]
    base.Annotation = PublishedAnnotation
    module = importlib.import_module(FoundationPoseModule.ANNOTATOR)
    yield module


# %% registration observations


@dataclass
class RegistrationObservation:
    """
    A masked observation with controlled post-filter depth.
    """

    estimator: Any
    """
    Real estimator with learned computation replaced.
    """

    color: np.ndarray
    """
    Color pixels in the registration grid.
    """

    depth: np.ndarray
    """
    Metric raw depth before GPU preprocessing.
    """

    mask: np.ndarray
    """
    Selected target pixels.
    """

    intrinsics: np.ndarray
    """
    Pinhole calibration of the observation.
    """

    filtered_depth: torch.Tensor
    """
    Depth that survives deterministic GPU preprocessing.
    """

    def register(self) -> Any:
        """
        Register the observation through the public estimator boundary.
        """
        return self.estimator.register(
            rgb=self.color,
            depth=self.depth,
            cam_intrinsics=self.intrinsics,
            obj_mask=self.mask,
        )


@pytest.fixture
def observation(
    estimator_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> RegistrationObservation:
    """
    Create a CPU observation whose raw depth exceeds the existing threshold.
    """
    side = (
        math.isqrt(2 * estimator_module.FoundationPose.MINIMUM_VALID_DEPTH_PIXELS) + 1
    )
    depth = np.ones((side, side), dtype=np.float32)
    filtered_depth = torch.zeros_like(torch.from_numpy(depth))
    monkeypatch.setattr(estimator_module, "set_seed", Mock())
    monkeypatch.setattr(
        estimator_module, "erode_depth", Mock(return_value=torch.from_numpy(depth))
    )
    monkeypatch.setattr(
        estimator_module, "bilateral_filter_depth", Mock(return_value=filtered_depth)
    )
    original_to = torch.Tensor.to

    def move_to_cpu(
        tensor: torch.Tensor, *arguments: Any, **keywords: Any
    ) -> torch.Tensor:
        """
        Keep tensors on CPU when production requests the CUDA device.
        """
        if arguments == (Device.CUDA,):
            return tensor
        return original_to(tensor, *arguments, **keywords)

    monkeypatch.setattr(torch.Tensor, "to", move_to_cpu)
    estimator = object.__new__(estimator_module.FoundationPose)
    return RegistrationObservation(
        estimator,
        np.zeros((side, side, 3), dtype=np.uint8),
        depth,
        np.ones_like(depth, dtype=bool),
        np.eye(3, dtype=np.float32),
        filtered_depth,
    )
