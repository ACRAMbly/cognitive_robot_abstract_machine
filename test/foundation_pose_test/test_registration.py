"""
Registration distinguishes unusable depth from successful poses.
"""

from __future__ import annotations

from types import ModuleType
from unittest.mock import Mock

import numpy as np
import pytest
import torch

from .conftest import RegistrationObservation

# %% insufficient filtered depth


def test_raw_depth_does_not_override_filtered_depth(
    estimator_module: ModuleType, observation: RegistrationObservation
) -> None:
    """
    Sufficient raw depth cannot turn empty filtered depth into a pose.
    """
    result = observation.register()
    assert isinstance(result, estimator_module.InsufficientDepth)


def test_one_pixel_below_threshold_has_typed_count(
    estimator_module: ModuleType, observation: RegistrationObservation
) -> None:
    """
    Report the surviving pixel count immediately below the estimator minimum.
    """
    count = observation.estimator.MINIMUM_VALID_DEPTH_PIXELS - 1
    observation.filtered_depth.flatten()[:count] = 1
    assert observation.register() == estimator_module.InsufficientDepth(count)


def test_unmasked_filtered_depth_does_not_satisfy_threshold(
    estimator_module: ModuleType, observation: RegistrationObservation
) -> None:
    """
    Count only filtered depth belonging to the selected target.
    """
    observation.filtered_depth[:] = 1
    observation.mask[:] = False
    assert observation.register() == estimator_module.InsufficientDepth(0)


# %% successful registration


@pytest.mark.parametrize("additional_pixels", [0, 1])
def test_sufficient_filtered_depth_preserves_ranked_original_cad_poses(
    estimator_module: ModuleType,
    observation: RegistrationObservation,
    monkeypatch: pytest.MonkeyPatch,
    additional_pixels: int,
) -> None:
    """
    At and above the minimum, rank refined poses and restore the CAD origin.
    """
    count = observation.estimator.MINIMUM_VALID_DEPTH_PIXELS + additional_pixels
    observation.filtered_depth.flatten()[:count] = 1
    estimator = observation.estimator
    refined_poses = torch.eye(4).repeat(2, 1, 1)
    refined_poses[1, :3, :3] = torch.tensor(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    refined_poses[1, :3, 3] = torch.tensor([1.0, 2.0, 3.0])
    centered_transform = torch.eye(4)
    centered_transform[0, 3] = -0.25
    estimator.generate_random_pose_hypo = Mock(return_value=refined_poses)
    estimator.refiner = Mock()
    estimator.refiner.predict.return_value = (refined_poses, None)
    estimator.scorer = Mock()
    estimator.scorer.predict.return_value = (torch.tensor([0.1, 0.9]), None)
    estimator.get_tf_to_centered_mesh = Mock(return_value=centered_transform)
    estimator.debug = False
    estimator.mesh = None
    estimator.mesh_tensors = None
    estimator.glctx = None
    estimator.diameter = 1
    monkeypatch.setattr(
        estimator_module,
        "depth2xyzmap_batch",
        Mock(return_value=torch.zeros((1, *observation.depth.shape, 3))),
    )
    expected = torch.matmul(refined_poses[1:2], centered_transform)
    torch.testing.assert_close(observation.register(), expected)
