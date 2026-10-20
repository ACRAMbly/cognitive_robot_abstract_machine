"""
Exercise inference with deterministic CPU model outputs.
"""

from __future__ import annotations

import importlib
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock
from typing_extensions import Any, Iterator

import numpy as np
import pytest
import torch
from omegaconf import OmegaConf

# %% reference observations


@dataclass
class ReferenceTemplates:
    """
    Supply two distinguishable object models without rendering assets.
    """

    template_dir: str
    """Directory for the descriptor cache."""

    obj_ids: list[int] = field(default_factory=lambda: [1, 2])
    """
    Object identities in template order.
    """

    def __len__(self) -> int:
        """
        Return the number of object models.
        """
        return len(self.obj_ids)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        """
        Return one tiny template and its segmentation mask.
        """
        return {
            "templates": torch.full((1, 3, 4, 4), float(index)),
            "template_masks": torch.ones((1, 4, 4)),
        }


# %% CPU inference setup


@pytest.fixture
def detector_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[Any]:
    """
    Replace unavailable model frameworks while loading real inference code.
    """
    source = Path(__file__).resolve().parents[2] / "noctis_acrambly"
    monkeypatch.syspath_prepend(str(source))
    for name in tuple(sys.modules):
        if name == "src" or name.startswith("src."):
            monkeypatch.delitem(sys.modules, name)
    lightning = ModuleType("pytorch_lightning")
    lightning.__dict__["LightningModule"] = torch.nn.Module
    monkeypatch.setitem(sys.modules, lightning.__name__, lightning)
    for name, class_name in (
        ("src.model.sam", "CustomSamAutomaticMaskGenerator"),
        ("src.model.grounded_sam", "GroundedSAM"),
    ):
        module = ModuleType(name)
        module.__dict__[class_name] = type(class_name, (), {})
        monkeypatch.setitem(sys.modules, name, module)
    module = importlib.import_module("src.model.detector")
    yield module
    for name in tuple(sys.modules):
        if name == "src" or name.startswith("src."):
            sys.modules.pop(name)


@pytest.fixture
def detector(detector_module: Any, tmp_path: Path) -> Any:
    """
    Build real inference around small, controlled model tensors.
    """
    source = Path(__file__).resolve().parents[2] / "noctis_acrambly"
    configuration = OmegaConf.load(source / "configs/model/grounded_sam.yaml")
    configuration.onboarding_config.reset_descriptors = True
    configuration.matching_config.appearance_score_weight_factor = 0
    segmentor = Mock(spec=detector_module.GroundedSAM)
    segmentor.move_to_device = Mock()
    segmentor.generate_masks = Mock(
        return_value={
            "boxes": torch.tensor([[0, 0, 4, 4]]),
            "masks": torch.ones((1, 4, 4)),
            "boxes_scores": torch.ones(1),
            "masks_scores": torch.ones(1),
        }
    )
    descriptor = Mock()
    descriptor.model.to.return_value = descriptor.model
    descriptor.compute_cls_and_patch_features.side_effect = [
        (torch.tensor([[1.0, 0.0]]), torch.ones((1, 2, 2))),
        (torch.tensor([[0.0, 1.0]]), torch.ones((1, 2, 2))),
    ]
    descriptor.return_value = (torch.tensor([[1.0, 0.0]]), torch.ones((1, 2, 2)))
    result = detector_module.NOCTIS(
        segmentor,
        descriptor,
        configuration.onboarding_config,
        configuration.matching_config,
        configuration.post_processing_config,
        name_prediction_file="test",
        log_interval=1,
        log_dir=str(tmp_path),
        return_detections=True,
    )
    result.device = torch.device("cpu")
    result.ref_dataset = ReferenceTemplates(str(tmp_path))
    return result


@pytest.fixture
def observation() -> dict[str, torch.Tensor]:
    """
    Provide an observation independent of its source frame number.
    """
    return {
        "image": torch.zeros((1, 3, 4, 4)),
        "scene_id": torch.tensor([1]),
        "frame_id": torch.tensor([37]),
    }


# %% initialization lifecycle


def test_nonzero_first_observation_initializes_once(
    detector: Any, observation: dict[str, torch.Tensor]
) -> None:
    """
    Source identities and repeated zero identities do not reload models.
    """
    for identity in (37, 38, 0):
        result = detector.test_step(observation, identity)
        assert result["detections"].object_ids.tolist() == [
            detector.ref_dataset.obj_ids[0]
        ]
    detector.segmentor_model.move_to_device.assert_called_once_with(detector.device)
    assert detector.descriptor_model.compute_cls_and_patch_features.call_count == len(
        detector.ref_dataset
    )


def test_explicit_initialization_recomputes_references_once(
    detector: Any, observation: dict[str, torch.Tensor]
) -> None:
    """
    An initialization request prepares references before any observation arrives.
    """
    detector.initialize()
    detector.initialize()
    assert detector.descriptor_model.compute_cls_and_patch_features.call_count == len(
        detector.ref_dataset
    )
    detector.test_step(observation, 91)
    assert detector.descriptor_model.compute_cls_and_patch_features.call_count == len(
        detector.ref_dataset
    )
    detector.segmentor_model.move_to_device.assert_called_once_with(detector.device)


# %% empty scenes


def test_zero_proposals_return_empty_detections(
    detector: Any, observation: dict[str, torch.Tensor]
) -> None:
    """
    An empty proposal batch never reaches descriptor computation.
    """
    detector.segmentor_model.generate_masks.return_value = {
        "boxes": torch.empty((0, 4)),
        "masks": torch.empty((0, 4, 4)),
    }
    detections = detector.test_step(observation, 0)["detections"]
    assert detections.object_ids.tolist() == []
    assert detections.boxes.shape == (0, 4)
    assert detections.masks.shape == (0, *observation["image"].shape[-2:])
    assert detections.scores.shape == (0,)
    assert detections.score_distribution.shape == (0, len(detector.ref_dataset))
    assert detections.object_ids.dtype == np.int64
    detector.descriptor_model.assert_not_called()


def test_filtered_masks_return_empty_detections(
    detector: Any, observation: dict[str, torch.Tensor]
) -> None:
    """
    Proposals with no usable mask stop before query descriptors.
    """
    detector.segmentor_model.generate_masks.return_value["masks"].zero_()
    detections = detector.test_step(observation, 0)["detections"]
    assert detections.object_ids.tolist() == []
    assert detections.boxes.shape == (0, 4)
    assert detections.masks.shape == (0, *observation["image"].shape[-2:])
    assert detections.scores.shape == (0,)
    detector.descriptor_model.assert_not_called()


def test_rejected_matches_return_empty_detections(
    detector: Any, observation: dict[str, torch.Tensor]
) -> None:
    """
    A proposal with no reference similarity creates no object detection.
    """
    detector.descriptor_model.return_value = (
        torch.tensor([[-1.0, -1.0]]),
        torch.ones((1, 2, 2)),
    )
    detections = detector.test_step(observation, 0)["detections"]
    assert detections.object_ids.tolist() == []
    assert detections.boxes.shape == (0, 4)
    assert detections.masks.shape == (0, *observation["image"].shape[-2:])
    assert detections.scores.shape == (0,)
    assert detections.score_distribution.shape == (0, len(detector.ref_dataset))


# %% retained recognition results


def test_threshold_and_best_per_model_preserve_detection_data(
    detector: Any, observation: dict[str, torch.Tensor]
) -> None:
    """
    Strict thresholding and per-model selection retain the matching rectangles and
    masks.
    """
    threshold = detector.matching_config.final_confidence_threshold
    proposals = {
        "boxes": torch.tensor(
            [[0, 0, 2, 4], [2, 0, 4, 4], [0, 0, 4, 4], [0, 0, 4, 4], [0, 0, 4, 4]]
        ),
        "masks": torch.stack(
            [torch.full((4, 4), value) for value in (0.5, 0.6, 0.7, 0.8, 0.9)]
        ),
        "boxes_scores": torch.tensor([0.6, 0.9, 0.8, threshold, threshold / 2]),
        "masks_scores": torch.tensor([0.6, 0.9, 0.8, threshold, threshold / 2]),
    }
    detector.segmentor_model.generate_masks.return_value = proposals
    detector.descriptor_model.return_value = (
        torch.tensor([[1.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 0.0], [1.0, 0.0]]),
        torch.ones((5, 2, 2)),
    )
    detections = detector.test_step(observation, 0)["detections"]
    selected = [1, 2]
    assert detections.object_ids.tolist() == detector.ref_dataset.obj_ids
    np.testing.assert_array_equal(
        detections.scores, proposals["boxes_scores"][selected].numpy()
    )
    np.testing.assert_array_equal(
        detections.boxes, proposals["boxes"][selected].numpy()
    )
    np.testing.assert_array_equal(
        detections.masks, proposals["masks"][selected].numpy()
    )


def test_reset_references_replaces_existing_cache_once(
    detector: Any, observation: dict[str, torch.Tensor], tmp_path: Path
) -> None:
    """
    A requested recomputation replaces stale cached descriptors before inference.
    """
    cache = tmp_path / "descriptors.pth"
    torch.save({}, cache)
    detector.initialize()
    cached = torch.load(cache, weights_only=True)
    torch.testing.assert_close(
        cached["cls_descriptors"], torch.eye(len(detector.ref_dataset)).unsqueeze(1)
    )
    for identity in (19, 20):
        detector.test_step(observation, identity)
    assert detector.descriptor_model.compute_cls_and_patch_features.call_count == len(
        detector.ref_dataset
    )


def test_empty_detections_can_be_saved(
    detector: Any, observation: dict[str, torch.Tensor], tmp_path: Path
) -> None:
    """
    File-based inference retains the ordinary empty detection contract.
    """
    detector.return_detections = False
    detector.dataset_name = "recording"
    detector.segmentor_model.generate_masks.return_value = {
        "boxes": torch.empty((0, 4)),
        "masks": torch.empty((0, 4, 4)),
    }
    assert detector.test_step(observation, 0) == 0
    predictions = list((tmp_path / "predictions").rglob("*.npz"))
    saved = next(path for path in predictions if not path.stem.endswith("_runtime"))
    with np.load(saved) as detections:
        assert detections["category_id"].tolist() == []
        assert detections["bbox"].shape == (0, 4)
        assert detections["segmentation"].shape == (0, *observation["image"].shape[-2:])
        assert detections["image_id"] == observation["frame_id"][0].item()


@pytest.mark.parametrize("confidence_factor", [0.5, 1.0, 1.5])
def test_final_threshold_is_strict(
    detector: Any, observation: dict[str, torch.Tensor], confidence_factor: float
) -> None:
    """
    Only confidence strictly above the configured threshold retains a detection.
    """
    confidence = detector.matching_config.final_confidence_threshold * confidence_factor
    proposals = detector.segmentor_model.generate_masks.return_value
    proposals["boxes_scores"].fill_(confidence)
    proposals["masks_scores"].fill_(confidence)
    detections = detector.test_step(observation, 0)["detections"]
    expected = detector.ref_dataset.obj_ids[:1] if confidence_factor > 1 else []
    assert detections.object_ids.tolist() == expected
