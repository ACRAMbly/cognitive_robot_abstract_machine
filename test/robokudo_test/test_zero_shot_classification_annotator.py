"""
Tests for the CLIP-based classification of whole images and object hypotheses.

The CLIP model is replaced by a stand-in that embeds an image as its mean color and a
text as the color it names, so the expected classification of every input is known
without downloading model weights.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import pytest
import torch
from PIL import Image
from py_trees.common import Status
from transformers import BatchEncoding, BatchFeature
from transformers.modeling_outputs import BaseModelOutputWithPooling
from typing_extensions import List, Sequence, Union

import robokudo.annotators.zero_shot_classification_annotator
from robokudo.annotators.zero_shot_classification_annotator import (
    ZeroShotClassificationAnnotator,
)
from robokudo.cas import CASViews
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification
from robokudo.types.scene import ObjectHypothesis

pytestmark = pytest.mark.usefixtures("annotator_output_buffer")

# %% color embedding stand-in for CLIP

COLOR_BY_WORD = {
    "blue": np.array([255, 0, 0], dtype=np.uint8),
    "green": np.array([0, 255, 0], dtype=np.uint8),
    "red": np.array([0, 0, 255], dtype=np.uint8),
}
"""
The color each vocabulary word names, in the channel order of the CAS color image.
"""

COLOR_VOCABULARY = list(COLOR_BY_WORD)
"""
A vocabulary in which every word names one color channel.
"""

CLIP_LOGIT_SCALE = 100.0
"""
The temperature CLIP's similarity logits are scaled with.
"""


UNNAMED_COLOR = np.array([1, 1, 1], dtype=np.uint8)
"""
The color a text naming none of the vocabulary colors is embedded as.
"""


def _named_color(text: str) -> np.ndarray:
    """
    :return: The color named by the vocabulary word contained in the given text.
    """
    return next(
        (color for word, color in COLOR_BY_WORD.items() if word in text),
        UNNAMED_COLOR,
    )


@dataclass
class ColorTokenizer:
    """
    Tokenizer stand-in that encodes each text as the color it names.
    """

    received_texts: List[str] = field(default_factory=list)
    """
    The texts this tokenizer was asked to encode.
    """

    def __call__(
        self, texts: List[str], padding: bool, return_tensors: str
    ) -> BatchEncoding:
        self.received_texts.extend(texts)
        colors = np.stack([_named_color(text) for text in texts])
        return BatchEncoding({"colors": torch.tensor(colors, dtype=torch.float32)})


@dataclass
class ColorProcessor:
    """
    CLIP processor stand-in that encodes each image as its mean color.
    """

    tokenizer: ColorTokenizer = field(default_factory=ColorTokenizer)
    """
    The tokenizer used for vocabulary texts.
    """

    def __call__(
        self,
        images: Union[np.ndarray, Image.Image, Sequence[Image.Image]],
        return_tensors: str,
    ) -> BatchFeature:
        image_list = images if isinstance(images, list) else [images]
        mean_colors = np.stack(
            [np.asarray(image).reshape(-1, 3).mean(axis=0) for image in image_list]
        )
        return BatchFeature({"colors": torch.tensor(mean_colors, dtype=torch.float32)})


@dataclass
class ColorModelConfiguration:
    """
    Model configuration stand-in exposing the embedding size.
    """

    projection_dim: int = 3
    """
    Size of the shared image and text embedding: one dimension per color channel.
    """


@dataclass
class ColorEmbeddingModel:
    """
    CLIP model stand-in whose image and text embeddings are the encoded colors.
    """

    config: ColorModelConfiguration = field(default_factory=ColorModelConfiguration)
    """
    The model configuration.
    """

    device: torch.device = field(default_factory=lambda: torch.device("cpu"))
    """
    The device the model runs on.
    """

    logit_scale: torch.Tensor = field(
        default_factory=lambda: torch.tensor(math.log(CLIP_LOGIT_SCALE))
    )
    """
    Logarithm of the temperature the similarity logits are scaled with.
    """

    def get_text_features(self, colors: torch.Tensor) -> BaseModelOutputWithPooling:
        return BaseModelOutputWithPooling(pooler_output=colors.clone())

    def get_image_features(self, colors: torch.Tensor) -> BaseModelOutputWithPooling:
        return BaseModelOutputWithPooling(pooler_output=colors.clone())


@dataclass
class ColorEmbeddingClip:
    """
    The stand-in model and processor handed out in place of pretrained CLIP weights.
    """

    model: ColorEmbeddingModel = field(default_factory=ColorEmbeddingModel)
    """
    The stand-in CLIP model.
    """

    processor: ColorProcessor = field(default_factory=ColorProcessor)
    """
    The stand-in CLIP processor.
    """


@pytest.fixture
def color_embedding_clip(monkeypatch) -> ColorEmbeddingClip:
    """
    Make the annotator load the color embedding stand-in instead of pretrained CLIP.
    """
    clip = ColorEmbeddingClip()
    monkeypatch.setattr(
        robokudo.annotators.zero_shot_classification_annotator.CLIPModel,
        "from_pretrained",
        lambda model_name: clip.model,
    )
    monkeypatch.setattr(
        robokudo.annotators.zero_shot_classification_annotator.CLIPProcessor,
        "from_pretrained",
        lambda model_name: clip.processor,
    )
    return clip


# %% scene setup


def _solid_image(color: np.ndarray, height: int = 20, width: int = 40) -> np.ndarray:
    """
    :return: A color image filled with one color.
    """
    return np.full((height, width, 3), color, dtype=np.uint8)


def _object_hypothesis(x: int, y: int, width: int, height: int) -> ObjectHypothesis:
    """
    :return: An object hypothesis covering the given image region.
    """
    object_hypothesis = ObjectHypothesis()
    object_hypothesis.roi.roi.pos.x = x
    object_hypothesis.roi.roi.pos.y = y
    object_hypothesis.roi.roi.width = width
    object_hypothesis.roi.roi.height = height
    return object_hypothesis


def _color_descriptor() -> ZeroShotClassificationAnnotator.Descriptor:
    """
    :return: A descriptor classifying against the color vocabulary on the CPU.
    """
    descriptor = ZeroShotClassificationAnnotator.Descriptor()
    descriptor.parameters.device = torch.device("cpu")
    descriptor.parameters.vocabulary = COLOR_VOCABULARY
    descriptor.parameters.save_top_k = len(COLOR_VOCABULARY)
    return descriptor


def _annotator_in_pipeline(
    descriptor: ZeroShotClassificationAnnotator.Descriptor, color_image: np.ndarray
) -> ZeroShotClassificationAnnotator:
    """
    :return: An annotator whose CAS holds the given color image.
    """
    annotator = ZeroShotClassificationAnnotator(descriptor=descriptor)
    pipeline = Pipeline("Sequence")
    pipeline.add_child(annotator)
    pipeline.setup()
    annotator.get_cas().set(CASViews.COLOR_IMAGE, color_image)
    return annotator


def _scene_classifications(
    annotator: ZeroShotClassificationAnnotator,
) -> List[Classification]:
    return annotator.get_cas().filter_annotations_by_type(Classification)


def _hypothesis_classifications(
    object_hypothesis: ObjectHypothesis,
) -> List[Classification]:
    return [
        annotation
        for annotation in object_hypothesis.annotations
        if isinstance(annotation, Classification)
    ]


# %% construction


def test_annotator_without_descriptor_indexes_default_vocabulary(
    color_embedding_clip,
):
    """
    An annotator built without a descriptor falls back to the default parameters.
    """
    default_vocabulary = (
        ZeroShotClassificationAnnotator.Descriptor().parameters.vocabulary
    )

    annotator = ZeroShotClassificationAnnotator()

    assert annotator.vocabulary_index.ntotal == len(default_vocabulary)


def test_vocabulary_is_encoded_through_the_template(color_embedding_clip):
    descriptor = _color_descriptor()
    descriptor.parameters.vocabulary_template = "A photo of something {}"

    ZeroShotClassificationAnnotator(descriptor=descriptor)

    assert color_embedding_clip.processor.tokenizer.received_texts == [
        descriptor.parameters.vocabulary_template.format(word)
        for word in COLOR_VOCABULARY
    ]


# %% scene scope


def test_scene_scope_ranks_word_naming_image_color_first(color_embedding_clip):
    annotator = _annotator_in_pipeline(
        _color_descriptor(), _solid_image(COLOR_BY_WORD["green"])
    )

    status = annotator.compute()

    assert status == Status.SUCCESS
    assert _scene_classifications(annotator)[0].classname == "green"


def test_scene_scope_keeps_save_top_k_classifications(color_embedding_clip):
    descriptor = _color_descriptor()
    descriptor.parameters.save_top_k = 2
    annotator = _annotator_in_pipeline(descriptor, _solid_image(COLOR_BY_WORD["red"]))

    annotator.compute()

    assert len(_scene_classifications(annotator)) == descriptor.parameters.save_top_k


def test_classifications_name_the_annotator_as_source(color_embedding_clip):
    annotator = _annotator_in_pipeline(
        _color_descriptor(), _solid_image(COLOR_BY_WORD["red"])
    )

    annotator.compute()

    assert {
        classification.source for classification in _scene_classifications(annotator)
    } == {annotator.name}


def test_softmax_confidences_over_whole_vocabulary_sum_to_one(color_embedding_clip):
    annotator = _annotator_in_pipeline(
        _color_descriptor(), _solid_image(COLOR_BY_WORD["blue"])
    )

    annotator.compute()

    confidences = [
        classification.confidence
        for classification in _scene_classifications(annotator)
    ]
    assert sum(confidences) == pytest.approx(1.0)


def test_confidence_without_softmax_is_rescaled_cosine_similarity(
    color_embedding_clip,
):
    """
    Without softmax, a cosine similarity in [-1, 1] is reported as a confidence in
    [0, 1]: the matching color has similarity 1, every other color similarity 0.
    """
    descriptor = _color_descriptor()
    descriptor.parameters.use_softmax = False
    annotator = _annotator_in_pipeline(descriptor, _solid_image(COLOR_BY_WORD["blue"]))

    annotator.compute()

    matching_similarity, orthogonal_similarity = 1.0, 0.0
    confidence_by_word = {
        classification.classname: classification.confidence
        for classification in _scene_classifications(annotator)
    }
    assert confidence_by_word == pytest.approx(
        {
            word: (
                (matching_similarity if word == "blue" else orthogonal_similarity) + 1.0
            )
            / 2.0
            for word in COLOR_VOCABULARY
        }
    )


def test_save_top_k_beyond_vocabulary_without_softmax_reports_each_word_once(
    color_embedding_clip,
):
    descriptor = _color_descriptor()
    descriptor.parameters.use_softmax = False
    descriptor.parameters.save_top_k = len(COLOR_VOCABULARY) + 2
    annotator = _annotator_in_pipeline(descriptor, _solid_image(COLOR_BY_WORD["blue"]))

    annotator.compute()

    assert sorted(
        classification.classname for classification in _scene_classifications(annotator)
    ) == sorted(COLOR_VOCABULARY)


# %% object hypothesis scope


def _two_color_scene() -> np.ndarray:
    """
    :return: An image whose left half is red and whose right half is blue.
    """
    image = _solid_image(COLOR_BY_WORD["red"])
    image[:, image.shape[1] // 2 :] = COLOR_BY_WORD["blue"]
    return image


def _object_hypothesis_descriptor() -> ZeroShotClassificationAnnotator.Descriptor:
    descriptor = _color_descriptor()
    descriptor.parameters.analysis_scope = ObjectHypothesis
    return descriptor


def test_object_hypothesis_scope_classifies_each_hypothesis_by_its_region(
    color_embedding_clip,
):
    annotator = _annotator_in_pipeline(
        _object_hypothesis_descriptor(), _two_color_scene()
    )
    red_object = _object_hypothesis(x=0, y=0, width=10, height=10)
    blue_object = _object_hypothesis(x=30, y=0, width=10, height=10)
    annotator.get_cas().annotations.extend([red_object, blue_object])

    status = annotator.compute()

    assert status == Status.SUCCESS
    assert _hypothesis_classifications(red_object)[0].classname == "red"
    assert _hypothesis_classifications(blue_object)[0].classname == "blue"


def test_object_hypothesis_scope_skips_hypotheses_rejected_by_filter(
    color_embedding_clip,
):
    descriptor = _object_hypothesis_descriptor()
    rejected_object = _object_hypothesis(x=0, y=0, width=10, height=10)
    descriptor.parameters.filter_fn = (
        lambda object_hypothesis: object_hypothesis is not rejected_object
    )
    annotator = _annotator_in_pipeline(descriptor, _two_color_scene())
    accepted_object = _object_hypothesis(x=30, y=0, width=10, height=10)
    annotator.get_cas().annotations.extend([rejected_object, accepted_object])

    annotator.compute()

    assert _hypothesis_classifications(rejected_object) == []
    assert len(_hypothesis_classifications(accepted_object)) == (
        descriptor.parameters.save_top_k
    )


def test_object_hypothesis_scope_without_hypotheses_adds_nothing(
    color_embedding_clip,
):
    annotator = _annotator_in_pipeline(
        _object_hypothesis_descriptor(), _two_color_scene()
    )

    status = annotator.compute()

    assert status == Status.SUCCESS
    assert annotator.get_cas().annotations == []


# %% unsupported scope


def test_unsupported_analysis_scope_fails(color_embedding_clip):
    descriptor = _color_descriptor()
    descriptor.parameters.analysis_scope = CASViews.DEPTH_IMAGE
    annotator = _annotator_in_pipeline(descriptor, _solid_image(COLOR_BY_WORD["red"]))

    assert annotator.compute() == Status.FAILURE
