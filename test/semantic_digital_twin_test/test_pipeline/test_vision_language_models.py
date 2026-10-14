"""
Finding a mug's handle with the real models.

Runs only where the ``vision`` extra is installed and a GPU is available; downloads
about 18 GB of weights on first use.
"""

from __future__ import annotations

import numpy as np
import pytest
import trimesh

torch = pytest.importorskip("torch")
pytest.importorskip("transformers")

from semantic_digital_twin.pipeline.part_locating import PartQuery
from semantic_digital_twin.pipeline.vision_language_models import (
    QwenBoxDetector,
    Sam2BoxSegmenter,
)
from semantic_digital_twin.pipeline.vision_language_part_locating import (
    VisionLanguagePartLocator,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Mug

pytestmark = pytest.mark.skipif(
    not torch.cuda.is_available(), reason="the models need a GPU"
)


def test_the_models_find_most_of_a_mugs_handle():
    cup = trimesh.creation.cylinder(radius=0.04, height=0.1, sections=64)
    handle = trimesh.creation.torus(
        major_radius=0.025, minor_radius=0.006, major_sections=48, minor_sections=12
    )
    handle.apply_transform(
        trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    )
    handle.apply_translation([0.045, 0.0, 0.0])
    shape = trimesh.util.concatenate([cup, handle])
    handle_faces = set(range(len(cup.faces), len(shape.faces)))
    locator = VisionLanguagePartLocator(
        detector=QwenBoxDetector(), segmenter=Sam2BoxSegmenter()
    )

    [located] = locator.locate(PartQuery.of_shape(shape, Mug, ["handle"]))

    found = set(located.faces)
    assert len(found & handle_faces) / len(found | handle_faces) > 0.5
