"""
Tests of :class:`robokudo.annotators.mesh_shape.MeshShapeAnnotator`, which gives the
classified object hypotheses of a pass that have a pose the mesh of their class.
"""

from __future__ import annotations

from dataclasses import replace

import numpy
import pytest
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.geometry import Mesh

from robokudo.annotators.foundation_pose import ObjectClassMesh
from robokudo.annotators.mesh_shape import MeshShapeAnnotator
from robokudo.cas import CAS
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import PoseAnnotation, Shape
from robokudo.types.scene import ObjectHypothesis

from ._object_hypotheses import make_object_hypothesis

# %% fixtures


@pytest.fixture
def class_mesh() -> ObjectClassMesh:
    """
    A small box in meters as the mesh of the cups.
    """
    return ObjectClassMesh(class_name="cup", shape=Mesh.box(extents=(0.04, 0.06, 0.1)))


def annotate_mesh_shapes(
    *, class_mesh: ObjectClassMesh, hypotheses: list[ObjectHypothesis]
) -> None:
    """
    Run an annotator configured with ``class_mesh`` on a CAS holding ``hypotheses``.
    """
    annotator = MeshShapeAnnotator(class_meshes=[class_mesh])
    pipeline = Pipeline("MeshShapes")
    pipeline.add_child(annotator)
    cas = CAS()
    cas.annotations.extend(hypotheses)
    pipeline.cas = cas
    annotator.tick_once()


# %% hypotheses that get a mesh shape


def test_classified_hypothesis_with_a_pose_gets_the_mesh_of_its_class_at_that_pose(
    *, class_mesh: ObjectClassMesh
) -> None:
    hypothesis = make_object_hypothesis(
        translation=(0.1, 0.2, 0.9), class_label=class_mesh.class_name
    )
    hypothesis.latest_annotation_of_type(annotation_type=PoseAnnotation).rotation = [
        0.0,
        0.0,
        numpy.sqrt(0.5),
        numpy.sqrt(0.5),
    ]

    annotate_mesh_shapes(class_mesh=class_mesh, hypotheses=[hypothesis])

    [shape] = [
        annotation
        for annotation in hypothesis.annotations
        if isinstance(annotation, Shape)
    ]
    quarter_turn_about_z_at_the_translation = numpy.array(
        [
            [0.0, -1.0, 0.0, 0.1],
            [1.0, 0.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.9],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    assert shape.geometry == replace(
        class_mesh.shape,
        origin=HomogeneousTransformationMatrix(
            data=quarter_turn_about_z_at_the_translation
        ),
    )


# %% hypotheses left unchanged


@pytest.mark.parametrize("class_label", ["plate", None])
def test_hypothesis_whose_class_has_no_mesh_is_left_unchanged(
    *, class_mesh: ObjectClassMesh, class_label: str | None
) -> None:
    hypothesis = make_object_hypothesis(class_label=class_label)
    annotations_before = list(hypothesis.annotations)

    annotate_mesh_shapes(class_mesh=class_mesh, hypotheses=[hypothesis])

    assert hypothesis.annotations == annotations_before


def test_hypothesis_without_a_pose_is_left_unchanged(
    *, class_mesh: ObjectClassMesh
) -> None:
    hypothesis = make_object_hypothesis(class_label=class_mesh.class_name)
    hypothesis.annotations = [
        annotation
        for annotation in hypothesis.annotations
        if not isinstance(annotation, PoseAnnotation)
    ]
    annotations_before = list(hypothesis.annotations)

    annotate_mesh_shapes(class_mesh=class_mesh, hypotheses=[hypothesis])

    assert hypothesis.annotations == annotations_before
