"""
Expected registration failures leave other targets available for publication.
"""

from __future__ import annotations

from types import ModuleType
from unittest.mock import Mock

import numpy as np
import torch

from .dataset.publication import MaskedTarget, PublishedPose, TargetClassification

# %% independent target outcomes


def test_insufficient_depth_publishes_no_pose_and_continues(
    estimator_module: ModuleType, annotator_module: ModuleType
) -> None:
    """
    Retain the typed failure and publish the next target's successful pose.
    """
    side = 4
    region = Mock()
    region.mask = np.ones((side, side), dtype=np.uint8)
    region.roi.pos.x = region.roi.pos.y = 0
    region.roi.width = region.roi.height = side
    targets = [
        MaskedTarget(region, [TargetClassification(identity)]) for identity in (1, 2)
    ]
    failure = estimator_module.InsufficientDepth(0)
    pose = torch.eye(4)[None]
    pose[0, :3, 3] = torch.tensor([1.0, 2.0, 3.0])
    stage = object.__new__(annotator_module.FoundationPoseAnnotator)
    stage.descriptor = annotator_module.FoundationPoseAnnotator.Descriptor()
    stage.descriptor.parameters = stage.descriptor.Parameters()
    stage.descriptor.parameters.global_with_depth = False
    stage.descriptor.parameters.global_with_visualization = False
    stage.with_estimation = True
    stage.with_tracking = False
    stage.twc = np.eye(4)
    stage.est = Mock()
    stage.est.register.side_effect = [failure, pose]
    stage.obj_id_to_mesh_data = {identity: (None, None, None) for identity in (1, 2)}
    scene = Mock()
    scene.filter_annotations_by_type.return_value = targets
    scene.get_copy.return_value = np.zeros((side, side, 3), dtype=np.uint8)
    scene.get.side_effect = [
        np.ones((side, side), dtype=np.float32),
        (1.0, 1.0),
        Mock(intrinsic_matrix=np.eye(3)),
    ]
    stage.get_cas = Mock(return_value=scene)

    assert stage.compute() == annotator_module.py_trees.common.Status.SUCCESS
    assert targets[0].annotations == [
        TargetClassification(1),
        annotator_module.InsufficientDepthAnnotation(
            outcome=failure, source=annotator_module.AnnotationSource.ESTIMATION
        ),
    ]
    assert len(targets[1].annotations) == 2
    published_pose = targets[1].annotations[1]
    assert isinstance(published_pose, PublishedPose)
    assert published_pose.translation == pose[0, :3, 3].tolist()
