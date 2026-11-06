# %% Imports

from __future__ import annotations

from copy import deepcopy

import numpy
import pytest
from typing_extensions import TYPE_CHECKING

from robokudo import world

from .dataset.semantic_observations import (
    ModelLabel,
    BeliefSnapshot,
    BeliefWorldSnapshot,
)
from .test_belief_creation import model_geometry
from .test_semantic_observations import (
    captured_cas,
    configured_connector,
    observation_example,
    semdt_cas,
)
from .test_semantic_world_connector import (
    _attach_connector_to_pipeline,
    _make_hypothesis,
)

if TYPE_CHECKING:
    from robokudo.annotators.semantic_world_connector import (
        SemanticDigitalTwinConnector,
    )
    from robokudo.cas import CAS
    from .dataset.semantic_observations import ObservationExample
    from .dataset.belief_geometry import OffsetModelGeometry

# %% Model compatibility


def test_different_models_cannot_match_identical_geometry(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """A different known model starts a new belief despite identical image and pose evidence."""
    _attach_connector_to_pipeline(configured_connector, captured_cas)
    initial = observation_example.create_hypothesis()
    initial.roi = _make_hypothesis().roi
    ((_, existing),) = configured_connector.associate_hypotheses_with_beliefs(
        [initial], [], captured_cas
    )
    snapshot = BeliefSnapshot.from_belief(existing)
    changed_kind = observation_example.create_hypothesis()
    changed_kind.roi = deepcopy(initial.roi)
    changed_kind.annotations[0].classname = ModelLabel.PANEL

    ((_, accepted),) = configured_connector.associate_hypotheses_with_beliefs(
        [changed_kind], [existing], captured_cas
    )

    assert accepted is not existing
    assert len(world.get_object_belief_states()) == 2
    assert BeliefSnapshot.from_belief(existing) == snapshot


# %% Geometry-preserving accepted poses


def test_pose_only_updates_preserve_geometry_and_identity(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
) -> None:
    """Accepted translations and rotations retain the model and rotate its local offset."""
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry
    }
    _attach_connector_to_pipeline(configured_connector, captured_cas)
    initial = observation_example.create_hypothesis()
    ((_, belief),) = configured_connector.associate_hypotheses_with_beliefs(
        [initial], [], captured_cas
    )
    body = belief.body
    identifier = belief.uuid
    annotations = list(world.world_instance().semantic_annotations)
    visual_shapes = list(body.visual.shapes)
    collision_shapes = list(body.collision.shapes)
    moved = deepcopy(observation_example.camera_pose)
    moved.translation[0] += 0.1
    moved.rotation = [0.0, 0.0, 0.5**0.5, 0.5**0.5]
    hypothesis = observation_example.create_hypothesis(moved)
    hypothesis.annotations[0].confidence = 0.0
    observation_example.capture_time.nanosec += 1
    captured_cas.data_timestamp += 1

    ((_, accepted),) = configured_connector.associate_hypotheses_with_beliefs(
        [hypothesis], [belief], captured_cas
    )

    assert accepted is belief
    assert accepted.body is body
    assert accepted.uuid == identifier
    assert accepted.latest_hypothesis is hypothesis
    assert hypothesis.annotations[-1].timestamp == observation_example.capture_time
    assert world.world_instance().semantic_annotations == annotations
    assert all(
        left is right
        for left, right in zip(world.world_instance().semantic_annotations, annotations)
    )
    assert len(body.visual.shapes) == len(visual_shapes)
    assert len(body.collision.shapes) == len(collision_shapes)
    assert body.visual.shapes[0] is visual_shapes[0]
    assert body.collision.shapes[0] is collision_shapes[0]
    expected_pose = numpy.array(
        [
            [-1.0, 0.0, 0.0, 8.0],
            [0.0, -1.0, 0.0, 21.1],
            [0.0, 0.0, 1.0, 33.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    numpy.testing.assert_allclose(
        body.global_transform.to_np(), expected_pose, atol=1e-12
    )
    center = body.visual.shapes[0].mesh.centroid
    numpy.testing.assert_allclose(
        (body.global_transform.to_np() @ numpy.append(center, 1))[:3],
        [6.5, 18.6, 36.5],
        atol=1e-12,
    )


# %% Retained absence


def test_partial_and_total_absence_preserve_last_accepted_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
) -> None:
    """An absent model retains its last observation, pose, capture time and geometry."""
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry,
        ModelLabel.PANEL: model_geometry,
    }
    _attach_connector_to_pipeline(configured_connector, captured_cas)
    initial = observation_example.create_hypothesis()
    panel = observation_example.create_hypothesis()
    panel.annotations[0].classname = ModelLabel.PANEL
    associations = configured_connector.associate_hypotheses_with_beliefs(
        [initial, panel], [], captured_cas
    )
    block_belief, panel_belief = [belief for _, belief in associations]
    absent_snapshot = BeliefSnapshot.from_belief(panel_belief)
    visible = observation_example.create_hypothesis()
    visible.annotations[-1].translation[0] += 0.1
    captured_cas.data_timestamp += 1

    ((_, accepted),) = configured_connector.associate_hypotheses_with_beliefs(
        [visible], [panel_belief, block_belief], captured_cas
    )

    assert accepted is block_belief
    assert block_belief.latest_hypothesis is visible
    assert BeliefSnapshot.from_belief(panel_belief) == absent_snapshot
    assert len(world.get_object_belief_states()) == len(associations)
    snapshot = BeliefWorldSnapshot.capture()
    assert (
        configured_connector.associate_hypotheses_with_beliefs(
            [], [block_belief, panel_belief], captured_cas
        )
        == []
    )
    assert BeliefWorldSnapshot.capture() == snapshot


# %% Hard association exclusion


def test_different_models_remain_incompatible_with_negative_similarity_threshold(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """Changing the similarity threshold cannot override model-kind incompatibility."""
    _attach_connector_to_pipeline(configured_connector, captured_cas)
    ((_, existing),) = configured_connector.associate_hypotheses_with_beliefs(
        [observation_example.create_hypothesis()], [], captured_cas
    )
    configured_connector.descriptor.parameters.confidence_threshold = -2.0
    changed_kind = observation_example.create_hypothesis()
    changed_kind.annotations[0].classname = ModelLabel.PANEL

    ((_, accepted),) = configured_connector.associate_hypotheses_with_beliefs(
        [changed_kind], [existing], captured_cas
    )

    assert accepted is not existing
    assert len(world.get_object_belief_states()) == 2
