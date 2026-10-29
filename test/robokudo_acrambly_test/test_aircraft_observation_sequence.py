# %% Imports

from __future__ import annotations

import numpy
import pytest
import trimesh
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

import robokudo.world as world
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import StampedPoseAnnotation
from robokudo.types.scene import ObjectHypothesis
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.types import AircraftPart

from ..robokudo_test.dataset.semantic_observations import BeliefSnapshot
from .dataset.aircraft_observations import (
    AircraftIdentity,
    AcceptedAircraftPart,
    AircraftSequence,
    AircraftStage,
)
from .test_aircraft_belief_creation import (
    aircraft_catalog,
    aircraft_connector,
    captured_cas,
    observation_example,
    semdt_cas,
)

if TYPE_CHECKING:
    from robokudo.annotators.semantic_world_connector import (
        SemanticDigitalTwinConnector,
    )
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog


# %% Local aircraft pipeline


@pytest.fixture
def aircraft_sequence() -> AircraftSequence:
    """
    Supply all six deterministic captures independently of processing time.
    """
    return AircraftSequence()


def test_aircraft_pipeline_retains_parts_through_movement_absence_and_reappearance(
    aircraft_catalog: AircraftModelCatalog,
    aircraft_connector: SemanticDigitalTwinConnector,
    aircraft_sequence: AircraftSequence,
    semdt_cas: CAS,
) -> None:
    """
    All configured kinds retain identity and geometry throughout six captures.
    """
    pipeline = Pipeline()
    pipeline.add_child(aircraft_connector)
    accepted_parts: dict[AircraftModel, AcceptedAircraftPart] = {}
    source_meshes = {
        model: trimesh.load_mesh(aircraft_catalog.mesh_directory / model.value.resource)
        for model in AircraftModel
    }

    for stage in AircraftStage:
        pipeline.create_new_cas()
        current_cas = pipeline.get_cas()
        current_cas.world_frame = semdt_cas.world_frame
        current_cas.camera_frame = semdt_cas.camera_frame
        observations = aircraft_sequence.populate(current_cas, stage)

        assert aircraft_connector.compute() is Status.SUCCESS

        expected_count = len(AircraftModel)
        if stage is AircraftStage.ABSENT:
            expected_count = 0
        elif stage is AircraftStage.PARTLY_ABSENT:
            expected_count -= len(aircraft_sequence.absent_models)
        assert (
            len(current_cas.filter_annotations_by_type(ObjectHypothesis))
            == expected_count
        )
        assert len(world.get_object_belief_states()) == len(AircraftModel)
        belief_world = world.world_instance()
        assert len(belief_world.get_semantic_annotations_by_type(AircraftPart)) == len(
            AircraftModel
        )
        assert len(
            {belief.uuid for belief in world.get_object_belief_states().values()}
        ) == len(AircraftModel)

        for model in AircraftModel:
            (annotation,) = belief_world.get_semantic_annotations_by_type(
                model.value.semantic_type
            )
            belief = world.get_object_belief_state(annotation.root.id)
            if stage is AircraftStage.INITIAL:
                identity = AircraftIdentity(
                    belief,
                    belief.body,
                    belief.uuid,
                    annotation,
                    list(belief.body.visual.shapes),
                    list(belief.body.collision.shapes),
                )
                accepted_parts[model] = AcceptedAircraftPart(
                    identity,
                    BeliefSnapshot.from_belief(belief),
                    belief.latest_hypothesis.id,
                    stage,
                )
            accepted = accepted_parts[model]
            original = accepted.identity
            assert belief is original.belief
            assert belief.body is original.body
            assert belief.uuid == original.identifier
            assert annotation is original.annotation
            assert annotation.class_label == model.value.label

            if model in observations:
                assert belief.latest_hypothesis is observations[model]
                if stage is not AircraftStage.INITIAL:
                    assert belief.latest_hypothesis.id != accepted.detection_identifier
                accepted.detection_identifier = belief.latest_hypothesis.id
                accepted.stage = stage
                (accepted_pose,) = current_cas.filter_by_type(
                    StampedPoseAnnotation, belief.latest_hypothesis.annotations
                )
                assert accepted_pose.frame == current_cas.world_frame
                assert (
                    accepted_pose.timestamp.sec
                    == aircraft_sequence.capture.capture_time.sec + int(stage)
                )
                assert (
                    accepted_pose.timestamp.nanosec
                    == aircraft_sequence.capture.capture_time.nanosec
                )
            else:
                assert BeliefSnapshot.from_belief(belief) == accepted.snapshot

            expected_transform = aircraft_sequence.expected_transform(
                model, accepted.stage
            )
            numpy.testing.assert_allclose(
                belief.body.global_transform.to_np(), expected_transform, atol=1e-12
            )
            source = source_meshes[model]
            for collection, retained_shapes in zip(
                (belief.body.visual, belief.body.collision),
                (original.visual_shapes, original.collision_shapes),
            ):
                assert len(collection.shapes) == len(retained_shapes)
                for shape, retained_shape in zip(collection.shapes, retained_shapes):
                    assert shape is retained_shape
                    assert collection.reference_frame is belief.body
                    assert shape.origin.reference_frame is belief.body
                    numpy.testing.assert_allclose(
                        shape.origin.to_np(), numpy.eye(4), atol=1e-12
                    )
                    numpy.testing.assert_allclose(
                        shape.mesh.bounds,
                        source.bounds * model.value.metres_per_mesh_unit,
                    )
                    local_center = numpy.append(shape.mesh.centroid, 1.0)
                    expected_center = numpy.append(
                        source.centroid * model.value.metres_per_mesh_unit, 1.0
                    )
                    numpy.testing.assert_allclose(
                        belief.body.global_transform.to_np()
                        @ shape.origin.to_np()
                        @ local_center,
                        expected_transform @ expected_center,
                        atol=1e-12,
                    )
            accepted.snapshot = BeliefSnapshot.from_belief(belief)
