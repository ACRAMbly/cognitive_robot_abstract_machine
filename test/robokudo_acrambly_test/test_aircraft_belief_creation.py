# %% Imports

from __future__ import annotations

from pathlib import Path

import numpy
import pytest
import trimesh
from typing_extensions import TYPE_CHECKING

from robokudo import world
from robokudo.annotators.semantic_world_connector import SemanticDigitalTwinConnector
from robokudo_acrambly.aircraft.models import AircraftModel, AircraftModelCatalog
from robokudo_acrambly.aircraft.types import AircraftPart

from ..robokudo_test.test_semantic_observations import (
    captured_cas,
    observation_example,
    semdt_cas,
)
from .test_aircraft_models import AircraftFixtureResource

if TYPE_CHECKING:
    from robokudo.cas import CAS
    from ..robokudo_test.dataset.semantic_observations import ObservationExample


# %% Application configuration


@pytest.fixture(scope="module")
def aircraft_catalog() -> AircraftModelCatalog:
    """
    Load the actual aircraft resources once for connector creation coverage.
    """
    return AircraftModelCatalog(
        Path(__file__).resolve().parents[2] / AircraftFixtureResource.SOURCE_DIRECTORY
    )


@pytest.fixture
def aircraft_connector(
    aircraft_catalog: AircraftModelCatalog,
) -> SemanticDigitalTwinConnector:
    """
    Supply application-owned semantics and geometry through generic configuration.
    """
    descriptor = SemanticDigitalTwinConnector.Descriptor()
    descriptor.parameters.semantic_types = {
        label: model.definition.semantic_type
        for label, model in aircraft_catalog.models.items()
    }
    descriptor.parameters.model_geometry = {
        label: model for label, model in aircraft_catalog.models.items()
    }
    return SemanticDigitalTwinConnector(descriptor=descriptor)


# %% Actual model creation


def test_all_aircraft_models_create_typed_mesh_beliefs(
    aircraft_catalog: AircraftModelCatalog,
    aircraft_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    All aircraft kinds enter the local world without per-observation geometry.
    """
    hypotheses = [observation_example.create_hypothesis() for model in AircraftModel]
    for hypothesis, model in zip(hypotheses, AircraftModel):
        hypothesis.annotations[0].classname = model.value.label

    associations = aircraft_connector.associate_hypotheses_with_beliefs(
        hypotheses, [], captured_cas
    )

    belief_world = world.world_instance()
    assert len(associations) == len(AircraftModel)
    assert len(world.get_object_belief_states()) == len(AircraftModel)
    assert len({belief.uuid for _, belief in associations}) == len(AircraftModel)
    assert len(belief_world.get_semantic_annotations_by_type(AircraftPart)) == len(
        AircraftModel
    )
    for (_, belief), model in zip(associations, AircraftModel):
        definition = model.value
        (annotation,) = belief_world.get_semantic_annotations_by_type(
            definition.semantic_type
        )
        assert annotation.root is belief.body
        assert annotation.class_label == definition.label
        assert world.get_object_belief_state(belief.uuid) is belief
        source = trimesh.load_mesh(
            aircraft_catalog.mesh_directory / definition.resource
        )
        for collection in (belief.body.visual, belief.body.collision):
            (shape,) = collection.shapes
            numpy.testing.assert_allclose(
                shape.mesh.bounds, source.bounds * definition.metres_per_mesh_unit
            )
            assert collection.reference_frame is belief.body
            assert shape.origin.reference_frame is belief.body
            numpy.testing.assert_array_equal(shape.origin.to_np(), numpy.eye(4))
        numpy.testing.assert_allclose(
            belief.body.global_transform.to_np()[:3, 3],
            observation_example.world_pose.translation,
        )
