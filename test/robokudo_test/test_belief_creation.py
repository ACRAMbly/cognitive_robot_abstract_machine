# %% Imports

from __future__ import annotations

import numpy
import pytest
import trimesh
from typing_extensions import TYPE_CHECKING

from robokudo import world
from robokudo.exceptions import RoboKudoError
from semantic_digital_twin.exceptions import PathResolutionError
from robokudo.utils.transform import get_transform_matrix_from_q
from semantic_digital_twin.world_description.geometry import Mesh, Scale

from .dataset.belief_geometry import (
    GeometryResource,
    OffsetModelGeometry,
    ResourceModelGeometry,
    InsertionGeometry,
    ShapeCounts,
    CreationRoute,
)
from .dataset.semantic_observations import (
    ModelLabel,
    ObservedBlock,
    BeliefWorldSnapshot,
)
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
    from pathlib import Path

    from robokudo.annotators.semantic_world_connector import (
        SemanticDigitalTwinConnector,
    )
    from robokudo.cas import CAS
    from .dataset.semantic_observations import ObservationExample


# %% Configured geometry


@pytest.fixture
def model_geometry(tmp_path: Path) -> OffsetModelGeometry:
    """
    Load a scaled mesh whose local center differs from its source origin.
    """
    source = trimesh.creation.box(extents=[2.0, 4.0, 6.0])
    source.apply_translation([3.0, 5.0, 7.0])
    mesh_path = tmp_path / GeometryResource.OFFSET_MODEL
    source.export(mesh_path)
    geometry = Mesh(filename=str(mesh_path), scale=Scale(0.5, 0.5, 0.5))
    geometry.mesh
    return OffsetModelGeometry(geometry)


def test_configured_creation_has_complete_geometry_and_mesh_origin_pose(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
) -> None:
    """
    A model label and pose create both mesh collections at the observed origin.
    """
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry
    }
    hypothesis = observation_example.create_hypothesis()

    associations = configured_connector.associate_hypotheses_with_beliefs(
        [hypothesis], [], captured_cas
    )

    belief = associations[0][1]
    assert len(belief.body.visual.shapes) == 1
    assert len(belief.body.collision.shapes) == 1
    for collection in (belief.body.visual, belief.body.collision):
        shape = collection.shapes[0]
        assert isinstance(shape, Mesh)
        assert shape.scale == model_geometry.geometry.scale
        numpy.testing.assert_allclose(
            shape.mesh.bounds, [[1.0, 1.5, 2.0], [2.0, 3.5, 5.0]]
        )
        assert collection.reference_frame is belief.body
        assert shape.origin.reference_frame is belief.body
        numpy.testing.assert_array_equal(shape.origin.to_np(), numpy.eye(4))
    numpy.testing.assert_allclose(
        belief.body.global_transform.to_np(),
        get_transform_matrix_from_q(
            observation_example.world_pose.rotation,
            observation_example.world_pose.translation,
        ),
        atol=1e-12,
    )


# %% Typed registration


@pytest.mark.parametrize("with_geometry", [False, True])
def test_configured_creation_registers_one_typed_annotation_and_body_uuid(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
    with_geometry: bool,
) -> None:
    """
    Configured semantics register one annotation rooted at the persistent body.
    """
    if with_geometry:
        configured_connector.descriptor.parameters.model_geometry = {
            ModelLabel.BLOCK: model_geometry
        }
    hypothesis = observation_example.create_hypothesis()

    associations = configured_connector.associate_hypotheses_with_beliefs(
        [hypothesis], [], captured_cas
    )

    belief = associations[0][1]
    annotations = world.world_instance().get_semantic_annotations_by_type(ObservedBlock)
    assert len(annotations) == 1
    assert annotations[0].root is belief.body
    assert annotations[0].class_label == ModelLabel.BLOCK
    assert world.world_instance().semantic_annotations == annotations
    assert world.get_object_belief_states() == {belief.body.id: belief}
    assert (
        world.world_instance().get_kinematic_structure_entity_by_id(belief.uuid)
        is belief.body
    )
    assert belief.latest_hypothesis is hypothesis


# %% Preparation before insertion


def test_model_geometry_is_present_at_insertion(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
) -> None:
    """
    World model observers receive complete visual and collision collections.
    """
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry
    }
    observer = InsertionGeometry(_world=world.world_instance())
    associations = configured_connector.associate_hypotheses_with_beliefs(
        [observation_example.create_hypothesis()], [], captured_cas
    )
    observer.stop()
    assert observer.shape_counts == {associations[0][1].uuid: ShapeCounts(1, 1)}


@pytest.mark.parametrize("populated_world", [False, True])
def test_missing_model_resource_after_valid_hypothesis_preserves_persistent_state(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
    tmp_path: Path,
    populated_world: bool,
) -> None:
    """
    A later unavailable model prevents creation and updates of earlier objects.
    """
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry
    }
    if populated_world:
        configured_connector.associate_hypotheses_with_beliefs(
            [observation_example.create_hypothesis()], [], captured_cas
        )
    snapshot = BeliefWorldSnapshot.capture()
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry,
        ModelLabel.PANEL: ResourceModelGeometry(
            tmp_path / GeometryResource.MISSING_MODEL
        ),
    }
    missing_resource_hypothesis = observation_example.create_hypothesis()
    missing_resource_hypothesis.annotations[0].classname = ModelLabel.PANEL
    with pytest.raises(PathResolutionError):
        configured_connector.associate_hypotheses_with_beliefs(
            [observation_example.create_hypothesis(), missing_resource_hypothesis],
            list(world.get_object_belief_states().values()),
            captured_cas,
        )
    assert BeliefWorldSnapshot.capture() == snapshot


@pytest.mark.parametrize("creation_route", list(CreationRoute))
def test_association_creates_mesh_beliefs_for_rejected_or_unmatched_hypotheses(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
    model_geometry: OffsetModelGeometry,
    creation_route: CreationRoute,
) -> None:
    """
    Every association route that creates a belief retains configured geometry.
    """
    configured_connector.descriptor.parameters.model_geometry = {
        ModelLabel.BLOCK: model_geometry
    }
    initial_hypothesis = observation_example.create_hypothesis()
    initial_hypothesis.roi = _make_hypothesis().roi
    (previous,) = configured_connector.associate_hypotheses_with_beliefs(
        [initial_hypothesis], [], captured_cas
    )
    existing = previous[1]
    hypotheses = [observation_example.create_hypothesis()]
    if creation_route is CreationRoute.LOW_SIMILARITY:
        configured_connector.descriptor.parameters.confidence_threshold = 1.0
    else:
        extra_hypothesis = observation_example.create_hypothesis()
        extra_hypothesis.annotations[-1].translation[0] += 10.0
        hypotheses.append(extra_hypothesis)
    for hypothesis in hypotheses:
        hypothesis.roi = _make_hypothesis().roi
    _attach_connector_to_pipeline(configured_connector, captured_cas)
    associations = configured_connector.associate_hypotheses_with_beliefs(
        hypotheses, [existing], captured_cas
    )
    created = [belief for _, belief in associations if belief is not existing]
    assert len(created) == 1
    assert len(created[0].body.visual.shapes) == 1
    assert len(created[0].body.collision.shapes) == 1
    annotations = world.world_instance().get_semantic_annotations_by_type(ObservedBlock)
    assert len(annotations) == len(world.get_object_belief_states())
    assert (
        len(
            [
                annotation
                for annotation in annotations
                if annotation.root is created[0].body
            ]
        )
        == 1
    )


def test_missing_model_geometry_mapping_reports_configuration_error_before_creation(
    configured_connector: SemanticDigitalTwinConnector,
    captured_cas: CAS,
    observation_example: ObservationExample,
) -> None:
    """
    An enabled geometry mapping must contain every observed model label.
    """
    configured_connector.descriptor.parameters.model_geometry = {}
    snapshot = BeliefWorldSnapshot.capture()
    with pytest.raises(RoboKudoError):
        configured_connector.associate_hypotheses_with_beliefs(
            [observation_example.create_hypothesis()], [], captured_cas
        )
    assert BeliefWorldSnapshot.capture() == snapshot
