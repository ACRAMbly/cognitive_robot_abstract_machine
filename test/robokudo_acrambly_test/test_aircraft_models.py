# %% Imports

from __future__ import annotations

from pathlib import Path
from os import access, R_OK
from dataclasses import replace
from enum import StrEnum
from typing_extensions import TYPE_CHECKING

import numpy
import trimesh
import pytest

from robokudo_acrambly.aircraft.exceptions import MissingAircraftResource
from robokudo_acrambly.aircraft.models import (
    AircraftModel,
    AircraftModelCatalog,
)
from robokudo_acrambly.aircraft.types import AircraftPart
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from robokudo_acrambly.aircraft.models import AircraftModelDefinition
    from semantic_digital_twin.world import World


# %% Fixture resources


class AircraftFixtureResource(StrEnum):
    """
    Resource names used by aircraft catalog acceptance fixtures.
    """

    SOURCE_DIRECTORY = "plane"
    DATASET_DIRECTORY = "dataset"
    INVENTORY = "aircraft_models.csv"
    REPLACEMENT = "replacement.stl"


# %% Semantic model inventory


@pytest.mark.parametrize("model", list(AircraftModel))
def test_model_semantic_subtype_query(model: AircraftModel, mini_world: World) -> None:
    """
    Each configured kind retains its label and participates in subtype queries.
    """
    definition = model.value
    body = Body(name=PrefixedName(definition.label))
    annotation = definition.semantic_type(root=body, class_label=definition.label)
    with mini_world.modify_world():
        mini_world.add_connection(FixedConnection(parent=mini_world.root, child=body))
        mini_world.add_semantic_annotation(annotation)
    assert mini_world.get_semantic_annotations_by_type(AircraftPart) == [annotation]
    assert mini_world.get_semantic_annotations_by_type(definition.semantic_type) == [
        annotation
    ]
    assert annotation.class_label == definition.label
    assert annotation.root is body


# %% Resource loading


@pytest.mark.parametrize("model", list(AircraftModel))
def test_configured_model_geometry(model: AircraftModel) -> None:
    """
    Checked-in models retain their metric coordinates in both shape collections.
    """
    directory = (
        Path(__file__).resolve().parents[2] / AircraftFixtureResource.SOURCE_DIRECTORY
    )
    catalog = AircraftModelCatalog(directory)
    loaded = catalog.models[model.value.label]
    body = loaded.create_body(PrefixedName(model.value.label))
    source = trimesh.load_mesh(directory / model.value.resource)
    assert len(body.visual.shapes) == 1
    assert len(body.collision.shapes) == 1
    numpy.testing.assert_allclose(
        body.visual.shapes[0].mesh.bounds,
        source.bounds * model.value.metres_per_mesh_unit,
    )
    numpy.testing.assert_array_equal(
        body.visual.shapes[0].mesh.vertices, body.collision.shapes[0].mesh.vertices
    )
    assert body.visual.shapes[0].origin.reference_frame is body
    numpy.testing.assert_array_equal(body.visual.shapes[0].origin.to_np(), numpy.eye(4))


@pytest.mark.parametrize("model", list(AircraftModel))
def test_missing_model_resource(model: AircraftModel, tmp_path: Path) -> None:
    """
    Missing resources report their configured location and corrective action.
    """
    with pytest.raises(MissingAircraftResource) as caught:
        AircraftModelCatalog(tmp_path, definitions=(model.value,))
    assert caught.value.mesh_path == tmp_path / model.value.resource
    assert model.value.resource.name in caught.value.suggest_correction()


# %% Inventory and resource replacement


def test_model_inventory() -> None:
    """
    The configured semantic kinds and resources match the connector inventory.
    """
    inventory = (
        Path(__file__).parent
        / AircraftFixtureResource.DATASET_DIRECTORY
        / AircraftFixtureResource.INVENTORY
    )
    expected = [line.split(",") for line in inventory.read_text().splitlines()]
    assert [
        [model.value.label, model.value.resource.name] for model in AircraftModel
    ] == expected
    assert [model.value.semantic_type.__name__ for model in AircraftModel] == [
        label for label, _ in expected
    ]


@pytest.fixture
def replacement_model(tmp_path: Path) -> AircraftModelDefinition:
    """
    An offset mesh with a replacement resource and explicit metric conversion.
    """
    definition = replace(
        AircraftModel.BOTTOM_WING.value,
        resource=Path(AircraftFixtureResource.REPLACEMENT),
        metres_per_mesh_unit=0.5,
    )
    source = trimesh.creation.box(extents=[2.0, 4.0, 6.0])
    source.apply_translation([3.0, 5.0, 7.0])
    source.export(tmp_path / definition.resource)
    return definition


def test_replacement_preserves_label_and_offsets(
    tmp_path: Path, replacement_model: AircraftModelDefinition
) -> None:
    """
    Resource replacement preserves classification and source-local coordinates.
    """
    catalog = AircraftModelCatalog(tmp_path, definitions=(replacement_model,))
    assert list(catalog.models) == [AircraftModel.BOTTOM_WING.value.label]
    body = catalog.models[replacement_model.label].create_body(
        PrefixedName(replacement_model.label)
    )
    numpy.testing.assert_allclose(
        body.visual.shapes[0].mesh.bounds, [[1.0, 1.5, 2.0], [2.0, 3.5, 5.0]]
    )
    numpy.testing.assert_array_equal(body.visual.shapes[0].origin.to_np(), numpy.eye(4))


def test_loaded_model_survives_resource_removal(
    tmp_path: Path, replacement_model: AircraftModelDefinition
) -> None:
    """
    Body creation reuses loaded geometry after its source resource disappears.
    """
    catalog = AircraftModelCatalog(tmp_path, definitions=(replacement_model,))
    (tmp_path / replacement_model.resource).unlink()
    loaded = catalog.models[replacement_model.label]
    first = loaded.create_body(PrefixedName(replacement_model.label))
    second = loaded.create_body(PrefixedName(replacement_model.label))
    numpy.testing.assert_array_equal(
        first.visual.shapes[0].mesh.vertices, second.visual.shapes[0].mesh.vertices
    )
    numpy.testing.assert_array_equal(
        first.collision.shapes[0].mesh.vertices,
        second.collision.shapes[0].mesh.vertices,
    )


def test_bodies_have_independent_geometry(
    tmp_path: Path, replacement_model: AircraftModelDefinition
) -> None:
    """
    Mutating one body's shapes cannot change another body or its other collection.
    """
    loaded = AircraftModelCatalog(tmp_path, definitions=(replacement_model,)).models[
        replacement_model.label
    ]
    first = loaded.create_body(PrefixedName(replacement_model.label))
    second = loaded.create_body(PrefixedName(replacement_model.label))
    expected_vertices = second.visual.shapes[0].mesh.vertices.copy()
    first.visual.shapes[0].mesh.apply_translation([1.0, 2.0, 3.0])
    first.visual.shapes[0].origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        1.0, 2.0, 3.0, reference_frame=first
    )
    first.visual.shapes.clear()
    numpy.testing.assert_array_equal(
        second.visual.shapes[0].mesh.vertices, expected_vertices
    )
    numpy.testing.assert_array_equal(
        first.collision.shapes[0].mesh.vertices, expected_vertices
    )
    numpy.testing.assert_array_equal(
        second.collision.shapes[0].mesh.vertices, expected_vertices
    )
    numpy.testing.assert_array_equal(
        second.visual.shapes[0].origin.to_np(), numpy.eye(4)
    )
    assert second.visual.reference_frame is second
    assert second.collision.reference_frame is second
    assert second.visual.shapes[0].origin.reference_frame is second
    assert second.collision.shapes[0].origin.reference_frame is second


def test_unreadable_model_resource(
    tmp_path: Path, replacement_model: AircraftModelDefinition
) -> None:
    """
    Unreadable resources provide the same corrective guidance as missing files.
    """
    mesh_path = tmp_path / replacement_model.resource
    mesh_path.chmod(0)
    if access(mesh_path, R_OK):
        pytest.skip(
            "The current user can read files regardless of their permission bits."
        )
    with pytest.raises(MissingAircraftResource) as caught:
        AircraftModelCatalog(tmp_path, definitions=(replacement_model,))
    assert caught.value.mesh_path == mesh_path
