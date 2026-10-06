"""
Tests for where the bullet world demo puts its place setting: each object starts resting
on something and is laid resting on the table, so what watches the run sees it taken up
and put down.
"""

from __future__ import annotations

import importlib.util
from enum import StrEnum
from pathlib import Path
from types import ModuleType

import pytest

from semantic_digital_twin.api import WorldSpecification
from semantic_digital_twin.reasoning.predicates import SupportedBy
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bowl,
    Milk,
    Spoon,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection

DEMO_PATH = (
    Path(__file__).resolve().parents[2]
    / "coraplex"
    / "demos"
    / "coraplex_bullet_world_demo"
    / "demo.py"
)
"""
The bullet world demo, which lives outside any package and is loaded from its file.
"""


class ApartmentSurface(StrEnum):
    """
    The apartment's bodies the place setting rests on.
    """

    COUNTER = "island_countertop"
    SPOON_DRAWER = "cabinet10_drawer_top"
    TABLE = "table_area_main"


@pytest.fixture(scope="module")
def bullet_world_demo() -> ModuleType:
    """
    :return: The bullet world demo, loaded from its file.
    """
    specification = importlib.util.spec_from_file_location(
        "bullet_world_demo", DEMO_PATH
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _apartment_with_the_place_setting(bullet_world_demo: ModuleType) -> World:
    """
    :return: The apartment alone, without a robot, with the place setting where the demo
        starts it.
    """
    world = WorldSpecification.from_urdf(
        bullet_world_demo.SceneFile.APARTMENT.path
    ).to_domain_object()
    for placed_object in bullet_world_demo.BulletWorldDemonstration(
        used_robot=PR2
    ).place_setting:
        placed_object.spawn(world)
    return world


@pytest.fixture(scope="module")
def apartment_at_the_start(bullet_world_demo) -> World:
    """
    :return: The apartment with the place setting where the demo starts it.
    """
    return _apartment_with_the_place_setting(bullet_world_demo)


@pytest.fixture(scope="module")
def apartment_laid_out(bullet_world_demo) -> World:
    """
    :return: The apartment with the place setting fixed where the demo lays it, as
        releasing each object there leaves it.
    """
    world = _apartment_with_the_place_setting(bullet_world_demo)
    for placed_object in bullet_world_demo.BulletWorldDemonstration(
        used_robot=PR2
    ).place_setting:
        body = placed_object.annotation_in(world).root
        target = placed_object.target_location(world).to_homogeneous_matrix()
        with world.modify_world():
            world.remove_connection(body.parent_connection)
            world.add_connection(
                FixedConnection(
                    parent=world.root, child=body, parent_T_connection_expression=target
                )
            )
    return world


@pytest.mark.parametrize(
    "semantic_annotation_type, surface",
    [
        (Milk, ApartmentSurface.COUNTER),
        (Bowl, ApartmentSurface.COUNTER),
        (Spoon, ApartmentSurface.SPOON_DRAWER),
    ],
    ids=["Milk", "Bowl", "Spoon"],
)
def test_each_object_starts_resting_on_its_surface(
    apartment_at_the_start, semantic_annotation_type, surface
):
    world = apartment_at_the_start

    assert SupportedBy(
        world.get_body_by_name(semantic_annotation_type.__name__),
        world.get_body_by_name(surface),
    )()


@pytest.mark.parametrize(
    "semantic_annotation_type", [Milk, Bowl, Spoon], ids=["Milk", "Bowl", "Spoon"]
)
def test_each_object_is_laid_resting_on_the_table(
    apartment_laid_out, semantic_annotation_type
):
    world = apartment_laid_out

    assert SupportedBy(
        world.get_body_by_name(semantic_annotation_type.__name__),
        world.get_body_by_name(ApartmentSurface.TABLE),
    )()
