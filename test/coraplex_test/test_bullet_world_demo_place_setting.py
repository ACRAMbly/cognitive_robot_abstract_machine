"""
Tests for where the bullet world demo lays its place setting: each object is laid resting
on the table, so what watches the run sees it put there.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest
from typing_extensions import Tuple, Type

from semantic_digital_twin.reasoning.predicates import SupportedBy
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootKinematicStructureEntity,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bowl,
    Milk,
    Spoon,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

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

PLACE_SETTING_TYPES = [Milk, Bowl, Spoon]
"""
What the demo lays on the table.
"""


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


def _laid_on_the_table(
    bullet_world_demo: ModuleType,
    semantic_annotation_type: Type[HasRootKinematicStructureEntity],
) -> Tuple[World, Body, Body]:
    """
    Build the demo's scene and fix the object of ``semantic_annotation_type`` to the
    world where the demo lays it, as releasing it there leaves it.

    :return: The world, the object's body, and the table it is laid on.
    """
    demonstration = bullet_world_demo.BulletWorldDemonstration(used_robot=PR2)
    world = demonstration.build_simulated_world()
    demonstration.populate_scene(world)
    placed_object = next(
        candidate
        for candidate in demonstration.place_setting
        if candidate.semantic_annotation_type is semantic_annotation_type
    )
    body = placed_object.annotation_in(world).root
    target = placed_object.target_location(world).to_homogeneous_matrix()
    with world.modify_world():
        world.remove_connection(body.parent_connection)
        world.add_connection(
            FixedConnection(
                parent=world.root, child=body, parent_T_connection_expression=target
            )
        )
    table = world.get_body_by_name(bullet_world_demo.ApartmentBody.TABLE)
    return world, body, table


@pytest.mark.parametrize(
    "semantic_annotation_type",
    PLACE_SETTING_TYPES,
    ids=[laid.__name__ for laid in PLACE_SETTING_TYPES],
)
def test_each_object_is_laid_resting_on_the_table(
    bullet_world_demo, semantic_annotation_type
):
    _, body, table = _laid_on_the_table(bullet_world_demo, semantic_annotation_type)

    assert SupportedBy(body, table)()
