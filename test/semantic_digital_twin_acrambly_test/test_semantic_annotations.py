"""
Tests that the aircraft, its parts and the tetris blocks are semantic annotations a
world can be queried for.
"""

# %% Imports

from __future__ import annotations

import pytest

from krrood.entity_query_language.factories import an, entity, variable
from krrood.patterns.specificity_ranking import concrete_subclasses
from semantic_digital_twin.part_finding.relationships import PartRelationshipInspector
from semantic_digital_twin.world import World
from semantic_digital_twin_acrambly.semantic_annotations import (
    Aircraft,
    AircraftPart,
    BottomWing,
    Chassis,
    CornerTetrisBlock,
    FrontWheel,
    LeftRearWheel,
    MotorGrill,
    Propeller,
    RearWing,
    RightRearWheel,
    StraightTetrisBlock,
    TetrisBlock,
    TopWing,
    UnderBody,
    UpperBody,
    Window,
)

# %% World


@pytest.fixture
def world() -> World:
    """
    :return: An empty world.
    """
    return World.create_with_root_body("root")


# %% Aircraft parts


def test_aircraft_parts_are_the_parts_of_the_aircraft() -> None:
    """
    The concrete aircraft parts are exactly the twelve pieces the aircraft is assembled
    from.
    """
    assert set(concrete_subclasses(AircraftPart)) == {
        BottomWing,
        Chassis,
        FrontWheel,
        MotorGrill,
        Propeller,
        LeftRearWheel,
        RightRearWheel,
        RearWing,
        TopWing,
        UnderBody,
        UpperBody,
        Window,
    }


# %% Tetris blocks


def test_tetris_blocks_are_the_straight_and_the_corner_block() -> None:
    """
    The concrete tetris blocks are exactly the straight and the corner block.
    """
    assert set(concrete_subclasses(TetrisBlock)) == {
        StraightTetrisBlock,
        CornerTetrisBlock,
    }


# %% Queries for parts


@pytest.mark.parametrize(
    "abstract_type, part_type",
    [(AircraftPart, part_type) for part_type in concrete_subclasses(AircraftPart)]
    + [(TetrisBlock, block_type) for block_type in concrete_subclasses(TetrisBlock)],
)
def test_part_is_found_by_its_class_and_by_its_abstract_class(
    world: World,
    abstract_type: type[AircraftPart | TetrisBlock],
    part_type: type[AircraftPart | TetrisBlock],
) -> None:
    """
    An aircraft part or a tetris block attached to a body is found by a query for its
    own class and by one for every aircraft part or every tetris block.
    """
    with world.modify_world():
        part = part_type.create_with_new_body_in_world(
            name=part_type.__name__, world=world
        )

    assert entity(variable(part_type, world.semantic_annotations)).tolist() == [part]
    assert entity(variable(abstract_type, world.semantic_annotations)).tolist() == [
        part
    ]


# %% Aircraft


def aircraft_part_types_besides_chassis() -> list[type[AircraftPart]]:
    """
    :return: The concrete aircraft parts that are mounted onto the chassis.
    """
    return [
        part_type
        for part_type in concrete_subclasses(AircraftPart)
        if part_type is not Chassis
    ]


@pytest.fixture
def aircraft(world: World) -> Aircraft:
    """
    :return: An aircraft rooted at the body of its chassis, with no other part mounted.
    """
    with world.modify_world():
        chassis = Chassis.create_with_new_body_in_world(name="chassis", world=world)
        aircraft = Aircraft(root=chassis.root)
        world.add_semantic_annotation(aircraft)
    return aircraft


def test_aircraft_with_only_its_chassis_has_no_parts(
    world: World, aircraft: Aircraft
) -> None:
    """
    An aircraft rooted at the body of its chassis is found in the world before any other
    part is mounted, and has no parts yet.
    """
    assert an(Aircraft).from_(domain=world.semantic_annotations).tolist() == [aircraft]
    assert aircraft.parts == []


@pytest.mark.parametrize("part_type", aircraft_part_types_besides_chassis())
def test_mounting_a_part_fills_only_its_field(
    world: World, aircraft: Aircraft, part_type: type[AircraftPart]
) -> None:
    """
    Mounting an aircraft part onto the aircraft fills the field of that part's class,
    leaves every other field empty, and attaches the part's body to the chassis body.
    """
    with world.modify_world():
        part = part_type.create_with_new_body_in_world(
            name=part_type.__name__, world=world
        )
    with world.modify_world():
        aircraft.add(part)

    filled_relationships = [
        relationship
        for relationship in PartRelationshipInspector.all_of_type(Aircraft)
        if relationship.is_filled_in(aircraft)
    ]
    assert [relationship.part_type for relationship in filled_relationships] == [
        part_type
    ]
    assert aircraft.parts == [part]
    assert part.root.parent_kinematic_structure_entity is aircraft.root


def test_part_relationships_report_every_mounted_part_of_the_aircraft(
    world: World, aircraft: Aircraft
) -> None:
    """
    The aircraft has one part-whole relationship per aircraft part besides the chassis,
    and once all of them are mounted the relationships report exactly those parts.
    """
    part_types = aircraft_part_types_besides_chassis()
    with world.modify_world():
        parts = [
            part_type.create_with_new_body_in_world(
                name=part_type.__name__, world=world
            )
            for part_type in part_types
        ]
    with world.modify_world():
        for part in parts:
            aircraft.add(part)

    relationships = PartRelationshipInspector.all_of_type(Aircraft)
    assert {relationship.part_type for relationship in relationships} == set(part_types)
    assert {
        part
        for relationship in relationships
        for part in relationship.parts_of(aircraft)
    } == set(parts)
