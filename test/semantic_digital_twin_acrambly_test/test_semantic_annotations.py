"""
Tests that the aircraft, its parts and the tetris blocks are semantic annotations a
world can be queried for.
"""

# %% Imports

from __future__ import annotations

import pytest

from krrood.entity_query_language.factories import a, an, entity, variable
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


@pytest.mark.parametrize("part_type", concrete_subclasses(AircraftPart))
def test_aircraft_part_is_found_by_its_class_and_as_an_aircraft_part(
    part_type: type[AircraftPart],
) -> None:
    """
    An aircraft part attached to a body is found by a query for its own class and by one
    for every aircraft part.
    """
    world = World.create_with_root_body("root")
    with world.modify_world():
        part = part_type.create_with_new_body_in_world(
            name=part_type.__name__, world=world
        )

    assert entity(variable(part_type, world.semantic_annotations)).tolist() == [part]
    assert an(AircraftPart).from_(domain=world.semantic_annotations).tolist() == [part]


# %% Tetris blocks


def test_tetris_blocks_are_the_straight_and_the_corner_block() -> None:
    """
    The concrete tetris blocks are exactly the straight and the corner block.
    """
    assert set(concrete_subclasses(TetrisBlock)) == {
        StraightTetrisBlock,
        CornerTetrisBlock,
    }


@pytest.mark.parametrize("block_type", concrete_subclasses(TetrisBlock))
def test_tetris_block_is_found_by_its_class_and_as_a_tetris_block(
    block_type: type[TetrisBlock],
) -> None:
    """
    A tetris block attached to a body is found by a query for its own class and by one
    for every tetris block.
    """
    world = World.create_with_root_body("root")
    with world.modify_world():
        block = block_type.create_with_new_body_in_world(
            name=block_type.__name__, world=world
        )

    assert entity(variable(block_type, world.semantic_annotations)).tolist() == [block]
    assert a(TetrisBlock).from_(domain=world.semantic_annotations).tolist() == [block]


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
def aircraft_world() -> World:
    """
    :return: An empty world for an aircraft to be assembled in.
    """
    return World.create_with_root_body("root")


@pytest.fixture
def aircraft(aircraft_world: World) -> Aircraft:
    """
    :return: An aircraft rooted at the body of its chassis, with no other part mounted.
    """
    with aircraft_world.modify_world():
        chassis = Chassis.create_with_new_body_in_world(
            name="chassis", world=aircraft_world
        )
        aircraft = Aircraft(root=chassis.root)
        aircraft_world.add_semantic_annotation(aircraft)
    return aircraft


def test_aircraft_with_only_its_chassis_has_no_parts(
    aircraft_world: World, aircraft: Aircraft
) -> None:
    """
    An aircraft rooted at the body of its chassis is found in the world before any other
    part is mounted, and has no parts yet.
    """
    assert an(Aircraft).from_(domain=aircraft_world.semantic_annotations).tolist() == [
        aircraft
    ]
    assert aircraft.parts == []


@pytest.mark.parametrize("part_type", aircraft_part_types_besides_chassis())
def test_mounting_a_part_fills_only_its_field(
    aircraft_world: World, aircraft: Aircraft, part_type: type[AircraftPart]
) -> None:
    """
    Mounting an aircraft part onto the aircraft fills the field of that part's class,
    leaves every other field empty, and attaches the part's body to the chassis body.
    """
    with aircraft_world.modify_world():
        part = part_type.create_with_new_body_in_world(
            name=part_type.__name__, world=aircraft_world
        )
    with aircraft_world.modify_world():
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
    aircraft_world: World, aircraft: Aircraft
) -> None:
    """
    The aircraft has one part-whole relationship per aircraft part besides the chassis,
    and once all of them are mounted the relationships report exactly those parts.
    """
    with aircraft_world.modify_world():
        parts = [
            part_type.create_with_new_body_in_world(
                name=part_type.__name__, world=aircraft_world
            )
            for part_type in aircraft_part_types_besides_chassis()
        ]
    with aircraft_world.modify_world():
        for part in parts:
            aircraft.add(part)

    relationships = PartRelationshipInspector.all_of_type(Aircraft)
    assert {relationship.part_type for relationship in relationships} == set(
        aircraft_part_types_besides_chassis()
    )
    assert {
        part
        for relationship in relationships
        for part in relationship.parts_of(aircraft)
    } == set(parts)
