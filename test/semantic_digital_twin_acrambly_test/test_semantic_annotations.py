"""
Tests that the aircraft parts and tetris blocks are semantic annotations a world can be
queried for.
"""

# %% Imports

from __future__ import annotations

import pytest

from krrood.entity_query_language.factories import a, an, entity, variable
from krrood.patterns.specificity_ranking import concrete_subclasses
from semantic_digital_twin.world import World
from semantic_digital_twin_acrambly.semantic_annotations import (
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
