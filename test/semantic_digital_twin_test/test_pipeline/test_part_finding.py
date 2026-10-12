"""
Finding an object's handle or rim in its shape and splitting it off.
"""

import pytest
from typing_extensions import Optional
import trimesh

from krrood.adapters.json_serializer import from_json, to_json
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.pipeline.handle_finding import (
    NarrowEndHandleFinder,
    ProtrudingHandleFinder,
)
from semantic_digital_twin.pipeline.part_splitting import SplitPartFromShape
from semantic_digital_twin.pipeline.rim_finding import RimFinder
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% shapes

BODY_OUTER_RADIUS = 0.04
"""
Outer radius of the round body of the synthetic mug.
"""

HANDLE_REACH = 0.02
"""
How far the synthetic mug's handle sticks out of its body.
"""

TOOL_HANDLE_LENGTH = 0.1
"""
Length of the synthetic tool's handle, which runs from x = -0.1 to x = 0.
"""

TOOL_HANDLE_WIDTH = 0.01
"""
Width of the synthetic tool's handle.
"""


def cup_without_handle() -> trimesh.Trimesh:
    """
    :return: A round body with a wall, open at the top.
    """
    return trimesh.creation.annulus(
        r_min=BODY_OUTER_RADIUS - 0.005, r_max=BODY_OUTER_RADIUS, height=0.1
    )


def mug() -> trimesh.Trimesh:
    """
    :return: The cup with a box-shaped handle sticking out towards positive x.
    """
    handle = trimesh.creation.box(extents=[HANDLE_REACH, 0.01, 0.06])
    handle.apply_translation([BODY_OUTER_RADIUS + HANDLE_REACH / 2, 0.0, 0.0])
    return trimesh.util.concatenate([cup_without_handle(), handle])


def tool() -> trimesh.Trimesh:
    """
    :return: A flat tool, its narrow handle towards negative x and its wide head
        towards positive x.
    """
    handle = trimesh.creation.box(
        extents=[TOOL_HANDLE_LENGTH, TOOL_HANDLE_WIDTH, 0.005]
    )
    handle.apply_translation([-TOOL_HANDLE_LENGTH / 2, 0.0, 0.0])
    head = trimesh.creation.box(extents=[0.05, 0.04, 0.005])
    head.apply_translation([0.025, 0.0, 0.0])
    return trimesh.util.concatenate([handle, head])


# %% handles sticking out of a round body


def test_the_handle_of_a_mug_is_what_sticks_out_of_its_round_body():
    split = ProtrudingHandleFinder().split(mug())

    lowest, highest = split.part.bounds
    assert lowest[0] >= BODY_OUTER_RADIUS - 1e-9
    assert highest[0] == pytest.approx(BODY_OUTER_RADIUS + HANDLE_REACH)
    assert split.rest.bounds[1][0] == pytest.approx(BODY_OUTER_RADIUS)
    assert len(split.part.faces) + len(split.rest.faces) == len(mug().faces)


def test_a_cup_without_a_handle_has_none():
    assert ProtrudingHandleFinder().split(cup_without_handle()) is None


def cup_with_handle(reach: float) -> trimesh.Trimesh:
    """
    :param reach: How far the handle sticks out of the cup's body.
    :return: The cup with a box-shaped handle sticking out towards positive x.
    """
    handle = trimesh.creation.box(extents=[reach, 0.01, 0.01])
    handle.apply_translation([BODY_OUTER_RADIUS + reach / 2, 0.0, 0.0])
    return trimesh.util.concatenate([cup_without_handle(), handle])


def test_a_long_handle_does_not_pull_the_body_outline_off_the_body():
    reach = 3 * BODY_OUTER_RADIUS
    split = ProtrudingHandleFinder().split(cup_with_handle(reach))

    lowest, highest = split.part.bounds
    assert lowest[0] >= BODY_OUTER_RADIUS - 1e-9
    assert highest[0] == pytest.approx(BODY_OUTER_RADIUS + reach)
    assert split.rest.bounds[1][0] == pytest.approx(BODY_OUTER_RADIUS)


def test_what_barely_sticks_out_stays_with_the_body():
    assert (
        ProtrudingHandleFinder().split(cup_with_handle(0.1 * BODY_OUTER_RADIUS)) is None
    )


def test_a_body_that_is_not_round_has_no_protruding_handle():
    box = trimesh.creation.box(extents=[0.08, 0.08, 0.1])

    assert ProtrudingHandleFinder().split(box) is None


# %% handles at the narrow end of a tool


def test_the_handle_of_a_tool_is_its_narrow_end_cut_off_where_it_widens():
    split = NarrowEndHandleFinder().split(tool())

    lowest, highest = split.part.bounds
    assert lowest[0] == pytest.approx(-TOOL_HANDLE_LENGTH)
    assert highest[0] == pytest.approx(0.0)
    assert highest[1] - lowest[1] == pytest.approx(TOOL_HANDLE_WIDTH)
    assert split.rest.bounds[0][0] == pytest.approx(0.0)
    assert split.part.is_watertight and split.rest.is_watertight


def test_a_bar_of_even_width_has_no_handle():
    bar = trimesh.creation.box(extents=[0.2, 0.02, 0.02])

    assert NarrowEndHandleFinder().split(bar) is None


# %% rims of open containers


def test_the_rim_of_a_cup_is_the_band_below_its_top():
    finder = RimFinder()

    split = finder.split(cup_without_handle())

    top = cup_without_handle().bounds[1][2]
    assert split.part.bounds[0][2] == pytest.approx(top - finder.depth)
    assert split.part.bounds[1][2] == pytest.approx(top)
    assert split.rest.bounds[1][2] == pytest.approx(top - finder.depth)
    assert split.part.is_watertight and split.rest.is_watertight
    assert split.part.volume + split.rest.volume == pytest.approx(
        cup_without_handle().volume
    )


# %% splitting a part off an annotation in a world


def door_with_mug_shape(world: Optional[World] = None) -> Door:
    """
    :param world: The empty world to put the door in; a new one if not given.
    :return: A door, alone in its world, shaped like the mug, so that it has a handle to
        split off.
    """
    world = world or World()
    body = Body(name=PrefixedName("door"))
    shapes = ShapeCollection(
        [
            Mesh.from_trimesh(
                mesh=mug(), origin=HomogeneousTransformationMatrix(reference_frame=body)
            )
        ],
        reference_frame=body,
    )
    body.visual = shapes
    body.collision = shapes
    door = Door(root=body)
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(door)
    return door


def split_handle(door: Door) -> Handle:
    """
    :return: The handle split off the door's shape.
    """
    return handle_splitter().split(door)


def handle_splitter() -> SplitPartFromShape:
    """
    :return: Splits the handle off the shape of a door.
    """
    return SplitPartFromShape(
        whole_type=Door, part_type=Handle, finder=ProtrudingHandleFinder()
    )


def test_a_part_is_split_off_only_the_first_time_it_is_asked_for():
    door = door_with_mug_shape()
    splitter = handle_splitter()

    handle = splitter.part_of(door)

    assert door.handle is handle
    assert splitter.part_of(door) is handle
    assert len(door._world.bodies) == 2


def test_applying_the_split_again_leaves_the_world_as_it_is():
    door = door_with_mug_shape()
    world = door._world
    splitter = handle_splitter()

    splitter.apply(world)
    faces = len(door.root.collision.combined_mesh.faces)
    splitter.apply(world)

    assert len(world.bodies) == 2
    assert len(door.root.collision.combined_mesh.faces) == faces


def test_rolling_back_a_split_restores_the_whole_shape():
    door = door_with_mug_shape()
    world = door._world
    split_handle(door)

    world.rollback_modification_blocks()

    assert len(world.bodies) == 1
    assert door.handle is None
    assert len(door.root.visual.combined_mesh.faces) == len(mug().faces)
    assert len(door.root.collision.combined_mesh.faces) == len(mug().faces)
    assert door.root.collision.reference_frame is door.root


def test_a_split_is_reproduced_in_a_synchronized_world():
    door = door_with_mug_shape()
    world = door._world
    split_handle(door)

    copy = World()
    for block in world.modification_history:
        kwargs = WorldEntityWithIDKwargsTracker.from_world(copy).create_kwargs()
        with copy.modify_world():
            from_json(to_json(block), **kwargs).apply(copy)

    root = copy.get_kinematic_structure_entity_by_id(door.root.id)
    assert len(copy.bodies) == 2
    assert root.collision.reference_frame is root
    assert len(root.collision.combined_mesh.faces) == len(
        door.root.collision.combined_mesh.faces
    )
    assert root.collision.combined_mesh.bounds[1][0] == pytest.approx(BODY_OUTER_RADIUS)
