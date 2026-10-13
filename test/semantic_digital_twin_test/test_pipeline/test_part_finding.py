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
from semantic_digital_twin.pipeline.part_splitting import (
    PartSplitter,
    SplitPartFromShape,
)
from semantic_digital_twin.pipeline.rim_finding import RimFinder
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color, Mesh
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


def handle_splitter() -> PartSplitter:
    """
    :return: Splits the handle off the shape of a door.
    """
    return PartSplitter(part_type=Handle, finder=ProtrudingHandleFinder())


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
    step = SplitPartFromShape(whole_type=Door, splitter=handle_splitter())

    step.apply(world)
    assert door.handle is not None
    faces = len(door.root.collision.combined_mesh.faces)
    step.apply(world)

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


def test_a_part_is_split_off_a_single_body_without_annotating_it():
    door = door_with_mug_shape()
    world = door._world

    part = handle_splitter().split_body(door.root)

    assert part.parent_kinematic_structure_entity is door.root
    assert door.handle is None
    assert len(world.semantic_annotations) == 1
    assert part.collision.combined_mesh.bounds[1][0] == pytest.approx(
        BODY_OUTER_RADIUS + HANDLE_REACH
    )
    assert door.root.collision.combined_mesh.bounds[1][0] == pytest.approx(
        BODY_OUTER_RADIUS
    )


def test_the_pieces_keep_the_color_of_the_shape():
    door = door_with_mug_shape()
    red = Color(1.0, 0.0, 0.0, 1.0)
    door.root.visual.dye_shapes(red)

    handle = split_handle(door)

    for shapes in (handle.root.visual, door.root.visual):
        colors = shapes.combined_mesh.visual.face_colors
        assert (colors == [255, 0, 0, 255]).all()


def test_the_pieces_keep_the_colors_of_the_faces_they_were_cut_from():
    door = door_with_mug_shape()
    split = ProtrudingHandleFinder().split(mug())
    split.part.visual.face_colors = [0, 255, 0, 255]
    split.rest.visual.face_colors = [0, 0, 255, 255]
    mesh = trimesh.util.concatenate([split.part, split.rest])
    shapes = ShapeCollection(
        [
            Mesh.from_trimesh(
                mesh=mesh,
                origin=HomogeneousTransformationMatrix(reference_frame=door.root),
            )
        ],
        reference_frame=door.root,
    )
    with door._world.modify_world():
        door.root.update_shapes(visual=shapes, collision=shapes)

    handle = split_handle(door)

    handle_colors = handle.root.visual.combined_mesh.visual.vertex_colors
    body_colors = door.root.visual.combined_mesh.visual.vertex_colors
    assert (handle_colors == [0, 255, 0, 255]).all()
    assert (body_colors == [0, 0, 255, 255]).all()


def test_parts_split_off_the_same_body_have_distinct_names():
    door = door_with_mug_shape()
    splitter = PartSplitter(part_type=Handle, finder=RimFinder())

    first = splitter.split_body(door.root)
    second = splitter.split_body(door.root)

    assert str(first.name) == "door/handle"
    assert str(second.name) == "door/handle_1"


def test_shapes_without_a_reference_frame_are_taken_relative_to_the_body():
    door = door_with_mug_shape()
    shapes = ShapeCollection([Mesh.from_trimesh(mesh=mug())])

    with door._world.modify_world():
        door.root.update_shapes(visual=shapes, collision=shapes)

    assert all(
        shape.origin.reference_frame is door.root for shape in door.root.collision
    )
