import numpy as np
import pytest

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import (
    PointOccupiedError,
    UnboundedSearchSpaceError,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import BoundingBox, Box, Scale
from semantic_digital_twin.world_description.graph_of_convex_sets import (
    GraphOfConvexSets,
)
from semantic_digital_twin.world_description.graph_of_convex_sets_drake import (
    DrakeGraphOfConvexSets,
    IrisSeedingSettings,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from semantic_digital_twin.world_description.world_entity import Body


@pytest.fixture
def unit_box_world() -> World:
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("map"))
        world.add_kinematic_structure_entity(root)
        obstacle = Body(name=PrefixedName("unit_cube"))
        world.add_connection(FixedConnection.create_with_dofs(world, root, obstacle))
        obstacle.collision.append(Box(scale=Scale(1.0, 1.0, 1.0)))
    return world


@pytest.fixture
def unit_box_search_space(unit_box_world: World) -> BoundingBoxCollection:
    return BoundingBoxCollection(
        [
            BoundingBox(
                min_x=-2,
                max_x=3,
                min_y=-2,
                max_y=3,
                min_z=-0.5,
                max_z=0.5,
                origin=HomogeneousTransformationMatrix(
                    reference_frame=unit_box_world.root
                ),
            )
        ],
        unit_box_world.root,
    )


@pytest.fixture
def unit_box_gcs(
    unit_box_world: World, unit_box_search_space: BoundingBoxCollection
) -> DrakeGraphOfConvexSets:
    return DrakeGraphOfConvexSets.from_world(unit_box_world, unit_box_search_space)


class TestDrakeGraphOfConvexSets:
    def test_is_a_graph_of_convex_sets(self, unit_box_gcs: DrakeGraphOfConvexSets):
        assert isinstance(unit_box_gcs, GraphOfConvexSets)

    def test_from_world_grows_at_least_one_region(
        self, unit_box_gcs: DrakeGraphOfConvexSets
    ):
        assert unit_box_gcs.region_count > 0
        assert len(unit_box_gcs.regions) == unit_box_gcs.region_count

    def test_no_region_contains_the_obstacle(
        self, unit_box_gcs: DrakeGraphOfConvexSets
    ):
        for region in unit_box_gcs.regions:
            assert not region.PointInSet(np.array([0.0, 0.0, 0.0]))

    def test_path_from_to_finds_a_path_around_the_cube(
        self, unit_box_world: World, unit_box_gcs: DrakeGraphOfConvexSets
    ):
        start = Point3(-1.5, -1.5, 0.0, reference_frame=unit_box_world.root)
        goal = Point3(2.5, 2.5, 0.0, reference_frame=unit_box_world.root)

        path = unit_box_gcs.path_from_to(start, goal)

        assert path is not None
        assert path[0] is start
        assert path[-1] is goal
        assert len(path) >= 2

    def test_path_from_to_raises_for_an_occupied_start(
        self, unit_box_world: World, unit_box_gcs: DrakeGraphOfConvexSets
    ):
        occupied = Point3(0.0, 0.0, 0.0, reference_frame=unit_box_world.root)
        free = Point3(2.5, 2.5, 0.0, reference_frame=unit_box_world.root)
        with pytest.raises(PointOccupiedError):
            unit_box_gcs.path_from_to(occupied, free)

    def test_path_from_to_raises_for_an_occupied_goal(
        self, unit_box_world: World, unit_box_gcs: DrakeGraphOfConvexSets
    ):
        occupied = Point3(0.0, 0.0, 0.0, reference_frame=unit_box_world.root)
        free = Point3(-1.5, -1.5, 0.0, reference_frame=unit_box_world.root)
        with pytest.raises(PointOccupiedError):
            unit_box_gcs.path_from_to(free, occupied)

    def test_repeated_queries_reuse_the_persistent_region_subgraph(
        self, unit_box_world: World, unit_box_gcs: DrakeGraphOfConvexSets
    ):
        start = Point3(-1.5, -1.5, 0.0, reference_frame=unit_box_world.root)
        goal = Point3(2.5, 2.5, 0.0, reference_frame=unit_box_world.root)

        for start, goal in [(start, goal), (goal, start), (start, goal)]:
            path = unit_box_gcs.path_from_to(start, goal)
            assert path is not None
            # region subgraph + one source + one target subgraph, regardless of how
            # many queries have already been solved.
            assert len(unit_box_gcs._trajectory_optimization.GetSubgraphs()) == 3

    def test_from_world_rejects_a_default_unbounded_search_space(
        self, unit_box_world: World
    ):
        with pytest.raises(UnboundedSearchSpaceError):
            DrakeGraphOfConvexSets.from_world(unit_box_world, None)

    def test_from_world_rejects_a_multi_box_search_space(self, unit_box_world: World):
        origin = HomogeneousTransformationMatrix(reference_frame=unit_box_world.root)
        two_boxes = BoundingBoxCollection(
            [
                BoundingBox(-2, -2, -0.5, -1, -1, 0.5, origin),
                BoundingBox(1, 1, -0.5, 2, 2, 0.5, origin),
            ],
            unit_box_world.root,
        )
        with pytest.raises(UnboundedSearchSpaceError):
            DrakeGraphOfConvexSets.from_world(unit_box_world, two_boxes)

    def test_extra_seed_points_are_seeded_before_the_coverage_grid(
        self, unit_box_world: World, unit_box_search_space: BoundingBoxCollection
    ):
        far_corner = Point3(-1.9, -1.9, 0.0, reference_frame=unit_box_world.root)
        gcs = DrakeGraphOfConvexSets.from_world(
            unit_box_world,
            unit_box_search_space,
            seeding_settings=IrisSeedingSettings(grid_resolution=1, max_regions=1),
            extra_seed_points=[far_corner],
        )
        assert gcs.region_count == 1
        assert gcs.regions[0].PointInSet(np.array([-1.9, -1.9, 0.0]))
