from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from pydrake.geometry.optimization import HPolyhedron, Iris, IrisOptions, Point
from pydrake.planning import GcsTrajectoryOptimization
from typing_extensions import List, Optional, Sequence, Tuple

from semantic_digital_twin.exceptions import (
    PointOccupiedError,
    UnboundedSearchSpaceError,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticEnvironmentAnnotation,
)
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import BoundingBox
from semantic_digital_twin.world_description.graph_of_convex_sets import (
    GraphOfConvexSets,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)

Subgraph = GcsTrajectoryOptimization.Subgraph
"""
Type alias for the ``Subgraph`` handle returned by
``GcsTrajectoryOptimization.AddRegions``.
"""


def _default_iris_options() -> IrisOptions:
    """
    Build the default :class:`IrisOptions` used by :class:`DrakeGraphOfConvexSets`.

    :return: IRIS options with a bounded iteration count so a single region-growth call
        cannot dominate build time on large scenes.
    """
    options = IrisOptions()
    options.iteration_limit = 10
    options.termination_threshold = 2e-2
    options.relative_termination_threshold = 1e-2
    return options


@dataclass
class IrisSeedingSettings:
    """
    Settings controlling how many IRIS regions :class:`DrakeGraphOfConvexSets` grows,
    and from where.

    Drake's IRIS grows one convex region at a time from a single seed point, so
    covering a scene requires calling it repeatedly from different seeds. Seeding here
    is a simple, deterministic coverage heuristic (a regular grid), not a tuned or
    optimal strategy: a production seeding strategy (e.g. clique-cover-based, or seeded
    from known robot poses via ``extra_seed_points`` on
    :meth:`DrakeGraphOfConvexSets.from_world`) would likely produce a different, more
    efficient region layout.
    """

    grid_resolution: int = 5
    """
    Number of candidate seed points per axis in the coverage grid.
    """

    max_regions: int = 40
    """
    Upper bound on the number of IRIS regions grown.
    """

    iris_options: IrisOptions = field(default_factory=_default_iris_options)
    """
    Options forwarded to every :func:`pydrake.geometry.optimization.Iris` call.
    """


def _world_frame_bounds(box: BoundingBox) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute a box's world-frame lower/upper corners as plain floats.

    ``box.origin.x``/``y``/``z`` are symbolic
    :class:`krrood.symbolic_math.symbolic_math.Scalar` objects; converting to ``float``
    once here, rather than repeating the conversion inside a tight per-axis loop, avoids
    a roughly two order-of-magnitude slowdown.

    :param box: The bounding box to read.
    :return: The box's ``(lower, upper)`` corners as plain-float 3-vectors.
    """
    origin = np.array([float(box.origin.x), float(box.origin.y), float(box.origin.z)])
    lower = origin + np.array([box.min_x, box.min_y, box.min_z])
    upper = origin + np.array([box.max_x, box.max_y, box.max_z])
    return lower, upper


def _box_to_hpolyhedron(box: BoundingBox) -> HPolyhedron:
    """
    Convert a semdt axis-aligned :class:`BoundingBox` into a Drake ``HPolyhedron``,
    expressed in the world frame.
    """
    lower, upper = _world_frame_bounds(box)
    return HPolyhedron.MakeBox(lower, upper)


def _validate_and_convert_domain(search_space: BoundingBoxCollection) -> HPolyhedron:
    """
    Convert a search space into the single, finite ``HPolyhedron`` domain IRIS grows
    regions within.

    :param search_space: The search space to convert.
    :raises UnboundedSearchSpaceError: If ``search_space`` is not exactly one finite
        bounding box.
    """
    boxes = search_space.bounding_boxes
    if len(boxes) != 1:
        raise UnboundedSearchSpaceError()
    lower, upper = _world_frame_bounds(boxes[0])
    if not (np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))):
        raise UnboundedSearchSpaceError()
    return HPolyhedron.MakeBox(lower, upper)


def _point_to_world_array(world: World, point: Point3) -> np.ndarray:
    """
    Express a point as a plain float 3-vector in ``world.root``.
    """
    point_in_world = world.transform(point, world.root)
    return np.array(
        [float(point_in_world.x), float(point_in_world.y), float(point_in_world.z)]
    )


def _coverage_grid_seeds(
    lower: np.ndarray, upper: np.ndarray, resolution: int
) -> List[np.ndarray]:
    """
    Build a regular grid of candidate seed points strictly inside ``[lower, upper]``.
    """
    axes = [np.linspace(lower[i], upper[i], resolution + 2)[1:-1] for i in range(3)]
    grid = np.meshgrid(*axes, indexing="ij")
    return [np.array(point) for point in np.stack(grid, axis=-1).reshape(-1, 3)]


class DrakeGraphOfConvexSets(GraphOfConvexSets):
    """
    A graph of convex sets whose regions are grown by Drake's IRIS algorithm and
    solved with Drake's ``GcsTrajectoryOptimization`` (Marcucci et al.,
    https://arxiv.org/abs/2205.04422).

    Unlike :class:`~semantic_digital_twin.world_description.graph_of_convex_sets.GraphOfBoundingBoxes`,
    which exhaustively partitions free space into many small axis-aligned boxes, IRIS
    covers free space with a handful of large, non-axis-aligned convex regions -- a
    sufficient cover for solving path queries, not a complete map of free space (some
    free points may not be covered by any region, in which case ``path_from_to``
    returns None for them rather than raising, since they are not necessarily
    occupied).

    Region growth (in :meth:`from_world`) is the expensive part of building this
    class; once built, :meth:`path_from_to` can be called repeatedly and cheaply for
    different start/goal pairs, since only the per-query endpoint subgraphs are added
    and removed rather than rebuilding the whole graph.
    """

    obstacles: List[BoundingBox]
    """
    The obstacle bounding boxes every region was grown to avoid, expressed relative to
    ``world.root``.
    """

    regions: List[HPolyhedron]
    """
    The IRIS regions covering the search space.
    """

    _gcs: GcsTrajectoryOptimization
    """
    The persistent trajectory-optimization instance holding the region subgraph and,
    while a query is in flight, its source/target point subgraphs.
    """

    _region_subgraph: Optional[Subgraph]
    """
    The subgraph spanning ``regions``, built once in :meth:`from_world` and reused by
    every :meth:`path_from_to` query.
    """

    _query_subgraphs: Optional[Tuple[Subgraph, Subgraph]]
    """
    The current query's ``(source, target)`` point subgraphs, or None between queries.
    """

    def __init__(
        self, world: World, search_space: Optional[BoundingBoxCollection] = None
    ):
        super().__init__(world, search_space)
        self.obstacles = []
        self.regions = []
        self._gcs = GcsTrajectoryOptimization(3)
        self._region_subgraph = None
        self._query_subgraphs = None

    @property
    def region_count(self) -> int:
        """
        :return: The number of IRIS regions in this graph.
        """
        return len(self.regions)

    @classmethod
    def from_world(
        cls,
        world: World,
        search_space: BoundingBoxCollection,
        bloat_obstacles: float = 0.0,
        seeding_settings: Optional[IrisSeedingSettings] = None,
        extra_seed_points: Sequence[Point3] = (),
    ) -> DrakeGraphOfConvexSets:
        """
        Grow IRIS regions covering the free space of *world* within *search_space*.

        :param world: The belief state to plan in.
        :param search_space: The navigable search volume; must consist of exactly one
            finite bounding box (see
            :class:`~semantic_digital_twin.exceptions.UnboundedSearchSpaceError`).
        :param bloat_obstacles: Amount to expand each obstacle bounding box in x/y,
            closing gaps between the individual panel boxes that make up a single
            piece of furniture (mirrors ``GraphOfBoundingBoxes``'s bloat parameter).
        :param seeding_settings: IRIS seeding strategy and options; defaults to
            :class:`IrisSeedingSettings`'s defaults.
        :param extra_seed_points: Points to seed a region from first, before the
            regular coverage grid -- e.g. known robot poses that should be covered by
            a region even if the grid would otherwise miss them.
        :return: A graph ready to answer repeated :meth:`path_from_to` queries.
        """
        result = cls(world, search_space)
        settings = seeding_settings or IrisSeedingSettings()
        domain = _validate_and_convert_domain(result.search_space)

        semantic_annotation = SemanticEnvironmentAnnotation(
            root=world.root, _world=world
        )
        bloated_obstacles = cls._build_bloated_obstacle_collection(
            result.search_space, semantic_annotation, bloat_obstacles=bloat_obstacles
        )
        result.obstacles = list(bloated_obstacles)
        obstacle_polyhedra = [_box_to_hpolyhedron(box) for box in result.obstacles]

        lower, upper = _world_frame_bounds(result.search_space.bounding_boxes[0])
        seeds = [
            _point_to_world_array(world, point) for point in extra_seed_points
        ] + _coverage_grid_seeds(lower, upper, settings.grid_resolution)

        regions: List[HPolyhedron] = []
        for seed in seeds:
            if len(regions) >= settings.max_regions:
                break
            if any(obstacle.PointInSet(seed) for obstacle in obstacle_polyhedra):
                continue
            if any(region.PointInSet(seed) for region in regions):
                continue
            regions.append(
                Iris(obstacle_polyhedra, seed, domain, settings.iris_options)
            )

        result.regions = regions
        result._region_subgraph = result._gcs.AddRegions(regions, order=1)
        result._gcs.AddPathLengthCost()
        return result

    def path_from_to(self, start: Point3, goal: Point3) -> Optional[List[Point3]]:
        """
        Calculate a connected path from a start pose to a goal pose.

        :param start: The start pose.
        :param goal: The goal pose.
        :return: The path as a sequence of points to navigate to, or None if no region
            covers a path between them.
        :raises PointOccupiedError: If ``start`` or ``goal`` lies inside an obstacle.
        """
        if any(obstacle.contains(start) for obstacle in self.obstacles):
            raise PointOccupiedError(start)
        if any(obstacle.contains(goal) for obstacle in self.obstacles):
            raise PointOccupiedError(goal)

        start_array = _point_to_world_array(self.world, start)
        goal_array = _point_to_world_array(self.world, goal)

        self._clear_previous_query()
        source_subgraph = self._gcs.AddRegions([Point(start_array)], order=0)
        target_subgraph = self._gcs.AddRegions([Point(goal_array)], order=0)
        self._gcs.AddEdges(source_subgraph, self._region_subgraph)
        self._gcs.AddEdges(self._region_subgraph, target_subgraph)
        self._query_subgraphs = (source_subgraph, target_subgraph)

        trajectory, result = self._gcs.SolvePath(source_subgraph, target_subgraph)
        if not result.is_success():
            return None

        breakpoints = trajectory.get_segment_times()
        interior_waypoints = [
            Point3(*trajectory.value(t).flatten(), reference_frame=self.world.root)
            for t in breakpoints[1:-1]
        ]
        return [start] + interior_waypoints + [goal]

    def _clear_previous_query(self) -> None:
        if self._query_subgraphs is None:
            return
        for subgraph in self._query_subgraphs:
            self._gcs.RemoveSubgraph(subgraph)
        self._query_subgraphs = None
