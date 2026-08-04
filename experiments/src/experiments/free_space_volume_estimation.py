"""
Monte Carlo estimation of free-space volume.

Used by :mod:`experiments.graph_of_convex_sets_experiments` as a mesh-accurate ground
truth for how much free space an environment actually has, and to score how much of
that free space a set of convex regions (e.g. IRIS regions grown by
:class:`~semantic_digital_twin.world_description.graph_of_convex_sets.polygons.GraphOfConvexPolygons`)
covers.

Sampling is used only where no closed form exists: the volume of a union of possibly
overlapping obstacle meshes, and the volume of a union of possibly overlapping convex
regions. Both are reported as confidence-interval bounds rather than point estimates.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
import trimesh
from typing_extensions import TYPE_CHECKING, List, Sequence, Tuple

from experiments.experiment_definitions import VolumeBound
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.geometry import BoundingBox, Shape

if TYPE_CHECKING:
    from pydrake.geometry.optimization import HPolyhedron

    from semantic_digital_twin.world_description.world_entity import (
        KinematicStructureEntity,
    )


# %% obstacle containment


@dataclass
class ObstacleContainmentTest:
    """
    Tests whether world-frame points fall inside a single obstacle shape.

    Uses the shape's own mesh when it is watertight (exact), and falls back to its axis-
    aligned bounding box otherwise, since :meth:`trimesh.Trimesh.contains` is only
    reliable on a closed manifold. The bounding-box fallback over-approximates rather
    than silently reporting no obstacle at all.
    """

    world_mesh: trimesh.Trimesh
    """
    The obstacle's mesh, expressed in the target frame.
    """

    world_bounding_box: BoundingBox
    """
    The obstacle's bounding box, expressed in the target frame; used when
    :attr:`world_mesh` is not watertight.
    """

    @classmethod
    def for_shape(
        cls, shape: Shape, target_frame: KinematicStructureEntity
    ) -> ObstacleContainmentTest:
        """
        Build a containment test for *shape*, expressed relative to *target_frame*.
        """
        world_bounding_box = shape.local_frame_bounding_box.transform_to_origin(
            HomogeneousTransformationMatrix(reference_frame=target_frame)
        )
        return cls(shape.mesh_in_frame(target_frame), world_bounding_box)

    def contains(self, points: np.ndarray) -> np.ndarray:
        """
        :param points: An ``(N, 3)`` array of points expressed in the target frame.
        :return: A boolean ``(N,)`` array, ``True`` where the obstacle contains the
            point.
        """
        if self.world_mesh.is_watertight:
            return self.world_mesh.contains(points)
        bounds = self.world_bounding_box.to_array_bounds()
        return np.all((points >= bounds.lower) & (points <= bounds.upper), axis=1)


# %% Monte Carlo sampling


@dataclass
class MonteCarloFreeSpaceSampler:
    """
    Estimates free-space volume by uniformly sampling points inside a search-space
    bounding box and classifying them against real obstacle geometry.

    The same sample points are reused to score how much of that free space a set of
    convex regions covers (:meth:`region_coverage_bound`), so the two quantities are
    directly comparable. Both are reported as Wilson-score confidence-interval bounds,
    since neither has a closed-form volume when the underlying obstacles or regions
    overlap.
    """

    search_space_bounding_box: BoundingBox
    """
    The volume to sample points within.
    """

    obstacle_containment_tests: List[ObstacleContainmentTest] = field(
        default_factory=list
    )
    """
    Containment tests for every obstacle in the scene.
    """

    sample_count: int = 2000
    """
    Number of points to sample.

    :meth:`region_coverage_bound` costs ``O(sample_count * region_count)`` Drake point-
    in-set queries, so this is a tradeoff between confidence-interval width and runtime.
    """

    confidence_level: float = 0.95
    """
    Confidence level used for every Wilson-score interval this sampler produces.
    """

    random_seed: int = 0
    """
    Seed for the sample points, so repeated runs over the same scene are comparable.
    """

    @classmethod
    def for_obstacle_shapes(
        cls,
        search_space_bounding_box: BoundingBox,
        obstacle_shapes: Sequence[Shape],
        target_frame: KinematicStructureEntity,
        sample_count: int = 2000,
        confidence_level: float = 0.95,
        random_seed: int = 0,
    ) -> MonteCarloFreeSpaceSampler:
        """
        Build a sampler from raw obstacle shapes, expressed relative to *target_frame*.
        """
        return cls(
            search_space_bounding_box=search_space_bounding_box,
            obstacle_containment_tests=[
                ObstacleContainmentTest.for_shape(shape, target_frame)
                for shape in obstacle_shapes
            ],
            sample_count=sample_count,
            confidence_level=confidence_level,
            random_seed=random_seed,
        )

    @cached_property
    def search_space_volume(self) -> float:
        """
        :return: The volume of :attr:`search_space_bounding_box`.
        """
        bounds = self.search_space_bounding_box.to_array_bounds()
        return float(np.prod(bounds.upper - bounds.lower))

    @cached_property
    def sample_points(self) -> np.ndarray:
        """
        :return: ``sample_count`` points drawn uniformly from
            :attr:`search_space_bounding_box`, deterministic for a fixed
            :attr:`random_seed`.
        """
        bounds = self.search_space_bounding_box.to_array_bounds()
        rng = np.random.default_rng(self.random_seed)
        return rng.uniform(bounds.lower, bounds.upper, size=(self.sample_count, 3))

    @cached_property
    def occupied_mask(self) -> np.ndarray:
        """
        :return: A boolean ``(sample_count,)`` array, ``True`` where the sample point
            falls inside any obstacle.
        """
        points = self.sample_points
        occupied = np.zeros(len(points), dtype=bool)
        for containment_test in self.obstacle_containment_tests:
            occupied |= containment_test.contains(points)
        return occupied

    def free_volume_bound(self) -> VolumeBound:
        """
        :return: A confidence-interval bound on the true free-space volume, from the
            fraction of sample points that fell outside every obstacle.
        """
        free_count = int(np.count_nonzero(~self.occupied_mask))
        return self._volume_bound_for_count(free_count)

    def region_coverage_bound(self, regions: Sequence[HPolyhedron]) -> VolumeBound:
        """
        :param regions: Convex regions to test sample points against.
        :return: A confidence-interval bound on the volume covered by the union of
            *regions*, from the same sample points used by :meth:`free_volume_bound`.
        """
        covered_count = sum(
            1
            for point in self.sample_points
            if any(region.PointInSet(point) for region in regions)
        )
        return self._volume_bound_for_count(covered_count)

    def _volume_bound_for_count(self, success_count: int) -> VolumeBound:
        lower_fraction, upper_fraction = self.wilson_score_interval(
            success_count, self.sample_count, self.confidence_level
        )
        return VolumeBound(
            lower=lower_fraction * self.search_space_volume,
            upper=upper_fraction * self.search_space_volume,
        )

    @staticmethod
    def wilson_score_interval(
        success_count: int, total: int, confidence_level: float
    ) -> Tuple[float, float]:
        """
        Wilson score confidence interval for the true proportion of successes among
        *total* Bernoulli trials.

        :param success_count: Number of observed successes.
        :param total: Number of trials.
        :param confidence_level: Two-sided confidence level, e.g. ``0.95``.
        :return: The ``(lower, upper)`` bounds on the true proportion, each clipped to
            ``[0, 1]``.
        """
        z = statistics.NormalDist().inv_cdf(1 - (1 - confidence_level) / 2)
        p_hat = success_count / total
        denominator = 1 + z**2 / total
        center = p_hat + z**2 / (2 * total)
        adjustment = z * math.sqrt(p_hat * (1 - p_hat) / total + z**2 / (4 * total**2))
        lower = max(0.0, (center - adjustment) / denominator)
        upper = min(1.0, (center + adjustment) / denominator)
        return lower, upper
