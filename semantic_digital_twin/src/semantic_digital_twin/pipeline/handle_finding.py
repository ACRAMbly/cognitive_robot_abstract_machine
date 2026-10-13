"""
Finding the piece of an object's shape that is its handle, so that it can be split off
into a :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Handle`
of its own by :class:`~semantic_digital_twin.pipeline.part_splitting.PartSplitter`.

Shapes are taken in the frame of the object's root body, its z-axis pointing up.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
from functools import cached_property

import numpy as np
from numpy.typing import NDArray
import trimesh
from scipy.spatial import ConvexHull
from typing_extensions import Optional

from semantic_digital_twin.datastructures.definitions import MeshArrayAxis
from semantic_digital_twin.pipeline.part_splitting import PartFinder, ShapeSplit
from semantic_digital_twin.spatial_types import Point2

# %% finding handles


class HandleFinder(PartFinder, ABC):
    """
    Finds the piece of a shape that is its handle.
    """


@dataclass
class RoundOutline:
    """
    A circle in the horizontal plane.
    """

    center: Point2
    """
    The circle's center.
    """

    radius: float
    """
    The circle's radius.
    """

    @classmethod
    def fitted_to(cls, points: NDArray[np.float64]) -> RoundOutline:
        """
        :param points: At least three points in the horizontal plane, as rows of x
            and y.
        :return: The circle closest to the points, in the least squares sense.
        """
        coefficients, *_ = np.linalg.lstsq(
            np.column_stack([2 * points, np.ones(len(points))]),
            np.sum(points**2, axis=1),
            rcond=None,
        )
        x, y, offset = coefficients
        return cls(
            center=Point2(x, y),
            radius=float(np.sqrt(offset + x**2 + y**2)),
        )

    def distances(self, points: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        :param points: Points in the horizontal plane, as rows of x and y.
        :return: Each point's distance from the circle's center.
        """
        return np.linalg.norm(points - self.center.to_np(), axis=1)


@dataclass
class ProtrudingHandleFinder(HandleFinder):
    """
    Finds a handle that sticks out sideways from a round body, as a mug's or a pot's
    does.

    The body's outline, seen from above, is a circle; the handle is what lies outside of
    it. The circle is the one most of the outline follows, so that a long handle does not
    pull it off the body, and every length that decides what sticks out is a fraction of
    its radius, so that the same finder fits a cup and a pot. Each piece that sticks out
    far enough belongs to the handle: both handles of a pot, but also a teapot's spout.
    """

    clearance: float = 0.1
    """
    How far outside the round body a face has to lie to belong to the handle, as a
    fraction of the body's radius.
    """

    minimum_reach: float = 0.2
    """
    How far a piece has to stick out of the round body to belong to the handle, as a
    fraction of the body's radius. Smaller ones, such as a rolled lip, stay with the body.
    """

    outline_tolerance: float = 0.02
    """
    How far from the circle a point of the outline may lie and still follow it, as a
    fraction of the circle's radius.
    """

    minimum_roundness: float = 0.3
    """
    How much of the outline has to follow the circle for the body to be round.
    """

    outline_points: int = 360
    """
    How many points, evenly spaced along the outline, the circle is fitted to.
    """

    trials: int = 200
    """
    How many circles through three points of the outline are tried.
    """

    def split(self, shape: trimesh.Trimesh) -> Optional[ShapeSplit]:
        outline = self._round_body_outline(shape)
        if outline is None:
            return None
        beyond = (
            outline.distances(shape.triangles_center[:, :2]) - outline.radius
        ) / outline.radius
        outside = beyond > self.clearance
        connected = shape.copy()
        connected.merge_vertices(merge_tex=True, merge_norm=True)
        adjacency = connected.face_adjacency
        in_handle = np.zeros(len(shape.faces), dtype=bool)
        for piece in trimesh.graph.connected_components(
            adjacency[outside[adjacency].all(axis=1)], nodes=np.flatnonzero(outside)
        ):
            in_handle[piece] = beyond[piece].max() >= self.minimum_reach
        return ShapeSplit.by_faces(shape, in_handle)

    def _round_body_outline(self, shape: trimesh.Trimesh) -> Optional[RoundOutline]:
        """
        Find the circle that most of the outline of the shape, seen from above, follows,
        by trying circles through three of its points.

        :param shape: The object's shape.
        :return: The outline of its round body; ``None`` if too little of the outline
            follows a circle that fits into it.
        """
        outline = self._evenly_spaced_outline(shape)
        largest_radius = (
            np.ptp(outline, axis=0).max() / 2 * (1 + self.outline_tolerance)
        )
        generator = np.random.default_rng(0)
        best_following = np.zeros(len(outline), dtype=bool)
        for _ in range(self.trials):
            three = outline[generator.choice(len(outline), size=3, replace=False)]
            candidate = RoundOutline.fitted_to(three)
            if not candidate.radius <= largest_radius:
                continue
            following = self._following(candidate, outline)
            if following.sum() > best_following.sum():
                best_following = following
        if best_following.mean() < self.minimum_roundness:
            return None
        return RoundOutline.fitted_to(outline[best_following])

    def _following(
        self, circle: RoundOutline, outline: NDArray[np.float64]
    ) -> NDArray[np.bool_]:
        """
        :param circle: A circle in the horizontal plane.
        :param outline: Points of the outline, as rows of x and y.
        :return: For each point, whether it lies on the circle.
        """
        return (
            np.abs(circle.distances(outline) - circle.radius)
            <= self.outline_tolerance * circle.radius
        )

    def _evenly_spaced_outline(self, shape: trimesh.Trimesh) -> NDArray[np.float64]:
        """
        :param shape: The object's shape.
        :return: :attr:`outline_points` points, evenly spaced along the convex outline
            of the shape seen from above, as rows of x and y.
        """
        horizontal = shape.vertices[:, :2]
        corners = horizontal[ConvexHull(horizontal).vertices]
        closed = np.vstack([corners, corners[:1]])
        along = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
        )
        spaced = np.linspace(0.0, along[-1], self.outline_points, endpoint=False)
        return np.column_stack(
            [
                np.interp(spaced, along, closed[:, 0]),
                np.interp(spaced, along, closed[:, 1]),
            ]
        )


@dataclass
class ElongatedShape:
    """
    A shape that is longer than it is wide, measured along its length and across its
    width at its vertices and the centers of its faces.
    """

    shape: trimesh.Trimesh
    """
    The shape.
    """

    length_axis: MeshArrayAxis
    """
    The axis the shape is longest along.
    """

    width_axis: MeshArrayAxis
    """
    The axis its width is measured along.
    """

    end_fraction: float = 0.15
    """
    How much of the shape's length, from each end, is compared to find the narrower end.
    """

    @cached_property
    def points(self) -> NDArray[np.float64]:
        """
        :return: The points the shape is measured at, as rows of x, y and z.
        """
        return np.vstack([self.shape.vertices, self.shape.triangles_center])

    @property
    def along(self) -> NDArray[np.float64]:
        """
        :return: Each point's distance from the end towards the negative length axis.
        """
        coordinates = self.points[:, self.length_axis]
        return coordinates - coordinates.min()

    @property
    def across(self) -> NDArray[np.float64]:
        """
        :return: Each point's coordinate across the shape.
        """
        return self.points[:, self.width_axis]

    @property
    def length(self) -> float:
        """
        :return: The shape's length.
        """
        return float(np.ptp(self.points[:, self.length_axis]))

    def narrower_end_is_positive(self) -> bool:
        """
        :return: Whether the end towards the positive length axis is the narrower one.
        """
        band = self.end_fraction * self.length
        negative_end = self.across[self.along <= band]
        positive_end = self.across[self.along >= self.length - band]
        return np.ptp(positive_end) < np.ptp(negative_end)


@dataclass
class NarrowEndHandleFinder(HandleFinder):
    """
    Finds the handle of an elongated object lying flat, such as a piece of cutlery or a
    tool: the stretch from its narrower end up to where it widens, cut off across the
    object's length.
    """

    widening: float = 1.8
    """
    How many times wider than at its narrow end the object has to get for the handle to
    end there.
    """

    slice_length: float = 0.005
    """
    The length of the slices the object's width is measured in, in meters.
    """

    end_fraction: float = 0.15
    """
    How much of the object's length, from each end, is compared to find the narrower
    end.
    """

    def split(self, shape: trimesh.Trimesh) -> Optional[ShapeSplit]:
        lowest, highest = shape.bounds
        extents = highest - lowest
        length_axis, width_axis = (
            (MeshArrayAxis.X, MeshArrayAxis.Y)
            if extents[MeshArrayAxis.X] >= extents[MeshArrayAxis.Y]
            else (MeshArrayAxis.Y, MeshArrayAxis.X)
        )
        elongated = ElongatedShape(
            shape=shape,
            length_axis=length_axis,
            width_axis=width_axis,
            end_fraction=self.end_fraction,
        )
        along = elongated.along
        narrow_end_is_positive = elongated.narrower_end_is_positive()
        if narrow_end_is_positive:
            along = elongated.length - along
        handle_length = self._handle_length(elongated.across, along)
        if handle_length is None:
            return None
        towards_narrow_end = np.zeros(3)
        towards_narrow_end[length_axis] = 1.0 if narrow_end_is_positive else -1.0
        narrow_end = highest if narrow_end_is_positive else lowest
        return ShapeSplit.by_plane(
            shape, narrow_end - towards_narrow_end * handle_length, towards_narrow_end
        )

    def _handle_length(
        self, across: NDArray[np.float64], along: NDArray[np.float64]
    ) -> Optional[float]:
        """
        :param across: The points' coordinates across the object.
        :param along: The points' distances from the narrow end.
        :return: How far from the narrow end the object first gets :attr:`widening`
            times as wide as at that end; ``None`` if it never does.
        """
        widths = []
        for start in np.arange(0.0, along.max(), self.slice_length):
            in_slice = across[(along >= start) & (along < start + self.slice_length)]
            if len(in_slice) > 1:
                widths.append((start, float(np.ptp(in_slice))))
        end_width = max(widths[0][1], 1e-4)
        for start, width in widths:
            if width > self.widening * end_width:
                return start
        return None
