"""
Locating parts by asking a vision-language model where they are in views of a shape.

The shape is rendered from a ring of cameras, a detector puts boxes around the part in
each view, a segmenter masks the part inside the boxes, and the faces of the shape vote
on whether they belong to the part by how often the views that show them mask them.

The detector and the segmenter are abstract, so this module runs without any model; the
models themselves are in
:mod:`~semantic_digital_twin.pipeline.vision_language_models`.
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
import trimesh
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from krrood.adapters.json_serializer import to_json
from semantic_digital_twin.pipeline.part_locating import (
    LocatedPart,
    PartLocator,
    PartQuery,
)

# %% rendering the views


@dataclass
class ViewDirection:
    """
    The direction a camera looks at a shape from.
    """

    azimuth: float
    """
    The angle about the vertical axis, from the x-axis towards the y-axis, in degrees.
    """

    elevation: float
    """
    The angle above the horizontal plane, in degrees.
    """


def _ring_of_eleven() -> list[ViewDirection]:
    """
    :return: Eight directions all around, 25 degrees from above, and three steeper
        ones, 65 degrees from above, so that handles on any side and flat objects are
        seen.
    """
    return [ViewDirection(float(azimuth), 25.0) for azimuth in range(0, 360, 45)] + [
        ViewDirection(float(azimuth), 65.0) for azimuth in (0, 120, 240)
    ]


@dataclass
class RenderedViews:
    """
    Gray images of a shape, each with the face that each of its pixels shows.
    """

    images: NDArray[np.uint8]
    """
    The images, as an array of views, rows and columns.
    """

    faces: NDArray[np.int64]
    """
    For each pixel of each image, the index of the face it shows; -1 where it shows the
    background.
    """

    @property
    def silhouettes(self) -> NDArray[np.bool_]:
        """
        :return: For each pixel of each image, whether it shows the shape.
        """
        return self.faces >= 0


@dataclass
class CameraRing:
    """
    Cameras around a shape that render it as gray images, shaded by how directly each
    face looks at the camera.
    """

    directions: list[ViewDirection] = field(default_factory=_ring_of_eleven)
    """
    The directions the shape is looked at from.
    """

    resolution: int = 448
    """
    The width and height of each image, in pixels.
    """

    margin: float = 1.05
    """
    How much larger than the shape the square each camera sees is.
    """

    darkest_shade: int = 40
    """
    The brightness of a face seen edge-on.
    """

    brightest_shade: int = 240
    """
    The brightness of a face seen head-on.
    """

    background: int = 255
    """
    The brightness of the background.
    """

    def render(self, shape: trimesh.Trimesh) -> RenderedViews:
        """
        :param shape: The shape, its z-axis pointing up.
        :return: One orthographic view of the shape from each of :attr:`directions`.
        """
        rendered = [
            self._render_view(shape, direction) for direction in self.directions
        ]
        return RenderedViews(
            images=np.concatenate([view.images for view in rendered]),
            faces=np.concatenate([view.faces for view in rendered]),
        )

    def _render_view(
        self, shape: trimesh.Trimesh, direction: ViewDirection
    ) -> RenderedViews:
        """
        :param shape: The shape.
        :param direction: Where the camera looks from.
        :return: The single view from there.
        """
        azimuth, elevation = np.radians(direction.azimuth), np.radians(
            direction.elevation
        )
        towards_camera = np.array(
            [
                np.cos(elevation) * np.cos(azimuth),
                np.cos(elevation) * np.sin(azimuth),
                np.sin(elevation),
            ]
        )
        forward = -towards_camera
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        diagonal = float(np.linalg.norm(shape.extents))
        half_width = diagonal / 2 * self.margin
        eye = shape.bounds.mean(axis=0) + towards_camera * diagonal
        steps = (np.arange(self.resolution) + 0.5) / self.resolution * 2 - 1
        across, upwards = np.meshgrid(steps, -steps)
        origins = eye + half_width * (
            across[..., None] * right + upwards[..., None] * up
        )
        triangles, rays = shape.ray.intersects_id(
            origins.reshape(-1, 3),
            np.broadcast_to(forward, (origins.shape[0] * origins.shape[1], 3)),
            multiple_hits=False,
        )
        faces = np.full(self.resolution * self.resolution, -1, dtype=np.int64)
        faces[rays] = triangles
        image = np.full(self.resolution * self.resolution, float(self.background))
        facing = np.abs(shape.face_normals[triangles] @ forward)
        image[rays] = (
            self.darkest_shade + (self.brightest_shade - self.darkest_shade) * facing
        )
        shape_of_views = (1, self.resolution, self.resolution)
        return RenderedViews(
            images=image.reshape(shape_of_views).astype(np.uint8),
            faces=faces.reshape(shape_of_views),
        )


# %% finding the part in each view


class PartBoxDetector(ABC):
    """
    Puts boxes around a part of an object in views of it.
    """

    @abstractmethod
    def detect(
        self, views: RenderedViews, part_name: str, object_name: str
    ) -> list[NDArray[np.float64]]:
        """
        :param views: The views of the object.
        :param part_name: What the part is called, such as ``"handle"``.
        :param object_name: What the object is called, such as ``"mug"``.
        :return: For each view, the boxes around the part in it, as rows of the
            smallest and largest column and the smallest and largest row, in pixels;
            no rows where the view does not show the part.
        """

    @property
    @abstractmethod
    def fingerprint(self) -> str:
        """
        :return: A description of the detector that changes whenever the boxes it
            finds could.
        """

    @abstractmethod
    def load(self) -> None:
        """
        Load the model, so that the first detection does not wait for it.
        """


class BoxSegmenter(ABC):
    """
    Masks the object inside boxes in views of it.
    """

    @abstractmethod
    def segment(
        self, views: RenderedViews, boxes: list[NDArray[np.float64]]
    ) -> NDArray[np.bool_]:
        """
        :param views: The views of the object.
        :param boxes: For each view, the boxes to mask the object in, as
            :meth:`PartBoxDetector.detect` returns them.
        :return: For each pixel of each view, whether it is masked.
        """

    @property
    @abstractmethod
    def fingerprint(self) -> str:
        """
        :return: A description of the segmenter that changes whenever the masks it
            makes could.
        """

    @abstractmethod
    def load(self) -> None:
        """
        Load the model, so that the first segmentation does not wait for it.
        """


@dataclass
class KnownPartBoxDetector(PartBoxDetector):
    """
    Boxes the faces it is told belong to each part, wherever a view shows them: a stand-
    in for a model wherever the parts are known beforehand, such as in tests.
    """

    part_faces: dict[str, NDArray[np.int64]] = field(default_factory=dict)
    """
    For each part name, the indices of the faces that belong to the part.
    """

    def detect(
        self, views: RenderedViews, part_name: str, object_name: str
    ) -> list[NDArray[np.float64]]:
        faces = self.part_faces.get(part_name, np.empty(0, dtype=np.int64))
        boxes = []
        for view_faces in views.faces:
            rows, columns = np.nonzero(np.isin(view_faces, faces))
            if len(rows) == 0:
                boxes.append(np.empty((0, 4)))
                continue
            boxes.append(
                np.array(
                    [[columns.min(), rows.min(), columns.max() + 1, rows.max() + 1]],
                    dtype=np.float64,
                )
            )
        return boxes

    @property
    def fingerprint(self) -> str:
        return f"{type(self).__name__} {sorted(self.part_faces)}"

    def load(self) -> None:
        pass


@dataclass
class FilledBoxSegmenter(BoxSegmenter):
    """
    Masks all of the object inside each box: a stand-in for a model wherever the boxes
    fit the part tightly, such as in tests.
    """

    def segment(
        self, views: RenderedViews, boxes: list[NDArray[np.float64]]
    ) -> NDArray[np.bool_]:
        masks = np.zeros(views.faces.shape, dtype=bool)
        for mask, view_boxes in zip(masks, boxes):
            for column_start, row_start, column_end, row_end in view_boxes.astype(int):
                mask[row_start:row_end, column_start:column_end] = True
        return masks & views.silhouettes

    @property
    def fingerprint(self) -> str:
        return type(self).__name__

    def load(self) -> None:
        pass


# %% voting on the faces


@dataclass
class ViewVote:
    """
    Decides which faces of a shape belong to a part from the masks of the part in views
    of the shape.
    """

    threshold: float = 0.5
    """
    The share of the views showing a face that have to mask it for the face to belong to
    the part.
    """

    minimum_visible_pixels: int = 20
    """
    How many pixels of the part a view has to show to have a say in how sure the vote
    is.
    """

    def faces_in_part(
        self, shape: trimesh.Trimesh, views: RenderedViews, masks: NDArray[np.bool_]
    ) -> NDArray[np.bool_]:
        """
        :param shape: The shape the views show.
        :param views: The views.
        :param masks: For each pixel of each view, whether it shows the part.
        :return: For each face, whether it belongs to the part. A face no view shows
            takes the vote of the closest face a view shows.
        """
        silhouettes = views.silhouettes
        shown_faces = views.faces[silhouettes]
        face_count = len(shape.faces)
        shown = np.bincount(shown_faces, minlength=face_count)
        masked = np.bincount(
            shown_faces, weights=masks[silhouettes], minlength=face_count
        )
        if not shown.any():
            return np.zeros(face_count, dtype=bool)
        is_shown = shown > 0
        share = np.zeros(face_count)
        share[is_shown] = masked[is_shown] / shown[is_shown]
        centers = shape.triangles_center
        _, closest = cKDTree(centers[is_shown]).query(centers[~is_shown])
        share[~is_shown] = share[is_shown][closest]
        return share > self.threshold

    def confidence(
        self,
        in_part: NDArray[np.bool_],
        views: RenderedViews,
        masks: NDArray[np.bool_],
    ) -> float:
        """
        :param in_part: For each face, whether it belongs to the part.
        :param views: The views the part was found in.
        :param masks: For each pixel of each view, whether it shows the part.
        :return: The share of the views showing at least
            :attr:`minimum_visible_pixels` pixels of the part that mask most of them;
            0 if no view shows that much of it.
        """
        agreeing = showing = 0
        for view_faces, mask in zip(views.faces, masks):
            shows_part = (view_faces >= 0) & in_part[np.maximum(view_faces, 0)]
            if shows_part.sum() < self.minimum_visible_pixels:
                continue
            showing += 1
            agreeing += mask[shows_part].mean() > 0.5
        return agreeing / showing if showing else 0.0


# %% the locator


@dataclass
class VisionLanguagePartLocator(PartLocator):
    """
    Locates parts by rendering the shape, finding each part in the views with a detector
    and a segmenter, and letting the faces vote.

    A relationship that holds any number of parts gets each connected piece of the faces
    found as a part of its own.
    """

    detector: PartBoxDetector
    """
    Puts boxes around a part in each view.
    """

    segmenter: BoxSegmenter
    """
    Masks the part inside the boxes.
    """

    cameras: CameraRing = field(default_factory=CameraRing)
    """
    Renders the views.
    """

    vote: ViewVote = field(default_factory=ViewVote)
    """
    Decides from the masks which faces belong to a part.
    """

    def locate(self, query: PartQuery) -> list[LocatedPart]:
        shape = query.shape
        views = self.cameras.render(shape)
        located = []
        for relationship in query.relationships:
            boxes = self.detector.detect(
                views, relationship.part_name, query.object_name
            )
            masks = self.segmenter.segment(views, boxes)
            in_part = self.vote.faces_in_part(shape, views, masks)
            if not in_part.any():
                continue
            pieces = (
                self._connected_pieces(shape, in_part)
                if relationship.holds_many
                else [in_part]
            )
            located.extend(
                LocatedPart(
                    relationship_name=relationship.name,
                    faces=np.flatnonzero(piece),
                    confidence=self.vote.confidence(piece, views, masks),
                )
                for piece in pieces
            )
        return located

    def load(self) -> None:
        """
        Load the models, so that the first query does not wait for them.
        """
        self.detector.load()
        self.segmenter.load()

    @property
    def fingerprint(self) -> str:
        """
        :return: A description of the locator that changes whenever the parts it
            locates could.
        """
        return " ".join(
            [
                self.detector.fingerprint,
                self.segmenter.fingerprint,
                json.dumps(to_json(self.cameras), sort_keys=True),
                json.dumps(to_json(self.vote), sort_keys=True),
            ]
        )

    @staticmethod
    def _connected_pieces(
        shape: trimesh.Trimesh, in_part: NDArray[np.bool_]
    ) -> list[NDArray[np.bool_]]:
        """
        :param shape: A shape.
        :param in_part: For each face, whether it belongs to the parts.
        :return: For each piece of the faces that is connected over shared vertices,
            which faces belong to it.
        """
        connected = shape.copy()
        connected.merge_vertices()
        adjacency = connected.face_adjacency
        pieces = []
        for faces in trimesh.graph.connected_components(
            adjacency[in_part[adjacency].all(axis=1)], nodes=np.flatnonzero(in_part)
        ):
            piece = np.zeros(len(shape.faces), dtype=bool)
            piece[faces] = True
            pieces.append(piece)
        return pieces
