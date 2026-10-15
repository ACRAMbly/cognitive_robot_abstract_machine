"""
A camera view at a constant distance that YOLO annotators under test are run on.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

from robokudo.types.cv import ColorToDepthRatio
from robokudo.annotators.core import BaseAnnotator
from robokudo.cas import CASViews
from robokudo.pipeline import Pipeline

# %% camera scene


@dataclass
class CameraScene:
    """
    A black color image and a depth image measuring the same distance everywhere.
    """

    models_directory: Path
    """
    Directory the annotators under test load pretrained models from.
    """

    color_image_shape: tuple[int, int, int] = (40, 80, 3)
    """
    Shape of the color images in the CAS.
    """

    depth_in_millimeters: int = 1000
    """
    Distance every depth image pixel measures.
    """

    millimeters_per_meter: float = 1000.0
    """
    Scale between the depth image values and point coordinates.
    """

    def color_image(self) -> np.ndarray:
        """
        :return: A black color image of the scene's shape.
        """
        return np.zeros(self.color_image_shape, dtype=np.uint8)

    def place_in_pipeline(
        self, annotator: BaseAnnotator, color_to_depth_ratio: float = 1.0
    ) -> None:
        """
        Set the annotator up in a pipeline whose CAS holds the scene's color image and a
        matching depth image.

        :param color_to_depth_ratio: Size of the depth image relative to the color
            image.
        """
        annotator.setup()
        pipeline = Pipeline("Sequence")
        pipeline.add_child(annotator)
        pipeline.setup()

        color_height, color_width = self.color_image_shape[:2]
        depth_height = int(color_height * color_to_depth_ratio)
        depth_width = int(color_width * color_to_depth_ratio)
        cas = annotator.get_cas()
        cas.set(CASViews.COLOR_IMAGE, self.color_image())
        cas.set(
            CASViews.DEPTH_IMAGE,
            np.full(
                (depth_height, depth_width),
                self.depth_in_millimeters,
                dtype=np.uint16,
            ),
        )
        cas.set(
            CASViews.COLOR2DEPTH_RATIO,
            ColorToDepthRatio(
                horizontal=color_to_depth_ratio, vertical=color_to_depth_ratio
            ),
        )
        cas.set(
            CASViews.CAMERA_INTRINSIC,
            o3d.camera.PinholeCameraIntrinsic(
                color_width,
                color_height,
                100.0,
                100.0,
                color_width / 2.0,
                color_height / 2.0,
            ),
        )
