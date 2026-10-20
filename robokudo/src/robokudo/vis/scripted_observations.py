"""
Draw diagnostic images for scripted perception observations.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray
from typing_extensions import TYPE_CHECKING

from robokudo.io.scripted_observations import PreviewColor
from robokudo.types.annotation import Classification

if TYPE_CHECKING:
    from robokudo.io.scripted_observations import ScriptedCamera, ScriptedObservation
    from robokudo.types.scene import ObjectHypothesis

# %% diagnostic image


@dataclass(frozen=True)
class ObservationPreview:
    """
    Show scripted regions and labels without a 3D renderer.
    """

    camera: ScriptedCamera
    """
    Camera defining the diagnostic image dimensions.
    """

    margin: int = 10
    """
    Image border reserved around labels.
    """

    heading_height: int = 30
    """
    Baseline of the scene label in pixels.
    """

    legend_line_height: int = 20
    """
    Vertical spacing between model labels in the legend.
    """

    label_gap: int = 5
    """
    Gap between each region and its model label.
    """

    font_scale: float = 0.5
    """
    OpenCV label font scale.
    """

    font_thickness: int = 1
    """
    OpenCV label stroke width.
    """

    def draw(
        self, observation: ScriptedObservation, hypotheses: list[ObjectHypothesis]
    ) -> NDArray[np.uint8]:
        """
        Render the current scene label and its synthetic detections.
        """
        image = np.full(
            (self.camera.height, self.camera.width, 3),
            PreviewColor.BACKGROUND.value,
            dtype=np.uint8,
        )
        self.draw_label(image, observation.step.value, self.margin, self.heading_height)
        for row, hypothesis in enumerate(hypotheses, start=1):
            region = hypothesis.roi.roi
            cv2.rectangle(
                image,
                (region.pos.x, region.pos.y),
                (region.pos.x + region.width - 1, region.pos.y + region.height - 1),
                PreviewColor.FOREGROUND.value,
                cv2.FILLED,
            )
            classification = next(
                annotation
                for annotation in hypothesis.annotations
                if isinstance(annotation, Classification)
            )
            self.draw_label(
                image,
                f"{classification.class_id}: {classification.classname}",
                self.margin,
                self.heading_height + row * self.legend_line_height,
            )
            self.draw_label(
                image,
                str(classification.class_id),
                region.pos.x,
                max(self.margin, region.pos.y - self.label_gap),
            )
        return image

    def draw_label(
        self, image: NDArray[np.uint8], text: str, horizontal: int, vertical: int
    ) -> None:
        """
        Place a readable scene or model label in the diagnostic image.
        """
        cv2.putText(
            image,
            text,
            (horizontal, vertical),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.font_scale,
            PreviewColor.TEXT.value,
            self.font_thickness,
            cv2.LINE_8,
        )
