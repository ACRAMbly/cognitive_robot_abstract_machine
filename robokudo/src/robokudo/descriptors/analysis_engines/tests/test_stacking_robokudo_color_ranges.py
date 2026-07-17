"""Tests for stacking-pipeline colored-block segmentation."""

from __future__ import annotations

from robokudo.descriptors.analysis_engines.stacking_robokudo import (
    create_color_cluster_descriptor,
)


def test_descriptor_covers_measured_block_hues() -> None:
    """Cover the observed blue, red, and yellow block hue intervals."""
    descriptor = create_color_cluster_descriptor()

    assert descriptor.parameters.color_name_to_hsv_range == {
        'blue': {
            'hsv_min': (135, 130, 85),
            'hsv_max': (165, 255, 255),
        },
        'red': {
            'hsv_min': (0, 150, 95),
            'hsv_max': (15, 255, 255),
        },
        'yellow': {
            'hsv_min': (22, 130, 85),
            'hsv_max': (65, 255, 255),
        },
    }
