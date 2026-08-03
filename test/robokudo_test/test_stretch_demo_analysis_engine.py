"""
Tests for the Stretch apartment demo's perception pipeline configuration.
"""

from robokudo.annotators.pointcloud_crop import PointcloudCropAnnotator
from robokudo.descriptors.analysis_engines.stretch_demo import (
    TARGET_SHELF_LAYER_MAX_WORLD_Z,
    TARGET_SHELF_LAYER_MIN_WORLD_Z,
    AnalysisEngine,
)
from test.robokudo_test.test_analysis_engine_query_composition import (
    bounded_build_time,
)


def test_pointcloud_crop_is_narrowed_to_the_target_shelf_layer():
    """
    Without a height-bounded, world-relative crop, the pipeline's wide RealSense field
    of view sees every shelf layer at once and answers a query with one untyped
    candidate per layer instead of just the object on the targeted one.
    """
    with bounded_build_time():
        pipeline = AnalysisEngine().implementation()

    crop_annotator = next(
        node for node in pipeline.children if isinstance(node, PointcloudCropAnnotator)
    )
    parameters = crop_annotator.descriptor.parameters

    assert parameters.relative_to_world is True
    assert parameters.min_z == TARGET_SHELF_LAYER_MIN_WORLD_Z
    assert parameters.max_z == TARGET_SHELF_LAYER_MAX_WORLD_Z


def test_target_shelf_layer_bounds_are_the_midpoints_to_its_neighbours():
    """
    The bounds are derived from the apartment demo's own shelf layer heights (0.283m,
    0.63m, 1.265m, 1.613m), targeting the second layer at 0.63m -- pinned here so a
    change to either side silently drifting out of sync is caught.
    """
    shelf_layer_heights = [0.283, 0.63, 1.265, 1.613]
    target_layer_height = shelf_layer_heights[1]

    assert (
        TARGET_SHELF_LAYER_MIN_WORLD_Z
        == (shelf_layer_heights[0] + target_layer_height) / 2
    )
    assert (
        TARGET_SHELF_LAYER_MAX_WORLD_Z
        == (target_layer_height + shelf_layer_heights[2]) / 2
    )
