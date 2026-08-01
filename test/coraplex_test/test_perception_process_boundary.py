"""
The RoboKudo perception source against a pipeline running in its own process.

Everything else about perception is covered in ``test_perception.py`` against a stand-in
node inside the test's own interpreter, which shares an rclpy context with the client.
This is the only test where the pipeline is discovered across a real process boundary,
and so the only one that exercises what happens on the robot: cross-context discovery,
and the ``server_timeout`` budget that a locally created server never makes the client
spend.
"""

from __future__ import annotations

from datetime import timedelta

import numpy as np
import pytest
from rclpy.action import get_action_names_and_types

from coraplex.exceptions import PerceptionSourceUnavailable
from coraplex.perception import (
    ROBOKUDO_QUERY_ACTION_NAME,
    PerceptionQuery,
    RoboKudoPerception,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk

from ..dataset import PERCEPTION_PIPELINE_STAND_IN_PATH
from ..standalone_process import StandaloneProcess

REPORTED_CLASS_LABEL = "Milk"
"""
Label the pipeline in the other process is told to report.
"""

REPORTED_POSITION = (2.6, 2.2, 1.05)
"""
Position the pipeline in the other process is told to report.
"""

MISSING_SOURCE_TIMEOUT = timedelta(seconds=2)
"""
How long to look for a pipeline that is not running.

Short enough to keep the test quick, long enough that a slow machine does not mistake
discovery latency for an absent server.
"""


@pytest.fixture
def perception_pipeline_process(rclpy_node):
    """
    A perception pipeline serving the query action from its own process.
    """
    pytest.importorskip("robokudo_msgs")

    def is_serving_queries() -> bool:
        return any(
            name.lstrip("/") == ROBOKUDO_QUERY_ACTION_NAME
            for name, _ in get_action_names_and_types(rclpy_node)
        )

    with StandaloneProcess(
        launcher_path=PERCEPTION_PIPELINE_STAND_IN_PATH,
        is_ready=is_serving_queries,
        arguments=[
            "--class-label",
            REPORTED_CLASS_LABEL,
            "--position",
            *(str(coordinate) for coordinate in REPORTED_POSITION),
        ],
    ) as process:
        yield process


def test_detection_crosses_a_process_boundary(
    immutable_model_world, whole_scene_region, rclpy_node, perception_pipeline_process
):
    """
    A pipeline that shares no interpreter state with the test is discovered through the
    middleware alone, and what it reports arrives intact.
    """
    world, view, context = immutable_model_world
    query = PerceptionQuery(Milk, whole_scene_region, view, world)

    detections = RoboKudoPerception(ros_node=rclpy_node).detect(query)

    assert [detection.class_label for detection in detections] == [REPORTED_CLASS_LABEL]
    np.testing.assert_allclose(
        detections[0].pose.to_position().to_np().flatten()[:3],
        REPORTED_POSITION,
        atol=1e-9,
    )


def test_absent_perception_source_is_reported(
    immutable_model_world, whole_scene_region, rclpy_node
):
    """
    With nothing serving the action, the source has to give up and say so rather than
    block forever or return an empty result that reads like "saw nothing".
    """
    pytest.importorskip("robokudo_msgs")
    world, view, context = immutable_model_world
    query = PerceptionQuery(Milk, whole_scene_region, view, world)

    with pytest.raises(PerceptionSourceUnavailable):
        RoboKudoPerception(
            ros_node=rclpy_node, server_timeout=MISSING_SOURCE_TIMEOUT
        ).detect(query)
