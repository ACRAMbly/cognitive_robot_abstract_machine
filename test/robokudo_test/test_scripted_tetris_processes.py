"""
Verify static CAD replication through separately launched applications.
"""

from __future__ import annotations

import subprocess
import sys
from contextlib import ExitStack
from pathlib import Path
from time import monotonic
from uuid import uuid4

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from robokudo.descriptors.scripted_tetris_observations import create_tetris_scenario
from robokudo.scripts.scripted_tetris_sync import ProcessRole
from robokudo.synchronized_tetris import ReceivingTopic

# %% process replication


def stop_process(process: subprocess.Popen) -> None:
    """
    Terminate a demonstration and require orderly application shutdown.
    """
    if process.poll() is None:
        process.terminate()
    process.wait(timeout=15)
    assert process.returncode == 0


def test_separate_receiver_publishes_initial_geometry_and_transforms(
    node: Node,
    tmp_path: Path,
) -> None:
    """
    Receive CAD meshes, boxes, and accepted poses solely from another process.
    """
    namespace = f"tetris_{uuid4().hex}"
    topic = f"/{namespace}/world_sync"
    visuals: list[MarkerArray] = []
    collisions: list[MarkerArray] = []
    transforms: list[TFMessage] = []
    with ExitStack() as cleanup:
        for receiving_topic, callback in (
            (ReceivingTopic.VISUAL, visuals.append),
            (ReceivingTopic.COLLISION, collisions.append),
        ):
            subscription = node.create_subscription(
                MarkerArray,
                f"/{namespace}/{receiving_topic.lstrip('/')}",
                callback,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            cleanup.callback(node.destroy_subscription, subscription)
        subscription = node.create_subscription(
            TFMessage,
            f"/{namespace}/tf",
            transforms.append,
            10,
        )
        cleanup.callback(node.destroy_subscription, subscription)
        receiver_log = cleanup.enter_context(
            (tmp_path / ProcessRole.RECEIVER).open("w")
        )
        receiver = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "robokudo.scripts.scripted_tetris_sync",
                ProcessRole.RECEIVER,
                "--topic",
                topic,
                "--namespace",
                namespace,
            ],
            stdout=receiver_log,
            stderr=subprocess.STDOUT,
        )
        cleanup.callback(stop_process, receiver)
        deadline = monotonic() + 30
        while node.count_subscribers(topic) == 0 and monotonic() < deadline:
            assert receiver.poll() is None, (
                tmp_path / ProcessRole.RECEIVER
            ).read_text()
            rclpy.spin_once(node, timeout_sec=0.01)
        assert node.count_subscribers(topic) == 1
        source_log = cleanup.enter_context((tmp_path / ProcessRole.SOURCE).open("w"))
        source = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "robokudo.scripts.scripted_tetris_sync",
                ProcessRole.SOURCE,
                "--topic",
                topic,
                "--namespace",
                namespace,
            ],
            stdout=source_log,
            stderr=subprocess.STDOUT,
        )
        cleanup.callback(stop_process, source)
        while monotonic() < deadline:
            assert receiver.poll() is None, (
                tmp_path / ProcessRole.RECEIVER
            ).read_text()
            assert source.poll() is None, (tmp_path / ProcessRole.SOURCE).read_text()
            rclpy.spin_once(node, timeout_sec=0.01)
            if (
                visuals
                and collisions
                and len(visuals[-1].markers) == 2
                and len(collisions[-1].markers) == 2
                and transforms
            ):
                break
        assert visuals and collisions and transforms
        assert [marker.type for marker in visuals[-1].markers] == [
            Marker.MESH_RESOURCE
        ] * 2
        assert [marker.type for marker in collisions[-1].markers] == [Marker.CUBE] * 2
        expected_translations = sorted(
            tuple(detection.pose.translation)
            for detection in create_tetris_scenario().observations[0].detections
        )
        accepted_translations = []
        while monotonic() < deadline:
            accepted_translations = sorted(
                (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                )
                for transform in transforms[-1].transforms
                if transform.child_frame_id
                in {marker.header.frame_id for marker in visuals[-1].markers}
            )
            if len(accepted_translations) == len(expected_translations) and np.allclose(
                accepted_translations, expected_translations
            ):
                break
            rclpy.spin_once(node, timeout_sec=0.01)
        np.testing.assert_allclose(accepted_translations, expected_translations)
        assert {
            endpoint.node_name
            for endpoint in node.get_publishers_info_by_topic(f"/{namespace}/tf")
        } == {"scripted_tetris_receiver"}
