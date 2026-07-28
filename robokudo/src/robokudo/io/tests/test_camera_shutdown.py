"""Tests for orderly camera-interface shutdown."""

from __future__ import annotations

from dataclasses import dataclass

from robokudo.descriptors import CollectionReaderAnnotator
from robokudo.io import camera_interface


@dataclass(frozen=True, slots=True)
class PassiveCameraConfiguration:
    """Provide configuration sufficient for collection-reader construction."""


@dataclass(slots=True)
class ShutdownTrackingCameraInterface:
    """Record collection-reader ownership cleanup."""

    events: list[str]
    """Shared shutdown event sequence."""

    interface_type: str = 'ShutdownTrackingCamera'
    """Interface name exposed to the collection reader."""

    def shutdown(self) -> None:
        """Record camera-interface shutdown."""
        self.events.append('camera')


@dataclass(slots=True)
class OrderedCameraExecutor:
    """Record executor shutdown."""

    events: list[str]
    """Shared shutdown event sequence."""

    def spin(self) -> None:
        """Provide the camera executor spin contract."""

    def shutdown(self) -> None:
        """Record executor shutdown."""
        self.events.append('executor')


@dataclass(slots=True)
class OrderedCameraThread:
    """Record camera-thread lifecycle operations."""

    events: list[str]
    """Shared shutdown event sequence."""

    def start(self) -> None:
        """Record thread startup."""
        self.events.append('thread_start')

    def join(self) -> None:
        """Record thread joining."""
        self.events.append('thread_join')


@dataclass(slots=True)
class OrderedCameraNode:
    """Record camera-node destruction."""

    events: list[str]
    """Shared shutdown event sequence."""

    def destroy_node(self) -> None:
        """Record node destruction."""
        self.events.append('node')


def test_collection_reader_shuts_down_owned_camera_interfaces() -> None:
    """Release collection-reader camera resources with the behavior tree."""
    events: list[str] = []
    camera = ShutdownTrackingCameraInterface(events)
    descriptor = CollectionReaderAnnotator.Descriptor(
        camera_config=PassiveCameraConfiguration(),
        camera_interface=camera,
    )
    reader = CollectionReaderAnnotator(descriptor=descriptor)

    reader.shutdown()

    assert events == ['camera']


def test_camera_resources_stop_executor_before_destroying_node() -> None:
    """Join the camera executor thread before destroying its ROS node."""
    events: list[str] = []
    resources = camera_interface.CameraRuntimeResources(
        node=OrderedCameraNode(events),
        executor=OrderedCameraExecutor(events),
        thread=OrderedCameraThread(events),
    )

    resources.start()
    resources.shutdown()
    resources.shutdown()

    assert events == [
        'thread_start',
        'executor',
        'thread_join',
        'node',
    ]
