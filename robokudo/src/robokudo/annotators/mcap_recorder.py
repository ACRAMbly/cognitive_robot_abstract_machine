"""ROS 2 topic recording to MCAP files."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path

from py_trees.common import Status

import rclpy

from robokudo.annotators.core import BaseAnnotator

import rosbag2_py

from typing_extensions import Any, Protocol


def _default_output_directory() -> Path:
    """Create a unique default path for an Orbbec recording."""
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S_%f')
    return Path.home() / 'blue' / 'data' / f'robokudo_orbbec_{timestamp}'


class McapStorageFormat(Enum):
    """Identify the rosbag storage format used by the recorder."""

    MCAP = 'mcap'


class McapStoragePreset(Enum):
    """Identify supported MCAP writer profiles."""

    ZSTD_FAST = 'zstd_fast'


class McapRecordingPathExistsError(FileExistsError):
    """Indicate that recording would overwrite an existing bag directory."""


class McapStoragePluginUnavailableError(RuntimeError):
    """Indicate that rosbag2 cannot write the MCAP format."""


class McapRecorderNotStartedError(RuntimeError):
    """Indicate that the recorder was ticked before successful setup."""


@dataclass(frozen=True, slots=True)
class McapRecorderConfiguration:
    """Configure recording of one set of ROS sensor topics."""

    topics: tuple[str, ...]
    """ROS topics required to replay the configured sensor input."""

    output_directory: Path = field(default_factory=_default_output_directory)
    """Directory in which rosbag2 stores the MCAP recording."""

    storage_preset: McapStoragePreset = McapStoragePreset.ZSTD_FAST
    """MCAP profile controlling chunking and compression."""

    node_name: str = 'robokudo_mcap_recorder'
    """ROS node name used by the rosbag recorder."""

    def __post_init__(self) -> None:
        """Validate values required by rosbag2."""
        if not self.topics:
            raise ValueError('At least one ROS topic must be recorded.')
        if not self.node_name:
            raise ValueError('The recorder node name must not be empty.')


class McapRecorderBackend(Protocol):
    """Provide the rosbag recorder lifecycle used by the annotator."""

    def start_spin(self) -> None:
        """Start topic discovery and subscription processing."""

    def record(self) -> None:
        """Start writing discovered topic messages."""

    def stop(self) -> None:
        """Finish writing and close the recording."""

    def stop_spin(self) -> None:
        """Stop topic discovery and subscription processing."""


class McapRecorderFactory(Protocol):
    """Create a configured MCAP recorder backend."""

    def create(
        self,
        storage_options: rosbag2_py.StorageOptions,
        record_options: rosbag2_py.RecordOptions,
        node_name: str,
    ) -> McapRecorderBackend:
        """Create a recorder for the supplied rosbag options."""


class ShutdownContext(Protocol):
    """Register cleanup work with the active ROS context."""

    def on_shutdown(self, callback: Callable[[], None]) -> None:
        """Register a callback invoked during ROS shutdown."""


@dataclass(frozen=True, slots=True)
class RosbagMcapRecorderFactory:
    """Create recorder backends through the installed rosbag2 transport."""

    def create(
        self,
        storage_options: rosbag2_py.StorageOptions,
        record_options: rosbag2_py.RecordOptions,
        node_name: str,
    ) -> McapRecorderBackend:
        """Create a recorder after verifying MCAP writer availability."""
        if (
            McapStorageFormat.MCAP.value
            not in rosbag2_py.get_registered_writers()
        ):
            raise McapStoragePluginUnavailableError(
                'The rosbag2 MCAP storage plugin is not installed.'
            )
        return rosbag2_py.Recorder(
            storage_options,
            record_options,
            'info',
            node_name,
        )


class McapRecorderAnnotator(BaseAnnotator):
    """Record configured ROS topics to MCAP during pipeline execution."""

    def __init__(
        self,
        configuration: McapRecorderConfiguration,
        name: str = 'McapRecorder',
        recorder_factory: McapRecorderFactory | None = None,
        shutdown_context: ShutdownContext | None = None,
    ) -> None:
        """Initialize the recorder without starting ROS subscriptions.

        :param name: Annotator name.
        :param configuration: Recording path, topics, and MCAP profile.
        :param recorder_factory: Factory used to create the rosbag backend.
        :param shutdown_context: ROS context that owns recorder cleanup.
        """
        super().__init__(name=name, descriptor=BaseAnnotator.Descriptor())
        self.configuration = configuration
        """Recording settings used when :meth:`setup` starts the backend."""

        self.recorder_factory = (
            recorder_factory
            if recorder_factory is not None
            else RosbagMcapRecorderFactory()
        )
        """Factory responsible for constructing the recorder backend."""

        self.shutdown_context = (
            shutdown_context
            if shutdown_context is not None
            else rclpy.get_default_context()
        )
        """ROS context that finalizes the bag during application shutdown."""

        self.recorder: McapRecorderBackend | None = None
        """Active recorder backend, when setup has completed."""

        self.is_recording = False
        """Whether the backend is currently accepting topic messages."""

    def setup(self, **setup_arguments: Any) -> bool:
        """Start MCAP recording and register shutdown cleanup.

        :param setup_arguments: Behaviour-tree setup arguments.
        :return: Whether recording started successfully.
        """
        if self.is_recording:
            return True

        output_directory = self.configuration.output_directory
        if output_directory.exists():
            raise McapRecordingPathExistsError(
                f'MCAP recording directory already exists: {output_directory}'
            )
        output_directory.parent.mkdir(parents=True, exist_ok=True)

        storage_options = rosbag2_py.StorageOptions(
            uri=str(output_directory),
            storage_id=McapStorageFormat.MCAP.value,
            storage_preset_profile=self.configuration.storage_preset.value,
        )
        record_options = rosbag2_py.RecordOptions()
        record_options.topics = list(self.configuration.topics)
        record_options.all_topics = False
        record_options.disable_keyboard_controls = True

        self.recorder = self.recorder_factory.create(
            storage_options=storage_options,
            record_options=record_options,
            node_name=self.configuration.node_name,
        )
        self.recorder.start_spin()
        self.recorder.record()
        self.is_recording = True
        self.shutdown_context.on_shutdown(self.shutdown)

        self.rk_logger.info(
            'Recording Orbbec sensor topics to MCAP at %s',
            output_directory,
        )
        return True

    def update(self) -> Status:
        """Keep recording while the perception pipeline processes data."""
        if not self.is_recording:
            raise McapRecorderNotStartedError(
                'MCAP recorder setup did not complete before the '
                'pipeline tick.'
            )
        self.feedback_message = (
            f'Recording sensor topics to {self.configuration.output_directory}'
        )
        return Status.SUCCESS

    def shutdown(self) -> None:
        """Stop topic discovery and finalize the MCAP recording once."""
        if not self.is_recording or self.recorder is None:
            return

        recorder = self.recorder
        self.is_recording = False
        try:
            try:
                recorder.stop()
            finally:
                recorder.stop_spin()
        finally:
            self.recorder = None

        self.rk_logger.info(
            'Finalized MCAP recording at %s',
            self.configuration.output_directory,
        )
