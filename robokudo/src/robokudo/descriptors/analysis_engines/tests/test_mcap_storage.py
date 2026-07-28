"""Tests for recording Orbbec sensor topics to MCAP."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import pytest

from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.annotators.mcap_recorder import (
    McapRecorderAnnotator,
    McapRecorderConfiguration,
    McapRecordingPathExistsError,
)
from robokudo.descriptors.analysis_engines import storage


@dataclass
class RecordingLifecycleBackend:
    """Record lifecycle calls without opening a rosbag."""

    events: list[str] = field(default_factory=list)
    """Recorder operations in invocation order."""

    def start_spin(self) -> None:
        """Record that ROS topic discovery started."""
        self.events.append('start_spin')

    def record(self) -> None:
        """Record that message recording started."""
        self.events.append('record')

    def stop(self) -> None:
        """Record that message recording stopped."""
        self.events.append('stop')

    def stop_spin(self) -> None:
        """Record that ROS topic discovery stopped."""
        self.events.append('stop_spin')


@dataclass
class OptionCapturingRecorderFactory:
    """Capture rosbag options and return a lifecycle backend."""

    backend: RecordingLifecycleBackend = field(
        default_factory=RecordingLifecycleBackend
    )
    """Backend returned for each creation request."""

    storage_options: object | None = None
    """Storage options received from the annotator."""

    record_options: object | None = None
    """Record options received from the annotator."""

    node_name: str | None = None
    """ROS node name received from the annotator."""

    creation_count: int = 0
    """Number of recorder creation requests."""

    def create(
        self,
        storage_options: object,
        record_options: object,
        node_name: str,
    ) -> RecordingLifecycleBackend:
        """Capture one creation request and return the backend."""
        self.storage_options = storage_options
        self.record_options = record_options
        self.node_name = node_name
        self.creation_count += 1
        return self.backend


@dataclass
class CallbackShutdownContext:
    """Collect callbacks registered for ROS shutdown."""

    callbacks: list[Callable[[], None]] = field(default_factory=list)
    """Callbacks registered by runtime resources."""

    def on_shutdown(self, callback: Callable[[], None]) -> None:
        """Register a callback for later invocation."""
        self.callbacks.append(callback)


@dataclass
class AvailableSensorDataInterface:
    """Mimic a camera interface with immediately available sensor data."""

    interface_type: str = 'MimicCamera'
    """Interface name exposed to the collection reader."""

    def has_new_data(self) -> bool:
        """Report that one sensor frame is available."""
        return True

    def set_data(self, cas: object) -> None:
        """Accept the target CAS without modifying it."""


@dataclass(frozen=True)
class OrbbecTopicConfiguration:
    """Mimic an RGB-D camera configuration with Orbbec topic names."""

    topic_depth: str = '/camera/depth/image_raw'
    """Depth image topic."""

    topic_color: str = '/camera/color/image_raw/compressed'
    """Compressed color image topic."""

    topic_cam_info: str = '/camera/color/camera_info'
    """Camera calibration topic."""


def test_default_recording_directory_is_blue_data(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """Place generated recordings below the user's blue data path."""
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)

    configuration = McapRecorderConfiguration(topics=('/camera/depth',))

    assert configuration.output_directory.parent == tmp_path / 'blue' / 'data'
    assert configuration.output_directory.name.startswith('robokudo_orbbec_')


def test_setup_records_expected_orbbec_topics_to_compressed_mcap(
    tmp_path: Path,
) -> None:
    """Start an indexed compressed MCAP recording for every required topic."""
    output_directory = tmp_path / 'orbbec_recording'
    recorder_factory = OptionCapturingRecorderFactory()
    shutdown_context = CallbackShutdownContext()
    annotator = McapRecorderAnnotator(
        configuration=McapRecorderConfiguration(
            output_directory=output_directory,
            topics=(
                '/camera/depth/image_raw',
                '/camera/color/image_raw/compressed',
                '/camera/color/camera_info',
                '/tf',
                '/tf_static',
            ),
        ),
        recorder_factory=recorder_factory,
        shutdown_context=shutdown_context,
    )

    assert annotator.setup()

    storage_options = recorder_factory.storage_options
    assert storage_options is not None
    assert storage_options.uri == str(output_directory)
    assert storage_options.storage_id == 'mcap'
    assert storage_options.storage_preset_profile == 'zstd_fast'

    record_options = recorder_factory.record_options
    assert record_options is not None
    assert record_options.topics == [
        '/camera/depth/image_raw',
        '/camera/color/image_raw/compressed',
        '/camera/color/camera_info',
        '/tf',
        '/tf_static',
    ]
    assert not record_options.all_topics
    assert record_options.disable_keyboard_controls
    assert recorder_factory.node_name == 'robokudo_mcap_recorder'
    assert recorder_factory.backend.events == ['start_spin', 'record']
    assert len(shutdown_context.callbacks) == 1

    shutdown_context.callbacks[0]()
    annotator.shutdown()

    assert recorder_factory.backend.events == [
        'start_spin',
        'record',
        'stop',
        'stop_spin',
    ]


def test_setup_rejects_an_existing_recording_directory(tmp_path: Path) -> None:
    """Refuse to overwrite an existing bag directory."""
    output_directory = tmp_path / 'existing_recording'
    output_directory.mkdir()
    recorder_factory = OptionCapturingRecorderFactory()
    annotator = McapRecorderAnnotator(
        configuration=McapRecorderConfiguration(
            output_directory=output_directory,
            topics=('/camera/depth/image_raw',),
        ),
        recorder_factory=recorder_factory,
        shutdown_context=CallbackShutdownContext(),
    )

    with pytest.raises(McapRecordingPathExistsError):
        annotator.setup()

    assert recorder_factory.creation_count == 0


def test_storage_engine_uses_mcap_recorder_after_preprocessing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Preserve the sensor pipeline and replace MongoDB with MCAP recording."""
    camera_descriptor = CollectionReaderAnnotator.Descriptor(
        camera_config=OrbbecTopicConfiguration(),
        camera_interface=AvailableSensorDataInterface(),
    )
    monkeypatch.setattr(
        storage.CrDescriptorFactory,
        'create_descriptor',
        lambda camera: camera_descriptor,
    )

    pipeline = storage.AnalysisEngine().implementation()

    recorder = pipeline.children[-1]
    assert isinstance(recorder, McapRecorderAnnotator)
    assert recorder.configuration.topics == (
        '/camera/depth/image_raw',
        '/camera/color/image_raw/compressed',
        '/camera/color/camera_info',
        '/tf',
        '/tf_static',
    )
    assert [
        type(child).__name__
        for child in pipeline.children
    ] == [
        'Sequence',
        'CollectionReaderAnnotator',
        'ImagePreprocessorAnnotator',
        'McapRecorderAnnotator',
    ]
