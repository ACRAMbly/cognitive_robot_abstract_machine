# %% Imports

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack
from enum import IntEnum
from time import monotonic

import pytest
import rclpy
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING
from visualization_msgs.msg import MarkerArray

from robokudo import world
from robokudo.vis.cv_visualizer import CVVisualizer
from robokudo.vis.o3d_visualizer import O3DVisualizer
from robokudo.vis.ros_visualizer import AllAnnotatorROSVisualizer, SharedROSVisualizer
from robokudo.vis.visualizer import Visualizer
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.aircraft_observations.inspection import AircraftBeliefInspection
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)

from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_aircraft_native_preview import PreviewKey
from .test_scripted_tetris import ControlledClock
from .test_scripted_tetris_visualization import DesktopKey, native_key_events

if TYPE_CHECKING:
    from unittest.mock import Mock

    from rclpy.node import Node
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames


# %% Inspection lifecycle


class InspectionLimit(IntEnum):
    """Bounds for local inspection message delivery."""

    DELIVERY_SECONDS = 5
    """Maximum wait for the final latched geometry message."""


@pytest.fixture
def inspection(
    aircraft_catalog: AircraftModelCatalog,
    aircraft_frames: AircraftFrames,
    semdt_cas: CAS,
    native_key_events: Mock,
    monkeypatch: pytest.MonkeyPatch,
    node: Node,
) -> Iterator[AircraftBeliefInspection]:
    """Inspect a single completed aircraft capture in an isolated belief world."""
    world.clear_world()
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    engine = AnalysisEngine(frames=aircraft_frames, clock=ControlledClock())
    engine.scenario.timing.duration_nanoseconds = 0
    with AircraftBeliefInspection(node=node, engine=engine) as inspection:
        yield inspection


def test_inspection_opens_the_standard_annotator_visualizers(
    inspection: AircraftBeliefInspection,
) -> None:
    """Inspection displays annotator output through all default visualizers."""
    AircraftBeliefCapture(inspection.pipeline).complete()
    inspection.tick()

    assert [type(visualizer) for visualizer in Visualizer.instances] == [
        CVVisualizer,
        O3DVisualizer,
        SharedROSVisualizer,
        AllAnnotatorROSVisualizer,
    ]


def test_finished_inspection_keeps_annotator_controls_responsive(
    inspection: AircraftBeliefInspection,
    native_key_events: Mock,
) -> None:
    """Annotator selection and rendering remain interactive after playback ends."""
    AircraftBeliefCapture(inspection.pipeline).complete()
    inspection.tick()
    final_capture = inspection.pipeline.cas
    image_backend = next(
        visualizer
        for visualizer in Visualizer.instances
        if isinstance(visualizer, CVVisualizer)
    )
    geometry_backend = next(
        visualizer
        for visualizer in Visualizer.instances
        if isinstance(visualizer, O3DVisualizer)
    )
    scene_ticks = geometry_backend.viewer3d.event_ticks
    key_polls = native_key_events.call_count
    source, connector = inspection.pipeline.get_annotators()

    image_backend.handle_keycallback(DesktopKey.NEXT_ANNOTATOR)
    inspection.tick()
    assert geometry_backend.shared_visualizer_state.active_annotator is connector
    image_backend.handle_keycallback(DesktopKey.PREVIOUS_ANNOTATOR)
    inspection.tick()

    assert geometry_backend.shared_visualizer_state.active_annotator is source
    assert geometry_backend.viewer3d.event_ticks == scene_ticks + 2
    assert native_key_events.call_count == key_polls + 2
    assert inspection.pipeline.cas is final_capture


@pytest.mark.parametrize("window_closed", [False, True])
@pytest.mark.parametrize("playback_finished", [False, True])
def test_inspection_stops_when_a_standard_visualizer_requests_exit(
    inspection: AircraftBeliefInspection,
    native_key_events: Mock,
    window_closed: bool,
    playback_finished: bool,
) -> None:
    """Escape and Open3D closure end inspection during playback or final viewing."""
    if not playback_finished:
        inspection.engine.scenario.timing.duration_nanoseconds = (
            inspection.engine.scenario.timing.sample_period_nanoseconds
        )
    AircraftBeliefCapture(inspection.pipeline).complete()
    inspection.tick()
    final_capture = inspection.pipeline.cas
    if window_closed:
        geometry_backend = next(
            visualizer
            for visualizer in Visualizer.instances
            if isinstance(visualizer, O3DVisualizer)
        )
        geometry_backend.viewer3d.is_open = False
    else:
        image_backend = next(
            visualizer
            for visualizer in Visualizer.instances
            if isinstance(visualizer, CVVisualizer)
        )
        image_backend.handle_keycallback(PreviewKey.EXIT_REQUESTED)
    native_key_events.side_effect = [DesktopKey.NONE]

    inspection.run()

    assert inspection.pipeline.cas is final_capture


def test_inspection_publishes_geometry_from_the_local_belief_world(
    inspection: AircraftBeliefInspection,
) -> None:
    """Every accepted belief supplies its geometry to the inspection markers."""
    AircraftBeliefCapture(inspection.pipeline).complete()

    assert {marker.ns for marker in inspection.markers.markers.markers} == {
        str(belief.body.name) for belief in world.get_object_belief_states().values()
    }
    assert len(inspection.markers.markers.markers) == len(
        inspection.engine.scenario.parts
    )


def test_finished_inspection_retains_beliefs_and_refreshes_transforms(
    inspection: AircraftBeliefInspection,
) -> None:
    """Idle inspection retains the accepted capture and publishes fresh TF stamps."""
    AircraftBeliefCapture(inspection.pipeline).complete()
    final_capture = inspection.pipeline.cas
    final_markers = inspection.markers.markers
    inspection.tick()
    previous_stamp = (
        inspection.markers.tf_publisher.tf_model_callback.tf_message.transforms[
            0
        ].header.stamp
    )
    inspection.tick()

    assert inspection.pipeline.status is Status.FAILURE
    assert inspection.pipeline.cas is final_capture
    assert inspection.markers.markers is final_markers
    assert inspection.node.context.ok()
    current_stamp = (
        inspection.markers.tf_publisher.tf_model_callback.tf_message.transforms[
            0
        ].header.stamp
    )
    assert (current_stamp.sec, current_stamp.nanosec) > (
        previous_stamp.sec,
        previous_stamp.nanosec,
    )


def test_late_subscriber_receives_final_belief_geometry(
    inspection: AircraftBeliefInspection,
) -> None:
    """An RViz subscriber opened after playback receives the completed belief world."""
    deadline = monotonic() + InspectionLimit.DELIVERY_SECONDS
    while inspection.pipeline.status is not Status.FAILURE and monotonic() < deadline:
        rclpy.spin_once(inspection.node, timeout_sec=inspection.tick_period_seconds)
    assert inspection.pipeline.status is Status.FAILURE

    received: list[MarkerArray] = []
    with ExitStack() as resources:
        subscription = inspection.node.create_subscription(
            MarkerArray,
            inspection.markers.topic_name,
            received.append,
            inspection.markers.qos_profile,
        )
        resources.callback(inspection.node.destroy_subscription, subscription)
        deadline = monotonic() + InspectionLimit.DELIVERY_SECONDS
        expected = inspection.markers.markers
        while (not received or received[-1] != expected) and monotonic() < deadline:
            rclpy.spin_once(inspection.node, timeout_sec=inspection.tick_period_seconds)

    assert received[-1] == expected
