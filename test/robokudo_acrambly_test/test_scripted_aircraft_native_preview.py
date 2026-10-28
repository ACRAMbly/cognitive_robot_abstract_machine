# %% Imports

from __future__ import annotations

from enum import IntEnum
from subprocess import CompletedProcess
from unittest.mock import Mock

import numpy
import pytest
from py_trees.common import Status
from typing_extensions import TYPE_CHECKING

import robokudo.vis.cv_visualizer as native_image_windows
from robokudo.annotators.testing import EmptyAnnotator
from robokudo.garden import grow_tree
from robokudo.vis.cv_visualizer import CVVisualizer
from robokudo.vis.o3d_visualizer import O3DVisualizer
from robokudo.vis.visualizer import Visualizer

from .test_scripted_aircraft_preview import (
    aircraft_frames,
    preview_engine,
    replacement_model,
)
from .test_scripted_tetris_visualization import (
    DesktopKey,
    InteractiveScene,
    native_key_events,
)

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
        AnalysisEngine,
    )


# %% Native preview controls


class PreviewKey(IntEnum):
    """
    Native exit event exercised without a desktop.
    """

    EXIT_REQUESTED = 27


def test_aircraft_native_preview_retains_an_interactive_final_scene(
    preview_engine: AnalysisEngine,
    node: Node,
    native_key_events: Mock,
) -> None:
    """
    Final-scene rendering and annotator controls remain responsive after exhaustion.
    """
    period = preview_engine.scenario.timing.sample_period_nanoseconds
    preview_engine.scenario.timing.duration_nanoseconds = period
    pipeline = preview_engine.implementation()
    comparison_annotator = EmptyAnnotator()
    pipeline.add_child(comparison_annotator)
    tree = grow_tree(
        pipeline,
        node=node,
        include_gui=True,
        visualizer_types=[CVVisualizer, O3DVisualizer],
    )
    tree.tick()
    preview_engine.clock.timestamp_nanoseconds += period
    tree.tick()
    completed_observation = pipeline.cas
    source = pipeline.get_annotators()[0]
    completed_geometry = source.get_annotator_output_struct().geometries
    tree.tick()
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
    scene = geometry_backend.viewer3d
    assert type(scene) is InteractiveScene
    original_scene_ticks = scene.event_ticks
    original_key_polls = native_key_events.call_count
    for _ in range(2):
        tree.tick()
        assert pipeline.cas is completed_observation
    assert scene.event_ticks == original_scene_ticks + 2
    assert native_key_events.call_count == original_key_polls + 2
    numpy.testing.assert_array_equal(
        numpy.asarray(scene.geometries[0].vertices),
        numpy.asarray(completed_geometry[0].vertices),
    )

    assert image_backend.handle_keycallback(DesktopKey.NEXT_ANNOTATOR)
    tree.tick()
    assert (
        geometry_backend.shared_visualizer_state.active_annotator
        is comparison_annotator
    )
    assert scene.geometries == []
    assert image_backend.handle_keycallback(DesktopKey.PREVIOUS_ANNOTATOR)
    tree.tick()
    assert geometry_backend.shared_visualizer_state.active_annotator is source
    numpy.testing.assert_array_equal(
        numpy.asarray(scene.geometries[0].vertices),
        numpy.asarray(completed_geometry[0].vertices),
    )


@pytest.mark.parametrize("window_closed", [False, True])
def test_aircraft_native_preview_preserves_the_final_capture_on_exit(
    preview_engine: AnalysisEngine,
    node: Node,
    native_key_events: Mock,
    monkeypatch: pytest.MonkeyPatch,
    window_closed: bool,
) -> None:
    """
    Escape and closing the native scene terminate viewing without new captures.
    """
    preview_engine.scenario.timing.duration_nanoseconds = 0
    pipeline = preview_engine.implementation()
    tree = grow_tree(
        pipeline,
        node=node,
        include_gui=True,
        visualizer_types=[CVVisualizer, O3DVisualizer],
    )
    tree.tick()
    completed_observation = pipeline.cas
    tree.tick()
    geometry_backend = next(
        visualizer
        for visualizer in Visualizer.instances
        if isinstance(visualizer, O3DVisualizer)
    )
    if window_closed:
        geometry_backend.viewer3d.is_open = False
    else:
        image_backend = next(
            visualizer
            for visualizer in Visualizer.instances
            if isinstance(visualizer, CVVisualizer)
        )
        window_title = CompletedProcess(
            args=[],
            returncode=0,
            stdout=image_backend.window_title().encode(),
            stderr=b"",
        )
        monkeypatch.setattr(
            native_image_windows.subprocess, "run", Mock(return_value=window_title)
        )
        native_key_events.return_value = PreviewKey.EXIT_REQUESTED
    tree.tick()

    assert tree.root.children[0].status is Status.FAILURE
    assert pipeline.cas is completed_observation


def test_aircraft_headless_playback_completes_without_native_windows(
    preview_engine: AnalysisEngine,
    node: Node,
    native_key_events: Mock,
) -> None:
    """
    Finite observation delivery requires no native event processing.
    """
    preview_engine.preview = None
    preview_engine.scenario.timing.duration_nanoseconds = 0
    pipeline = preview_engine.implementation()
    tree = grow_tree(pipeline, node=node, include_gui=False)
    tree.tick()
    tree.tick()

    assert tree.root.children[0].status is Status.FAILURE
    assert Visualizer.instances == []
    assert native_key_events.call_count == 0
