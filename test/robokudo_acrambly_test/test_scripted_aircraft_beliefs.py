# %% Imports

from __future__ import annotations

import numpy
import pytest
from py_trees.common import Status
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING

from robokudo import world
from robokudo.types.annotation import PoseAnnotation
from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.types import AircraftPart
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from robokudo_acrambly.playback.motion import VisibilityBoundary

from ..robokudo_test.dataset.semantic_observations import BeliefSnapshot
from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .dataset.aircraft_observations import AircraftIdentity
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from rclpy.node import Node

    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames

    from .conftest import AircraftPoseComparison


# %% Synthetic observations accepted into local beliefs


def test_scripted_aircraft_updates_local_beliefs_through_absence_and_reappearance(
    aircraft_catalog: AircraftModelCatalog,
    aircraft_frames: AircraftFrames,
    semdt_cas: CAS,
    monkeypatch: pytest.MonkeyPatch,
    node: Node,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    aircraft_pose_comparison: AircraftPoseComparison,
) -> None:
    """
    The launchable engine retains typed bodies and geometry across its scenario.
    """
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    clock = ControlledClock()
    engine = AnalysisEngine(frames=aircraft_frames, clock=clock)
    engine.scenario.timing.sample_period_nanoseconds = (
        VisibilityBoundary.REAPPEARANCE - VisibilityBoundary.TOTAL_ABSENCE_START
    )
    pipeline = engine.implementation()
    pipeline.setup()
    sequence_start = clock.timestamp_nanoseconds
    retained_parts: dict[AircraftModel, AircraftIdentity] = {}
    previous_snapshots: dict[AircraftModel, BeliefSnapshot] = {}
    timing = engine.scenario.timing

    for offset in range(
        0, timing.duration_nanoseconds + 1, timing.sample_period_nanoseconds
    ):
        clock.timestamp_nanoseconds = sequence_start + offset
        AircraftBeliefCapture(pipeline).complete()

        assert pipeline.status is Status.SUCCESS
        beliefs = world.get_object_belief_states()
        assert len(beliefs) == len(AircraftModel)
        assert len(
            world.world_instance().get_semantic_annotations_by_type(AircraftPart)
        ) == len(AircraftModel)
        visible_parts = {
            part.model: hypothesis
            for part, hypothesis in zip(
                (part for part in engine.scenario.parts if part.is_visible(offset)),
                pipeline.cas.annotations,
                strict=True,
            )
        }
        if offset == VisibilityBoundary.TOTAL_ABSENCE_START:
            assert visible_parts == {}
        for model in AircraftModel:
            (annotation,) = world.world_instance().get_semantic_annotations_by_type(
                model.value.semantic_type
            )
            belief = world.get_object_belief_state(annotation.root.id)
            if offset == 0:
                retained_parts[model] = AircraftIdentity(
                    belief,
                    belief.body,
                    belief.uuid,
                    annotation,
                    list(belief.body.visual.shapes),
                    list(belief.body.collision.shapes),
                )
            original = retained_parts[model]
            assert belief is original.belief
            assert belief.body is original.body
            assert belief.uuid == original.identifier
            assert annotation is original.annotation
            assert annotation.class_label == model.value.label
            for collection, original_shapes in zip(
                (belief.body.visual, belief.body.collision),
                (original.visual_shapes, original.collision_shapes),
                strict=True,
            ):
                (shape,) = collection.shapes
                (original_shape,) = original_shapes
                assert shape is original_shape
            if model not in visible_parts:
                assert BeliefSnapshot.from_belief(belief) == previous_snapshots[model]
                continue
            hypothesis = visible_parts[model]
            assert belief.latest_hypothesis is hypothesis
            (camera_pose,) = pipeline.cas.filter_by_type(
                PoseAnnotation, hypothesis.annotations
            )
            camera_T_part = numpy.eye(4)
            camera_T_part[:3, :3] = Rotation.from_quat(camera_pose.rotation).as_matrix()
            camera_T_part[:3, 3] = camera_pose.translation
            world_T_part = (
                aircraft_frames.camera_to_world_transform.to_np() @ camera_T_part
            )
            numpy.testing.assert_allclose(
                belief.body.global_transform.to_np(),
                world_T_part,
                atol=aircraft_pose_comparison.absolute_tolerance,
            )
            previous_snapshots[model] = BeliefSnapshot.from_belief(belief)
