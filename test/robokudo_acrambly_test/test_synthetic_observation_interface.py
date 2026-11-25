"""
What RoboKudo's collection reader puts into the CAS from a synthetic observation
interface.
"""

from __future__ import annotations

# %% Imports

from dataclasses import dataclass

import numpy
import pytest
from py_trees.common import Status
from robokudo.annotators.collection_reader import CollectionReaderAnnotator
from robokudo.pipeline import Pipeline
from robokudo.types.annotation import Classification, PoseAnnotation
from robokudo.types.scene import ObjectHypothesis
from semantic_digital_twin.robots.robot_parts import Camera

from robokudo_acrambly.synthetic_observations.camera_interface import (
    SyntheticCameraConfig,
    SyntheticObservationInterface,
)
from robokudo_acrambly.synthetic_observations.parts import AircraftPart
from robokudo_acrambly.synthetic_observations.scenario import PlaneLayout, Scenario

from .dataset.clocks import SteppedClock


# %% Reader fixture


@dataclass(kw_only=True)
class SyntheticReaderPipeline:
    """
    A pipeline whose collection reader reads a scenario on a controlled clock.
    """

    scenario: Scenario
    """
    The scenario the reader reads.
    """

    reader: CollectionReaderAnnotator
    """
    The reader of the pipeline.
    """

    pipeline: Pipeline
    """
    The pipeline owning the CAS.
    """

    start: int
    """
    When the observation sequence starts, in nanoseconds since the Unix epoch.
    """

    def read_pass(self, *, pass_index: int) -> None:
        """
        Let the reader read every pass up to and including ``pass_index``.
        """
        for _ in range(pass_index + 1):
            assert self.reader.update() == Status.SUCCESS


@pytest.fixture
def synthetic_reader(clock: SteppedClock) -> SyntheticReaderPipeline:
    """
    A reader of the aircraft scenario whose sequence has started at the time of
    ``clock``.
    """
    scenario = Scenario(parts=PlaneLayout().arrange(labels=list(AircraftPart)))
    camera_config = SyntheticCameraConfig(scenario=scenario)
    camera_interface = SyntheticObservationInterface(
        camera_config=camera_config, clock=clock
    )
    camera_interface.has_new_data()
    reader = CollectionReaderAnnotator(
        CollectionReaderAnnotator.Descriptor(
            camera_config=camera_config, camera_interface=camera_interface
        )
    )
    return SyntheticReaderPipeline(
        scenario=scenario,
        reader=reader,
        pipeline=Pipeline(children=[reader]),
        start=clock.now,
    )


def object_hypotheses(*, pipeline: Pipeline) -> list[ObjectHypothesis]:
    """
    :return: The object hypotheses in the CAS of ``pipeline``.
    """
    return pipeline.cas.filter_annotations_by_type(ObjectHypothesis)


# %% Object hypotheses


@pytest.mark.parametrize("pass_index", [0, 37])
def test_each_part_gets_one_hypothesis_with_its_label_and_scripted_pose(
    synthetic_reader: SyntheticReaderPipeline, clock: SteppedClock, pass_index: int
) -> None:
    scenario = synthetic_reader.scenario
    elapsed = scenario.schedule.elapsed_at(pass_index=pass_index)
    clock.now = synthetic_reader.start + int(
        numpy.timedelta64(elapsed, "ns").astype(numpy.int64)
    )
    synthetic_reader.read_pass(pass_index=pass_index)

    hypotheses = object_hypotheses(pipeline=synthetic_reader.pipeline)
    assert [hypothesis.class_label for hypothesis in hypotheses] == [
        part.label for part in scenario.parts
    ]
    for hypothesis, part in zip(hypotheses, scenario.parts):
        classification, pose_annotation = hypothesis.annotations
        assert type(classification) is Classification
        assert classification.confidence == 1.0
        assert type(pose_annotation) is PoseAnnotation
        expected_pose = part.path.pose_at(elapsed=elapsed)
        assert pose_annotation.translation == pytest.approx(
            expected_pose.position.to_np()[:3].tolist()
        )
        assert pose_annotation.rotation == pytest.approx(
            expected_pose.quaternion.to_np().tolist()
        )


def test_hypotheses_name_the_interface_as_their_source(
    synthetic_reader: SyntheticReaderPipeline,
) -> None:
    synthetic_reader.read_pass(pass_index=0)
    for hypothesis in object_hypotheses(pipeline=synthetic_reader.pipeline):
        assert {annotation.source for annotation in hypothesis.annotations} == {
            SyntheticObservationInterface.__name__
        }


# %% Camera observation


def test_camera_observation_carries_the_scheduled_capture_time(
    synthetic_reader: SyntheticReaderPipeline, clock: SteppedClock
) -> None:
    clock.now = synthetic_reader.start + 60_000_000_000
    synthetic_reader.read_pass(pass_index=37)
    observation = synthetic_reader.pipeline.cas.camera_observation
    assert observation.timestamp_nanoseconds == synthetic_reader.start + 3_700_000_000


def test_camera_observation_has_the_scenario_camera_and_no_world_pose(
    synthetic_reader: SyntheticReaderPipeline,
) -> None:
    synthetic_reader.read_pass(pass_index=0)
    observation = synthetic_reader.pipeline.cas.camera_observation
    assert observation.world_T_camera is None
    assert observation.effective_camera_model is synthetic_reader.scenario.camera_model
    assert observation.camera.root.name.name == synthetic_reader.scenario.camera_frame


def test_reading_passes_adds_nothing_to_the_world(
    synthetic_reader: SyntheticReaderPipeline, clock: SteppedClock
) -> None:
    world = synthetic_reader.pipeline.world
    clock.now = synthetic_reader.start + 60_000_000_000
    synthetic_reader.read_pass(pass_index=2)
    assert world.bodies == []
    assert world.get_semantic_annotations_by_type(Camera) == []


# %% Waiting and the end of the sequence


def test_reader_waits_for_the_next_capture_time(
    synthetic_reader: SyntheticReaderPipeline,
) -> None:
    synthetic_reader.read_pass(pass_index=0)
    first_cas = synthetic_reader.pipeline.cas
    assert synthetic_reader.reader.update() == Status.RUNNING
    assert synthetic_reader.pipeline.cas is first_cas


def test_reader_waits_after_the_last_pass(
    synthetic_reader: SyntheticReaderPipeline, clock: SteppedClock
) -> None:
    clock.now = synthetic_reader.start + 60_000_000_000
    synthetic_reader.read_pass(
        pass_index=synthetic_reader.scenario.schedule.pass_count - 1
    )
    last_cas = synthetic_reader.pipeline.cas
    assert synthetic_reader.reader.update() == Status.RUNNING
    assert synthetic_reader.pipeline.cas is last_cas
