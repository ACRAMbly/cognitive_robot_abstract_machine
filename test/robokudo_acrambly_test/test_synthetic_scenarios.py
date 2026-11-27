"""
The parts of the synthetic scenarios and how they move over an observation sequence.
"""

from __future__ import annotations

# %% Imports

import math
from dataclasses import dataclass

import numpy
import pytest

from robokudo_acrambly.descriptors.analysis_engines import (
    synthetic_aircraft,
    synthetic_tetris,
)
from robokudo_acrambly.synthetic_observations.scenario import Scenario
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin_acrambly.semantic_annotations import (
    BottomWing,
    Chassis,
    CornerTetrisBlock,
    FrontWheel,
    LeftRearWheel,
    MotorGrill,
    Propeller,
    RearWing,
    RightRearWheel,
    StraightTetrisBlock,
    TopWing,
    UnderBody,
    UpperBody,
    Window,
)


# %% Motion limits


@dataclass(kw_only=True)
class MotionLimits:
    """
    The motion the specification allows a scenario part, so that an object matcher can
    follow each part from one pass to the next.
    """

    maximum_speed: float = 0.05
    """
    Fastest a part may move, in metres per second.
    """

    maximum_turn_rate: float = math.radians(15.0)
    """
    Fastest a part may turn, in radians per second.
    """

    minimum_centre_distance: float = 0.3
    """
    Closest two part centres may come, in metres.
    """


@pytest.fixture
def motion_limits() -> MotionLimits:
    """
    The limits of the specification.
    """
    return MotionLimits()


@dataclass(kw_only=True)
class ScenarioUnderTest:
    """
    A scenario of an analysis engine and the parts it has to contain.
    """

    scenario: Scenario
    """
    The scenario the analysis engine plays back.
    """

    expected_part_types: list[type[IsGraspable]]
    """
    The annotation classes of the parts the scenario has to contain, one part each, in
    the order the scenario arranges them.
    """


@pytest.fixture(
    params=[
        (
            synthetic_tetris.AnalysisEngine,
            [StraightTetrisBlock, CornerTetrisBlock],
        ),
        (
            synthetic_aircraft.AnalysisEngine,
            [
                BottomWing,
                Chassis,
                FrontWheel,
                MotorGrill,
                Propeller,
                LeftRearWheel,
                RightRearWheel,
                RearWing,
                TopWing,
                UnderBody,
                UpperBody,
                Window,
            ],
        ),
    ],
    ids=["tetris", "aircraft"],
)
def scenario_under_test(request: pytest.FixtureRequest) -> ScenarioUnderTest:
    """
    The scenario of each synthetic analysis engine.
    """
    engine_type, expected_part_types = request.param
    return ScenarioUnderTest(
        scenario=engine_type().scenario, expected_part_types=expected_part_types
    )


# %% Part positions and headings


@dataclass(kw_only=True)
class PartPlacement:
    """
    Where a part is and which way it faces at one pass, in the camera optical frame.
    """

    position: numpy.ndarray
    """
    The part's centre, in metres.
    """

    quaternion: numpy.ndarray
    """
    The part's orientation as x, y, z, w.
    """


def placements_at(*, scenario: Scenario, pass_index: int) -> list[PartPlacement]:
    """
    :return: The placement of every part of ``scenario`` at the pass ``pass_index``.
    """
    elapsed = scenario.schedule.elapsed_at(pass_index=pass_index)
    placements = []
    for part in scenario.parts:
        pose = part.path.pose_at(elapsed=elapsed)
        placements.append(
            PartPlacement(
                position=pose.position.to_np()[:3],
                quaternion=pose.quaternion.to_np(),
            )
        )
    return placements


def turned_angle(*, before: numpy.ndarray, after: numpy.ndarray) -> float:
    """
    :return: The angle, in radians, between the orientations ``before`` and ``after``.
    """
    alignment = min(1.0, abs(float(numpy.dot(before, after))))
    return 2.0 * math.acos(alignment)


# %% Scenario contents


def test_scenario_contains_one_part_per_annotation_class(
    scenario_under_test: ScenarioUnderTest,
) -> None:
    labels = [part.label for part in scenario_under_test.scenario.parts]
    assert labels == [
        part_type.__name__ for part_type in scenario_under_test.expected_part_types
    ]


def test_scenario_lasts_thirty_seconds_at_ten_passes_per_second(
    scenario_under_test: ScenarioUnderTest,
) -> None:
    schedule = scenario_under_test.scenario.schedule
    assert schedule.pass_count == 300
    assert schedule.elapsed_at(pass_index=299).total_seconds() == pytest.approx(29.9)


# %% Motion between passes


def test_parts_move_no_faster_than_the_limit(
    scenario_under_test: ScenarioUnderTest, motion_limits: MotionLimits
) -> None:
    scenario = scenario_under_test.scenario
    period = scenario.schedule.pass_period.total_seconds()
    previous = placements_at(scenario=scenario, pass_index=0)
    for pass_index in range(1, scenario.schedule.pass_count):
        current = placements_at(scenario=scenario, pass_index=pass_index)
        for before, after in zip(previous, current):
            distance = float(numpy.linalg.norm(after.position - before.position))
            assert distance / period <= motion_limits.maximum_speed
        previous = current


def test_parts_turn_no_faster_than_the_limit(
    scenario_under_test: ScenarioUnderTest, motion_limits: MotionLimits
) -> None:
    scenario = scenario_under_test.scenario
    period = scenario.schedule.pass_period.total_seconds()
    previous = placements_at(scenario=scenario, pass_index=0)
    for pass_index in range(1, scenario.schedule.pass_count):
        current = placements_at(scenario=scenario, pass_index=pass_index)
        for before, after in zip(previous, current):
            angle = turned_angle(before=before.quaternion, after=after.quaternion)
            assert angle / period <= motion_limits.maximum_turn_rate
        previous = current


def test_parts_turn_during_the_sequence(
    scenario_under_test: ScenarioUnderTest,
) -> None:
    scenario = scenario_under_test.scenario
    first = placements_at(scenario=scenario, pass_index=0)
    last = placements_at(scenario=scenario, pass_index=scenario.schedule.pass_count - 1)
    for before, after in zip(first, last):
        assert turned_angle(before=before.quaternion, after=after.quaternion) > 0.0


# %% Layout in view of the camera


def test_part_centres_keep_their_distance(
    scenario_under_test: ScenarioUnderTest, motion_limits: MotionLimits
) -> None:
    scenario = scenario_under_test.scenario
    for pass_index in range(scenario.schedule.pass_count):
        positions = [
            placement.position
            for placement in placements_at(scenario=scenario, pass_index=pass_index)
        ]
        for index, position in enumerate(positions):
            for other in positions[index + 1 :]:
                distance = float(numpy.linalg.norm(position - other))
                assert distance >= motion_limits.minimum_centre_distance


def test_part_centres_stay_inside_the_image(
    scenario_under_test: ScenarioUnderTest,
) -> None:
    scenario = scenario_under_test.scenario
    camera_model = scenario.camera_model
    resolution = camera_model.resolution
    for pass_index in range(scenario.schedule.pass_count):
        for placement in placements_at(scenario=scenario, pass_index=pass_index):
            assert placement.position[2] > 0.0
            pixel = camera_model.intrinsic_matrix @ placement.position
            column, row = pixel[:2] / pixel[2]
            assert 0.0 <= column < resolution.width
            assert 0.0 <= row < resolution.height
