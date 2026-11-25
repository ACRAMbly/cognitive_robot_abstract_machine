# %% Imports

from __future__ import annotations

import pytest
from typing_extensions import TYPE_CHECKING

from robokudo_acrambly.aircraft.models import AircraftModel
from robokudo_acrambly.aircraft.types import AircraftPart
from robokudo_acrambly.aircraft_observations.configuration import AircraftEnvironment
from robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft import (
    AnalysisEngine,
)
from robokudo_acrambly.shared_world.world_source import ProvidedWorldSource

from .dataset.aircraft_beliefs import AircraftBeliefCapture
from .dataset.robot_world import RobotWorldFrame, create_robot_world
from .test_aircraft_belief_creation import aircraft_catalog, semdt_cas
from .test_scripted_aircraft import aircraft_frames
from .test_scripted_tetris import ControlledClock

if TYPE_CHECKING:
    from robokudo.annotators.outputs import AnnotatorOutputPerPipelineMap
    from robokudo.cas import CAS
    from robokudo_acrambly.aircraft.models import AircraftModelCatalog
    from robokudo_acrambly.aircraft_observations.scenario import AircraftFrames


# %% World frame from the robot world's root


def test_accepted_parts_are_placed_directly_under_the_root_of_the_supplied_world(
    aircraft_frames: AircraftFrames,
    aircraft_catalog: AircraftModelCatalog,
    semdt_cas: CAS,
    annotator_output_buffer: AnnotatorOutputPerPipelineMap,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    A root not named ``map`` becomes the world frame, and no frame body is added.
    """
    monkeypatch.setenv(
        AircraftEnvironment.MESH_DIRECTORY, str(aircraft_catalog.mesh_directory)
    )
    robot_world = create_robot_world(root_frame=RobotWorldFrame.TRACY_MAP)
    engine = AnalysisEngine(
        frames=aircraft_frames,
        clock=ControlledClock(),
        world_source=ProvidedWorldSource(world=robot_world),
    )
    engine.scenario.timing.duration_nanoseconds = 0
    pipeline = engine.implementation()
    pipeline.setup()

    AircraftBeliefCapture(pipeline).complete()

    parts = robot_world.get_semantic_annotations_by_type(AircraftPart)
    assert len(parts) == len(AircraftModel)
    assert set(robot_world.bodies) == {robot_world.root} | {part.root for part in parts}
    assert all(part.root.parent_connection.parent is robot_world.root for part in parts)
    assert pipeline.cas.world_frame == RobotWorldFrame.TRACY_MAP
