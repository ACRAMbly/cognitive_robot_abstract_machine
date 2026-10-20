"""
Illustrative camera-relative observations of the supplied Tetris blocks.
"""

from math import sqrt

from robokudo.io.scripted_observations import (
    ObservationStep,
    ScriptedDetection,
    ScriptedObservation,
    ScriptedScenario,
    TetrisBlock,
)
from robokudo.types.tf import Pose

# %% demonstration data


def create_tetris_scenario() -> ScriptedScenario:
    """
    Create motion, absence, and reappearance scenes in the original CAD frames.
    """
    first = ScriptedDetection(
        TetrisBlock.FIRST,
        Pose(translation=[-0.30, -0.20, 0.90], rotation=[0.0, 0.0, 0.0, 1.0]),
    )
    second = ScriptedDetection(
        TetrisBlock.SECOND,
        Pose(translation=[0.0, -0.20, 0.90], rotation=[0.0, 0.0, 0.0, 1.0]),
    )
    translated_first = ScriptedDetection(
        TetrisBlock.FIRST,
        Pose(translation=[-0.25, -0.15, 1.0], rotation=[0.0, 0.0, 0.0, 1.0]),
    )
    translated_second = ScriptedDetection(
        TetrisBlock.SECOND,
        Pose(translation=[0.05, -0.15, 1.0], rotation=[0.0, 0.0, 0.0, 1.0]),
    )
    quarter_turn_component = sqrt(0.5)
    rotated_first = ScriptedDetection(
        TetrisBlock.FIRST,
        Pose(
            translation=[0.0, -0.15, 1.0],
            rotation=[0.0, 0.0, quarter_turn_component, quarter_turn_component],
        ),
    )
    rotated_second = ScriptedDetection(
        TetrisBlock.SECOND,
        Pose(
            translation=[0.30, -0.15, 1.0],
            rotation=[0.0, 0.0, quarter_turn_component, quarter_turn_component],
        ),
    )
    return ScriptedScenario(
        observations=(
            ScriptedObservation(ObservationStep.BOTH_VISIBLE, (first, second)),
            ScriptedObservation(
                ObservationStep.TRANSLATED, (translated_first, translated_second)
            ),
            ScriptedObservation(
                ObservationStep.ROTATED, (rotated_first, rotated_second)
            ),
            ScriptedObservation(ObservationStep.FIRST_ABSENT, (rotated_second,)),
            ScriptedObservation(ObservationStep.BOTH_ABSENT, ()),
            ScriptedObservation(
                ObservationStep.REAPPEARED, (rotated_first, rotated_second)
            ),
        )
    )
