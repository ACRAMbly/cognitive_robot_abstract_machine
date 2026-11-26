"""
The labels of the parts the synthetic scenarios contain.
"""

from __future__ import annotations

# %% Part labels

from enum import StrEnum


class TetrisBlock(StrEnum):
    """
    The tetris blocks, labelled with the name of their mesh file.
    """

    CHILD_CUBE_0 = "child_cube_0_colored"
    CHILD_CUBE_2 = "child_cube_2_colored"


class AircraftPart(StrEnum):
    """
    The parts of the toy aircraft, labelled with the name of their mesh file.
    """

    BOTTOM_WING = "BottomWing"
    CHASSIS = "Chassis"
    FRONT_WHEEL = "FrontWheel"
    MOTOR_GRILL = "MotorGrill"
    PROPELLER = "Propeller"
    REAR_WHEEL_LEFT = "RearWheelLeft"
    REAR_WHEEL_RIGHT = "RearWheelRight"
    REAR_WING = "RearWing"
    TOP_WING = "TopWing"
    UNDER_BODY = "UnderBody"
    UPPER_BODY = "UpperBody"
    WINDOW = "Window"
