"""
Semantic annotations of the objects of the ACRAMbly assembly tasks: the aircraft parts
and the tetris blocks.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass

from semantic_digital_twin.semantic_annotations.mixins import IsGraspable

# %% aircraft parts


@dataclass(eq=False, kw_only=True)
class AircraftPart(IsGraspable, ABC):
    """
    One of the distinct pieces the aircraft is assembled from, each with its own shape.
    """


@dataclass(eq=False, kw_only=True)
class BottomWing(AircraftPart):
    """
    The bottom wing of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class Chassis(AircraftPart):
    """
    The chassis of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class FrontWheel(AircraftPart):
    """
    The front wheel of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class MotorGrill(AircraftPart):
    """
    The motor grill of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class Propeller(AircraftPart):
    """
    The propeller of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class LeftRearWheel(AircraftPart):
    """
    The left rear wheel of the aircraft, whose mesh is named ``RearWheelLeft``.
    """


@dataclass(eq=False, kw_only=True)
class RightRearWheel(AircraftPart):
    """
    The right rear wheel of the aircraft, whose mesh is named ``RearWheelRight``.
    """


@dataclass(eq=False, kw_only=True)
class RearWing(AircraftPart):
    """
    The rear wing of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class TopWing(AircraftPart):
    """
    The top wing of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class UnderBody(AircraftPart):
    """
    The under body of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class UpperBody(AircraftPart):
    """
    The upper body of the aircraft.
    """


@dataclass(eq=False, kw_only=True)
class Window(AircraftPart):
    """
    The window of the aircraft.
    """


# %% tetris blocks


@dataclass(eq=False, kw_only=True)
class TetrisBlock(IsGraspable, ABC):
    """
    One of the toy blocks built from unit cubes, in a straight or a three-dimensional
    shape.
    """


@dataclass(eq=False, kw_only=True)
class StraightTetrisBlock(TetrisBlock):
    """
    A tetris block that is a bar of four cubes, whose mesh is named
    ``child_cube_0_colored``.
    """


@dataclass(eq=False, kw_only=True)
class CornerTetrisBlock(TetrisBlock):
    """
    A tetris block that is a row of three cubes with one cube sideways and one upwards
    from the same end, whose mesh is named ``child_cube_2_colored``.
    """
