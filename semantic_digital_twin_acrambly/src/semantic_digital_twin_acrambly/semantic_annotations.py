"""
Semantic annotations of the objects of the ACRAMbly assembly tasks: the aircraft, its
parts and the tetris blocks.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field

from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    IsGraspable,
    PartWholeRelationship,
)
from semantic_digital_twin.semantic_annotations.part_whole import (
    IsPartWholeRelationship,
)

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


# %% aircraft


@dataclass(eq=False, kw_only=True)
class Aircraft(HasRootBody, PartWholeRelationship):
    """
    The toy aircraft assembled from the aircraft parts, rooted at the body of its
    :class:`Chassis`; each other part fills its field once it is mounted.
    """

    bottom_wing: BottomWing | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The bottom wing, once it is mounted.
    """

    front_wheel: FrontWheel | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The front wheel, once it is mounted.
    """

    motor_grill: MotorGrill | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The motor grill, once it is mounted.
    """

    propeller: Propeller | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The propeller, once it is mounted.
    """

    left_rear_wheel: LeftRearWheel | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The left rear wheel, once it is mounted.
    """

    right_rear_wheel: RightRearWheel | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The right rear wheel, once it is mounted.
    """

    rear_wing: RearWing | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The rear wing, once it is mounted.
    """

    top_wing: TopWing | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The top wing, once it is mounted.
    """

    under_body: UnderBody | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The under body, once it is mounted.
    """

    upper_body: UpperBody | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The upper body, once it is mounted.
    """

    window: Window | None = field(
        default=None, metadata=IsPartWholeRelationship().as_dict()
    )
    """
    The window, once it is mounted.
    """
