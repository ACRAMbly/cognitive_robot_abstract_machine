# %% Imports

from __future__ import annotations

from dataclasses import dataclass

from semantic_digital_twin.semantic_annotations.mixins import HasRootBody, IsPerceivable

# %% Aircraft parts


@dataclass(eq=False)
class AircraftPart(HasRootBody, IsPerceivable):
    """
    A physical aircraft component with a body and accepted perception label.
    """


@dataclass(eq=False)
class BottomWing(AircraftPart):
    """
    The aircraft's bottom wing component.
    """


@dataclass(eq=False)
class Chassis(AircraftPart):
    """
    The aircraft's chassis component.
    """


@dataclass(eq=False)
class FrontWheel(AircraftPart):
    """
    The aircraft's front wheel component.
    """


@dataclass(eq=False)
class MotorGrill(AircraftPart):
    """
    The aircraft's motor grill component.
    """


@dataclass(eq=False)
class Propeller(AircraftPart):
    """
    The aircraft's propeller component.
    """


@dataclass(eq=False)
class RearWheelLeft(AircraftPart):
    """
    The aircraft's left rear wheel component.
    """


@dataclass(eq=False)
class RearWheelRight(AircraftPart):
    """
    The aircraft's right rear wheel component.
    """


@dataclass(eq=False)
class RearWing(AircraftPart):
    """
    The aircraft's rear wing component.
    """


@dataclass(eq=False)
class TopWing(AircraftPart):
    """
    The aircraft's top wing component.
    """


@dataclass(eq=False)
class UnderBody(AircraftPart):
    """
    The aircraft's under body component.
    """


@dataclass(eq=False)
class UpperBody(AircraftPart):
    """
    The aircraft's upper body component.
    """


@dataclass(eq=False)
class Window(AircraftPart):
    """
    The aircraft's window component.
    """
