from __future__ import annotations

from enum import Enum, auto


class JointStateType(Enum): ...


class GripperState(JointStateType):
    OPEN = auto()
    CLOSE = auto()
    MEDIUM = auto()


class TorsoState(JointStateType):
    HIGH = auto()
    MID = auto()
    LOW = auto()


class StaticJointState(JointStateType):
    PARK = auto()


class AxisIdentifier(Enum):
    """
    An axis of a frame, as the unit vector along it.
    """

    X = (1, 0, 0)
    Y = (0, 1, 0)
    Z = (0, 0, 1)
    Undefined = (0, 0, 0)

    @classmethod
    def from_tuple(cls, axis_tuple: tuple[int, int, int]) -> AxisIdentifier | None:
        """
        :param axis_tuple: A unit vector along an axis.
        :return: The axis the vector points along, or ``None`` if it is none of them.
        """
        return next((axis for axis in cls if axis.value == axis_tuple), None)
