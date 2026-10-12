from enum import Enum, IntEnum, auto


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


class MeshArrayAxis(IntEnum):
    """
    An axis of a frame, as the column that holds the coordinates along it in an array of
    points, such as a mesh's vertices.
    """

    X = 0
    Y = 1
    Z = 2
