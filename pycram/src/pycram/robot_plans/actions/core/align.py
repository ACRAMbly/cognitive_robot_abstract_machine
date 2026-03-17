from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.manipulation import AlignMotion


@dataclass
class AlignAction(ActionDescription):
    """
    Performs a precise, slow 6-DOF alignment of the TCP to a target pose.

    This is typically used as a **precursor** to `InsertionAction` or `SnapAction`
    when sub-millimetre accuracy is needed before the main motion begins.
    The reduced velocity (default 2 cm/s, 0.05 rad/s) allows the robot's
    impedance controller to settle accurately.
    """

    target: PoseStamped
    """Target 6D pose for the TCP after alignment."""

    arm: Arms
    """Arm to align."""

    reference_linear_velocity: float = 0.02
    """Linear approach speed (m/s). Keep ≤ 0.05 for fine alignment."""

    reference_angular_velocity: float = 0.05
    """Angular speed (rad/s)."""

    threshold: float = 0.001
    """Convergence threshold in metres (1 mm)."""

    def execute(self) -> None:
        SequentialPlan(
            self.context,
            AlignMotion(
                target=self.target,
                arm=self.arm,
                reference_linear_velocity=self.reference_linear_velocity,
                reference_angular_velocity=self.reference_angular_velocity,
                threshold=self.threshold,
            ),
        ).perform()

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        pass

    @classmethod
    def description(
        cls,
        target: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        reference_linear_velocity: Union[Iterable[float], float] = 0.02,
        reference_angular_velocity: Union[Iterable[float], float] = 0.05,
        threshold: Union[Iterable[float], float] = 0.001,
    ) -> PartialDesignator["AlignAction"]:
        return PartialDesignator[AlignAction](
            AlignAction,
            target=target,
            arm=arm,
            reference_linear_velocity=reference_linear_velocity,
            reference_angular_velocity=reference_angular_velocity,
            threshold=threshold,
        )


AlignActionDescription = AlignAction.description
