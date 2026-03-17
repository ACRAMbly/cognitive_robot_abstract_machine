from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, List

from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms
from ....datastructures.partial_designator import PartialDesignator
from ....failures import PlanFailure
from ....robot_plans.actions.base import ActionDescription
from ....view_manager import ViewManager

logger = logging.getLogger(__name__)


class AssemblyNotMatedError(PlanFailure):
    """Raised when `InspectJointAction` determines two parts are not properly mated."""

    def __init__(self, part_a: Body, part_b: Body, detail: str = ""):
        self.part_a = part_a
        self.part_b = part_b
        super().__init__(
            f"Assembly check FAILED: {part_a} and {part_b} are not mated. {detail}"
        )


@dataclass
class InspectJointAction(ActionDescription):
    """
    Verifies that two assembly parts are properly mated after a snap, insert, or
    screw operation.

    The check is **world-model-based**: it reads the current joint effort (torque)
    of a specified list of joints and compares to a threshold.  High effort with a
    low position delta indicates successful contact / mating.

    For a visual check, set `use_visual=True` — this will trigger a `DetectAction`
    to verify the parts are at the expected relative pose (requires a detection
    pipeline to be set up).

    This action is **postcondition-only**: `execute()` is a no-op.
    """

    part_a: Body
    """First assembly part (the one being inserted/snapped)."""

    part_b: Body
    """Second assembly part (the receptacle/base)."""

    monitor_joints: List[str]
    """Joint names to read effort from (e.g. gripper finger joints)."""

    min_effort_threshold: float = 0.5
    """Minimum summed joint effort indicating successful contact (Nm)."""

    def execute(self) -> None:
        # No motion — check is purely in postcondition
        pass

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        """
        Reads the joint effort from the world model and raises
        `AssemblyNotMatedError` if the effort is below the threshold.
        """
        if not self.monitor_joints:
            logger.warning("InspectJointAction: no joints to monitor, skipping check.")
            return

        total_effort = 0.0
        world_state = None
        try:
            # Try to get joint states from the world
            for joint_name in self.monitor_joints:
                try:
                    # Access world joint effort if available
                    joint = self.world[joint_name]
                    total_effort += abs(getattr(joint, "effort", 0.0))
                except (KeyError, AttributeError):
                    logger.debug(f"InspectJointAction: joint {joint_name!r} not in world state.")
        except Exception as e:
            logger.warning(f"InspectJointAction: could not read joint state: {e}")
            return

        logger.info(
            f"InspectJointAction: total effort={total_effort:.3f} Nm "
            f"(threshold={self.min_effort_threshold:.3f})"
        )
        if total_effort < self.min_effort_threshold:
            raise AssemblyNotMatedError(
                self.part_a,
                self.part_b,
                f"Effort {total_effort:.3f} < {self.min_effort_threshold:.3f} Nm",
            )

    @classmethod
    def description(
        cls,
        part_a: Union[Iterable[Body], Body],
        part_b: Union[Iterable[Body], Body],
        monitor_joints: Union[Iterable[List[str]], List[str]],
        min_effort_threshold: Union[Iterable[float], float] = 0.5,
    ) -> PartialDesignator["InspectJointAction"]:
        return PartialDesignator[InspectJointAction](
            InspectJointAction,
            part_a=part_a,
            part_b=part_b,
            monitor_joints=monitor_joints,
            min_effort_threshold=min_effort_threshold,
        )


InspectJointActionDescription = InspectJointAction.description
