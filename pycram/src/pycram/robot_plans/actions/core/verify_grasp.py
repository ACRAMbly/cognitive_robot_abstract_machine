from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms
from ....datastructures.partial_designator import PartialDesignator
from ....failures import ObjectNotGraspedError
from ....robot_plans.actions.base import ActionDescription
from ....view_manager import ViewManager

logger = logging.getLogger(__name__)


@dataclass
class VerifyGraspAction(ActionDescription):
    """
    Verifies that the gripper is holding an object of the expected width by
    reading the gripper joint position from the world state.

    This is a **postcondition-only** action: `execute()` is a no-op.
    The actual check is performed in `validate_postcondition`, which raises
    `ObjectNotGraspedError` if the joint position indicates an empty hand.

    Usage example::

        SequentialPlan(
            context,
            PickUpActionDescription(obj, arm=Arms.RIGHT, grasp_description=g),
            VerifyGraspActionDescription(arm=Arms.RIGHT, expected_object_width=0.05),
        ).perform()
    """

    arm: Arms
    """Arm whose gripper to verify."""

    expected_object_width: float
    """Expected object width at the gripper (m). Used to validate joint position."""

    tolerance: float = 0.01
    """Acceptable deviation from expected position (m)."""

    def execute(self) -> None:
        # No motion required — verification is purely state-based.
        pass

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        """
        Reads the current gripper joint state from the world model and checks
        whether the position is consistent with holding an object of
        `expected_object_width`.  Raises `ObjectNotGraspedError` on failure.
        """
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot_view)

        # Get all gripper DOFs
        gripper_dofs = end_effector.gripper.get_all_dofs() if hasattr(end_effector, "gripper") else []

        if not gripper_dofs:
            logger.warning(
                f"VerifyGraspAction: no gripper DOFs found for arm {self.arm}. Skipping check."
            )
            return

        # Read current positions from world
        positions = [self.world[dof.id].position for dof in gripper_dofs]
        avg_pos = sum(positions) / len(positions)

        # Check against expected width ± tolerance
        if abs(avg_pos - self.expected_object_width) > self.tolerance:
            # Check for fully-closed (empty hand) case
            fully_closed = end_effector.get_joint_state_by_type(
                __import__(
                    "semantic_digital_twin.datastructures.definitions",
                    fromlist=["GripperState"],
                ).GripperState.CLOSE
            )
            avg_close = sum(fully_closed.target_values) / len(fully_closed.target_values)
            logger.warning(
                f"VerifyGraspAction: gripper at {avg_pos:.4f}, expected {self.expected_object_width:.4f} "
                f"(tolerance {self.tolerance}). "
                f"{'Empty hand (fully closed).' if abs(avg_pos - avg_close) < self.tolerance else 'Wrong position.'}"
            )
            raise ObjectNotGraspedError(None, self.robot_view, self.arm, None)

    @classmethod
    def description(
        cls,
        arm: Union[Iterable[Arms], Arms],
        expected_object_width: Union[Iterable[float], float],
        tolerance: Union[Iterable[float], float] = 0.01,
    ) -> PartialDesignator["VerifyGraspAction"]:
        return PartialDesignator[VerifyGraspAction](
            VerifyGraspAction,
            arm=arm,
            expected_object_width=expected_object_width,
            tolerance=tolerance,
        )


VerifyGraspActionDescription = VerifyGraspAction.description
