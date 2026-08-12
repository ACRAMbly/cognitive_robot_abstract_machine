"""
Error Recovery Module for Tracy
Recovery strategies:
1. Gripper failures -> retry with alternative grasps from GraspClassifier
2. Manipulation failures -> jiggle motion + park + retry
3. Placement failures -> try nearby alternative positions
4. Generic fallback -> park arms safely
"""

from __future__ import annotations

import time
import random
import logging
from typing import Dict, Any, Callable, Type, Optional, List

import numpy as np

from semantic_digital_twin.world import World as SemanticWorld
from semantic_digital_twin.world_description.world_entity import Body

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.grasp import GraspDescription
from pycram.failures import (
    PlanFailure,
    LowLevelFailure,
    ManipulationGoalNotReached,
    ManipulationPoseUnreachable,
    GripperGoalNotReached,
    GripperClosedCompletely,
    GripperIsNotOpen,
    ObjectNotPlacedAtTargetLocation,
    ObjectStillInContact,
    IKError,
    ConfigurationNotReached,
)

logger = logging.getLogger(__name__)


def loginfo(msg):
    logger.info(msg)
    print(f"[RECOVERY-INFO] {msg}")


def logwarn(msg):
    logger.warning(msg)
    print(f"[RECOVERY-WARN] {msg}")


# CUSTOM FAILURE TYPES

class GraspUnfeasibleFailure(LowLevelFailure):
    """Raised when no feasible grasp can be found."""
    pass


class PlacementFailedError(LowLevelFailure):
    """Raised when PlaceAction fails at all attempted locations."""
    pass


class RecoveryExhaustedError(PlanFailure):
    """Raised when all recovery strategies have been exhausted."""
    pass


# PLAN GUARDIAN

class PlanGuardian:
    """
    Error handler that diagnoses manipulation failures and
    attempts recovery using contextual strategies.

    Features:
    - Retry logic with configurable max attempts per action
    - Alternative grasp selection via GraspClassifier
    - Jiggle motion to escape IK/motion planning local minima
    - Alternative placement positions when target is occupied/unreachable
    - Error logging and statistics for debugging
    """

    def __init__(
        self,
        world: SemanticWorld,
        robot_view,
        grasp_classifier=None,
        max_retries: int = 2,
    ):
        self.world = world
        self.robot_view = robot_view
        self.grasp_classifier = grasp_classifier
        self.max_retries = max_retries

        # Track retry attempts per action instance
        self.retry_counts: Dict[str, int] = {}

        # Error log for debugging and statistics
        self.error_log: List[Dict] = []

        # Map failure types to recovery strategies
        self.recovery_strategies: Dict[Type[Exception], Callable] = {
            GripperGoalNotReached: self._recover_gripper_failure,
            GripperClosedCompletely: self._recover_gripper_failure,
            GripperIsNotOpen: self._recover_gripper_failure,
            ManipulationGoalNotReached: self._recover_manipulation_failure,
            ManipulationPoseUnreachable: self._recover_unreachable,
            IKError: self._recover_manipulation_failure,
            ConfigurationNotReached: self._recover_manipulation_failure,
            GraspUnfeasibleFailure: self._recover_unreachable,
            ObjectNotPlacedAtTargetLocation: self._recover_placement_failure,
            ObjectStillInContact: self._recover_placement_failure,
            PlacementFailedError: self._recover_placement_failure,
        }


    # PUBLIC API

    def handle_error(
        self, error: Exception, failed_action: Any, context: Dict
    ) -> None:
        """
        Main entry point for error handling.

        Args:
            error: The caught exception.
            failed_action: The action instance that failed.
            context: Dict with keys like 'object_designator', 'arm',
                     'target_location', 'grasp_description', etc.
        """
        action_name = type(failed_action).__name__ if failed_action else "<unknown>"
        action_id = str(id(failed_action)) if failed_action else "no_action"

        logwarn(
            f"PlanGuardian caught: {type(error).__name__} "
            f"during action: {action_name}"
        )
        self._log_error(error, failed_action, context)

        # Step 1: Try simple retry for transient errors
        if self._should_retry(action_id, error):
            try:
                self._simple_retry(failed_action, action_id)
                loginfo("Simple retry succeeded!")
                self._reset_retry_count(action_id)
                return
            except Exception as retry_error:
                logwarn(f"Retry attempt failed: {retry_error}")

        # Step 2: Use recovery strategy based on error type
        strategy = self._find_strategy(error)

        try:
            strategy(error, failed_action, context)
            loginfo("Recovery succeeded.")
            self._reset_retry_count(action_id)
        except Exception as recovery_error:
            logwarn(f"Recovery failed: {recovery_error}")
            self._reset_retry_count(action_id)
            raise RecoveryExhaustedError(
                f"Could not recover from {type(error).__name__}: {recovery_error}"
            ) from error

    def get_error_statistics(self) -> Dict[str, Any]:
        """Return statistics about errors encountered."""
        if not self.error_log:
            return {"total_errors": 0}

        error_types: Dict[str, int] = {}
        for entry in self.error_log:
            etype = entry["error_type"]
            error_types[etype] = error_types.get(etype, 0) + 1

        return {
            "total_errors": len(self.error_log),
            "error_types": error_types,
            "first_error": self.error_log[0]["timestamp"],
            "last_error": self.error_log[-1]["timestamp"],
        }

    def print_error_summary(self) -> None:
        """Print a human-readable summary of all errors."""
        stats = self.get_error_statistics()
        print("---ERROR RECOVERY SUMMARY---")
        print(f"Total errors: {stats['total_errors']}")
        if stats["total_errors"] > 0:
            print("Error types:")
            for etype, count in stats["error_types"].items():
                print(f"  {etype}: {count}")


    # RETRY LOGIC

    def _should_retry(self, action_id: str, error: Exception) -> bool:
        """Check if a simple retry should be attempted."""
        current = self.retry_counts.get(action_id, 0)

        # Only retry transient errors
        retryable = (
            GripperGoalNotReached,
            ManipulationGoalNotReached,
            ConfigurationNotReached,
            IKError,
        )

        return isinstance(error, retryable) and current < self.max_retries

    def _simple_retry(self, action: Any, action_id: str) -> None:
        """Retry the same action without modifications."""
        if not action or not hasattr(action, "perform"):
            raise ValueError("Action cannot be retried - no perform method")

        self.retry_counts[action_id] = self.retry_counts.get(action_id, 0) + 1
        retry_num = self.retry_counts[action_id]

        loginfo(f"Simple retry #{retry_num}/{self.max_retries}...")
        time.sleep(0.5)
        action.perform()

    def _reset_retry_count(self, action_id: str) -> None:
        """Reset the retry counter after success or final failure."""
        self.retry_counts.pop(action_id, None)


    # RECOVERY STRATEGIES

    def _find_strategy(self, error: Exception) -> Callable:
        """Find the best recovery strategy for an error, walking the MRO."""
        # Exact match first
        if type(error) in self.recovery_strategies:
            return self.recovery_strategies[type(error)]

        # Walk the inheritance chain
        for error_type in type(error).__mro__:
            if error_type in self.recovery_strategies:
                return self.recovery_strategies[error_type]

        return self._generic_recovery

    def _recover_gripper_failure(
        self, error: Exception, action: Any, context: Dict
    ) -> None:
        """
        Recovery for gripper failures (object not grasped, gripper stuck).

        Strategy:
        1. Park arms to reset posture
        2. If GraspClassifier available, try alternative grasps
        3. If no classifier, retry after parking
        """
        loginfo("Recovering from gripper failure...")

        from pycram.robot_plans.actions.core.robot_body import ParkArmsAction

        # Park arms first
        try:
            ParkArmsAction.description([Arms.BOTH]).perform()
        except Exception:
            pass

        time.sleep(0.5)

        obj = context.get("object_designator")
        arm = context.get("arm", Arms.LEFT)

        if self.grasp_classifier and obj:
            loginfo("Trying alternative grasps from classifier...")
            all_grasps = self.grasp_classifier.get_all_grasps_sorted()

            # Skip the first grasp (that's the one that failed)
            alternatives = all_grasps[1:6] if len(all_grasps) > 1 else []

            for i, grasp_info in enumerate(alternatives):
                loginfo(
                    f"Trying alternative grasp #{i + 1}: "
                    f"ID={grasp_info['id']}, "
                    f"Approach={grasp_info['approach_direction'].name}"
                )
                try:
                    if action and hasattr(action, "perform"):
                        if hasattr(action, "grasp_description"):
                            from pycram.view_manager import ViewManager

                            arm_view = ViewManager.get_arm_view(arm, self.robot_view)
                            action.grasp_description = GraspDescription(
                                approach_direction=grasp_info["approach_direction"],
                                vertical_alignment=grasp_info["vertical_alignment"],
                                manipulator=arm_view.manipulator,
                                rotate_gripper=False,
                            )
                        action.perform()
                        loginfo(f"Alternative grasp #{i + 1} succeeded!")
                        return
                except Exception as e:
                    logwarn(f"Alternative grasp #{i + 1} failed: {e}")

            raise PlanFailure("All alternative grasps exhausted.")
        else:
            # No classifier - just retry after parking
            if action and hasattr(action, "perform"):
                loginfo("Re-attempting action after parking...")
                action.perform()
            else:
                raise PlanFailure("Cannot recover: no action to retry.")

    def _recover_manipulation_failure(
        self, error: Exception, action: Any, context: Dict
    ) -> None:
        """
        Recovery for manipulation/IK failures.

        Strategy:
        1. Jiggle motion
        2. Park arms
        3. Retry the original action
        """
        loginfo("Recovering from manipulation failure...")

        arm = context.get("arm", Arms.LEFT)

        # Jiggle motion to escape local minima
        self._perform_jiggle_motion(arm)

        # Park arms
        from pycram.robot_plans.actions.core.robot_body import ParkArmsAction

        try:
            ParkArmsAction.description([Arms.BOTH]).perform()
        except Exception:
            pass

        time.sleep(0.5)

        # Retry
        if action and hasattr(action, "perform"):
            loginfo("Re-attempting action after jiggle + park...")
            action.perform()
        else:
            raise PlanFailure("Cannot recover: no action to retry.")

    def _recover_placement_failure(
        self, error: Exception, action: Any, context: Dict
    ) -> None:
        """
        Recovery for placement failures.

        Strategy:
        1. Try nearby alternative positions (offsets of 5cm)
        2. If all fail, try a safe discard position
        """
        loginfo("Recovering from placement failure...")

        target = context.get("target_location")
        obj = context.get("object_designator")
        arm = context.get("arm", Arms.LEFT)

        if not target or not obj:
            raise PlanFailure("Cannot recover: missing target_location or object.")

        from pycram.robot_plans.actions.core.placing import PlaceAction

        # Try nearby offsets
        offsets = [
            (0.05, 0, 0),
            (-0.05, 0, 0),
            (0, 0.05, 0),
            (0, -0.05, 0),
            (0.05, 0.05, 0),
            (-0.05, -0.05, 0),
        ]

        root = self.world.root

        for i, (dx, dy, dz) in enumerate(offsets):
            loginfo(f"Trying alternative placement #{i + 1}...")
            try:
                alt_pos = [
                    target.position.x + dx,
                    target.position.y + dy,
                    target.position.z + dz,
                ]
                alt_pose = PoseStamped.from_list(frame=root, position=alt_pos)

                PlaceAction.description(
                    object_designator=obj,
                    target_location=alt_pose,
                    arm=arm,
                ).perform()

                loginfo(f"Alternative placement #{i + 1} succeeded!")
                return

            except Exception as e:
                logwarn(f"Alternative placement #{i + 1} failed: {e}")

        # Last resort: discard zone
        logwarn("All alternative placements failed. Trying discard zone...")
        self._place_at_discard_zone(obj, arm)

    def _recover_unreachable(
        self, error: Exception, action: Any, context: Dict
    ) -> None:
        """
        Recovery for unreachable poses / unfeasible grasps.
        Parks arms and reports - the object truly cannot be reached.
        """
        logwarn("Object/pose is unreachable. Parking arms and aborting.")

        from pycram.robot_plans.actions.core.robot_body import ParkArmsAction

        try:
            ParkArmsAction.description([Arms.BOTH]).perform()
        except Exception:
            pass

        obj = context.get("object_designator")
        obj_name = getattr(obj, "name", str(obj)) if obj else "<unknown>"
        raise PlanFailure(
            f"Object '{obj_name}' is unreachable from the current robot position."
        )

    def _generic_recovery(
        self, error: Exception, action: Any, context: Dict
    ) -> None:
        """Fallback recovery: park arms and raise failure."""
        logwarn(
            f"No specific strategy for {type(error).__name__}. "
            f"Parking arms and aborting."
        )

        from pycram.robot_plans.actions.core.robot_body import ParkArmsAction

        try:
            ParkArmsAction.description([Arms.BOTH]).perform()
        except Exception:
            pass

        raise PlanFailure(
            f"Generic recovery triggered for {type(error).__name__}: {error}"
        )


    # HELPER METHODS

    def _perform_jiggle_motion(self, arm: Arms) -> None:
        """
        Perform small random TCP movements to escape IK/motion planning
        local minima. Moves the arm tip by 2-5cm in safe directions.
        """
        from pycram.robot_plans.motions.gripper import MoveTCPMotion
        from pycram.view_manager import ViewManager

        try:
            arm_view = ViewManager.get_arm_view(arm, self.robot_view)
            root = self.world.root

            # Get current tip position via FK
            tip_fk = self.world.compute_forward_kinematics(root, arm_view.tip)
            if hasattr(tip_fk, "translation"):
                current_pos = list(tip_fk.translation[:3])
            else:
                current_pos = list(np.array(tip_fk)[:3, 3])

            # Small random offset biased upward for safety
            jiggle_pos = [
                current_pos[0] + random.uniform(-0.03, 0.03),
                current_pos[1] + random.uniform(-0.03, 0.03),
                current_pos[2] + random.uniform(0.02, 0.05),
            ]

            jiggle_pose = PoseStamped.from_list(frame=root, position=jiggle_pos)

            loginfo(f"Jiggling {arm.name} arm by small offset...")
            MoveTCPMotion(target=jiggle_pose, arm=arm).perform()
            time.sleep(0.3)

        except Exception as e:
            logwarn(f"Jiggle motion failed (non-critical): {e}")

    def _place_at_discard_zone(self, obj: Body, arm: Arms) -> None:
        """
        Place object at a safe discard zone to free the gripper.
        """
        from pycram.robot_plans.actions.core.placing import PlaceAction
        from semantic_digital_twin.datastructures.definitions import GripperState
        from pycram.robot_plans.motions.gripper import MoveGripperMotion

        root = self.world.root
        discard_pose = PoseStamped.from_list(
            frame=root, position=[0.3, 0.0, 1.0]
        )

        loginfo(f"Placing '{obj.name}' at discard zone...")

        try:
            PlaceAction.description(
                object_designator=obj,
                target_location=discard_pose,
                arm=arm,
            ).perform()
            loginfo("Object placed at discard zone. Gripper freed.")
        except Exception:
            logwarn("Discard placement failed. Force-opening gripper...")
            try:
                MoveGripperMotion(GripperState.OPEN, arm).perform()
            except Exception:
                pass

    def _log_error(self, error: Exception, action: Any, context: Dict) -> None:
        """Log an error for debugging and statistics."""
        self.error_log.append(
            {
                "timestamp": time.time(),
                "error_type": type(error).__name__,
                "error_message": str(error),
                "action": type(action).__name__ if action else "<unknown>",
                "context": {k: str(v) for k, v in context.items()},
            }
        )



# DECORATOR FOR EASY INTEGRATION


def with_error_recovery(guardian: PlanGuardian):
    """
    Decorator that wraps a function with error recovery.

    Usage:
        guardian = PlanGuardian(world, robot_view, grasp_classifier)
    """

    def decorator(func: Callable) -> Callable:
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except PlanFailure as e:
                context = {
                    "args": str(args),
                    "kwargs": str(kwargs),
                }
                guardian.handle_error(e, None, context)

        return wrapper

    return decorator
