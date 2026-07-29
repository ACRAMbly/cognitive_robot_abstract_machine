"""
Exceptions raised while executing a trajectory on a robot.
"""

from __future__ import annotations

from dataclasses import dataclass

from giskardpy.data_types.exceptions import (
    DontPrintStackTrace,
    GiskardException,
    SetupException,
)


@dataclass
class ExecutionException(GiskardException):
    """
    Base class for errors that occur while executing a trajectory.
    """


@dataclass
class NoActiveGoalToCancelError(ExecutionException):
    """
    Raised when a goal cancellation is requested but no goal is active.
    """

    def error_message(self) -> str:
        return "Can't cancel goals, because there is no active one."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class ExecutionCanceledException(ExecutionException):
    """
    Raised when the execution of a goal is canceled.
    """

    action_server_name: str
    """
    The name of the action server whose goal was canceled.
    """

    goal_id: int
    """
    The id of the canceled goal.
    """

    def error_message(self) -> str:
        return f"'{self.action_server_name}' goal #{self.goal_id} canceled"

    def suggest_correction(self) -> str:
        return ""


@dataclass
class WorldModelModifiedDuringMotionError(ExecutionException, DontPrintStackTrace):
    """
    Raised when another process modified the world model while a motion was running.

    The motion statechart and the quadratic program are compiled against the structure
    of the world, so the modification cannot be applied under a running motion. The
    motion is terminated instead and the modification is applied once Giskard is idle
    again.
    """

    def error_message(self) -> str:
        return "The world model was modified by another process during the motion."

    def suggest_correction(self) -> str:
        return "Send the goal again; the modification is applied by then."


@dataclass
class ExecutionPreemptedException(ExecutionException):
    """
    Raised when the execution of a goal is preempted.
    """

    namespace: str
    """
    The namespace of the action server that was preempted.
    """

    def error_message(self) -> str:
        return f"'{self.namespace}' preempted. Stopping execution."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class ExecutionTimeoutException(ExecutionException):
    """
    Raised when the execution of a goal takes too long.
    """

    namespace: str
    """
    The namespace of the action server that timed out.
    """

    reason: str
    """
    A description of why the execution timed out.
    """

    def error_message(self) -> str:
        return f"'{self.namespace}' timed out. {self.reason}"

    def suggest_correction(self) -> str:
        return ""


@dataclass
class ExecutionAbortedException(ExecutionException):
    """
    Raised when the execution is aborted by Giskard.
    """

    def error_message(self) -> str:
        return "Execution aborted by Giskard."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class ExecutionSucceededPrematurely(ExecutionException):
    """
    Raised when the execution finishes before the minimum execution time.
    """

    namespace: str
    """
    The namespace of the action server that finished too early.
    """

    def error_message(self) -> str:
        return f"'{self.namespace}' executed too quickly, stopping execution."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class FollowJointTrajectoryError(ExecutionException):
    """
    Raised when a follow joint trajectory action server fails to execute a goal.
    """

    namespace: str
    """
    The namespace of the action server that failed.
    """

    error_description: str
    """
    A human-readable description of the action server error code.
    """

    def error_message(self) -> str:
        return f"'{self.namespace}' failed to execute goal. Error: '{self.error_description}'"

    def suggest_correction(self) -> str:
        return ""


@dataclass
class FollowJointTrajectory_INVALID_GOAL(FollowJointTrajectoryError):
    """
    Raised when the action server reports an invalid goal.
    """


@dataclass
class FollowJointTrajectory_INVALID_JOINTS(FollowJointTrajectoryError):
    """
    Raised when the action server reports invalid joints.
    """


@dataclass
class FollowJointTrajectory_OLD_HEADER_TIMESTAMP(FollowJointTrajectoryError):
    """
    Raised when the action server reports an outdated header timestamp.
    """


@dataclass
class FollowJointTrajectory_PATH_TOLERANCE_VIOLATED(FollowJointTrajectoryError):
    """
    Raised when the action server reports a path tolerance violation.
    """


@dataclass
class FollowJointTrajectory_GOAL_TOLERANCE_VIOLATED(FollowJointTrajectoryError):
    """
    Raised when the action server reports a goal tolerance violation.
    """


@dataclass
class AlreadyTrackedByTfFrameError(SetupException):
    """
    Raised when a connection is registered for tf tracking a second time.
    """

    connection_name: str
    """
    The name of the connection that is already tracked.
    """

    tf_parent_frame: str
    """
    The tf parent frame the connection is already tracked with.
    """

    tf_child_frame: str
    """
    The tf child frame the connection is already tracked with.
    """

    def error_message(self) -> str:
        return (
            f"Connection '{self.connection_name}' is already tracked with a tf frame: "
            f"'{self.tf_parent_frame}'<-'{self.tf_child_frame}'"
        )

    def suggest_correction(self) -> str:
        return ""


@dataclass
class ConnectionCannotBeTrackedByTfFrameError(SetupException):
    """
    Raised when a connection without 6 degrees of freedom is registered for tf tracking.
    """

    connection_name: str
    """
    The name of the connection that cannot be tracked.
    """

    connection_type: str
    """
    The type of the connection that cannot be tracked.
    """

    def error_message(self) -> str:
        return (
            f"Can only sync Connection6DoF with tf, but '{self.connection_name}' is of "
            f"type '{self.connection_type}'."
        )

    def suggest_correction(self) -> str:
        return ""
