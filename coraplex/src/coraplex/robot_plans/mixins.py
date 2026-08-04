from dataclasses import dataclass, field

from typing_extensions import Optional


@dataclass
class HasMaxJointVelocity:
    """
    Adds an optional joint velocity cap to an action or motion.
    """

    max_joint_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum joint velocity (in rad/s or m/s, per joint), enforced via
    :class:`~giskardpy.motion_statechart.tasks.joint_tasks.JointVelocityLimit`.

    ``None`` leaves the speed unconstrained.
    """
