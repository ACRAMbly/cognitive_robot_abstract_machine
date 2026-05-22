from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import TYPE_CHECKING

from pycram.motion_executor import MotionExecutor

if TYPE_CHECKING:

    from pycram.plans.plan import Plan
    from pycram.plans.plan_node import ActionNode
    from semantic_digital_twin.world import World


@dataclass
class ActionExecutor:
    action_node: ActionNode
    """
    Root node of the action sub-plan that should be executed
    """

    plan: Plan
    """
    Plan to which the action node belongs
    """

    world: World
    """
    World in which the action should be executed.
    """

    def execute(self):
        pass
