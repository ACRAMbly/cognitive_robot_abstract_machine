from __future__ import annotations

from dataclasses import dataclass, field

from coraplex.perception import PerceptionQuery
from coraplex.plans.executables import PerceptionExecutable
from coraplex.plans.plan_node import PlanNode


@dataclass
class PerceptionNode(PlanNode):
    """
    Node that answers a perception query and writes the detections into the world.

    Perception is an execution boundary rather than a motion: the motions planned after it
    have to be built against the world it produced, and on the real robot the surrounding
    motion state chart runs in the controller's process while the query is answered here.
    """

    query: PerceptionQuery = field(kw_only=True)
    """
    What to look for and where.
    """

    @property
    def is_execution_boundary(self) -> bool:
        return True

    def notify(self) -> None:
        pass

    def parse(self) -> PerceptionExecutable:
        return PerceptionExecutable(context=self.plan.context, query=self.query)
