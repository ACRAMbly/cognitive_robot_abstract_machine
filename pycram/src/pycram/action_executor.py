from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from itertools import groupby

from typing_extensions import TYPE_CHECKING, Any, List, Type, Dict

from giskardpy.motion_statechart.goals.templates import Sequence, Parallel
from giskardpy.motion_statechart.graph_node import Task, Goal, MotionStatechartNode

if TYPE_CHECKING:

    from pycram.plans.plan_node import ActionNode, PlanNode, MotionNode
    from pycram.robot_plans import BaseMotion
    from pycram.plans.condition_nodes import ConditionNode
    from pycram.language import LanguageNode


@dataclass
class GraphParser(ABC):
    """
    Base class for parsing a plan graph into executable units.
    """

    @abstractmethod
    def parse(self) -> Executable:
        """
        Parses the graph and returns the result.
        """
        pass

    def parse_children(self, children: List[PlanNode]) -> List[Executable]:
        """
        Expands sub-actions within a list of plan nodes into executables.

        :param children: The list of plan nodes to expand.
        :return: A list of executables.
        """
        from pycram.plans.plan_node import ActionNode, MotionNode
        from pycram.language import LanguageNode

        result = []

        for child in children:
            if isinstance(child, ActionNode):
                result.append(ActionGraphParser(child).parse())
            elif isinstance(child, LanguageNode):
                result.append(LanguageGraphParser(child).parse())
            elif isinstance(child, MotionNode):
                result.append(
                    MotionExecutable(
                        motion_mappings={child: child.designator.motion_chart}
                    )
                )
            else:
                result.append(child)
        return result

    def split_list_by_type(
        self, flat_list: List, cluster_type: Type[Any]
    ) -> List[List[Executable]]:
        groups = list(
            (
                list(g)
                for _, g in groupby(
                    flat_list, key=lambda m: isinstance(m, cluster_type)
                )
            )
        )
        return groups

    def group_by_type(
        self, flat_list: List[Any], group_type: Type[Any]
    ) -> List[List[Executable]]:
        groups = list(
            (
                list(g)
                for _, g in groupby(
                    flat_list, key=lambda m: not isinstance(m, group_type)
                )
            )
        )
        return groups

    def merge_motion_executables(
        self, executables: List[Executable]
    ) -> List[Executable]:
        result = []
        for exec in self.group_by_type(executables, MotionExecutable):
            if not isinstance(exec[0], MotionExecutable):
                result.extend(exec)
            else:
                new_mappings = self.merge_motion_mappings(exec)
                result.append(MotionExecutable(motion_mappings=new_mappings))
        return result

    def merge_motion_mappings(self, motions: List[Dict[MotionNode, Task]]):
        new_mappings = {}
        for motion in motions:
            new_mappings.update(motion.motion_mappings)
        return new_mappings


@dataclass
class ActionGraphParser(GraphParser):
    """
    Parser for action nodes in a plan graph.
    """

    action_node: ActionNode
    """
    The action node to parse.
    """

    def parse(self) -> Executable:
        """
        Parses the action node and its children into a list of executables.

        :return: A list of executables.
        """
        children = self.action_node.children
        pre_condition_executable = ConditionExecutable(condition_node=children.pop(0))
        post_condition_executable = ConditionExecutable(condition_node=children.pop(-1))

        child_execs = self.parse_children(children)

        exec_list = [pre_condition_executable, *child_execs, post_condition_executable]

        return Executable(self.merge_motion_executables(exec_list))


@dataclass
class LanguageGraphParser(GraphParser):

    language_node: LanguageNode

    def parse(self) -> Executable:

        child_executables = self.parse_children(self.language_node.children)

        all_motions = all([isinstance(m, MotionExecutable) for m in child_executables])
        if all_motions:
            tasks = [
                t for exe in child_executables for t in exe.motion_mappings.values()
            ]

            return LanguageExecutable(
                motion_mappings=self.merge_motion_mappings(child_executables),
                giskard_task=self.language_node.msc_template(nodes=tasks),
            )


@dataclass
class Executable:
    """
    Base class for executable units.
    """

    execution_list: List[Executable] = field(default_factory=list)

    def execute(self) -> None:
        """
        Executes the unit.
        """
        for e in self.execution_list:
            e.execute()


@dataclass
class GiskardExecutable(Executable):
    motion_mappings: Dict[MotionNode, Task] = field(kw_only=True)

    giskard_task: MotionStatechartNode = field(kw_only=True, default=None)

    def execute(self) -> None:
        pass


@dataclass
class LanguageExecutable(GiskardExecutable):

    @property
    def motion_state_chart(self):
        return Parallel(
            nodes=[
                motion.motion_node.designator.motion_chart for motion in self.motions
            ]
        )


@dataclass
class ConditionExecutable(Executable):
    """
    An executable unit for a condition node.
    """

    condition_node: ConditionNode = field(kw_only=True)
    """
    The condition node to execute.
    """

    def execute(self) -> None:
        """
        Executes the condition node.
        """
        pass


@dataclass
class MotionExecutable(GiskardExecutable): ...
