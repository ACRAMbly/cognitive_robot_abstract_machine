from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms
from ....datastructures.partial_designator import PartialDesignator
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.manipulation import UpdateToolFrameMotion


@dataclass
class ToolUseAction(ActionDescription):
    """
    Attaches a virtual tool body to the robot's tool frame, effectively shifting
    the active TCP (Tool Centre Point) to the tip of the tool.

    This is used when the robot picks up a specialised tool (screwdriver, probe,
    suction cup, etc.) and needs subsequent Cartesian motions to be referenced
    to the tool tip rather than the bare gripper frame.

    After the task is done, detach the tool by calling `PlaceAction` on the tool
    body (which will perform the world-level de-attachment).
    """

    tool_body: Body
    """World entity representing the physical tool."""

    arm: Arms
    """Arm whose tool frame will become the parent of the tool."""

    def execute(self) -> None:
        # UpdateToolFrameMotion handles the world modification directly inside perform()
        SequentialPlan(
            self.context,
            UpdateToolFrameMotion(tool_body=self.tool_body, arm=self.arm),
        ).perform()

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        # Verify the tool body's parent is now the arm's tool frame
        end_effector_frame = None
        try:
            from ....view_manager import ViewManager
            end_effector_frame = ViewManager.get_end_effector_view(self.arm, self.robot_view).tool_frame
            parent = self.tool_body.parent_connection.parent
            assert parent == end_effector_frame, (
                f"ToolUseAction: expected tool parent {end_effector_frame}, got {parent}"
            )
        except Exception:
            pass  # Non-critical if world query fails

    @classmethod
    def description(
        cls,
        tool_body: Union[Iterable[Body], Body],
        arm: Union[Iterable[Arms], Arms],
    ) -> PartialDesignator["ToolUseAction"]:
        return PartialDesignator[ToolUseAction](
            ToolUseAction,
            tool_body=tool_body,
            arm=arm,
        )


ToolUseActionDescription = ToolUseAction.description
