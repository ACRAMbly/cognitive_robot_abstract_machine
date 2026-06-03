from pycram.action_executor import (
    ActionGraphParser,
    ConditionExecutable,
    MotionExecutable,
    LanguageExecutable,
)
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.core.pick_up import ReachAction
from pycram.robot_plans.actions.core.robot_body import MoveTorsoAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.position_descriptions import (
    VerticalSemanticDirection,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose


def test_parse_simple_action(immutable_model_world):
    world, view, context = immutable_model_world

    plan = execute_single(MoveTorsoAction(TorsoState.HIGH), context=context)

    plan.notify()

    executable = ActionGraphParser(plan).parse()

    assert len(executable.execution_list) == 3
    assert type(executable.execution_list[0]) == ConditionExecutable
    assert type(executable.execution_list[1]) == MotionExecutable


def test_merge_motions(immutable_model_world):
    world, view, context = immutable_model_world

    plan = execute_single(
        ReachAction(
            Pose(reference_frame=world.root),
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.manipulator,
            ),
            world.get_body_by_name("milk.stl"),
        ),
        context=context,
    )

    plan.notify()

    executable = ActionGraphParser(plan).parse()

    assert len(executable.execution_list) == 3
    assert type(executable.execution_list[0]) == ConditionExecutable
    assert type(executable.execution_list[1]) == LanguageExecutable
