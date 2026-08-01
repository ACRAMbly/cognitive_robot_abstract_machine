from dataclasses import dataclass

import numpy as np

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    Arms,
    ApproachDirection,
    DetectionTechnique,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import simulated_robot
from coraplex.plans.attachment_nodes import ModelChangeNode
from coraplex.perception import PerceptionQuery
from coraplex.plans.executables import (
    ConditionExecutable,
    Executable,
    GiskardExecutable,
    ModelChangeExecutable,
    PerceptionExecutable,
)
from coraplex.plans.factories import execute_single, sequential
from coraplex.plans.perception_nodes import PerceptionNode
from coraplex.plans.plan_node import MotionNode, PlanNode
from coraplex.robot_plans import MoveToolCenterPointMotion
from coraplex.robot_plans.actions.composite.transporting import TransportAction
from coraplex.robot_plans.actions.core.misc import DetectAction
from coraplex.robot_plans.actions.core.pick_up import ReachAction, PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from coraplex.utils import split_list_by_type
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose, Point3
from semantic_digital_twin.world_description.geometry import BoundingBox


def test_parse_simple_action(immutable_model_world):
    world, view, context = immutable_model_world

    plan = execute_single(MoveTorsoAction(TorsoState.HIGH), context=context)

    plan.notify()

    executable = plan.parse()

    assert type(executable) == GiskardExecutable
    assert executable.pre_condition_node
    assert executable.post_condition_node
    assert len(executable.motion_mappings) == 1
    assert type(list(executable.motion_mappings.values())[0]) == JointPositionList


def test_merge_motions(immutable_model_world, rclpy_node):
    world, view, context = immutable_model_world

    world.get_body_by_name("milk.stl").parent_connection.origin = (
        HomogeneousTransformationMatrix.from_xyz_rpy(2, 1.5, 0.7, 0, 0, 0)
    )

    plan = execute_single(
        ReachAction(
            Pose.from_xyz_rpy(2, 1.5, 0.7, reference_frame=world.root),
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
            world.get_body_by_name("milk.stl"),
        ),
        context=context,
    )

    plan.notify()

    executable = plan.parse()

    assert type(executable) == GiskardExecutable
    assert len(executable.motion_mappings) == 2
    assert executable.pre_condition_node
    assert executable.post_condition_node

    with simulated_robot:
        executable.execute()


def test_parse_pick_up(immutable_model_world):
    world, view, context = immutable_model_world

    plan = execute_single(
        PickUpAction(
            world.get_body_by_name("milk.stl"),
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
        ),
        context=context,
    )

    plan.notify()

    # plan.plan.visualize()

    executable = plan.parse()

    assert len(executable.execution_list) == 3
    assert type(executable.execution_list[0]) == GiskardExecutable
    assert type(executable.execution_list[1]) == ModelChangeExecutable
    assert type(executable.execution_list[2]) == GiskardExecutable


def test_parse_pick_up_merges_motions_around_model_change(immutable_model_world):
    """
    The motions on each side of the model change (the attach) must be merged into a
    single giskard executable per side, so the model change splits the plan into exactly
    [merged motions, model change, merged motions].
    """
    world, view, context = immutable_model_world

    plan = execute_single(
        PickUpAction(
            world.get_body_by_name("milk.stl"),
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
        ),
        context=context,
    )

    plan.notify()
    executable = plan.parse()

    # The four motions before the attach (open gripper, reach pre-pose, reach pose,
    # close gripper) merge into one executable; the lift after it into another.
    assert len(executable.execution_list[0].motion_mappings) == 4
    assert len(executable.execution_list[2].motion_mappings) == 1


def test_parse_complex_plan(immutable_model_world):
    world, view, context = immutable_model_world

    plan = sequential(
        [
            ParkArmsAction(Arms.BOTH),
            ReachAction(
                target_pose=Pose(
                    Point3.from_iterable([1, -2, 0.8]), reference_frame=world.root
                ),
                object_designator=world.get_body_by_name("milk.stl"),
                arm=Arms.LEFT,
                grasp_description=GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.NoAlignment,
                    view.right_arm.end_effector,
                ),
            ),
        ],
        context=context,
    )

    plan.notify()
    exec = plan.parse()
    assert type(exec) == GiskardExecutable
    assert len(exec.motion_mappings) == 3


def test_parsing_two_actions_into_one_exec(immutable_model_world):
    world, view, context = immutable_model_world

    plan = sequential(
        [
            ParkArmsAction(Arms.BOTH),
            ReachAction(
                target_pose=Pose(
                    Point3.from_iterable([1, -2, 0.8]), reference_frame=world.root
                ),
                object_designator=world.get_body_by_name("milk.stl"),
                arm=Arms.LEFT,
                grasp_description=GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.NoAlignment,
                    view.right_arm.end_effector,
                ),
            ),
        ],
        context=context,
    )

    plan.notify()
    exec = plan.parse()

    assert type(exec) == GiskardExecutable
    assert len(exec.motion_mappings) == 3


def test_parse_pick_place(immutable_model_world):
    world, view, context = immutable_model_world

    plan = sequential(
        [
            PickUpAction(
                world.get_body_by_name("milk.stl"),
                Arms.RIGHT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.NoAlignment,
                    view.right_arm.end_effector,
                ),
            ),
            PlaceAction(
                world.get_body_by_name("milk.stl"),
                Pose(reference_frame=world.root),
                Arms.RIGHT,
            ),
        ],
        context=context,
    )

    plan.notify()

    # plan.plan.visualize()

    executable = plan.parse()

    assert len(executable.execution_list) == 2
    assert len(executable.execution_list[0].execution_list) == 3
    assert len(executable.execution_list[1].execution_list) == 3


def test_parse_transport_plan(mutable_model_world, rclpy_node):
    world, view, context = mutable_model_world

    plan = sequential(
        [
            MoveTorsoAction(TorsoState.HIGH),
            ParkArmsAction(Arms.BOTH),
            TransportAction(
                world.get_body_by_name("milk.stl"),
                Pose.from_xyz_rpy(2.37, 2.5, 1.05, reference_frame=world.root),
                Arms.RIGHT,
            ),
        ],
        context=context,
    )

    plan.notify()
    exec = plan.parse()

    with simulated_robot:
        exec.execute()


# %% execution boundaries


@dataclass(eq=False, repr=False)
class BoundaryNode(PlanNode):
    """
    Node that declares itself an execution boundary and parses to a non-giskard
    executable.

    Stands in for any node that interrupts the merging of motions, so the split is pinned
    to the :attr:`~coraplex.plans.plan_node.PlanNode.is_execution_boundary` contract rather
    than to the node types that happen to declare it today.
    """

    @property
    def is_execution_boundary(self) -> bool:
        return True

    def notify(self) -> None:
        pass

    def parse(self) -> Executable:
        return Executable(context=self.plan.context)


def test_execution_boundary_splits_the_merged_motion_chart(immutable_model_world):
    """
    A node declaring itself an execution boundary separates the motions around it into
    one merged chart per side, instead of all of them collapsing into a single chart.
    """
    world, view, context = immutable_model_world

    plan = sequential(
        [
            MoveToolCenterPointMotion(Pose(reference_frame=world.root), Arms.LEFT),
            MoveToolCenterPointMotion(Pose(reference_frame=world.root), Arms.RIGHT),
            BoundaryNode(),
            MoveToolCenterPointMotion(Pose(reference_frame=world.root), Arms.LEFT),
        ],
        context=context,
    )

    plan.notify()
    executable = plan.parse()

    assert [type(child) for child in executable.execution_list] == [
        GiskardExecutable,
        Executable,
        GiskardExecutable,
    ]
    assert len(executable.execution_list[0].motion_mappings) == 2
    assert len(executable.execution_list[2].motion_mappings) == 1


def test_perception_node_parses_to_a_perception_executable(immutable_model_world):
    """
    Perception carries its query into execution and declares itself a boundary, so the
    motions planned after it are built against the world it produced.
    """
    world, view, context = immutable_model_world
    query = PerceptionQuery(
        Milk,
        BoundingBox(
            origin=HomogeneousTransformationMatrix(reference_frame=world.root),
            min_x=-10,
            min_y=-10,
            min_z=-10,
            max_x=10,
            max_y=10,
            max_z=10,
        ),
        view,
        world,
    )
    node = PerceptionNode(query=query)

    executable = execute_single(node, context=context).parse()

    assert node.is_execution_boundary
    assert type(executable) == PerceptionExecutable
    assert executable.query is query


def test_action_without_motions_evaluates_its_conditions_around_the_body(
    immutable_model_world,
):
    """
    An action whose body is only an execution boundary has no motion state chart to
    carry its conditions as monitors, so they run as their own executables around it.
    """
    world, view, context = immutable_model_world

    plan = execute_single(
        DetectAction(DetectionTechnique.TYPES, object_sem_annotation=Milk),
        context=context,
    )
    plan.notify()
    executable = plan.parse()

    assert [type(child) for child in executable.execution_list] == [
        ConditionExecutable,
        PerceptionExecutable,
        ConditionExecutable,
    ]


def test_action_without_motions_drops_its_conditions_when_they_are_not_evaluated(
    immutable_model_world,
):
    """
    A context that does not evaluate conditions leaves the action's body on its own, so
    nothing is run around it.
    """
    world, view, _ = immutable_model_world
    context = Context(world, view, evaluate_conditions=False)

    plan = execute_single(
        DetectAction(DetectionTechnique.TYPES, object_sem_annotation=Milk),
        context=context,
    )
    plan.notify()
    executable = plan.parse()

    assert [type(child) for child in executable.execution_list] == [
        PerceptionExecutable
    ]


# %% expansion-time pose capture


def test_pick_up_motions_follow_the_object_moved_after_expansion(immutable_model_world):
    """
    The whole plan is expanded before the first motion runs, so a pick-up that captured
    the object's pose in world coordinates could never act on a pose corrected in
    between (for example by a detection).

    Keeping the motion targets in the object's own frame is what lets them follow it.
    """
    world, view, context = immutable_model_world
    milk_body = world.get_body_by_name("milk.stl")

    plan = execute_single(
        PickUpAction(
            milk_body,
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
        ),
        context=context,
    )
    plan.notify()
    targets = [
        node.designator.target
        for node in plan.descendants
        if isinstance(node, MotionNode)
        and isinstance(node.designator, MoveToolCenterPointMotion)
    ]
    positions_before = [
        world.transform(target, world.root).to_position().to_np().flatten()[:3]
        for target in targets
    ]

    displacement = np.array([0.25, -0.4, 0.1])
    milk_body.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        *(milk_body.global_pose.to_position().to_np().flatten()[:3] + displacement),
        reference_frame=world.root,
    )

    assert targets
    assert all(target.reference_frame is milk_body for target in targets)
    for target, position_before in zip(targets, positions_before):
        np.testing.assert_allclose(
            world.transform(target, world.root).to_position().to_np().flatten()[:3],
            position_before + displacement,
            atol=1e-9,
        )


# %% splitting helper


def test_split_by_type(immutable_model_world):
    world, view, context = immutable_model_world

    split_list = [
        MoveToolCenterPointMotion(Pose(), Arms.LEFT),
        ModelChangeNode(body=world.get_body_by_name("milk.stl"), new_parent=world.root),
        MoveToolCenterPointMotion(Pose(), Arms.RIGHT),
    ]

    splitted_list = split_list_by_type(split_list, ModelChangeNode)

    assert len(splitted_list) == 3
    assert len(splitted_list[0]) == 1
    assert len(splitted_list[1]) == 1
    assert len(splitted_list[2]) == 1


def test_split_by_type_empty_list():
    assert split_list_by_type([], ModelChangeNode) == []


def test_split_by_type_without_match_stays_one_group():
    no_model_change = [
        MoveToolCenterPointMotion(Pose(), Arms.LEFT),
        MoveToolCenterPointMotion(Pose(), Arms.RIGHT),
    ]

    splitted_list = split_list_by_type(no_model_change, ModelChangeNode)

    assert len(splitted_list) == 1
    assert splitted_list[0] == no_model_change


def test_split_by_type_groups_consecutive_elements(immutable_model_world):
    world, view, context = immutable_model_world
    model_change = ModelChangeNode(
        body=world.get_body_by_name("milk.stl"), new_parent=world.root
    )

    split_list = [
        MoveToolCenterPointMotion(Pose(), Arms.LEFT),
        MoveToolCenterPointMotion(Pose(), Arms.RIGHT),
        model_change,
        MoveToolCenterPointMotion(Pose(), Arms.LEFT),
    ]

    splitted_list = split_list_by_type(split_list, ModelChangeNode)

    assert [len(group) for group in splitted_list] == [2, 1, 1]
    assert splitted_list[1] == [model_change]
    assert all(not isinstance(element, ModelChangeNode) for element in splitted_list[0])


def test_split_by_type_leading_and_trailing_match(immutable_model_world):
    world, view, context = immutable_model_world
    first_model_change = ModelChangeNode(
        body=world.get_body_by_name("milk.stl"), new_parent=world.root
    )
    last_model_change = ModelChangeNode(
        body=world.get_body_by_name("milk.stl"), new_parent=world.root
    )

    split_list = [
        first_model_change,
        MoveToolCenterPointMotion(Pose(), Arms.LEFT),
        last_model_change,
    ]

    splitted_list = split_list_by_type(split_list, ModelChangeNode)

    assert [len(group) for group in splitted_list] == [1, 1, 1]
    assert splitted_list[0] == [first_model_change]
    assert splitted_list[2] == [last_model_change]
