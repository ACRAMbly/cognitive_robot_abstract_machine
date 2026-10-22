
from __future__ import annotations

import mujoco

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from datetime import timedelta

from coraplex.execution_environment import PhysicallySimulatedRobot
from semantic_digital_twin.datastructures.definitions import StaticJointState


world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy, evaluate_conditions=False)

plan = sequential([
    ParkArmsAction(tracy.all_arms),

    ],
    context=context).plan

for arm in tracy.all_arms:
    arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)


simulation = MujocoSim(
    world=world,
    headless=False,
    step_size=1e-3,
    cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
    impratio=10.0,
    noslip_iterations=10,
)

simulation.start_stepped_simulation()

try:
    simulation.step_simulation(timedelta(seconds=1))
    with PhysicallySimulatedRobot(simulation=simulation):
        plan.perform()
finally:
    simulation.stop_simulation()