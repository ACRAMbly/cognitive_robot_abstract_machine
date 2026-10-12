"""
The montessori demo carried out against MuJoCo physics, in a MuJoCo viewer.

The physics is stepped from the control loop rather than from a thread of its own: every
tick of the controller writes its command into the world, the servos take it as their set
point, and the physics advances one cycle before the next tick reads the world back. That
is what :class:`~coraplex.execution_environment.PhysicallySimulatedRobot` has every
motion of a plan do.
"""

from __future__ import annotations

from datetime import timedelta
from typing import TYPE_CHECKING

import mujoco

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import PhysicallySimulatedRobot
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.contact import (
    ContactFriction,
    ContactImpedance,
    ContactParameters,
    ContactStiffness,
)

if TYPE_CHECKING:
    from demo import MontessoriEnvironment

HEADLESS = False
"""
Whether the simulation runs without a viewer.
"""

STEP_SIZE = 1e-3
"""
How much simulated time one physics step covers, in seconds.
"""

SETTLING_DURATION = timedelta(seconds=1)
"""
How long the scene is left to settle before the robot starts, so the pieces come to rest
on the table rather than being grasped mid-fall.
"""

FRICTION_CONE = mujoco.mjtCone.mjCONE_ELLIPTIC
"""
How MuJoCo bounds a contact's friction.

The elliptic cone is the exact one; the default pyramid only approximates it and lets a
squeezed piece creep between the pads.
"""

IMPEDANCE_RATIO = 10.0
"""
How much stiffer MuJoCo makes friction than the push along the contact normal
(``impratio``).

MuJoCo's own guidance for grasping with an elliptic cone; at the default of 1 a held
piece slides out under its own weight despite a firm squeeze.
"""

NO_SLIP_ITERATIONS = 10
"""
How many passes MuJoCo spends removing the slip its soft contacts leave, so that a piece
held as firmly as the real gripper holds it stays where the pads hold it.
"""

RIGID_CONTACT_STIFFNESS = ContactStiffness(
    time_constant=timedelta(seconds=2 * STEP_SIZE), damping_ratio=1.0
)
"""
How stiffly two bodies push back once they touch: as stiffly as MuJoCo stays stable at,
a time constant of two physics steps.

The wooden pieces, the board and the gripper's pads are solid, so they must not sink
into each other; MuJoCo's default of 20 ms lets a firmly held piece sink several
millimetres into the pads.
"""

RIGID_CONTACT_IMPEDANCE = ContactImpedance(minimum=0.99, maximum=0.999, width=0.001)
"""
How little a contact gives way under load: nearly not at all, as between solid bodies.
"""


def run(environment: MontessoriEnvironment, world: World) -> None:
    """
    Carry the sorting plan out against MuJoCo physics.

    :param environment: The scene, with the plan that sorts its pieces.
    :param world: The world the environment stood Tracy, the board and the pieces in.
    """
    _make_every_contact_rigid(world)
    [robot] = world.get_semantic_annotations_by_type(Tracy)
    context = Context(
        world=world,
        robot=robot,
        evaluate_conditions=False,
    )
    plan = environment.build_plan(context)

    simulation = MujocoSim(
        world=world,
        headless=HEADLESS,
        step_size=STEP_SIZE,
        cone=FRICTION_CONE,
        impratio=IMPEDANCE_RATIO,
        noslip_iterations=NO_SLIP_ITERATIONS,
    )
    simulation.start_stepped_simulation()
    try:
        simulation.step_simulation(SETTLING_DURATION)
        with PhysicallySimulatedRobot(simulation=simulation):
            plan.perform()
    finally:
        simulation.stop_simulation()


def _make_every_contact_rigid(world: World) -> None:
    """
    Make every body of ``world`` push back as a solid does, keeping the friction each one
    already has.

    :param world: The world whose bodies are made rigid.
    """
    for body in world.bodies_with_collision:
        for shape in body.collision.shapes:
            contact = shape.get_simulator_property_of_type(ContactParameters)
            if contact is None:
                contact = ContactParameters(friction=ContactFriction())
                shape.add_simulator_property(contact)
            contact.stiffness = RIGID_CONTACT_STIFFNESS
            contact.impedance = RIGID_CONTACT_IMPEDANCE
