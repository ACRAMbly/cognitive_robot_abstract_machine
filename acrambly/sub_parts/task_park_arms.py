"""
Task: Park Arms
===============
Trivial task that just sends both arms to their park position.
Demonstrates the simplest possible task module – it needs no perception and
no world objects.

Callable signature
------------------
``setup_and_build_plan(world, tracy, context, node) -> Plan | None``
"""

from rclpy.node import Node
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.available_plans import build_park_arms_plan


def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """No perception or world-setup needed – just return the park-arms plan."""
    return build_park_arms_plan(context)