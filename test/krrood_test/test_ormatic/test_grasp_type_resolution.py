from coraplex.datastructures.grasp import GraspDescription
from krrood.class_diagrams.utils import get_type_hints_of_object
from semantic_digital_twin.robots.robot_parts import EndEffector


def test_grasp_description_end_effector_type_resolves():
    """The ``end_effector`` annotation of :class:`GraspDescription` must be resolvable
    by krrood's class-diagram type resolver.

    ``EndEffector`` is only referenced in annotations, so a missing import left the
    name out of the module scope and made the resolver raise ``CouldNotResolveType``
    while building the entity-query class diagram for ``set_of`` translations.
    """
    resolved_type_hints = get_type_hints_of_object(GraspDescription)

    assert resolved_type_hints["end_effector"] is EndEffector
