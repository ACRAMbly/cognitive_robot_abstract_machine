"""
Rewiring the expression graph finds expressions by identity, since ``==`` on symbolic
expressions builds a comparison.

The private graph methods are driven directly because no public operation reaches them.
"""

from krrood.entity_query_language.factories import variable

from ...dataset.example_classes import KRROODPosition
from ...dataset.value_comparisons import IsGreaterThan


def identifiers(expressions):
    """
    :param expressions: The expressions to identify.
    :return: The identifiers of the expressions, in order.
    """
    return [expression._id_ for expression in expressions]


# %% removing a parent


def test_detaching_a_child_keeps_its_siblings():
    position = variable(KRROODPosition, [])
    x, y = position.x, position.y
    predicate = IsGreaterThan(x, y)

    y._parent_ = None

    assert identifiers(predicate._children_) == identifiers([x])


def test_detaching_a_parent_keeps_the_other_parents():
    position = variable(KRROODPosition, [])
    other_position = variable(KRROODPosition, [])
    x, y = position.x, position.y

    y._replace_child_(position, other_position)

    assert identifiers(position._parents_) == identifiers([x])


# %% replacing a child that occurs more than once


def test_replacing_a_child_replaces_it_on_both_sides_of_a_comparison():
    position = variable(KRROODPosition, [])
    other_position = variable(KRROODPosition, [])
    x, other_x = position.x, other_position.x
    comparison = x > x

    comparison._replace_child_field_(x, other_x)

    assert identifiers([comparison.left, comparison.right]) == identifiers(
        [other_x, other_x]
    )


def test_replacing_a_child_replaces_every_argument_it_is_given_as():
    position = variable(KRROODPosition, [])
    other_position = variable(KRROODPosition, [])
    x, other_x = position.x, other_position.x
    predicate = IsGreaterThan(x, x)

    predicate._replace_child_field_(x, other_x)

    assert identifiers(predicate._child_variables_.values()) == identifiers(
        [other_x, other_x]
    )
