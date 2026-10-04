"""
Selecting the statements of a condition by whether they hold.
"""

from krrood.entity_query_language.factories import (
    and_,
    get_false_statements,
    get_true_statements,
    variable,
)

from ...dataset.example_classes import KRROODPosition
from ...dataset.value_comparisons import IsGreaterThan

# %% which children are statements


def test_attributes_a_statement_takes_are_not_statements_of_it():
    position = variable(KRROODPosition, [KRROODPosition(0.0, 1.0, 0.0)])

    statement = IsGreaterThan(position.x, position.y)

    assert get_false_statements(statement) == []
    assert get_true_statements(statement) == []


def test_calculated_operands_of_a_statement_are_not_statements_of_it():
    position = variable(KRROODPosition, [KRROODPosition(0.0, 1.0, 0.0)])

    statement = IsGreaterThan(position.x + 1, position.y + 1)

    assert get_false_statements(statement) == []
    assert get_true_statements(statement) == []


# %% selecting statements by truth


def test_false_statements_are_the_statements_that_do_not_hold():
    position = variable(KRROODPosition, [KRROODPosition(2.0, 0.0, 0.0)])
    false_statement = IsGreaterThan(position.y, position.x)
    true_statement = IsGreaterThan(position.x, position.y)

    statements = get_false_statements(and_(true_statement, false_statement))

    assert [statement._id_ for statement in statements] == [false_statement._id_]


def test_true_statements_are_the_statements_that_hold():
    position = variable(KRROODPosition, [KRROODPosition(2.0, 0.0, 0.0)])
    false_statement = IsGreaterThan(position.y, position.x)
    true_statement = IsGreaterThan(position.x, position.y)

    statements = get_true_statements(and_(true_statement, false_statement))

    assert [statement._id_ for statement in statements] == [true_statement._id_]


# %% statements judged together with the values the other conditions bind


def test_false_statements_hold_for_no_value_the_other_conditions_allow():
    """
    A statement that holds for some value of a variable is still false when it holds for
    none of the values the other conditions allow.
    """
    position = variable(
        KRROODPosition,
        [KRROODPosition(2.0, 0.0, 0.0), KRROODPosition(0.0, 2.0, 0.0)],
    )
    large_x = IsGreaterThan(position.x, 1.0)
    large_y = IsGreaterThan(position.y, 1.0)

    statements = get_false_statements(and_(large_x, large_y))

    assert [statement._id_ for statement in statements] == [large_y._id_]


def test_true_statements_hold_for_a_value_the_other_conditions_allow():
    """
    A statement holds only when it holds for a value the other conditions allow.
    """
    position = variable(
        KRROODPosition,
        [KRROODPosition(2.0, 0.0, 0.0), KRROODPosition(0.0, 2.0, 0.0)],
    )
    large_x = IsGreaterThan(position.x, 1.0)
    large_y = IsGreaterThan(position.y, 1.0)

    statements = get_true_statements(and_(large_x, large_y))

    assert [statement._id_ for statement in statements] == [large_x._id_]


def test_statements_skipped_after_a_false_statement_are_neither_true_nor_false():
    """
    A statement that is never evaluated, because an earlier one ruled out every value,
    is not reported as false, nor as true.
    """
    position = variable(KRROODPosition, [KRROODPosition(0.0, 0.0, 0.0)])
    large_x = IsGreaterThan(position.x, 1.0)
    large_y = IsGreaterThan(position.y, 1.0)
    condition = and_(large_x, large_y)

    false_statements = get_false_statements(condition)
    true_statements = get_true_statements(condition)

    assert [statement._id_ for statement in false_statements] == [large_x._id_]
    assert true_statements == []
