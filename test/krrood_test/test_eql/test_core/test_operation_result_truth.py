"""
Characterization of the truth value every truth-bearing expression reports.

Pins the observable ``OperationResult`` truth contract for each operator family, so that
moving truth onto a single source (``bindings[expression._id_]``) cannot silently change
an outcome. Every assertion is on the public contract (``is_true``/``is_false``), never
on how the value is stored.
"""

from typing_extensions import List, Optional

from krrood.entity_query_language.core.base_expressions import (
    OperationResult,
    SymbolicExpression,
)
from krrood.entity_query_language.factories import (
    and_,
    entity,
    exists,
    for_all,
    not_,
    or_,
    variable_from,
)
from krrood.entity_query_language.operators.set_operations import Union


def truth_values(
    expression: SymbolicExpression, sources: Optional[OperationResult] = None
) -> List[bool]:
    """
    :param expression: The expression to evaluate.
    :param sources: The incoming result to evaluate against, if any.
    :return: The truth value of every ``OperationResult`` *expression* yields, in order.
    """
    return [result.is_true for result in expression._evaluate_(sources)]


# %% comparators


def test_comparator_is_true_when_its_comparison_holds():
    value = variable_from([6])

    assert truth_values(value > 5) == [True]


def test_comparator_is_false_when_its_comparison_fails():
    value = variable_from([3])

    assert truth_values(value > 5) == [False]


def test_comparator_reports_one_truth_value_per_domain_value():
    value = variable_from([3, 6])

    assert truth_values(value > 5) == [False, True]


# %% negation


def test_negation_inverts_a_true_child():
    value = variable_from([6])

    assert truth_values(not_(value > 5)) == [False]


def test_negation_inverts_a_false_child():
    value = variable_from([3])

    assert truth_values(not_(value > 5)) == [True]


# %% conjunction


def test_conjunction_is_true_when_both_sides_hold():
    value = variable_from([6])

    assert truth_values(and_(value > 5, value < 10)) == [True]


def test_conjunction_is_false_and_skips_the_right_side_when_the_left_fails():
    value = variable_from([3])

    assert truth_values(and_(value > 5, value < 10)) == [False]


def test_conjunction_is_false_when_its_left_side_yields_nothing():
    """
    Negation as failure: an empty domain produces no result to judge, which the
    conjunction reports as false rather than as no result at all.
    """
    empty_value = variable_from([])
    other_value = variable_from([1])

    assert truth_values(and_(empty_value > 5, other_value > 0)) == [False]


# %% disjunction


def test_disjunction_is_true_and_skips_the_right_side_when_the_left_holds():
    value = variable_from([6])

    assert truth_values(or_(value > 5, value < 0)) == [True]


def test_disjunction_falls_back_to_a_true_right_side():
    value = variable_from([3])

    assert truth_values(or_(value > 5, value < 10)) == [True]


def test_disjunction_is_false_when_neither_side_holds():
    value = variable_from([3])

    assert truth_values(or_(value > 5, value < 0)) == [False]


def test_disjunction_is_false_when_its_left_side_yields_nothing():
    empty_value = variable_from([])
    other_value = variable_from([1])

    assert truth_values(or_(empty_value > 5, other_value > 0)) == [False]


# %% quantifiers


def test_existential_quantifier_is_true_when_one_value_satisfies_the_condition():
    quantified_value = variable_from([1, 3, 7])

    assert truth_values(exists(quantified_value, quantified_value > 5)) == [True]


def test_existential_quantifier_is_false_when_no_value_satisfies_the_condition():
    quantified_value = variable_from([1, 3])

    assert truth_values(exists(quantified_value, quantified_value > 5)) == [False]


def test_universal_quantifier_is_true_when_every_value_satisfies_the_condition():
    quantified_value = variable_from([6, 7])

    assert truth_values(for_all(quantified_value, quantified_value > 5)) == [True]


def test_universal_quantifier_yields_nothing_when_a_value_fails_the_condition():
    quantified_value = variable_from([3, 7])

    assert truth_values(for_all(quantified_value, quantified_value > 5)) == []


# %% union


def test_union_reports_the_truth_value_of_each_child_result():
    value = variable_from([6])
    other_value = variable_from([3])

    assert truth_values(Union((value > 5, other_value > 5))) == [True, False]


# %% queries


def test_query_yields_only_results_whose_condition_holds():
    """
    A query filters internally, so every result it yields is already true; a domain
    value failing the condition produces no result rather than a false one.
    """
    value = variable_from([3, 6])
    query = entity(value).where(value > 5)
    query.build()

    assert truth_values(query) == [True]


def test_query_yields_only_its_true_results_to_callers():
    value = variable_from([3, 6])

    assert list(entity(value).where(value > 5).evaluate()) == [6]


def test_query_selecting_a_falsy_value_reports_its_result_as_true():
    """
    A query's own binding is its selection, not a truth claim, so selecting ``0`` or an
    empty collection says nothing about whether the query was satisfied.

    Any consumer reading the truth of a query's result — a subquery evaluated as a
    condition, or a caller filtering results — would otherwise discard a legitimately
    selected falsy value.
    """
    value = variable_from([0, 1])
    query = entity(value)
    query.build()

    assert truth_values(query) == [True, True]


def test_query_selecting_an_empty_collection_reports_its_result_as_true():
    query = entity(variable_from([[], [1]]))
    query.build()

    assert truth_values(query) == [True, True]
