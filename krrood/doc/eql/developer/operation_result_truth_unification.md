# Unification of OperationResult Truth Semantics

## Status

**Implemented.**

## Motivation

`OperationResult` used to carry two truth-related attributes:

- `is_false: bool` — a raw dataclass field, set explicitly by the logical operators, and
  the one `is_true` read.
- `is_condition_false: property` — the rule condition evaluation actually applied:
  value-based for expressions with a binding, flag-based for operators without one.

The two disagreed by construction, so reaching for the obvious-looking `is_false` on a
result that bound a value gave the wrong answer. The unification removes the duplicate
state: truth is no longer stored on the result at all, only read from the bindings.

## The invariant

**An expression's truth is read from the binding it recorded.**

`OperationResult.is_false` reads `bindings[operand._id_]` directly and applies one rule:
an empty collection or a falsy value is false. An operand that recorded no binding makes
no truth claim and is therefore not false.

## Truth and value share one binding

Each expression has exactly one binding, so truth and value cannot both live there. The
two kinds of expression are told apart by `TruthValuedExpression`:

- **Truth-valued** — a logical operator, a quantifier, a rule-tree selector, a union, a
  comparator. Its binding *is* a truth value; it has no separate value to report, so
  `TruthValuedExpression._process_result_` returns the result's bindings rather than
  presenting the truth value as a selectable one.
- **Value-producing** — a variable, an attribute, an aggregator, an arithmetic operator,
  a query. Its binding is a value. In a condition context that value's truthiness *is*
  its truth (`where(flag)`, or a count of zero), which is why `is_false` applies the
  same rule to every binding. Outside a condition it makes no truth claim: `evaluate()`
  filters by truth only for a root that records truth (`_records_truth_`), so a query
  selecting `0` or an empty list still yields it.

## Recording truth

Every truth-valued expression records its own truth through
`SymbolicExpression._build_operation_result_with_truth_`, which copies the bindings
before writing so that a sibling evaluation branch sharing them is unaffected — the
discipline `Comparator.get_operation_result` already followed.

`TruthValueOperator._evaluate_child_as_condition_` no longer rebuilds its children's
results to fix up their truth; a child already reports it. The method remains as the
place where a child is evaluated in a condition context.

## Satisfied-condition tracking

`SatisfiedConditionTracker.on_conclusions_processed` no longer needs a separate rule for
logical operators, nor the truth map it used to build by walking the
`previous_operation_result` chain. One lookup per condition participant covers every
expression, and an expression short-circuited by an operator recorded no binding, so it
is correctly not satisfied.
