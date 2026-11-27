# Label lookup prefers the most specific class

Status: needs-triage
Type: task

`annotation_class_by_label` in
`semantic_digital_twin/src/semantic_digital_twin/reasoning/queries.py` returns the
first `IsPerceivable` subclass whose lowercased name is contained in the label. When
several class names match, the result depends on subclass order. With
`IsPerceivable` on `TetrisBlock`, the label `StraightTetrisBlock` matches both
`StraightTetrisBlock` and the abstract `TetrisBlock`.

Wanted: when several classes match, the lookup returns the most specific one, the
one with the longest name.

Done when:

- a test, written first and failing, shows a base class winning over the class the
  label names;
- the synthetic tetris engine test asserts the exact class of each belief, not
  only that it is a `TetrisBlock` with the exact `class_label`.

Deferred from the design session of 2026-11-27. Changes `semantic_digital_twin`, so
it needs the developer's permission.

## Comments
