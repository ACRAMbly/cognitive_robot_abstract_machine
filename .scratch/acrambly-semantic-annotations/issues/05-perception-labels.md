# Perception labels from annotation classes

Status: resolved
Type: task
Blocked by: 03

Remove `robokudo_acrambly/src/robokudo_acrambly/synthetic_observations/parts.py`
and its `TetrisBlock` and `AircraftPart` `StrEnum`s. The synthetic tetris and
aircraft analysis engines arrange the concrete subclasses of the annotation
classes `TetrisBlock` and `AircraftPart`, each labelled with its class name.
Add `semantic_digital_twin_acrambly` to the dependencies of
`robokudo_acrambly`.

The tetris labels change from `child_cube_0_colored` and
`child_cube_2_colored` to `StraightTetrisBlock` and `CornerTetrisBlock`; the
aircraft labels `RearWheelLeft` and `RearWheelRight` change to
`LeftRearWheel` and `RightRearWheel`.

Update `test/robokudo_acrambly_test/test_synthetic_scenarios.py` to expect
those classes instead of the removed enums. This follows the API change; it is
not a fix to a failing test.

Done when the `robokudo_acrambly` tests pass and nothing imports `parts.py`.

## Comments

## Answer

Built and merged in `02605615ae`. `parts.py` and its enums are gone. The
tetris and aircraft engines label each part with its annotation class name,
taken from `concrete_subclasses` of `TetrisBlock` / `AircraftPart` in
definition order. `robokudo_acrambly` depends on
`semantic_digital_twin_acrambly`. `test_synthetic_scenarios.py` expects the
annotation classes. All 33 tests in `test/robokudo_acrambly_test` pass with ROS
sourced, and nothing imports `parts.py`.
