# Scenario parts and their motion

Status: resolved
Type: task
Blocked by: 01

A scenario names its parts by label and gives each one a smooth, deterministic
pose in the camera optical frame at any time of the 30 s sequence. Provide the
tetris and aircraft scenarios.

Done when tests show, for both scenarios at every pass: speed ≤ 5 cm/s, turn
rate ≤ 15 °/s, centres ≥ 0.3 m apart, every centre inside the 640 × 480 image.

## Answer

`synthetic_observations/scenario.py` (`PartPath`, `ScenarioPart`, `PlaneLayout`, `Scenario`) and `parts.py` (`TetrisBlock`, `AircraftPart`). Covered by `test_synthetic_scenarios.py`.

## Comments
