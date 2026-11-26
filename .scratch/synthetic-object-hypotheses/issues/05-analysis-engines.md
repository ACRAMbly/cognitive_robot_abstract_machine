# Analysis engines for both scenarios

Status: resolved
Type: task
Blocked by: 04

One analysis-engine descriptor per scenario, containing only the reader with the
synthetic source.

Done when tests build both pipelines and find the source of the right scenario.

## Answer

`synthetic_observations/analysis_engine.py` (`SyntheticObservationEngine`) and `descriptors/analysis_engines/synthetic_{tetris,aircraft}.py`. Covered by `test_synthetic_analysis_engines.py`, which loads both through RoboKudo's `ModuleLoader`.

## Comments
