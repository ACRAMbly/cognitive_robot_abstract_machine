# Notes for implementers of `.scratch/synthetic-hypotheses-in-shared-world`

Integration branch: `feature/synthetic-hypotheses-in-shared-world` (main checkout at
`/home/nasim/Desktop/cognitive_robot_abstract_machine`).

## Running tests

- Use `run_tests.sh` next to this file (machine-specific paths): `run_tests.sh <your-worktree-root> <pytest args>`.
  It sources ROS, puts every `*/src` of the checkout on `PYTHONPATH`, sets
  `CRAM_ORM_BUILD=never`, and caps memory at 16 GiB. Check `free -g` first.
- A fresh worktree has no generated `ormatic_interface.py` (git-ignored). None of the
  packages with one change in this spec, so copy them from the main checkout:
  `coraplex`, `giskardpy`, `segmind`, `semantic_digital_twin`, `experiments`
  (`<pkg>/src/<pkg>/orm/ormatic_interface.py`).
- Baseline on the integration branch before any ticket:
  `test/robokudo_acrambly_test test/semantic_digital_twin_acrambly_test` -> 64 passed.
- Run one package or a few test files at a time, never the whole suite.

## Formatting and checks

- Format modified Python files with `krrood format <files>` and check with
  `krrood check <files>` (the pre-commit hooks), using the `cram-env` interpreter's
  `krrood` (`/home/nasim/.virtualenvs/cram-env/bin/krrood`, if present).

## Scope

- Follow the spec exactly, nothing more or less. Code outside `robokudo_acrambly`,
  `semantic_digital_twin_acrambly`, `test/` and the new robokudo annotator is not
  approved. Never touch `acrambly/`.
- Commits are authored as the git user configured in the repo (AGENTS.md "Version
  control"): no `Co-Authored-By` trailer; a body line `Made with AI assistance` is fine.
  Do not push.
