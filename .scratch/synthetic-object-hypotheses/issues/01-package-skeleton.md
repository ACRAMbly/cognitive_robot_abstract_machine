# Clean robokudo_acrambly package

Status: resolved
Type: task
Blocked by:

Create `robokudo_acrambly/` from scratch: `pyproject.toml`, `README.md`,
`src/robokudo_acrambly/__init__.py`, `_version.py`. Delete the old
`test/robokudo_acrambly_test/` modules that test the removed package, keeping
the shared conftest.

Done when the package imports and the old tests are gone.

## Answer

New `robokudo_acrambly` package (pyproject, README, version); old tests of the removed package deleted, conftest kept and given a shared `clock` fixture.

## Comments
