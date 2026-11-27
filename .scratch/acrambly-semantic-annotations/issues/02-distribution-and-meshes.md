# Distribution and meshes

Status: resolved
Type: task
Blocked by: 01

Create the distribution `semantic_digital_twin_acrambly/`, laid out like
`robokudo_acrambly/`: `pyproject.toml` (depending on `semantic_digital_twin`),
`README.md`, `src/semantic_digital_twin_acrambly/__init__.py`, `_version.py`.
Register it in the root `pyproject.toml` under `dependencies` and
`[tool.uv.sources]`.

Move with `git mv`, keeping file names:

- `meshes/plane/*.stl`, `*.dae` and `Imported Components/` to
  `semantic_digital_twin_acrambly/resources/meshes/aircraft/`;
- `meshes/cube/*.stl` to
  `semantic_digital_twin_acrambly/resources/meshes/tetris_block/`.

Delete the then empty `meshes/` and the obsolete `assembly_models/`. Do not
touch `acrambly/`.

Done when the distribution installs and imports in the workspace environment,
and neither `meshes/` nor `assembly_models/` exists.

## Comments

## Answer

Built in `5219a49e60`, merged in `bdff0d3f27`. The distribution
`semantic_digital_twin_acrambly` depends on `semantic_digital_twin` and
`krrood-core` and is registered in the root `pyproject.toml` (`dependencies`,
workspace `members`, `[tool.uv.sources]`, `[tool.poetry.dependencies]`). It is
installed editable in the workspace environment. The meshes were moved with
`git mv`, and `meshes/` and `assembly_models/` are gone.
`test/semantic_digital_twin_acrambly_test/test_meshes.py` passes.

Open: `resources/` sits outside `src/`, so a wheel does not ship the meshes.
