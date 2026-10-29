# 05: Document aircraft usage and verify retained behavior

Status: resolved
Type: task
Blocked by: 01, 02, 03, 04.

Implementation permission: granted by the user's explicit ticket 05 implementation
request. Follow the approved
[specification](../spec.md); completion records must describe executed checks.

## Outcome

Developers can run the aircraft source headlessly, opt into mesh verification,
and understand the minimal CAS output and separate PR #13 consumer setup.

## Acceptance criteria

- [x] Make `scripted_aircraft` the primary synthetic example in the package
  README while retaining the working Tetris command and guides. Document separate
  commands and configuration for the independent engines; no combined mode is
  introduced.
- [x] Document exact configured-interpreter/ROS setup, engine selection, required
  frame configuration, capture cadence, tick rate, and finite completion.
- [x] Document optional preview activation, explicit mesh-directory setup using
  `plane/` as a checkout example, and final-scene inspection/exit behavior.
- [x] Describe labels, mesh-origin camera poses, detection IDs, scheduled capture
  timestamps, frames, and transform fields. Explain that image evidence and
  per-detection mesh/box annotations are absent.
- [x] Distinguish source emission from PR #13 consumption: application-owned frame
  initialization and catalog semantic/geometry mappings are consumer prerequisites;
  the source-only engine creates no beliefs or world objects.
- [x] Document editing poses, motion, holds, absence membership, and timing in
  structured configuration without copying the model inventory into a new source
  of truth.
- [x] Run new aircraft source/compatibility/preview coverage and relevant existing
  Tetris, pipeline, CAS, and native lifecycle regressions in focused batches.
- [x] Exercise finite headless completion with documented configuration; perform
  any native visual inspection only in an available display environment and
  report whether it was performed.
- [x] Record exact commands, results, formatter status for modified Python files,
  and preexisting failures or unavailable dependencies. Report only observed
  success, and leave unresolved checks explicit.

## Verification

After permission, follow the specification's TDD, memory, interpreter, ORM, and
formatting requirements. Tests require no live services or credentials. Use the
existing package CI entry, retaining always-running local-fixture coverage.
Review documentation against actual implemented configuration and public output.

## Completion record

Delivered documentation and launcher configuration after tickets 01–04.
Executed checks and unresolved baseline failures are recorded below; completion
does not assert that the existing package suite passes.

## Answer

The package [README](../../../robokudo_acrambly/README.md) now leads with
`scripted_aircraft`. The [usage guide](../../../robokudo_acrambly/doc/scripted_aircraft.md)
covers ROS Jazzy, the configured Python 3.12 interpreter, frame calibration,
independent engine selection, capture/tick timing, finite completion, editable
scenario descriptors, minimal CAS output, optional catalog preview, native exit
controls, and separate application-owned PR #13 consumer prerequisites. Existing
Tetris commands, guides, implementation, fixtures, and tests were retained.

Ticket 03 deferred launcher wiring here. `AIRCRAFT_FRAME_CONFIGURATION` now
selects an explicit JSON calibration document; the documented
[`aircraft_frames.json`](../../../robokudo_acrambly/doc/aircraft_frames.json)
supplies an illustrative translated camera pose. Missing configuration raises
`MissingAircraftFrameConfiguration`. The existing public pose validation rejects
missing translation/orientation instead of acquiring spatial-constructor defaults.
`AIRCRAFT_MESH_DIRECTORY` explicitly enables catalog preview; without it, the
headless source loads no meshes. Explicit programmatic frames/preview remain
supported. Launcher frame references remain detached from the belief world.

The user confirmed `ModuleLoader.load_ae(...).implementation()` as the new test
boundary. Thirteen configuration cases cover documented and rotated frame input,
preview disabled/enabled, missing configuration/components, and world isolation.
Preview fixtures reuse existing local mesh resources and need no display or live
services. The existing package CI matrix entry remains unchanged.

### Test-first evidence

Each command used `/tmp/aircraft-pytest` with
`test/robokudo_acrambly_test/test_scripted_aircraft_configuration.py -q --tb=short`:

- Before configuration support: collection failed with the missing configuration
  module; after implementation, **1 passed**. Logs:
  `/tmp/aircraft-05-configuration-red.log`, `/tmp/aircraft-05-configuration-green.log`.
- Before optional mesh-directory wiring: **1 failed, 1 passed**, with no native
  geometry; after wiring, **2 passed**. Logs:
  `/tmp/aircraft-05-preview-red.log`, `/tmp/aircraft-05-preview-green.log`.
- Before the missing-configuration exception: collection failed with the missing
  exception module; after implementation, **3 passed**. Logs:
  `/tmp/aircraft-05-missing-configuration-red.log`,
  `/tmp/aircraft-05-missing-configuration-green.log`.
- Missing camera pose components initially produced **2 failed, 3 passed**;
  existing public pose validation then produced **5 passed**. Logs:
  `/tmp/aircraft-05-incomplete-calibration-red.log`,
  `/tmp/aircraft-05-incomplete-calibration-green.log`.
- Rotated-input and world-isolation coverage passed on its first execution:
  **11 passed**, `/tmp/aircraft-05-configuration-final.log`. Review separated the
  preview-disabled assertion into its own test, yielding thirteen cases in the
  final aircraft batch below. No artificial failure was introduced for existing
  correct behavior, and no existing failing test was edited.

### Executed regression batches

All pytest invocations ran from the repository root through `/tmp/aircraft-pytest`.
That wrapper acquires `/tmp/aircraft-pytest.lock`, checks free memory and existing
Python test processes, requires at least 6 GiB available, sets
`ulimit -v 6291456`, sources `/tmp/aircraft-env.sh`, and invokes
`/home/nasimpcm/.virtualenvs/cram-env/bin/python -m pytest --orm-build never`.
Runs were serialized. Recorded available memory exceeded 12 GiB for the final
focused batches. The environment loads ROS Jazzy, the existing temporary ROS
overlay and prerequisites, current checkout sources, and existing dependency
paths. No dependency was added, and ORM regeneration was not needed.

```bash
/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_aircraft*.py test/robokudo_acrambly_test/test_aircraft*.py -q --tb=short
```

**111 passed**, including thirteen launcher cases, complete playback and PR #13
compatibility, local-fixture and actual `plane/` preview resources, native
lifecycle coverage with desktop boundaries mocked, and existing aircraft catalog
and persistent-belief regressions. Log: `/tmp/aircraft-05-final-aircraft.log`.

```bash
/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_tetris*.py -q --tb=short
```

**17 passed, 33 failed, 13 errors**. Every reported failure/error has the existing
`KeyError: CASViews.COLOR_TO_DEPTH_RATIO` in the unchanged Tetris projection
path. Retained Tetris image, geometry, playback, and native visual assertions
remain unresolved where this prevents execution. Log: `/tmp/aircraft-05-tetris.log`.

```bash
/tmp/aircraft-pytest test/robokudo_test/test_pipeline.py test/robokudo_test/test_cas.py test/robokudo_test/test_cas_view_codecs.py test/robokudo_test/test_cas_annotation_codecs.py test/robokudo_test/test_garden.py test/robokudo_test/test_gui.py test/robokudo_test/test_behaviours_clear_annotator_outputs.py test/robokudo_test/test_semantic_observations.py test/robokudo_test/test_semantic_world_connector.py -q --tb=short
```

**107 passed**. Log: `/tmp/aircraft-05-generic-regressions.log`.

The initial five-file aircraft baseline also passed **55 tests** before launcher
changes (`/tmp/aircraft-05-aircraft.log`). The package-wide check was attempted
without running the monorepo suite in one invocation:

```bash
/tmp/aircraft-pytest test/robokudo_acrambly_test -q --tb=short
/tmp/aircraft-pytest test/robokudo_acrambly_test --continue-on-collection-errors -q --tb=short
/tmp/aircraft-pytest test/robokudo_acrambly_test/test_robokudo_cram_integration.py::test_new_query_can_reach_query_annotator_during_initialization -q --tb=short
```

The first command stopped with **1 collection error**: existing
`test_package_loading.py` imports missing `CrDescriptorFactory` from
`robokudo.descriptors`. The continuation produced **43 passed, 1 failed, 140
errors**: the existing query initialization test returned `RUNNING` instead of
`SUCCESS`, and its teardown left a `None` query server, causing subsequent
blackboard fixture errors. A fresh isolated run of that existing query test
reproduced **1 failed, 1 teardown error** without collecting any aircraft test.
These existing sources/tests were not changed. Logs:
`/tmp/aircraft-05-application-regressions.log`,
`/tmp/aircraft-05-application-continued.log`, `/tmp/aircraft-05-existing-query.log`.
The fresh focused batches above avoid that teardown contamination; a clean full
application-suite result remains unresolved.

### Headless execution, formatting, and review

With `/tmp/aircraft-env.sh` loaded and the same configured interpreter, free-memory
check, and 6 GiB virtual-memory cap, executed:

```bash
export AIRCRAFT_FRAME_CONFIGURATION="$PWD/robokudo_acrambly/doc/aircraft_frames.json"
unset AIRCRAFT_MESH_DIRECTORY
timeout 50 python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae scripted_aircraft _headless _tickrate 20
```

The documented default thirty-second run **exited 0 before the timeout**, without
mesh resources or window processing. Log: `/tmp/aircraft-05-headless.log`.
The final-code repeat also **exited 0 before the timeout**
(`/tmp/aircraft-05-headless-final.log`).
ROS DDS sandbox socket diagnostics occurred but did not prevent the completed
headless run or passing focused batches. `timeout 5 xdpyinfo` reported
`unable to open display ":0"`; **no native desktop visual inspection was
performed**. Native lifecycle and geometry were verified by the tests above.

```bash
python scripts/format_docstrings.py robokudo_acrambly/src/robokudo_acrambly/aircraft_observations/configuration.py robokudo_acrambly/src/robokudo_acrambly/aircraft_observations/exceptions.py robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py test/robokudo_acrambly_test/test_scripted_aircraft_configuration.py
python -m compileall -q robokudo_acrambly/src/robokudo_acrambly/aircraft_observations/configuration.py robokudo_acrambly/src/robokudo_acrambly/aircraft_observations/exceptions.py robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py test/robokudo_acrambly_test/test_scripted_aircraft_configuration.py
python -m mypy robokudo_acrambly/src/robokudo_acrambly/aircraft_observations/configuration.py robokudo_acrambly/src/robokudo_acrambly/aircraft_observations/exceptions.py robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py test/robokudo_acrambly_test/test_scripted_aircraft_configuration.py
git diff --check
```

The formatter initially changed files; its final repeated run **exited 0** with
no changes (`/tmp/aircraft-05-format-final.log`). Compilation and whitespace checks
passed. Static typechecking **could not run**: the configured interpreter reports
`No module named mypy` (`/tmp/aircraft-05-typecheck.log`); compilation is not a
substitute for that unresolved check.

The code-review skill ran separate Standards and Spec reviews against starting
commit `af0a975c046ee10e8df931af807b6fe3e5c3fefb`. Six localized standards findings
were corrected and re-reviewed; **no remaining Standards or Spec findings** were
reported. Preexisting edits to `AGENTS.md` and earlier feature-map records were
preserved.
