# Aircraft feature review

Status: resolved

Reviewed the complete feature using `git diff 812ded39b...HEAD` against
[the approved specification](spec.md), including the later extraction approval
in [the feature map](map.md). The review started at
`c7ef8bbfa1508fc1268a9643014e7d18b2a0bd02`; the fixed point resolves to
`812ded39bcdca29edec8c89c2082236432bd9b87`. Separate Standards and Spec agents
reviewed the nonempty diff.

The reviewed commits, from `git log 812ded39b..HEAD --oneline` at review start:

```text
c7ef8bbfa docs: minor
53ca6e9b9 Document and verify scripted aircraft observations
af0a975c0 Merge aircraft input compatibility coverage
019d70674 Verify scripted aircraft semantic input compatibility
f75eb7921 Merge native aircraft mesh verification
715d38ec6 Add optional aircraft catalog mesh preview
6e2137bbc Merge finite aircraft playback
19a58ac89 Run finite aircraft observations with shared playback helpers
f2a4a5db0 Merge minimal aircraft observation source
2e83e3d11 Add minimal aircraft CAS observation source
```

## Standards

Five findings against `AGENTS.md` were corrected:

1. Document enum members in the shared playback modules, aircraft source,
   modified Tetris visibility boundaries, and aircraft test definitions.
2. Guard the annotation-only `PlaybackTiming` import in `playback/motion.py`
   with `TYPE_CHECKING`. Keep intentional compatibility re-exports intact.
3. Define fixed fixture frame labels with `AircraftFrameLabel`, a `StrEnum`.
4. Put the existing pose-comparison tolerance in the shared
   `AircraftPoseComparison` dataclass fixture instead of repeating its literal.
5. Carry the injected clock's actual initial timestamp in `AircraftCaptures`
   and use it for acceptance expectations instead of constructing an unused
   clock. This was the most consequential standards finding.

The changes preserve source behavior and require no public API renames. Existing
aircraft assertions now consume the shared definitions and clock input. The
follow-up Standards review confirmed **zero remaining findings**, including no
additional baseline code smells.

## Spec

**Zero findings.** The independent source, minimal payload, explicit frame and
capture metadata, complete finite timeline, interpolation, visibility,
compatibility with the public PR #13 acceptance boundary, world isolation, and
optional native mesh preview match the specification. The shared Tetris
extraction is covered by the later approval recorded in the map.

Coverage exercises the 301 captures, exact timing and boundary behavior,
fresh detections, nonidentity camera conversion, geometry preservation, and
native lifecycle. Documentation matches the launcher and separates source
emission from application-owned consumer initialization.

## Executed checks

Pytest commands ran from the checkout root through `/tmp/aircraft-pytest`, using
`/home/nasimpcm/.virtualenvs/cram-env/bin/python`, ROS Jazzy, the existing overlay
and prerequisites in `/tmp`, and current checkout source paths. The wrapper
serializes runs with `flock`, checks free memory and existing test processes,
requires at least 6 GiB available, caps virtual memory with
`ulimit -v 6291456`, and passes `--orm-build never`. More than 9 GiB was available
before each run. No dependency or ORM regeneration was needed.

```bash
/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_aircraft_playback.py test/robokudo_acrambly_test/test_scripted_aircraft_acceptance.py -q --tb=short
```

**35 passed**. Log: `/tmp/aircraft-feature-review-playback.log`.

```bash
COVERAGE_FILE=/tmp/aircraft-feature-review.coverage /tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_aircraft*.py test/robokudo_acrambly_test/test_aircraft*.py -q --tb=short --cov=robokudo_acrambly.aircraft_observations --cov=robokudo_acrambly.playback --cov=robokudo_acrambly.annotators.aircraft_observation --cov=robokudo_acrambly.descriptors.analysis_engines.scripted_aircraft --cov-report=term-missing --cov-report=json:/tmp/aircraft-feature-review-coverage.json --cov-fail-under=85
```

**111 passed**, with **96.52% statement coverage** across the selected aircraft
and shared playback modules; the 85% coverage gate passed. Log:
`/tmp/aircraft-feature-review-aircraft.log`. The JSON coverage report is
`/tmp/aircraft-feature-review-coverage.json`.

```bash
/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_tetris*.py -q --tb=short
```

**17 passed, 33 failed, 13 errors**, matching ticket 05's baseline. All reported
failures/errors have the existing `KeyError: CASViews.COLOR_TO_DEPTH_RATIO` in
the unchanged projection path. Log: `/tmp/aircraft-feature-review-tetris.log`.
These existing failing tests were not modified.

```bash
/tmp/aircraft-pytest test/robokudo_test/test_pipeline.py test/robokudo_test/test_cas.py test/robokudo_test/test_cas_view_codecs.py test/robokudo_test/test_cas_annotation_codecs.py test/robokudo_test/test_garden.py test/robokudo_test/test_gui.py test/robokudo_test/test_behaviours_clear_annotator_outputs.py test/robokudo_test/test_semantic_observations.py test/robokudo_test/test_semantic_world_connector.py -q --tb=short
```

**107 passed**. Log: `/tmp/aircraft-feature-review-generic.log`.

With the same configured interpreter and environment:

```bash
git diff --name-only -z -- '*.py' | xargs -0 /home/nasimpcm/.virtualenvs/cram-env/bin/python scripts/format_docstrings.py
python -m mypy robokudo_acrambly/src/robokudo_acrambly/aircraft_observations robokudo_acrambly/src/robokudo_acrambly/playback robokudo_acrambly/src/robokudo_acrambly/annotators/aircraft_observation.py robokudo_acrambly/src/robokudo_acrambly/descriptors/analysis_engines/scripted_aircraft.py
git diff --check
```

Formatting ran on all nine modified Python files; the final repeated run exited
zero with no further changes (`/tmp/aircraft-feature-review-format-stable.log`).
Whitespace checks passed. Static typechecking remains unavailable:
`No module named mypy` (`/tmp/aircraft-feature-review-typecheck.log`).

The review did not repeat native desktop inspection or the package-wide run.
Native lifecycle checks use mocked desktop boundaries. The preexisting package
import/query fixture failures and absence of a clean full-package result remain
explicit in [ticket 05](issues/05-documentation-and-regressions.md); they were not
treated as new feature findings. The monorepo suite was not run in one invocation.

## Temporary worktree cleanup

Before removal, `/tmp/aircraft-ticket-05` had no tracked or untracked changes,
and `git log HEAD..task/aircraft-05 --oneline` had no unique commits.
`git merge-base --is-ancestor task/aircraft-05 HEAD` succeeded. Executed:

```bash
git worktree remove /tmp/aircraft-ticket-05
git branch -d task/aircraft-05
git worktree list --porcelain
```

Removal succeeded without force. The merged temporary branch was deleted, and
the worktree list contains only the main checkout on
`feature/local-world-belief-updates`. Verification logs and shared `/tmp`
prerequisites were retained.

Standards: five findings resolved, zero remaining; Spec: zero findings.
