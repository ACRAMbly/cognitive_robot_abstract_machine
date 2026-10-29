# 04: Prove compatibility with PR #13's aircraft input

Status: resolved
Type: task
Blocked by: 01: Minimal aircraft CAS source; 02: Finite aircraft playback.

Implementation permission: granted by the user's `/implement-spec` request.
Follow the approved [specification](../spec.md) and PR #13's linked observation
contract.

## Outcome

Actual CAS observations emitted by the public aircraft engine satisfy existing
semantic acceptance without masks, regions, boxes, or per-detection meshes.

## Acceptance criteria

- [x] Generate observations through the public aircraft analysis engine rather
  than substituting the existing connector-test sequence for the new source.
- [x] Reuse the public PR #13 acceptance interfaces and existing application
  fixture configuration for all twelve model kinds.
- [x] Preinitialize fixture world/camera frames before the isolation baseline.
  Configure a camera transform containing translation and rotation.
- [x] Verify emitted labels and camera-relative mesh-origin poses resolve to the
  expected semantic types and independently calculated world poses, retaining
  exact integer capture timestamps.
- [x] Cover completed captures across movement, holds, partial absence, total
  absence, and reappearance. Empty CAS observations retain valid frame/capture
  metadata without requiring hypotheses to trigger validation.
- [x] Run compatibility checks with the minimal payload and no preview data.
  Verify the caller-supplied frame configuration is present after CAS replacement.
- [x] Source execution and compatibility preparation leave persistent beliefs,
  world bodies/connections, and semantic annotations unchanged from the baseline.
- [x] Keep persistent belief creation, association, UUID retention, and geometry
  update tests in PR #13's existing suite; this ticket does not add a connector
  stage or duplicate those acceptance tests.
- [x] Establish any missing source behavior with a failing test before fixing
  production code. Preserve existing failing tests and generic acceptance rules.

## Verification

After permission, keep aircraft-specific imports within application tests and
use existing ROS/world/CAS fixtures. Run the new source-compatibility tests and
existing aircraft/semantic acceptance regressions in bounded, serialized batches.
Follow the specification's interpreter, memory, ORM, and formatting requirements.

## Completion record

Implemented and verified against the public observation and acceptance boundaries.
See the answer below for executed checks and environment prerequisites.

## Answer

Added four tests in
[`test_scripted_aircraft_acceptance.py`](../../../test/robokudo_acrambly_test/test_scripted_aircraft_acceptance.py).
They consume actual public-engine observations using the existing ROS, capture,
application semantic mapping, controlled clock, and complete-sequence fixtures.
Fixture-owned frame initialization precedes the persistent-world baseline and
source construction. Every visible part across all 301 captures is accepted
without adding a connector stage or enabling preview.

World translations are checked with the fixture's independently worked
quarter-turn coordinate rule; orientations are composed independently from its
known camera/world pose pair. Public `ObservationCapture` validation runs on
every capture, including total absence, and verifies exact integer capture time
and frame meaning after CAS replacement. Full `BeliefWorldSnapshot` comparisons
prove source execution and preparation preserve existing bodies, connections,
semantic annotations, beliefs, world state, and recorded modifications. CAS
annotations, view keys, capture metadata, and camera transforms also remain
unchanged by preparation.

Each new test passed on its first execution against tickets 01 and 02. No
missing source behavior was found, so no production change or artificial failing
test was introduced. The source's failing-before-fix evidence remains in those
tickets. Existing belief creation, association, and persistent identity tests
were reused as regressions rather than duplicated.

All pytest commands ran from `/tmp/aircraft-ticket-04` through
`/tmp/aircraft-pytest`. The wrapper uses a shared `flock` to serialize runs,
checks free memory and other Python processes, sets `ulimit -v 6291456`, loads
`/tmp/aircraft-env.sh`, and invokes the configured
`/home/nasimpcm/.virtualenvs/cram-env/bin/python -m pytest --orm-build never`.
At least 11,735 MiB was available before each run. The environment loads ROS
Jazzy, current worktree source paths, existing dependency paths, and the
user-approved prerequisites under `/tmp`. No ORM regeneration was required.

- `/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_aircraft_acceptance.py -q --tb=short`:
  first slice **1 passed**, then **2**, **3**, and final **4 passed** as coverage
  was added. Logs: `/tmp/aircraft-04-first-acceptance.log`,
  `/tmp/aircraft-04-world-poses.log`, `/tmp/aircraft-04-capture-metadata.log`,
  and `/tmp/aircraft-04-isolation.log`.
- `/tmp/aircraft-pytest test/robokudo_acrambly_test/test_scripted_aircraft_acceptance.py test/robokudo_acrambly_test/test_aircraft_models.py test/robokudo_acrambly_test/test_aircraft_belief_creation.py test/robokudo_acrambly_test/test_aircraft_observation_sequence.py -q --tb=short`:
  **45 passed, 2 failed** because the local environment lacked `vhacdx`, an
  existing dependency needed by the two persistent mesh-belief tests.
  Log: `/tmp/aircraft-04-application-regressions.log`.
- After restoring that prerequisite under the user-approved `/tmp` directory,
  `/tmp/aircraft-pytest test/robokudo_acrambly_test/test_aircraft_belief_creation.py test/robokudo_acrambly_test/test_aircraft_observation_sequence.py -q --tb=short`:
  **2 passed**. No existing source or test was changed to obtain this result.
  Log: `/tmp/aircraft-04-belief-rerun.log`.
- `/tmp/aircraft-pytest test/robokudo_test/test_semantic_observations.py test/robokudo_test/test_semantic_world_connector.py -q --tb=short`:
  **76 passed**. Log: `/tmp/aircraft-04-semantic-regressions.log`.
- `python scripts/format_docstrings.py test/robokudo_acrambly_test/test_scripted_aircraft_acceptance.py`:
  first pass formatted the file; the second pass was stable with exit status 0.
- `git diff --check`: passed.

The latest integration tip, including optional preview, was merged into this
worktree before completion. Preview remains disabled for these tests. No live
service, display, hardware, credentials, or new application dependency is needed
by the compatibility coverage. Existing ROS DDS sandbox socket diagnostics did
not prevent the test batches from completing.
