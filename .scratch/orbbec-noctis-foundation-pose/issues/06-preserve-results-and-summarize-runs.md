# 06: Preserve completed results across failures and repeated runs

**What to build:** Completed pose records remain reviewable when processing is interrupted or infrastructure fails, and a later run preserves earlier output. Each run reports its processed inputs, target outcomes, unmatched inputs, and elapsed processing time, including a partial summary when shutdown can complete.

**Blocked by:** 04: Save stamped metric poses for selected observations.

**Status:** ready-for-agent

- [ ] Write failing engine-level pytest coverage for repeated runs, interruption, fatal execution failure, and result summaries before changing persistence behavior.
- [ ] Each run uses a separate subdirectory under the agreed fixed Git-ignored output location, without overwriting earlier results or requiring an output-directory option.
- [ ] Flush completed JSON Lines records incrementally so they remain readable before normal EOF and after a later fatal failure. Persist only complete result records.
- [ ] Invalid configuration, missing required models or CUDA, worker readiness failure, and a worker dying or becoming unusable during processing stop the run with actionable errors. Do not fabricate a successful result or model outcome for an unfinished observation.
- [ ] Preserve completed records after fatal failure and interruption, and release the worker and other owned resources during shutdown when execution permits it.
- [ ] Write a summary containing included and completed frame counts, typed target-outcome counts, unmatched input counts, and elapsed processing time. Compute counts from the same observation results and input diagnostics used by the engine.
- [ ] Write a partial summary identifying incomplete execution when shutdown can run; do not claim that an interrupted or failed run completed the recording.
- [ ] Tests compare persisted completed records and summaries with observed engine results for normal completion, a target failure, fatal worker failure after completed work, interruption, and a second run preserving the first.
- [ ] Register mock-based lifecycle and persistence tests with the existing CI suite and reuse applicable fixtures. These checks need no GPU, private recording, checkpoints, database-backed storage, network calls, or credentials.
