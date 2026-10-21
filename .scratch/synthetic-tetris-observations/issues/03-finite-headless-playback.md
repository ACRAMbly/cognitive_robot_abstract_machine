# 03: Run finite headless playback without losing observations

**What to build:** A developer can run a configured finite sequence of stationary
block observations headlessly. Each scheduled capture becomes a fresh completed
CAS, even when execution falls behind, and the process exits after the final
observation. Tests control time without real-time waiting.

**Blocked by:** 01: Emit a static observation with metric geometry.

**Status:** in-progress

**Claimed by:** playback implementer on `ticket/synthetic-tetris-03` in
`/tmp/cram-tetris-playback`, based on integration commit `33083169d`.

Implementation requires separate explicit user permission. Follow the approved
synthetic Tetris observation specification, domain glossary, and recorded decision
to preserve every scheduled observation. This ticket can proceed independently
of foreground-mask generation.

- [ ] Structured configuration controls the finite duration and sample period.
  Include both sequence endpoints and emit a sample shared by adjacent interval
  boundaries only once. Playback progress survives successful pipeline iterations.
- [ ] An injectable clock establishes the sequence's starting Unix time. Capture
  times use that start plus scheduled sample offsets, expressed in
  `CAS.data_timestamp` as integer nanoseconds since the Unix epoch. Camera capture
  metadata agrees with this time; `CAS.timestamp` retains its construction-time
  meaning. No production fixed-timestamp control is introduced.
- [ ] Every completed observation receives a fresh pipeline-owned CAS with its
  camera metadata and current hypotheses. Detection identifiers are deterministic
  and distinct for every detection in every observation, including stationary
  observations.
- [ ] Waiting ticks remain running and produce no additional completed
  observation. A completed observation succeeds. Early ticks neither advance
  playback nor duplicate a scheduled capture.
- [ ] Late ticks deliver all overdue observations in chronological order without
  dropping, merging, or replacing them. Their capture times stay on the schedule
  even if delivery takes longer than the configured sequence duration.
- [ ] Exhaustion uses the runner's existing failure-based termination contract
  and an engine tree arrangement that the current headless runner observes.
  The final CAS remains the final completed observation; later ticks cannot
  restart playback or create another observation. Keep shared-world startup
  and launcher import refactoring outside scope.
- [ ] Public-engine pytest coverage uses a controllable clock to exercise regular,
  early, and late ticks, fresh CAS instances, deterministic identifiers,
  scheduled metadata, final-sample uniqueness, and exhaustion. Tests require no
  real-time sleeps or display.
- [ ] Documentation provides exact setup and headless run commands, mesh-directory
  configuration, an adequate runner tick rate for the configured sample rate,
  output access, and waiting, success, and completion semantics. Explain that
  preserving overdue observations can extend actual runtime.
- [ ] Implement test-first with existing fixtures and local mesh resources.
  Run focused checks and relevant existing pipeline tests with the configured
  interpreter and `--orm-build never`, checking and capping memory before each
  pytest run. Format modified Python docstrings using the repository formatter.
