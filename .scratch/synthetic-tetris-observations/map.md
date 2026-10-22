# Synthetic Tetris observations: design interview

## Notes

- Plan: [synthetic_data_analysis_engine.md](../../synthetic_data_analysis_engine.md).
- Published spec: [spec.md](spec.md), marked `ready-for-agent`.
- Reviewed defaults: [playback-proposal.md](playback-proposal.md), now incorporated
  into the plan.
- The design interview and `to-spec` synthesis are complete. Static observations,
  image evidence, finite playback, motion, and native visualization are
  implemented; visibility remains open.
- Preserve the existing local changes and untracked assets.
- RoboKudo's standard runner imports its shared world before loading the engine:
  [main.py](../../robokudo/src/robokudo/scripts/main.py#L5),
  [world.py](../../robokudo/src/robokudo/world.py#L38).
- Runner shutdown checks the first child, which differs between headless and GUI
  trees: [main.py](../../robokudo/src/robokudo/scripts/main.py#L72),
  [garden.py](../../robokudo/src/robokudo/garden.py#L78).
- Hypothesis IDs have no established cross-observation identity guarantee:
  [core.py](../../robokudo/src/robokudo/types/core.py#L35),
  [pointcloud_cluster_extractor.py](../../robokudo/src/robokudo/annotators/pointcloud_cluster_extractor.py#L384).
- The supplied PLY files have no unit metadata. Existing block dimensions support
  the confirmed millimetre interpretation:
  [cube_perception.py](../../acrambly/sub_parts/real/cube_perception.py#L5).

## Decisions-so-far

- Q1: Preserve every scheduled observation and its scheduled capture time. Emit
  overdue samples in order without dropping them. Recorded in the plan and
  [ADR 0001](../../docs/adr/0001-preserve-scheduled-observations.md).
- Q2: Use the actual block outline for foreground masks. Carry the CAD mesh and
  mesh-derived offset bounding box separately for the later connector's visual
  and collision geometry. Recorded in the plan and
  [ADR 0002](../../docs/adr/0002-use-block-outlines-for-foreground-masks.md).
- Q3: The documented default command runs headlessly. Visualization is opt-in.
- Q4: Explicitly declare millimetres for both supplied meshes, with `0.001` metres
  per mesh unit in the model definitions.
- Q5: Give every detection a new deterministic identifier in every observation,
  keeping model labels separate; persistent object identity and
  cross-observation association belong to the later connector.
- Q6: Keep the work limited to synthetic observations. Existing empty shared-world
  startup is outside scope; the engine creates no beliefs and makes no world changes.
- Q7: Finite headless playback exits after the final observation. An optional
  viewer stays open on the final scene until closed.
- Q8: Continue observations during motion pauses, with unchanged poses, advancing
  capture times, and fresh detection identifiers.
- Q9: Inter-object occlusion and partially hidden blocks are out of scope. Keep
  the supplied scenario's blocks fully in frame and non-overlapping.
- Q10: Accept the proposed camera and poses: a 640-by-480 image, 60-degree field
  of view, and the first block translating ten centimetres and rotating ninety
  degrees while the second remains stationary. Exact poses are in the plan.
- Q11, revised duration: The user requires one thirty-second sequence. Keep ten
  observations per second, with six-second movements and three-second stationary
  and absence intervals. Emit boundary states once and include both sequence
  endpoints, producing 301 observations.
- Q12 clarification: The user needs one sequence. Ordinary capture timestamps
  start at the clock time when that sequence begins; tests control the clock to
  check timestamps without real-time waiting. The plan excludes looping and extra
  return motion.
- Established terminology is recorded in [GLOSSARY.md](../../GLOSSARY.md).
- Testing boundary confirmed: run the public analysis engine and inspect its
  completed CAS observations, including the full thirty-second sequence of 301
  observations. Connector world-update behavior remains in the separate change.
- Published the formal specification as `ready-for-agent` in the local Markdown
  issue tracker: [spec.md](spec.md).

- Resolved [ticket 01](issues/01-static-observation.md): public static engine,
  structured scene/model/camera definitions, configurable mesh resources, metric
  CAD geometry and offset boxes, exact camera/capture metadata, and world
  isolation. Focused engine and existing pipeline validation: 15 passed.
- Resolved [ticket 02](issues/02-projected-foreground-masks.md): cropped binary
  masks from projected CAD triangle surfaces preserve gaps, with exact static
  public-engine image assertions and fixture/supplied-model separation checks.
  Static engine, masks, and existing pipeline validation: 20 passed.
- Resolved [ticket 03](issues/03-finite-headless-playback.md): exact-nanosecond
  finite scheduling preserves every capture, waits without replacing the CAS,
  distinguishes detections across samples, and exposes stable exhaustion to the
  headless runner. Public engine, masks, playback, and pipeline checks: 29 passed.
  The default headless CLI also completed successfully with local mesh fixtures.
- Resolved [ticket 06](issues/06-native-annotator-visualization.md): completed
  captures publish native image overlays and copied camera-relative CAD, box,
  and frame geometry. Existing GUI lifecycle supports final-scene inspection,
  annotator navigation, and native exit. Engine/mask/playback/visualization checks
  passed 50 tests after motion integration; existing pipeline/garden/GUI checks
  passed 14 with Open3D 0.20.0.

- Resolved [ticket 04](issues/04-motion-and-pauses.md): editable visible pose
  keyframes, linear translation, shortest-path rotation, exact endpoints, motion
  pauses, stationary reference poses, and consistent projected/metric geometry.
  Full 301-capture footprints pass with fixture and supplied meshes; combined
  static/mask/playback/motion/pipeline validation: 46 passed.

## Design tree

- Observation contract: hypotheses only, as required by the supplied plan.
  - Detection identity: Q5 settled as identifiers for individual detections.
  - Image masks: Q2 settled as actual block outlines.
    - Q9: Inter-object occlusion and partially hidden blocks are out of scope.
    - The supplied scenario keeps blocks fully in frame and non-overlapping.
  - Model units: Q4 settled.
    - CAD origin preservation and metre/xyzw conventions are required by the plan.
- Playback timeline: Q1 settled.
  - Observations during motion pauses: Q8 settled as continued observations.
  - Q10: Illustrative camera and poses accepted and incorporated into the plan.
  - Q11: Sampling interval, stage boundaries, disappearance, and reappearance
    accepted and incorporated into the plan.
  - One finite sequence ends after reappearance; looping and extra return motion
    are excluded following the user's clarification.
  - Q12: Ordinary capture timestamps start at sequence start; tests control the
    clock.
- Runtime presentation: Q3 settled.
  - Existing shared-world startup is outside scope: Q6 clarified.
  - Finite playback process and viewer completion: Q7 settled.
  - Configuration and public output access follow the runtime decisions.
- Testing boundary: the user confirmed tests through completed engine CAS output.

## Fog

No open design questions remain. The published spec is ready for implementation.
