# 01: Make NOCTIS initialization reusable and empty detections safe

**What to build:** NOCTIS can recognize successive observations without treating a source frame number as an initialization signal. Its public inference interface returns ordinary empty detections when proposals, usable masks, or retained matches are absent, so an empty scene can pass through the later recorded-data engine safely. This is preparatory work for the persistent worker.

**Blocked by:** None (can start immediately).

**Status:** ready-for-human

- [x] Write meaningful failing pytest coverage through NOCTIS's public inference interface before changing the implementation.
- [x] Model and reference initialization can be requested independently of observation numbering. Processing a first observation with a nonzero source identity and then additional observations initializes each model and the references only once.
- [x] A requested reference-descriptor recomputation happens once during initialization; inference does not rebuild descriptors for each observation.
- [x] Zero proposals, zero usable masks after proposal filtering, and zero retained matches each return the public empty-detection result without calling later operations that require a nonempty batch.
- [x] Empty and nonempty results retain compatible identity, score, rectangle, and mask contracts. Empty results contain no invented object detections.
- [x] The existing configured score thresholds, object identity mapping, and best-per-model detection filtering retain their behavior for nonempty input.
- [x] Register deterministic mock-based tests with the existing CI suite. Replace heavyweight model computation and reuse applicable fixtures; no CUDA, checkpoints, private recording, network calls, or credentials are required.


## Comments

- Implemented reusable `NOCTIS.initialize()` and inference independent of source frame numbering. Empty tensor proposals, the configured segmentor's `None` result, filtered masks, and rejected matches retain the public detection contract.
- Deterministic CPU pytest coverage exercises initialization, recomputation and cached references, empty results, saved results, strict configured thresholds, and best-per-model data. Registered a dedicated CPU job in the existing CI workflow using the parent's pinned NOCTIS revision.
- Human follow-up: publish the nested NOCTIS commits before publishing the parent commit so remote CI can fetch the pinned source. No remote push was performed.
- Full workspace pytest was attempted but stopped during collection because the selected interpreter cannot import `cognitive_robot_abstract_machine` during ORM generation. `scripts/regenerate_all_orm.py` failed on the same missing import.
