# 05: Replay the recording completely without skipping observations

**What to build:** The offline engine processes the recording in order at inference speed and terminates at end of input. Timestamp matching preserves valid color-depth associations, and every included color frame is retained and exported with both target outcomes even when that frame cannot support pose estimation.

**Blocked by:** 04: Save stamped metric poses for selected observations.

**Status:** ready-for-agent

- [ ] Write failing engine-level pytest coverage for multi-observation replay and incomplete inputs before implementing the full replay behavior.
- [ ] The default run covers the recording and stops cleanly at EOF without looping. Expose bounded and frame-selected diagnostic runs, preserving recorded identities and order rather than renumbering source observations for model initialization.
- [ ] Process every usable observation through NOCTIS then FoundationPose before advancing to the next observation. Controlled slow model responses demonstrate that inference duration cannot skip source input or overlap outstanding inference requests.
- [ ] Match color and depth one-to-one by acquisition timestamps without a mandatory maximum offset. Never reuse a depth image and do not pair messages solely by position.
- [ ] Protect closer valid associations from an earlier unmatched color or a color lacking compatible calibration. Cover a leading unmatched color, competing timestamps, missing calibration, unused depth, and a valid association whose offset exceeds 2 ms.
- [ ] Preserve integer source timestamps and the signed paired-depth offset at nanosecond precision. Keep color acquisition time as the stamp for successful poses and do not substitute receipt, playback, or elapsed processing time.
- [ ] Produce exactly one exported record per included color frame, with one outcome for each configured target. Missing or incompatible calibration and unpaired color become typed no-pose outcomes without fabricated hypotheses or poses.
- [ ] Report unused depth messages in run diagnostics. Frame selection and bounded runs account for their included input without reporting intentionally excluded frames as processing failures.
- [ ] Expected input and per-target failures allow later observations to finish. Clean EOF leaves all completed included records available for export.
- [ ] Register deterministic CPU tests using small repository-owned or locally generated recorded fixtures and controlled model dependencies, reusing existing fixtures and requiring no GPU, private recording, checkpoints, live API, network, or credentials.
