# 04: Save stamped metric poses for selected observations

**What to build:** Selected recorded observations pass through one NOCTIS annotator followed by one FoundationPose annotator. Each target receives either a stamped camera-relative pose in its original CAD frame or a typed no-pose reason. Robokudo retains those outcomes and exports the same values as JSON Lines in the fixed output location.

**Blocked by:** 02: Replace FoundationPose initialization guesses with typed failures; 03: Recognize both targets in a selected recorded observation.

**Status:** ready-for-agent

- [ ] Write failing engine-level pytest tests before implementation, replacing model computation while exercising real acquisition, preprocessing, annotation, result retention, and export.
- [ ] Reuse the existing FoundationPose annotator and estimator with the two verified target-to-CAD mappings. Run NOCTIS then FoundationPose on the same observation, estimate independently for every processed observation, and disable tracking and reuse of earlier poses.
- [ ] Convert millimetre depth and CAD geometry consistently to metres, applying the agreed mesh scale of 0.001. Preserve the estimator's conversion from centered internal geometry back to the original CAD frame.
- [ ] Verify metric translation and preservation of an offset original CAD origin through controlled fixtures and the public registration boundary, using independently determined expected values.
- [ ] Publish successful results through Robokudo's existing stamped pose annotation in `camera_color_optical_frame`, with translation in metres, finite orientation in xyzw order, and the complete color acquisition timestamp including nanoseconds. Do not apply a world-frame transform.
- [ ] Validate malformed or non-finite estimator output before publishing a pose. Missing detections, invalid masks, typed insufficient filtered depth, and invalid estimates become distinct typed no-pose reasons.
- [ ] Retain observation metadata and exactly one structured outcome per configured target in Robokudo. A target has either its stamped pose or its no-pose reason, without a placeholder pose. Preserve source times, signed depth offset when available, observation and target identities, and recognition score when available; scores do not claim pose confidence.
- [ ] One target's expected failure does not suppress the other target's valid result. Export both outcomes for each selected included color frame and allow a subsequent selected observation to finish.
- [ ] Write result-only JSON Lines into a new run subdirectory beneath the agreed fixed Git-ignored repository-root output location. Use the existing KRROOD dataclass and subclass serialization support rather than a second hand-written result representation, and exclude sensor arrays and the complete scene.
- [ ] Round-trip both successful and failed results and compare exported values with the retained Robokudo outcomes, including typed reasons, full timestamps, camera frame, and quaternion order.
- [ ] Register deterministic tests in the existing CI suite using applicable fixtures, with no GPU, private data, checkpoints, network, or credentials. Fail with an actionable error when required FoundationPose startup configuration, assets, or CUDA are unavailable.
