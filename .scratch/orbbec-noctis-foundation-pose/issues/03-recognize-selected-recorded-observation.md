# 03: Recognize both targets in a selected recorded observation

**What to build:** A selected observation from the supplied MCAP recording is read directly, calibrated and rectified, then recognized by a NOCTIS annotator using one persistent local worker. Robokudo retains the verified target identities and usable segmentation hypotheses, providing a complete recorded-input-to-recognition path for pose estimation.

**Blocked by:** 01: Make NOCTIS initialization reusable and empty detections safe.

**Status:** ready-for-agent

- [ ] Write failing behavior tests through the Robokudo analysis engine and the public worker request/response contract before implementation. Reuse existing ROS lifecycle and isolated pipeline fixtures.
- [ ] Decode recorded JPEG color and little-endian 16UC1 depth through locally available ROS2 MCAP support, using acquisition timestamps from message headers. Preserve observation identity, camera frame, integer color and paired-depth timestamps, and their signed offset in documented dataclasses.
- [ ] Use the associated recorded color calibration. Treat the supplied depth as already registered to color and do not apply the physical depth-to-color extrinsic again.
- [ ] Rectify color and registered depth onto one pinhole grid using the recorded distortion and resulting intrinsics. Use nearest-neighbor depth resampling, preserve invalid zeros, and retain the recorded camera frame.
- [ ] Supply the correct color representation to Robokudo and explicitly convert to the model's expected input. Controlled image fixtures verify color order, common dimensions, intrinsics, and depth resampling.
- [ ] Use the supplied NOCTIS configuration: prompt `tetris block`, verified object identities ordered 1 then 2, existing level-zero templates and thresholds, including the strict final detection score greater than 0.4. Recompute descriptors once for this final configuration and reuse local model sources, weights, and rendered templates.
- [ ] Keep the accepted runtime boundary: Robokudo runs in the existing Python 3.12 environment and the NOCTIS annotator owns one persistent worker in the existing Python 3.8 environment. Wait for model and reference readiness before inference, allow one outstanding request, correlate responses with observation identities, and release the worker at shutdown.
- [ ] Transfer each retained target's identity, recognition score, pixel rectangle, and binary cropped mask into Robokudo object hypotheses using the verified CAD mapping. Keep the best detection per model and create no hypotheses for undetected targets.
- [ ] Validate image bounds and mask dimensions, convert fractional masks explicitly to binary masks, and retain target-specific diagnostics for invalid masks so subsequent result construction can report them. Empty detection results are ordinary outcomes.
- [ ] Invalid startup configuration, missing required local assets, unavailable required CUDA, readiness failure, a dead worker, and unusable or incorrectly correlated responses stop execution with actionable errors.
- [ ] Register mock-based engine and worker tests in the existing CI suite. Use small repository-owned or locally generated recorded fixtures and a separate CPU worker fixture where subprocess behavior needs exercising; require no private recording, GPU, checkpoints, live API, network, or credentials.
