# 07: Save projected CAD overlays for selected observations

**What to build:** Selected successful recorded observations produce saved CAD projections over their rectified color images, allowing a perception developer to inspect the initial 6D pose quality. The images use the same geometry, pose, image grid, and calibration as estimation.

**Blocked by:** 04: Save stamped metric poses for selected observations.

**Status:** ready-for-agent

- [ ] Write a failing engine-level pytest test for a selected overlay using controlled recorded input, a known pose, and known calibration before implementing projection.
- [ ] Save selected diagnostic images with their run output and preserve observation and target identity so each overlay can be associated with its retained and exported pose.
- [ ] Project the original CAD geometry with the agreed metric scale and its original CAD frame, using the same stamped camera-relative pose, rectified color grid, and pinhole intrinsics as estimation.
- [ ] Controlled fixtures verify expected projection, including geometry whose CAD origin differs from its geometric center, without copying the production projection calculation into test expectations.
- [ ] Only successful target poses are projected. Missing detections and typed no-pose outcomes remain visible in structured results without a fabricated CAD projection.
- [ ] Overlay generation leaves the retained and exported poses, timestamps, target identities, and intrinsics unchanged, and selection does not prevent other included observations from producing their ordinary result records.
- [ ] Register deterministic CPU projection coverage in the existing CI suite with model computation replaced. Learned-model quality is reserved for local visual review and is not claimed by these tests.
