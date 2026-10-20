# 08: Validate and document the complete GPU workflow

**What to build:** A perception developer can run the completed recorded-Orbbec analysis engine with one documented command using the agreed local environments. Local evidence establishes model startup, bounded and complete replay behavior, combined GPU memory use, and representative CAD-overlay quality on the target hardware.

**Blocked by:** 05: Replay the recording completely without skipping observations; 06: Preserve completed results across failures and repeated runs; 07: Save projected CAD overlays for selected observations.

**Status:** ready-for-agent

- [ ] Document the offline command with the selected project interpreter, required local ROS environment setup, the existing Python 3.8 NOCTIS worker environment, required assets, diagnostic selection and bounds, default EOF behavior, and the fixed result location.
- [ ] Resolve only packaging and startup problems necessary to execute this engine in the accepted separate runtimes. Keep exactly one NOCTIS annotator followed by one FoundationPose annotator, with sequential GPU computation on each observation.
- [ ] Confirm that the CPU/mock behavior tests from the implementation tickets execute in the existing CI suite without CUDA, private data, checkpoints, live external APIs, network calls, or credentials. Use pytest and the selected project interpreter for validation.
- [ ] Any new behavior or fix found during integration is first demonstrated by a meaningful failing pytest test and retains equivalent CPU/mock coverage. Keep real-model checks conditional on available local hardware and assets.
- [ ] Load both actual model stacks and the final references from available local assets, process representative observations containing both targets, and verify request lifecycle, independent estimation, output identities, stamped poses, and saved results.
- [ ] Measure and record combined peak GPU memory with both model sets resident and the stages executing sequentially on the target RTX 3080 with 10 GiB memory. Sequential execution alone is not accepted as evidence that the models fit.
- [ ] Run a bounded replay before the complete recording. Inspect selected CAD overlays for both targets, checking metric consistency, original CAD origins, rectified image alignment, and the recorded depth-registration assumption.
- [ ] After representative and bounded validation succeeds, run the complete recording, verify clean EOF and incremental output, reconcile the exported color-frame and unmatched-input counts with the inspected recording, and inspect representative saved overlays.
- [ ] Record the commands, environments, hardware, observed startup or compatibility findings, memory measurements, frame and outcome counts, elapsed processing time, and visual quality findings. Do not claim a numerical pose-error guarantee or a real-time throughput requirement.
