# 02: Replace FoundationPose initialization guesses with typed failures

**What to build:** A target with too little usable depth produces a typed insufficient-depth outcome rather than an initialization guess presented as a successful 6D pose. The existing FoundationPose annotator handles that outcome without publishing a pose and can continue with another target.

**Blocked by:** None (can start immediately).

**Status:** ready-for-human

- [x] Write meaningful failing pytest coverage through FoundationPose's existing public registration interface before changing the implementation, replacing GPU computation deterministically.
- [x] Keep the authoritative valid-masked-pixel check after depth erosion and filtering, and give its current threshold one meaningful estimator-owned named definition without changing its value.
- [x] Below that threshold, registration returns a typed insufficient-depth outcome and no guessed translation or identity-orientation pose is published as success.
- [x] Do not replace or duplicate the authoritative decision with a raw-depth precheck. Include a regression fixture with apparently sufficient raw depth that fails after filtering.
- [x] Exercise the threshold boundary using the production definition rather than retyping its value in tests, and preserve successful registration behavior when sufficient filtered depth remains.
- [x] The annotator recognizes the typed outcome, publishes no pose for the affected target, and permits another target to finish.
- [x] Register CPU-safe tests with the existing CI suite, reuse applicable fixtures, and keep all model, CUDA, checkpoint, private-data, network, and credential requirements out of the mock-based checks.

## Comments

- Registration returns `InsufficientDepth` with its surviving masked pixel count below `FoundationPose.MINIMUM_VALID_DEPTH_PIXELS`, retaining the existing post-filter threshold of 500. The annotator retains the failure in an `InsufficientDepthAnnotation` and continues without publishing a guessed pose.
- Wrote failing public-registration coverage before the estimator change and a failing continuation test before the annotator change. Six deterministic CPU tests cover filtered-depth rejection, masked pixel counting, below/at/above the production threshold, successful pose ranking and CAD-origin conversion, and continuation to another target. Registered them in the CI workflow. The thirteen existing NOCTIS CPU tests also pass.
- Tests run with `.venv-foundation-pose/bin/python`. Mypy passes for the new tests; production-file checks reproduce the same three errors as the starting revision (depth variable type, optional setup timeout, and unannotated detection dictionary). Ran the required docstring formatter on every modified Python file.
- Full workspace pytest stopped during collection because ORM generation cannot resolve `MetaData` for `WorldUpdateReferencesUnknownEntityError`. The prescribed `scripts/regenerate_all_orm.py` fails with the same error; no generated interfaces were accessed or edited.
- Separate standards and specification reviews completed; review corrections restore test import isolation and preserve the framework annotation contract. GPU model quality and kernel behavior are outside these CPU checks.
