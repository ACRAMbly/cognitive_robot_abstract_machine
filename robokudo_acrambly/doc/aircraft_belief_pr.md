## Summary

Accept configured aircraft-part observations as persistent local beliefs, preserving model geometry and physical identity through movement, absence, and reappearance.

```text
CAS hypotheses
  validate labels, captured poses, frames, and geometry before belief changes
  exclude incompatible model kinds from one-to-one association
  create typed mesh-bearing beliefs for unmatched observations
  update retained bodies for accepted matches
  retain absent beliefs and their last accepted state
```

The application owns the twelve aircraft types and replaceable model catalog. Original STL resources supply visual and collision geometry. The generic connector remains independent of aircraft types.

## Evidence

- **Before:** model-incompatible observations could update an existing belief; pose-only mesh updates failed without image masks and retained the old body pose.
  **After:** focused generic regressions pass **149 tests**, and aircraft coverage passes **43 tests**, including all twelve model kinds through movement, absence, and reappearance. Post-review checks repeat **69 generic** and **43 aircraft** cases successfully.
- Full-engine regressions: **2 passed, 2 failed** because the existing remote dataset hostname could not resolve. Existing complete-package collection errors remain: missing `get_scale_coordinates` in RoboKudo and `CrDescriptorFactory` in application package-loading tests.

Exact commands and limitations are recorded in [verification](aircraft_belief_verification.md). The six-stage local pipeline uses fresh public CAS observations, precise capture timestamps, changing detection identifiers, and a camera transform containing rotation and translation.

## Merge Danger

**Door:** two-way

No durable storage migration or external synchronization change. Revert the code and configuration to undo the feature.

**Blast Radius:** perception

Changes affect local belief acceptance and configured model association. Generic bounding-box behavior remains covered by existing connector regressions.
