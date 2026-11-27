# Matched objects take the perceived orientation

Status: needs-triage
Type: task

`PerceptionBelief.update_object_in_world` (via `_move` in
`robokudo/src/robokudo/perception_beliefs.py`) moves a matched object to the
perceived centre but keeps its old orientation. That was right for boxes fitted to
point clouds. It loses the orientation of a 6D pose estimate, such as the mesh
`Shape` placed at a `PoseAnnotation` (see `docs/adr/0003-pose-estimates-reach-the-world-as-mesh-shapes.md`).
Today a part keeps the orientation it was first seen with.

Wanted: a belief seen with a fitted `Shape` also updates the orientation of the
object it is matched to. Beliefs from a bounding box or points keep today's
behaviour.

Done when a test shows that a turning part seen in a later pass takes that pass's
orientation, and that the synthetic engine tests check the orientation on every
pass, not only the first.

Deferred from the design session of 2026-11-27, which put object hypotheses of the
synthetic scenarios into the shared world. Changes `robokudo`, so it needs the
developer's permission.

## Comments
