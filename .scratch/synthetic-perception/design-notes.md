# Synthetic perception design

## Confirmed scope

Build synthetic data and a RoboKudo analysis engine for the two supplied Tetris
blocks. Stop at CAS object hypotheses. Implementing or extending
`SemanticDigitalTwinConnector`, the RoboKudo belief world, `WorldSynchronizer`,
or a receiving world is outside this task.

The implementation must use test-driven development, with meaningful pytest
coverage of the synthetic observations and engine execution.

## Existing domain and source facts

- An analysis engine is a RoboKudo perception pipeline composed of annotators,
  as defined in [the glossary](../../GLOSSARY.md).
- The plan describes scripted observations producing CAS object hypotheses.
- The existing object mapping is model 1 to `child_cube_0_colored.ply` and model 2
  to `child_cube_2_colored.ply`. Their geometry uses millimetres, with a scale of
  `0.001` for metres. Pose results preserve the original CAD frame. See
  [the recorded-perception specification](../orbbec-noctis-foundation-pose/spec.md).
- `ObjectHypothesis`, `Classification`, and `PoseAnnotation` already provide
  the existing CAS representation. The source should reuse these types.
- The existing pipeline supports creating a fresh CAS. The source must arrange
  this for each observation; script progress belongs to the source rather than
  to a previous CAS.
- Later connector reuse requires a plain `PoseAnnotation`; a stamped pose alone
  is insufficient. Its translation is in metres and quaternion order is `xyzw`.
- An enclosing box can have a different origin from its CAD model. If bounding
  boxes are included, their poses must account for that offset.

## Accepted design

### Round 1

1. Emit scripted hypotheses directly, without rendering RGB-D or running inference.
2. Cover both blocks visible, translation and rotation, temporary absence of one
   block, absence of both blocks, and reappearance. Verify the CAS output through
   pytest.

The user accepted both recommendations with “lgtm”.

### Round 2

3. Include classification, plain camera-relative CAD-frame pose,
   mesh-derived bounding box, rectangular image region and binary region mask,
   plus capture time and camera frame. Region masks are synthetic rectangles rather
   than rendered object silhouettes.
4. Use finite deterministic scenarios for tests and repeating playback for the
   demo, with increasing capture timestamps and a configurable frame interval.
5. Include a simple 2D diagnostic image showing regions and labels without
   introducing a world or 3D renderer.

The user accepted all remaining design recommendations with “lgtm”.
Design approval and implementation are separate steps, with a planning commit
before implementation.

Persistent identity and observation acceptance belong to the excluded downstream
work. The scenario representation and class design are implementation details.

## Interview status

The design interview is complete. Verification uses the agreed CAS output
boundary, including execution through the analysis engine. Tests cover
the scenario sequence, geometry, metadata, playback, and diagnostic output.

## Implementation and validation

The engine is `scripted_tetris`; see [the run guide](../../robokudo/scripted_tetris.md).
The CLI smoke test exposed an unconditional world-marker publisher in the existing
runner. Startup now creates that publisher only when the perception world has a
root. This allows the CAS-only engine to start without initializing a belief world.

The focused and adjacent regression run passed all 31 tests, including 18 new
scripted-observation tests and the actual headless CLI invocation:

```bash
.venv-foundation-pose/bin/python -m pytest test/robokudo_test/test_scripted_observations.py test/robokudo_test/test_cas.py test/robokudo_test/test_pipeline.py test/robokudo_test/test_semantic_world_connector.py -q --tb=short --capture=fd
```

The run used the project interpreter after sourcing ROS Jazzy and the existing
ROS workspace overlay. The repository docstring formatter was run on every
modified Python file. The [six-frame diagnostic preview](preview.png) was
generated from the actual analysis engine and inspected visually.
