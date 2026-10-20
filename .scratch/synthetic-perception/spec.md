# Scripted Tetris perception

Implement the accepted [design](design-notes.md) as a synthetic observation source
and a runnable RoboKudo analysis engine. The pipeline ends at CAS object hypotheses.
Connector, belief-world, synchronization, and receiving-world implementation are
outside this task.

## Accepted behavior

- Use the two supplied CAD meshes, preserving their original origins and converting
  millimetres to metres once.
- Emit deterministic scenes covering visibility, translation, rotation, partial
  absence, complete absence, and reappearance.
- Write classification, camera-relative CAD pose, mesh-derived bounding box,
  rectangular image region and binary mask, capture time, and camera frame.
- Create a fresh CAS for every completed observation.
- Support finite playback for tests and repeating playback for the demonstration,
  with configurable speed and increasing capture timestamps.
- Publish a 2D diagnostic image through the CAS and annotator GUI output.

## Verification boundary

Use pytest to execute the actual analysis engine and inspect its CAS and diagnostic
output. Reuse the repository's ROS and blackboard fixtures. Read checked-in meshes;
use a controllable clock instead of waiting. Tests must run in the existing RoboKudo
CI suite without cameras, inference workers, external calls, or credentials.

## Intended entry point

The runnable analysis engine will be named `scripted_tetris`. Add its run guide
with the implementation.
