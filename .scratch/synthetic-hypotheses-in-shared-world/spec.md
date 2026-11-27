# Synthetic object hypotheses in the shared world

Status: ready-for-agent

## Problem Statement

The synthetic aircraft and tetris analysis engines read scripted observations of
the aircraft parts and the tetris blocks into the CAS of each pass. Nothing they
perceive reaches the shared world, so a coraplex plan has nothing to consume.
Four things stop it today:

- A pass has no camera pose in the world, so `WorldUpdatingAnnotator` leaves out
  the whole pass. The engines also never run that annotator.
- An object hypothesis carries only a classification and a 6D pose. A perception
  belief needs a shape, a 3D bounding box or points, so each hypothesis is
  dropped. RoboKudo's own FoundationPose annotator has the same gap.
- The aircraft parts and tetris blocks are not perceivable, so even with a shape
  every label would resolve to an unknown object.
- The tetris block meshes are in millimetres, while everything else is in metres.

## Solution

Each pass of a synthetic engine ends with one perception belief per part in the
shared world: of the part's class, shown and colliding with the part's own mesh,
at the pose the scenario scripted, seen from a camera the scenario places in the
world. Another process that joins the shared world, such as one running a coraplex
plan, finds the parts there.

Turning a classified 6D pose into a mesh shape is a generic RoboKudo step, as
ADR 0003 records. The ACRAMbly code only supplies the meshes of its parts.

## User Stories

1. As a plan developer, I want each aircraft part a synthetic pass perceives to be in the shared world, so that a coraplex plan can find it without its own perception.
2. As a plan developer, I want each tetris block a synthetic pass perceives to be in the shared world, so that a block-handling plan can find it.
3. As a plan developer, I want each aircraft part in the shared world to be of its exact semantic class, such as top wing or left rear wheel, so that a plan can query for one part type.
4. As a plan developer, I want each tetris block in the shared world to be a tetris block whose class label names its exact block type, so that a plan can tell the straight and the corner block apart.
5. As a plan developer, I want each part's body to carry the part's own mesh as visual and collision shape, so that grasping and assembly reason about the real shape and not a box.
6. As a plan developer, I want each part to enter the world at the pose its first pass scripted, orientation included, so that the world matches what the camera saw.
7. As a plan developer, I want a part to follow the scripted path in later passes, so that the world keeps up with where the part is.
8. As a plan developer, I want one body per part and not one per pass, so that repeated passes update the parts instead of piling up copies.
9. As a plan developer, I want the parts in the shared world to be sized in metres, so that distances, grasps and collisions are physically right.
10. As a plan developer, I want the camera of the scenario to be a frame of the shared world, so that the part poses relate to something in the world.
11. As a plan developer, I want the beliefs and their meshes to reach every process that joins the shared world, so that the plan process sees the same parts as perception.
12. As an ACRAMbly developer, I want both synthetic engines to put their beliefs into the world in the same way, so that the two scenarios differ only in their parts.
13. As an ACRAMbly developer, I want the synthetic object hypotheses to keep carrying only a classification and a 6D pose, so that they stay the same as what NOCTIS followed by FoundationPose leaves in the CAS.
14. As an ACRAMbly developer, I want the scenario to define where its camera sits in the world, with a default at the world root, so that I can place the parts somewhere else without changing the engine.
15. As an ACRAMbly developer, I want each part class and tetris block class to name its own mesh in one place, so that the mapping from class to mesh is written once, next to the meshes.
16. As an ACRAMbly developer, I want the aircraft parts and the tetris blocks to be perceivable, so that their labels resolve to their semantic classes.
17. As an ACRAMbly developer, I want the tetris mesh files themselves to be in metres, so that every user of them, such as FoundationPose and RViz, reads the true size without a scale factor.
18. As a perception developer, I want a generic RoboKudo annotator that turns a classified object hypothesis with a 6D pose into one seen with its class's mesh, so that any mesh-based pose estimate reaches the world.
19. As a perception developer, I want that annotator to be configured with the same object class meshes as the FoundationPose annotator, so that one pipeline configures its meshes once and nothing is reimplemented.
20. As a perception developer, I want the annotator to place the mesh at the pose of the mesh's own frame in the camera optical frame, so that it agrees with what FoundationPose estimates.
21. As a perception developer, I want an object hypothesis whose class has no mesh to pass through the annotator unchanged, so that other hypotheses in the same pass are still handled by the bounding box or points.
22. As a perception developer, I want an object hypothesis without a 6D pose to pass through the annotator unchanged, so that the annotator never guesses a pose.
23. As a perception developer, I want the annotator to know nothing about ACRAMbly, so that a kitchen or any other setup can use it with its own meshes.
24. As a perception developer, I want the annotator to run right before the world update, so that the world update sees the shapes it needs.
25. As a test maintainer, I want to drive a synthetic engine pass by pass on a controlled clock, so that tests reach later passes without waiting for real time.
26. As a test maintainer, I want the main test to check the world as another process serving it sees it, so that the test proves the parts reach the plan's side of the shared world.
27. As a test maintainer, I want the annotator tested with a test mesh and no ACRAMbly code, so that RoboKudo's tests stay independent of ACRAMbly.
28. As a test maintainer, I want the engine test to state the pipeline's order, reader then mesh shapes then world update, so that a reordering fails loudly.
29. As a future developer, I want the deferred orientation update and label-lookup fix recorded as issues, so that the limits of this work are not forgotten.

## Implementation Decisions

- **New RoboKudo annotator, generic.** A new annotator in `robokudo` is configured with a list of the existing object class meshes, the same type the FoundationPose annotator takes, imported from where it lives today. For each object hypothesis whose latest classification names a class with a mesh and that carries a pose annotation, it adds a shape annotation whose geometry is that mesh, with its origin at the pose. Every other hypothesis is left unchanged. It imports nothing from ACRAMbly. This is the decision ADR 0003 records.
- **Meaning of a pose annotation.** A pose annotation is the pose of the mesh's own file frame in the camera optical frame. This holds for the synthetic interface and for FoundationPose alike.
- **World update unchanged.** `WorldUpdatingAnnotator`, the perception belief and the object matcher are not changed. A belief seen with a shape already gets a body with that shape, at the camera pose composed with the shape's origin. A mesh already serializes its geometry, so it crosses the world synchronizer.
- **Meshes per class.** `semantic_digital_twin_acrambly` gets a path enumeration of its mesh files, subclassing krrood's path enumeration. Each concrete aircraft part and tetris block class names its member through a class property. The two left/right rear wheels name the files `RearWheelLeft` and `RearWheelRight`, and the tetris blocks name `child_cube_0_colored` (straight) and `child_cube_2_colored` (corner), as their docstrings already say.
- **Tetris meshes in metres.** The two tetris STL files are rewritten once in metres. No mesh carries a scale factor.
- **Perceivable parts.** `IsPerceivable` is mixed into the aircraft part base class and the tetris block base class. Until the label-lookup follow-up lands, a tetris label may resolve to the abstract tetris block base. That risk is accepted.
- **Camera in the world.** The scenario gets a field for the camera pose in the world, which defaults to the identity below the world root. The synthetic interface stops opting out of adding its camera, and it stores that pose as the camera pose of each pass. RoboKudo's camera interface then hangs the camera frame below the world root at that pose.
- **Engine composition.** The synthetic observation engine runs the collection reader, then the new annotator, configured with one object class mesh per part class of its scenario (class name equal to the label the interface already writes), then `WorldUpdatingAnnotator`. Both the aircraft and the tetris engine inherit this and still differ only in their scenario. The engine's docstring changes from "does nothing else with them" to what it now does.
- **Clock on the engine.** The synthetic observation engine gets a clock field, defaulting to the system clock, which it hands to its interface. Only tests set it. The developer approved this test seam.
- **Scheduled observations.** ADR 0001 still holds: every scheduled pass is delivered with its scheduled capture time, so each one updates the world in order.
- **Packages touched.** `robokudo` (the new annotator, with the developer's permission), `robokudo_acrambly` (camera, engine), `semantic_digital_twin_acrambly` (perceivable parts, mesh enumeration, tetris mesh files). `semantic_digital_twin` and RoboKudo's perception beliefs are not changed.

## Testing Decisions

- A good test drives the behaviour through the highest public seam and asserts what a user of that seam observes: bodies, classes, shapes and poses in the world, or annotations in the CAS. It does not assert which helper ran or how. Expected values come from the definitions the code consumed, such as the scenario's scripted path, its camera pose and the part classes, never from retyped literals.
- **Main seam: synthetic engine into the shared world.** In the `robokudo_acrambly` tests, a RoboKudo runs each real synthetic engine on a stepped clock and joins a world another process serves, using RoboKudo's `shared_world` and `node` fixtures, imported the way the conftest already imports RoboKudo fixtures. The test ticks pass by pass and checks the serving process's copy of the world:
  - the camera frame is in the world at the scenario's camera pose;
  - there is exactly one perceivable object per part, and still one after a later pass;
  - aircraft parts are of their exact class; tetris blocks are tetris blocks whose class label is their exact block name;
  - each body's visual and collision shape is its class's mesh;
  - after the first pass, each body's pose equals the camera pose composed with the scripted pose at that pass, orientation included;
  - after a later pass, each body's position follows the scripted path; orientation is not asserted after the first pass.
  
  Prior art: RoboKudo's process tests joining a served world, and the existing synthetic observation interface tests with their stepped clock.
- **Annotator seam.** In the RoboKudo tests, the new annotator is run on a CAS holding object hypotheses built with the existing hypothesis helper and configured with an object class mesh of a small test mesh. It checks that a classified hypothesis with a pose gets a shape whose geometry is that mesh at that pose, and that hypotheses without a matching mesh, or without a pose, are unchanged. Prior art: the world-updating annotator tests.
- **Mesh seam (existing).** The `semantic_digital_twin_acrambly` mesh tests are extended: every concrete aircraft part and tetris block class names a mesh file that exists, and each mesh's extent is in metres, which covers the tetris rescale. The perceivable mixin is checked through the main seam.
- **Engine composition test (rewritten).** The existing test asserting a reader-only pipeline is rewritten for the new behaviour: reader, then the new annotator configured with the scenario's part meshes, then the world update. It is rewritten because the specified behaviour changed, not to make a failing test pass.
- Every new test runs in CI; none needs credentials or live external calls.

## Out of Scope

- A coraplex plan that consumes the parts. This work ends at the shared world.
- Updating the orientation of a matched object. A part keeps the orientation it was first seen with. Deferred to the perceived-orientation-update issue.
- Making the label lookup prefer the most specific class. Deferred to the most-specific-label-lookup issue.
- Changes to the FoundationPose annotator, the perception belief, the object matcher or `WorldUpdatingAnnotator`.
- A stand-in world server process, RViz, NOCTIS or FoundationPose runs.
- Bolts, nuts and part holders; an ORM interface for `semantic_digital_twin_acrambly`.
- Anything under `acrambly/`.

## Further Notes

- Neighbouring parts of the plane layout stay more than the object matcher's maximum match distance apart, even at their widest drift, so beliefs of different parts are not matched to each other.
- The follow-up issues live in `.scratch/perceived-orientation-update/` and `.scratch/most-specific-label-lookup/`. When the orientation follow-up lands, the main test also checks orientation after later passes. When the label-lookup follow-up lands, the tetris assertions check the exact class.
