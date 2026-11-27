# 02: Object hypotheses reach the shared world

Status: claimed
Type: task
Blocked by: 01

**What to build:** each pass of both synthetic engines, aircraft and tetris, ends
with one perception belief per part in the shared world: of the part's class, shown
and colliding with its own mesh in metres, at the pose the scenario scripted, seen
from a camera the scenario places in the world. Another process serving the shared
world finds the parts there. See the spec.

- [ ] Each concrete aircraft part and tetris block class names its mesh file through a path enumeration in `semantic_digital_twin_acrambly`; the two tetris STL files are rewritten in metres.
- [ ] `AircraftPart` and `TetrisBlock` are perceivable.
- [ ] The scenario has a camera pose in the world, defaulting to the identity below the world root; the synthetic interface adds its camera frame there and stores that pose as the camera pose of each pass.
- [ ] The synthetic observation engine has a clock field, defaulting to the system clock, handed to its interface.
- [ ] The synthetic observation engine runs the collection reader, then the annotator of ticket 01 configured with one object class mesh per part class of its scenario, then the world-updating annotator; its docstring says so.
- [ ] Main test, for both engines: a RoboKudo on a stepped clock joins a world another process serves, and in the serving process's copy:
  - [ ] the camera frame is at the scenario's camera pose;
  - [ ] there is exactly one perceivable object per part, still after a later pass;
  - [ ] aircraft parts are of their exact class; tetris blocks are tetris blocks whose class label is their exact block name;
  - [ ] each body's visual and collision shape is its class's mesh;
  - [ ] after the first pass, each body's pose is the camera pose composed with the scripted pose, orientation included;
  - [ ] after a later pass, each body's position follows the scripted path (orientation is not asserted).
- [ ] The mesh tests check that every concrete class names an existing mesh and that every mesh's extent is in metres.
- [ ] The interface tests that asserted no world pose and nothing added to the world, and the engine test that asserted a reader-only pipeline, are rewritten for the new behaviour.

## Comments

2026-11-27, decided by the developer before implementation:

- **Tetris meshes re-centred.** `ObjectMatcher` measures from a belief's centre, which is
  its mesh origin, to the visual centre of the body already in the world. It matches
  only within 0.2 m. The tetris STL origins lie 0.21 m (straight) and 0.26 m (corner)
  from the block centres, so a new body would appear every pass. The two tetris STLs
  are rewritten in metres and re-centred on their bounding-box centre. Nothing changes
  in `robokudo`.
- **Later-pass position.** After a later pass, `PerceptionBelief` moves the body so
  that its visual centre lies at the seen position. The aircraft mesh origins lie
  0.4–3.3 cm off their centres. So after a later pass, the main test asserts that
  `ObjectMatcher.visual_center` of each body equals the camera pose composed with the
  scripted position. It does not assert the body origin.
- **Mesh class property on the bases.** With `IsPerceivable` mixed in, a tetris label
  resolves to `TetrisBlock` itself, and the world update instantiates that class. So
  the mesh class property must not be abstract. `AircraftPart` and `TetrisBlock` each
  declare it without making it abstract. On a base it raises a custom
  `DataclassException`, and every concrete class overrides it.
