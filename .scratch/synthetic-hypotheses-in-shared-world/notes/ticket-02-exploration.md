# Ticket 02 exploration: object hypotheses reach the shared world

Repo root `R=/home/nasim/Desktop/cognitive_robot_abstract_machine`. Branch
`feature/synthetic-hypotheses-in-shared-world`. **Ticket 01 has already landed on this branch**
(commit `6fcb6923f0`): `R/robokudo/src/robokudo/annotators/mesh_shape.py`,
`MeshShapeAnnotator(BaseAnnotator)`, `@dataclass(kw_only=True, eq=False)`, ctor
`MeshShapeAnnotator(class_meshes=[ObjectClassMesh(...)])` (`class_meshes: InitVar[list[ObjectClassMesh]]`,
stored as `object_class_meshes: ObjectClassMeshes` -> `.meshes_by_class_name: dict[str, ObjectClassMesh]`).
It sets the shape origin with `HomogeneousTransformationMatrix.from_xyz_quaternion(*translation, *rotation)`
of the latest `PoseAnnotation` and appends `Shape(geometry=replace(class_mesh.shape, origin=...))`.

## 0. Blocking findings - ask the developer before coding (verified by simulation)

Simulation script: `<scratchpad>/scripts/simulate.py` (runs PerceptionBelief + ObjectMatcher +
add/update exactly as `WorldUpdatingAnnotator` does, on the real scenarios and meshes; tetris rescaled
x0.001 into the scratchpad, nothing in the repo touched).

**A. Tetris parts are never matched from pass to pass -> a new body every pass.**
`PerceptionBelief.center` (perception_beliefs.py:200-205) is the position of `pose` = `world_T_camera @
shape.origin`, i.e. the *mesh file origin*, while `ObjectMatcher.visual_center` (:373-386) of the object
already in the world is the centre of its visual bounding box. Distance = |p_k - (p_{k-1} + R_first d)| ~ |d|,
d = bbox centre of the mesh in its own frame. Max match distance is `ObjectMatcher.maximum_match_distance
= 0.2` m (perception_beliefs.py:305). Tetris meshes are not only in mm, they are far off-centre:
straight |d| = 0.206 m, corner |d| = 0.265 m after /1000 -> both > 0.2. Simulation: tetris pass 0 -> 2
objects, pass 1 -> 4 objects, 0 matched. Aircraft |d| <= 0.033 m -> all 12 matched every pass.
"Rewrite once in metres" (spec) is therefore not enough; options for the developer: re-centre the tetris
STLs (changes the frame FoundationPose estimates too) or fix `PerceptionBelief.center` for meshes (robokudo
change, spec says perception belief unchanged). Spec's Further Notes only considered grid spacing.

**B. After a later pass the body origin is not at the scripted position.** `_move`
(perception_beliefs.py:250-268) sets origin = old origin + (belief.center - visual_center), so the
*visual bbox centre* lands on the scripted mesh-origin position, and body origin = p_k - R_first d.
Simulation (aircraft): |origin - scripted| = |d| per part (0.0035 Chassis ... 0.0334 RearWing),
|visual_center - scripted| = 0.0000 exactly. The ticket's "each body's position follows the scripted
path" can thus only be asserted honestly as `ObjectMatcher.visual_center(body=...)` == camera pose @
scripted position, unless meshes are re-centred (d = 0, then origin == scripted position too). Ask which.
After pass 0 (add_to_world, :209-224) the body origin == `world_T_camera @ scripted pose` exactly.

**C. `TetrisBlock` must stay instantiable.** With `IsPerceivable` on the base, `annotation_class_by_label`
returns the *base* for "StraightTetrisBlock" (verified with a mimic hierarchy, `<scratchpad>/scripts/lookup.py`:
`recursive_subclasses` lists direct subclasses first, and the query returns the first match). The belief then
does `TetrisBlock(root=Body(...), class_label=...)` (perception_beliefs.py:122-129). If the mesh class
property is declared `@classproperty @abstractmethod` on `TetrisBlock`, that raises
`TypeError: Can't instantiate abstract class ... 'mesh_file'` inside the world update. Aircraft labels resolve
to the concrete classes (no IsPerceivable class name is a substring of any part label; checked at runtime:
perceivables are Cup, Bowl, CheezeIt, Candy, Noodles, Cereal, Milk, SaltContainer, UnknownObject, Spoon,
Human, SaltPepperShaker). AGENTS.md wants an operation callers depend on declared in a base class -> ask how to
declare it without making `TetrisBlock` abstract (or accept a non-abstract base declaration).

**D. World without a root.** `bind_world_T_camera` needs a reference frame *name*. In a world of its own
(RoboKudo with no server; `Pipeline(children=[reader])` in the interface tests) `world.root is None`, and
bind then creates a root body named after `reference_frame`. Spec only says "identity below the world root".
Prior art for a name: `StaticCameraTransformComponent.static_world_frame = "map"`
(robokudo/descriptors/camera_configs/components.py:110-131). Decide/ask what frame an empty world gets.

**E. Which class does each part's mesh come from?** `ScenarioPart` carries only `label: str`
(scenario.py:104-118); `PlaneLayout.arrange(part_types=...)` (scenario.py:177-215) drops the types after
`label=part_type.__name__`. Engine must configure "one object class mesh per part class of its scenario";
either the part keeps its annotation class (e.g. a `part_type` field, label derived) or the engine maps.

## 1. robokudo_acrambly synthetic observations today

`R/robokudo_acrambly/src/robokudo_acrambly/synthetic_observations/`
- `scenario.py`: `PartPath` (36-101) `pose_at(self, *, elapsed: timedelta) -> HomogeneousTransformationMatrix`
  (pose in the *camera optical frame*): position = centre + drift_direction * amplitude*sin(2pi t/period + phase),
  rotation = axis-angle(turn_axis, initial_heading + turn_rate*t). `ScenarioPart(label, path)` (104-118).
  `PlaneLayout` (124-215): plane_height 0.25 (below optical axis, +y), nearest_depth 1.6, spacing 0.45,
  columns 4, drift_amplitude 0.06, drift_period 12 s, turn_rate 10 deg/s; `vertical_axis` classproperty
  `Vector3(0,-1,0)`. `Scenario` (221-253): `parts: list[ScenarioPart]`, `schedule: PassSchedule`,
  `camera_frame: str = "synthetic_camera_optical_frame"`, `camera_model: PinholeCameraModel` (640x480, f=525).
  **No camera pose field yet** -> add one (default identity, `field(default_factory=HomogeneousTransformationMatrix)`,
  as components.py:126 does).
- `camera_interface.py`: `@dataclass(kw_only=True, eq=False) class SyntheticObservationInterface(CameraInterface)`
  (39-122). Fields `camera_config: SyntheticCameraConfig`, `clock: Clock = field(default_factory=SystemClock)`,
  `playback: Playback = field(init=False)`; `__post_init__` calls `super().__init__(self.camera_config)`.
  Docstring 42-48 says "no camera pose in the world. Nothing is added to the world." -> update.
  Opt-out to remove: 78-82
  ```python
  def adds_missing_camera(self, *, world: World) -> bool:
      """Unlike :meth:`CameraInterface.adds_missing_camera`, never adds the camera."""
      return False
  ```
  `set_data(self, *, cas, world)` (84-99): `take_next_pass()`, then
  `self.store_camera_observation(cas=cas, world=world, camera_model=..., camera_frame=scenario.camera_frame,
  timestamp_nanoseconds=scheduled_pass.capture_time_nanoseconds, modalities=(), world_T_camera=None)`, then one
  hypothesis per part. `create_object_hypothesis` (101-122): `Classification(source=type(self).__name__,
  classname=part.label, confidence=1.0)` + `PoseAnnotation(translation=pose.position.to_np()[:3].tolist(),
  rotation=pose.quaternion.to_np().tolist())` (x,y,z,w). Keep these two only (user story 13).
  `SyntheticCameraConfig(BaseCameraConfig)` (128-148): `registry_name` classproperty, `interface_type`, `scenario`.
- `analysis_engine.py`: `@dataclass(kw_only=True) class SyntheticObservationEngine(AnalysisEngineInterface)`
  (177-205), field `scenario: Scenario`; docstring "reads the passes ... and does nothing else with them"
  -> change. `implementation()` builds `Pipeline(self.name())` with `[pipeline_init(),
  CollectionReaderAnnotator(descriptor=CollectionReaderAnnotator.Descriptor(camera_config=camera_config,
  camera_interface=SyntheticObservationInterface(camera_config=camera_config)))]`. **No clock field** -> add
  `clock: Clock = field(default_factory=SystemClock)` and pass `clock=self.clock` to the interface; append
  `MeshShapeAnnotator(class_meshes=...)` and `WorldUpdatingAnnotator()`.
  `AnalysisEngineInterface` is a `Protocol` (robokudo/analysis_engine.py:17) - existing, leave it.
- `playback.py`: `PassSchedule(duration=30 s, pass_period=100 ms)`, `pass_count`, `elapsed_at(*, pass_index)`;
  `ScheduledPass(index, elapsed, capture_time_nanoseconds)`; `Clock(ABC).now_nanoseconds()`; `SystemClock`;
  `Playback(schedule, clock)`: first `is_next_pass_due()` sets `start_nanoseconds = now`; pass i due when
  now >= start + elapsed_at(i); `take_next_pass()` hands out one pass per call even when late (ADR 0001).
- `exceptions.py`: `NoPassDue(DataclassException)` - put new ones here.
- Engines: `descriptors/analysis_engines/synthetic_aircraft.py` / `synthetic_tetris.py`:
  `@dataclass(kw_only=True) class AnalysisEngine(SyntheticObservationEngine)` with
  `scenario = field(default_factory=lambda: Scenario(parts=PlaneLayout().arrange(part_types=concrete_subclasses(AircraftPart|TetrisBlock))))`,
  `name()` -> "synthetic_aircraft"/"synthetic_tetris". `ModuleLoader().load_ae` instantiates with no args.
- robokudo_acrambly deps (pyproject): robokudo, semantic_digital_twin, semantic_digital_twin_acrambly,
  krrood-core, krrood_spatial_math, numpy, typing_extensions. No core-boundary section.

## 2. RoboKudo camera interface base (`R/robokudo/src/robokudo/io/camera_interface.py`)

- `class CameraInterface(object)` (91); `__init__(self, camera_config)` (106).
- `adds_missing_camera(self, *, world) -> bool` (133-140) returns True (the opt-out hook).
- `bind_world_T_camera(self, *, world, reference_frame: str, camera_frame: str, world_T_camera) -> HTM`
  (142-204): finds bodies by ROS frame name (`find_bodies_for_ros_frame`, utils/ros_frames.py:36-55: full
  `str(body.name)` match, else unprefixed name). Raises `UnknownWorldFrameError` if world has a root but no
  body for `reference_frame`. Missing camera body + `adds_missing_camera` -> `_standalone_mount` (221-265)
  creates (if needed) reference body, camera body and `Connection6DoF` named
  `robokudo_camera_mount_<camera id>` (`standalone_mount_prefix` classproperty, 99-104), then sets
  `mount.origin = HTM(data=world_T_camera, reference_frame=mount.parent, child_frame=mount.child)` inside
  `world.modify_world()`. Later passes reuse the mount (`is_mounted_standalone`). The input matrix is copied,
  not mutated. Empty world: reference body becomes the root (test_camera_interface_placement.py:60-76).
- `static_world_T_camera_if_configured(self, *, world, camera_frame)` (288-306): prior art of a configured
  fixed pose bound via `bind_world_T_camera(reference_frame=self.camera_config.static_world_frame, ...)`;
  used by `FileReaderInterface.set_data` (io/file_reader_interface.py:261-273): bind first, then
  `store_camera_observation(..., world_T_camera=world_T_camera)`. Same order as test helper
  `place_camera_of` (test/robokudo_test/_perception_world.py:161-199).
- `store_camera_observation(self, *, cas, world, camera_model, camera_frame, timestamp_nanoseconds,
  modalities, world_T_camera: HTM | None)` (308-354): resolves/creates `Camera` annotation for the body of
  `camera_frame` (`_resolve_or_create_camera`, 356-398), builds `CameraObservation`, `_register_camera`
  (400-429; skipped when `adds_missing_camera` is False) adds Camera annotation (+ body and free mount if the
  body is new), sets `cas.camera_observation`.
- Reference frame name for "below the world root": `str(world.root.name)` (`PrefixedName.__str__` = name
  when no prefix). Shared-world fixture root is `Body(name=PrefixedName("map"))` (`FrameName.WORLD`).
- `CameraObservation` (robokudo/types/camera.py:28-130): fields `camera`, `effective_camera_model`,
  `world_T_camera: HTM | None`, `timestamp_nanoseconds`; `__post_init__` requires `reference_frame` set;
  `world_T_camera_or_raise()`. CAS: `CASViews.CAMERA_OBSERVATION = "camera_observation"` (cas.py:72),
  property `cas.camera_observation` (cas.py:196-207).

## 3. World update and perception beliefs (unchanged by spec)

- `R/robokudo/src/robokudo/annotators/world_updating.py`: `@dataclass(kw_only=True, eq=False)
  class WorldUpdatingAnnotator(BaseAnnotator)`, field `minimum_class_confidence: float = 0.5`;
  `update()` (40-89): no camera pose / no reference frame -> warning "Leaving out a pass without a camera
  pose in the world." and SUCCESS; reference frame not in world -> `CameraPoseOutsideWorldError`; else in one
  `modify_world()`: `PerceptionBelief.from_hypothesis(hypothesis, world_T_camera, minimum_class_confidence)`
  per `ObjectHypothesis`, `ObjectMatcher().match(...)`, `add_to_world` / `update_object_in_world`.
- `R/robokudo/src/robokudo/perception_beliefs.py`:
  - `from_hypothesis` (81-132): needs latest `robokudo.types.annotation.Shape` (else 3D bbox, else >= 4
    points, else warning + None). `shape = replace(seen_geometry, origin=identity)`; `visual =
    ShapeCollection([shape])`; `annotation = semantic_class(root=Body(visual=visual,
    collision=visual.copy_without_reference_frame()), class_label=hypothesis.class_label)`;
    `pose = world_T_camera @ seen_geometry.origin`; `scale = seen_geometry.local_frame_bounding_box.scale`.
  - semantic class: `ObjectHypothesis.determine_semantic_class` (robokudo/types/scene.py:100-118) ->
    `annotation_class_by_label(label)` (semantic_digital_twin/reasoning/queries.py:166-190): first
    `recursive_subclasses(IsPerceivable)` whose lowercased name is *contained in* the lowercased label;
    None -> `UnknownObject`. Confidence < 0.5 -> `UnknownObject`.
  - `add_to_world` (209-224): `Connection6DoF` from `pose.reference_frame` (world root) to the body, origin
    = pose. `update_object_in_world` (226-248): `_move` keeps orientation and shape; keeps class unless
    the object was `UnknownObject`.
  - `ObjectMatcher` (292-427): `maximum_match_distance = 0.2`, `maximum_match_size_ratio = 2.0`,
    `class_mismatch_penalty = 0.2`; only `IsPerceivable` & `HasRootBody` annotations, one per body
    (`_perceivable_objects`). `visual_center(*, body)` static.
- Mesh geometry (semantic_digital_twin/world_description/geometry.py:710-947): `Mesh(Shape)` fields
  `scale: Scale`, `filename: str`; `Mesh.from_file(file_path)`; `mesh` cached_property loads in metres
  only if the file declares units (STL declares none -> read as written); `local_frame_bounding_box`;
  JSON drops `filename` and carries the geometry (`_serialized_fields`, `to_json`) -> crosses the
  synchronizer; round trip keeps vertices exactly (test_geometry/test_shape.py:615-626).
  Aircraft meshes serialize to ~9.3 MB JSON (x2 visual+collision) on the first pass - consider a longer
  `wait_until` timeout than the 5 s default.
- `ObjectClassMesh` (robokudo/annotators/foundation_pose.py:49-79): `@dataclass` with `class_name: str`,
  `shape: Mesh` (`Mesh.scale` brings unitless file coords to metres). Test prior art:
  `ObjectClassMesh(class_name=..., shape=Mesh(filename=str(mesh_file), scale=Scale(...)))`
  (test_foundation_pose.py:238-246). `ObjectClassMeshes.from_class_meshes` raises `DuplicateObjectClassMesh`.

## 4. semantic_digital_twin_acrambly

- `R/semantic_digital_twin_acrambly/src/semantic_digital_twin_acrambly/semantic_annotations.py`:
  `AircraftPart(IsGraspable, ABC)` (23-27), concrete: BottomWing, Chassis, FrontWheel, MotorGrill, Propeller,
  LeftRearWheel (docstring: mesh named ``RearWheelLeft``, 66-69), RightRearWheel (``RearWheelRight``, 72-76),
  RearWing, TopWing, UnderBody, UpperBody, Window. `TetrisBlock(IsGraspable, ABC)` (117-121),
  `StraightTetrisBlock` (``child_cube_0_colored``, 124-129), `CornerTetrisBlock` (``child_cube_2_colored``,
  132-137). All `@dataclass(eq=False, kw_only=True)`. `Aircraft(HasRootBody, PartWholeRelationship)` (143-225).
  Module has `# %% aircraft parts`, `# %% tetris blocks`, `# %% aircraft` sections.
- `IsPerceivable`: semantic_digital_twin/semantic_annotations/mixins.py:124-133, `@dataclass(eq=False)`,
  one field `class_label: str | None = field(default=None, kw_only=True)`. Mixed in *last*:
  `class Cup(HasHandle, HasRim, DrinkingContainer, IsPerceivable)` (semantic_annotations.py:1006),
  `class CheezeIt(Food, IsPerceivable)` (:1112), `class UnknownObject(HasRootBody, IsPerceivable)` (:1181).
  `IsGraspable(HasRootBody, Mixin)` (mixins.py:411) already has a classproperty `grasp_strategy` (422-427) -
  prior art for a classproperty on these annotations.
- `classproperty`: `krrood.patterns.classproperty.classproperty`
  (krrood/krrood-core/src/krrood/patterns/classproperty.py); supports `@classproperty` over
  `@abstractmethod` (`__isabstractmethod__`) - but see finding C.
- `PathEnumeration`: `krrood.patterns.path_enumeration.PathEnumeration(Path, ReprEnum)`
  (krrood/krrood-core/src/krrood/patterns/path_enumeration.py:10-40). `member.name` is the file name; the
  member's own name is `member._name_`. Prior art with file-relative absolute paths:
  `SceneFile.MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"`
  (coraplex/demos/coraplex_hsr_mujoco_demo/demo.py:58-63); `DatasetFile` (test/krrood_test/
  test_development_tools/dataset_files.py:29-41); `RepositoryFile` (cognitive_robot_abstract_machine/
  repository_files.py:14-43, relative paths); `LinuxMemoryFile` (krrood_compute_resources/memory.py:21-30).
  Resources live *outside* src: `semantic_digital_twin_acrambly/resources/meshes/{aircraft,tetris_block}`;
  from `src/semantic_digital_twin_acrambly/<module>.py`, `Path(__file__).resolve().parents[2] / "resources" /
  "meshes"` (same pattern as semantic_digital_twin robots, e.g. robots/pr2.py:737). README says tetris
  meshes "in millimetres" -> update. Distribution deps: semantic_digital_twin, krrood-core.
- Mesh files (all binary STL, no units, no colour attributes; `.dae` also exists for TopWing and UpperBody -
  use the STL). Extents (processed trimesh) / bbox centre in file frame:

  | file | extents | bbox centre |d| |
  |---|---|---|
  | aircraft/BottomWing.stl | 0.254, 0.104, 0.029 | 0.017 |
  | aircraft/Chassis.stl | 0.030, 0.120, 0.103 | 0.004 |
  | aircraft/FrontWheel.stl | 0.060, 0.030, 0.060 | 0.015 |
  | aircraft/MotorGrill.stl | 0.054, 0.054, 0.016 | 0.005 |
  | aircraft/Propeller.stl | 0.104, 0.119, 0.016 | 0.017 |
  | aircraft/RearWheelLeft.stl | 0.026, 0.040, 0.040 | 0.013 |
  | aircraft/RearWheelRight.stl | 0.029, 0.040, 0.040 | 0.015 |
  | aircraft/RearWing.stl | 0.115, 0.087, 0.089 | 0.033 |
  | aircraft/TopWing.stl | 0.103, 0.255, 0.035 | 0.026 |
  | aircraft/UnderBody.stl | 0.077, 0.187, 0.043 | 0.006 |
  | aircraft/UpperBody.stl | 0.239, 0.077, 0.092 | 0.015 |
  | aircraft/Window.stl | 0.048, 0.051, 0.033 | 0.017 |
  | tetris_block/child_cube_0_colored.stl | 49.998, 203.052, 49.998 (mm) | centre (99.6, 178.4, 25.0) mm, 205.8 mm |
  | tetris_block/child_cube_2_colored.stl | 152.034, 101.016, 101.016 (mm) | centre (225.8, 128.7, 50.5) mm, 264.7 mm |

  Aircraft already in metres (max 0.255 m). Rescale: `trimesh.load(p); m.apply_scale(0.001); m.export(p)`
  (binary STL, 40/186 faces; nothing to lose). Only references to the tetris files: the docstrings and
  test_meshes.py; no code applies a scale factor today.
- Label collision check after mixing in `IsPerceivable` (runtime): no aircraft label matches another
  perceivable name; tetris labels also match `TetrisBlock` (accepted risk, follow-up
  `.scratch/most-specific-label-lookup/issues/01-...md`).

## 5. Tests

- `R/test/robokudo_acrambly_test/conftest.py`: imports `annotator_output_buffer, isolate_blackboard, node,
  ros_default` from `..robokudo_test.conftest` (lines 12-17) and defines `clock() -> SteppedClock`. Add
  `other_process_node, shared_world` to that import for the main test.
- `dataset/clocks.py`: `@dataclass(kw_only=True) class SteppedClock(Clock)`, `now: int =
  1_700_000_000_000_000_000`, `now_nanoseconds()` returns `now`. Advance with
  `clock.now = start + int(numpy.timedelta64(elapsed, "ns").astype(numpy.int64))`
  (test_synthetic_observation_interface.py:108-111).
- `test_synthetic_analysis_engines.py`: `engine_module` fixture params `[synthetic_tetris,
  synthetic_aircraft]`, ids tetris/aircraft (29-36). To rewrite: `test_pipeline_only_reads_the_engine_scenario`
  (60-71): collects `BaseAnnotator`s from `pipeline.iterate()`, asserts `[CollectionReaderAnnotator]`,
  `annotators[0].descriptor.parameters.camera_interface` is a `SyntheticObservationInterface` whose
  `camera_config.scenario is engine.scenario`. New: `[CollectionReaderAnnotator, MeshShapeAnnotator,
  WorldUpdatingAnnotator]`, mesh annotator's `object_class_meshes.meshes_by_class_name` keyed by every
  `part.label`, each mesh file the part class's enumeration member.
- `test_synthetic_observation_interface.py`: fixture `synthetic_reader` (68-90) builds interface with
  `clock`, calls `has_new_data()` once (starts the sequence), wraps reader in `Pipeline(children=[reader])`
  (empty world! see finding D); `read_pass(pass_index)` calls `reader.update()` pass_index+1 times.
  To rewrite: `test_camera_observation_has_the_scenario_camera_and_no_world_pose` (154-161, asserts
  `observation.world_T_camera is None`) and `test_reading_passes_adds_nothing_to_the_world` (164-171,
  asserts `world.bodies == []` and no `Camera`). Keep the others (hypotheses, capture time, waiting).
- `test_synthetic_scenarios.py`: `placements_at(*, scenario, pass_index)` (146-160) is the prior art for
  expected poses; `MotionLimits` (35-55); these tests treat pose position as "part centre" (cf. finding A).
- `R/test/semantic_digital_twin_acrambly_test/test_meshes.py`: StrEnums `AircraftMeshFile`,
  `TetrisBlockMeshFile` retype file names (17-42); fixture `meshes_directory` =
  `Path(__file__).resolve().parents[2] / "semantic_digital_twin_acrambly" / "resources" / "meshes"`;
  two glob tests. Extend: every `concrete_subclasses(AircraftPart) + concrete_subclasses(TetrisBlock)` names
  an existing member; every member's extent is in metres (e.g. `Mesh.from_file(str(member)).mesh.extents`
  below a bound). `test_semantic_annotations.py` uses `concrete_subclasses` and `World.create_with_root_body`.
- RoboKudo fixtures (`R/test/robokudo_test/conftest.py`): `ros_default` (session, autouse, 127-144),
  `node` (147-154, registers in `ROSNodeRegistry`), `other_process_node` (157-172, spun by its own executor
  thread), `shared_world(other_process_node) -> SharedWorld` (175-185: `World()` with one body
  `PrefixedName(FrameName.WORLD)` = "map", served via `SharedWorld.served_through`), `isolate_blackboard`
  (autouse, 212-222).
- `R/test/robokudo_test/_perception_world.py`: `FrameName` (WORLD="map", CAMERA, HEAD), `wait_until(condition,
  timeout=timedelta(seconds=5), poll_interval=50 ms) -> bool` (75-94), `SharedWorld(world, server,
  synchronizer)` + `served_through(*, node, world)` + `close()` (233-273), `place_camera_of` (161-199).
- `R/test/robokudo_test/_perception_process.py`: `robokudo_of_its_own(node, engine)` (93-104,
  `world_discovery_timeout=timedelta(0)`), `put_pass_into_world` (110-131), `EngineFindingAnObject`.
- `R/test/robokudo_test/test_process.py`: `robokudo_joining_a_shared_world(node, engine)` (89-100):
  `RoboKudo(node=node, analysis_engine=engine, world_discovery_timeout=timedelta(seconds=10))`;
  `perceived_body_id(world)` (103-108); `joined_robokudo` fixture (205-214, `yield` then `close()`).
  Served-world assertions: `test_perceived_object_reaches_the_shared_world_at_its_pose` (379-408:
  `wait_until(lambda: body_id in [b.id for b in shared_world.world.bodies] and np.allclose(
  shared_world.world.get_kinematic_structure_entity_by_id(body_id).global_transform.to_np(), expected))`),
  class label (410-433), semantic class `[type(a) ...] == [Cup]` (435-462). Ticking a real engine:
  `TestObjectsOfAFinishedPass` (787-834): `robokudo.start(include_gui=False)`, `robokudo.tick_tree()`,
  read `robokudo.world`, `robokudo.close()`. Leaving publishing to the server with a shared world: 892-904.
- `R/test/robokudo_test/test_analysis_engine_world_updating.py` only discovers robokudo's own engines
  (`discovered_analysis_engine_module_names`, test_analysis_engine_query_composition.py:183-196); acrambly
  engines are not covered there.
- RoboKudo process (`R/robokudo/src/robokudo/process.py:58-301`): `__post_init__` joins the served world
  (`find_world_fetcher_service`, `join_shared_world`, `WorldSynchronizer.of_world`); `start(*, include_gui,
  visualizer_types)` sets every `Pipeline.world` and grows the tree; `tick_tree()` ticks once; `close()`.
  `Pipeline` is a `Sequence(memory=True)` (robokudo/pipeline.py:26-47); all annotators here are synchronous
  `BaseAnnotator`s, so one `tick_tree()` = one pass when a pass is due; the reader returns RUNNING otherwise
  (annotators/collection_reader.py:96-141; it calls `pipeline.create_new_cas()` then `set_data`).
  `pipeline_init()` (idioms.py:40-60) succeeds without a query server.

### Main test sketch (both engines, parametrized like `engine_module`)
```python
engine = engine_module.AnalysisEngine(clock=clock)          # new clock field
robokudo = RoboKudo(node=node, analysis_engine=engine, world_discovery_timeout=timedelta(seconds=10))
robokudo.start(include_gui=False)
start = clock.now; robokudo.tick_tree()                     # pass 0 (sequence starts on first has_new_data)
served = shared_world.world
# camera: served.get_body_by_name(scenario.camera_frame).global_transform ~ scenario.<camera pose>
# beliefs: served.get_semantic_annotations_by_type(IsPerceivable); pair by annotation.class_label == part.label
#   aircraft: type(annotation) is <part class>; tetris: isinstance(annotation, TetrisBlock)
# shapes: [s.mesh.vertices / faces for s in body.visual.shapes] and body.collision.shapes vs
#   Mesh.from_file(str(<class mesh member>)).mesh
# pass 0 pose: np.allclose(body.global_transform.to_np(),
#   camera_pose.to_np() @ part.path.pose_at(elapsed=schedule.elapsed_at(pass_index=0)).to_np())
for k in 1..K: clock.now = start + ns(schedule.elapsed_at(pass_index=k)); robokudo.tick_tree()
# later pass: still len(parts) perceivables; position per finding B; orientation not asserted
robokudo.close()
```
Wrap served-world reads in `wait_until(...)` (asynchronous sync; consider timeout > 5 s for aircraft meshes).
A non-identity scenario camera pose in the test would make the composition check meaningful (identity
default makes `camera @ scripted == scripted`); the engines take `scenario=` so a test can pass a modified one.

## 6. Expected values

- Pose of part at pass i in camera optical frame: `part.path.pose_at(elapsed=scenario.schedule.elapsed_at(pass_index=i))`
  (pass i at `start + i*100 ms`). In the world: `<scenario camera pose> @ that` (parent = world root at identity
  in the served world). Camera pose: the new scenario field, default identity below the root.
- Body shape: one visual and one collision `Mesh` at identity origin in the body frame (the mesh file frame);
  serving process gets it via JSON with the same vertices; `filename` not serialized.
- Neighbour separation: >= spacing - 2*amplitude = 0.33 m > 0.2 m + aircraft |d| (0.033) -> no cross-matching.

## 7. Process notes

- AGENTS.md: dataclasses kw_only; keyword-only params; `# %%` sections; docstrings on every field; no
  try/except; custom exceptions in nearest `exceptions.py`; no module constants (classproperty / enum);
  `typing_extensions`; tests with plain pytest, assert against definitions (scenario, classproperty, enum
  member), never retyped literals. Memory note: stick exactly to the spec.
- Run tests one package at a time with the configured interpreter
  `/home/nasim/.virtualenvs/cram-env/bin/python -m pytest`, `CRAM_ORM_BUILD=never`, memory-capped
  (e.g. `systemd-run --user --scope -p MemoryMax=...`), check free memory first.
- Packages touched: robokudo_acrambly (scenario, camera_interface, analysis_engine), semantic_digital_twin_acrambly
  (IsPerceivable, mesh PathEnumeration module, tetris STLs, README); robokudo already done by ticket 01.
- Not verified: that a `Camera` semantic annotation added in RoboKudo's world crosses the synchronizer into the
  served world without error (no existing test does exactly this; robot worlds with cameras do sync). Check early.
