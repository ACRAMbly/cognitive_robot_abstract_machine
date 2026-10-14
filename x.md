# Native RoboKudo connector: implementation handoff

Adapted from [synchronization_research_sources.md](cognitive_robot_abstract_machine/semantic_digital_twin/doc/synchronization_research_sources.md), revised 2026-10-11, for **external Noctis + FoundationPose with one CAS adapter**. Scope: **option 1 — reuse `SemanticDigitalTwinConnector`, extend the body-update path for FoundationPose poses, and use native SDT `WorldSynchronizer`**. The integration requirements below are proposed work; the source did not verify an end-to-end integration or benchmark, and the external-worker integration is not an existing implementation.

## Implementation target

Run Noctis segmentation and FoundationPose pose estimation as external workers. Use one adapter inside RoboKudo to exchange camera data and inference results with those workers and convert the results into CAS annotations. Use the existing connector to associate observations with persistent SDT bodies, update accepted object state, and distribute changes to Coraplex and Giskard through native synchronization.

Keep RoboKudo, Noctis, and FoundationPose in separate environments so their Python, PyTorch, and CUDA dependencies can be managed independently. Communicate across process boundaries through an explicit request/result contract; RoboKudo's environment needs the adapter and transport dependencies, without importing either inference stack. Separate environments can avoid Python, PyTorch, and CUDA dependency conflicts.

The camera is fixed relative to the workspace.

## Required integration work

### 1. Connect external inference through one CAS adapter

- Implement a single `ExternalPerceptionAnnotator` CAS adapter in RoboKudo and configure it after the camera reader and before `SemanticDigitalTwinConnector`. It coordinates external inference and writes the resulting annotations; Noctis and FoundationPose execute in their own worker processes.
- Have the adapter read the camera RGB image, aligned depth, and intrinsics from the CAS. Send RGB to Noctis, then pass its masks and labels, the same frame's RGB/depth/intrinsics, and the resolved CAD model identifiers to FoundationPose. Keep the workers independent of RoboKudo imports and CAS objects.
- Define the request/result contract and transport explicitly. Carry a request/frame identifier, capture timestamp, camera frame, image dimensions, depth units, CAD model identifiers, and per-detection identifiers through both stages. Return ROIs, masks, labels, classification confidence, and camera-relative model poses with pose-validity information. Match all results to their originating frame and detection before writing the CAS.
- Have this one adapter write an `ObjectHypothesis` for each detection with its ROI, segmentation mask, and `Classification`, plus a `PoseAnnotation` when FoundationPose returns a valid pose. Classification confidence is not pose confidence. A detection without a valid pose must not trigger a pose-based body update. Verify the classification-to-CAD-mesh mapping used by the external FoundationPose worker.
- The inspected FoundationPose wrapper requires masks, classification-to-mesh mapping, and depth. Preserve those input requirements in the external worker; Noctis being RGB-only does not remove the pose stage's depth requirement. Normalize depth to meters at one declared boundary. The existing wrapper's depth reader converts millimeters to meters, so reusing its logic requires avoiding a second conversion.
- Read intrinsics through this checkout's `CASViews.CAMERA_INTRINSIC` in the adapter. The inspected wrapper references `CASViews.CAM_INTRINSIC`; resolve that mismatch only if reusing its CAS-facing code. The external worker receives intrinsics through the request contract.
- Verify the external worker's outputs, model frame, units, timestamps, and tracking behavior. The existing wrapper is a reference for extracting inference behavior, not an additional annotator to instantiate in RoboKudo. Findings about that wrapper came from static inspection, not reproduced runtime failures.
- Require FoundationPose results and the adapter's plain pose annotations to remain camera-relative. The inspected wrapper emits `self.twc @ poses_tco`; remove or bypass that world conversion when reusing the inference logic. The connector owns camera-to-world conversion, avoiding a double transform.
- Handle worker failures, timeouts, and partial results explicitly. Reject mismatched or out-of-order responses and do not relabel an old result with the current CAS frame's timestamp. An inference failure is not evidence that an object disappeared.
- Apply the calibrated mapping exactly once:

  ```text
  world_T_body = world_T_camera @ camera_T_model @ model_T_body
  ```

  The connector already applies camera-to-world conversion for association. Reuse that conversion and include model-to-SDT-body alignment in the accepted body-update path when those origins differ. `world_T_camera` and the model-to-SDT-body alignment remain fixed. Preserve the capture timestamp and explicit frame through the workers, adapter, and connector for freshness and ordering. Any conversion involving a moving robot frame still needs its transform at capture time. Renew calibration if the camera mount or workspace reference changes.

### 2. Update bodies from FoundationPose while preserving geometry

- The connector currently compares image regions, world-frame stamped poses, and 3D bounding boxes.
- The current belief creation/update helpers derive body poses from the latest `BoundingBox3DAnnotation`, through `_create_world_origin_and_scale_from_latest_bbox` and `_update_belief_body_from_latest_bbox`. Without a 3D bounding box, that path skips the body pose update even when a plain `PoseAnnotation` is present. Extend this body-update path to consume valid external FoundationPose poses.
- RoboKudo already supports CAD meshes: [world descriptors](cognitive_robot_abstract_machine/robokudo/src/robokudo/world_descriptor.py) can load visual meshes with box collision geometry, and the [tabletop descriptor](cognitive_robot_abstract_machine/robokudo/src/robokudo/descriptors/worlds/world_semdt_raytracer_tabletop.py) uses meshes for both visual and collision geometry. Reuse this support and verify mesh-frame/body-frame alignment.
- The existing bounding-box update clears visual shapes before inserting a box; it does not clear collision shapes. Separate pose updates from this visual-geometry replacement so an accepted FoundationPose pose retains the body's CAD mesh and collision geometry.
- Successful matches already move and rotate the existing body while preserving its UUID. Reuse this identity handling. For newly perceived bodies, extend creation to select CAD/collision geometry and add the semantic annotations required by Coraplex plans; the current belief-creation path starts with a bare body and adds a visual box. Preloaded bodies may already have these properties and need no reinitialization once associated with a belief.
- Treat body, mesh, and semantic additions as structural changes at appropriate plan boundaries; ongoing pose updates are state changes.

### 3. Extend association rules and observation validity

- Reuse the connector's hypothesis-to-belief association. `ObjectBeliefState` holds an SDT `Body` and its persistent UUID.
- Current association uses Hungarian assignment and creates new beliefs when matches fall below its association threshold. The threshold measures matching similarity, not calibrated FoundationPose pose confidence.
- Use the existing geometry-based association for the current object set, where different object kinds have distinct shapes. A class/CAD model identifies an object kind; duplicate instances require distinct persistent UUIDs.
- If using FoundationPose tracking, route each persistent instance's previous camera-relative model pose or worker tracking handle through the adapter's request contract. Bind worker tracking state to the connector's persistent UUID after association; a per-frame detection identifier is not a persistent identity. Reset or reinitialize tracking when the binding is lost or ambiguous. Enabling external tracking alone does not establish instance association.
- Every accepted observation must resolve to a stable twin UUID and retain capture time and frame.
- Track last-seen time, association quality, and pose validity separately. The connector records pose timestamps but does not check capture age, ordering, or FoundationPose validity before updating beliefs. Assign acceptance checks to the adapter, a dedicated component, or the connector; do not duplicate checks already enforced elsewhere. Reject out-of-order observations and gate implausible pose jumps before applying world updates.
- Existing beliefs are already retained when observations are absent or unmatched. Add explicit stale/lost status, rediscovery, and removal criteria where plans require them; retention alone does not ensure freshness or correct reassociation. The current connector does not provide calibrated pose-uncertainty handling.
- Preserve identity during temporary occlusion. Record hidden or out-of-view objects as unobserved and age their last accepted perception poses; invisibility alone is not evidence that an object left the world. Resolve ambiguous matches before reaching.
- Editing `semantic_world_connector.py` is conditional: use existing comparator configuration and adapter validation where they satisfy the contract, and extend the connector only for behavior not covered by those mechanisms.

### 4. Initialize RoboKudo's world and attach synchronization

- RoboKudo's central perception `World` is local to its process. Updating it alone does not update Coraplex or Giskard.
- External workers produce observations; the adapter populates the CAS and the connector accepts world updates. The workers do not publish SDT world changes or maintain competing authoritative world instances.
- Bootstrap a compatible world model and explicitly attach/configure `WorldSynchronizer` during application startup after obtaining RoboKudo's runtime world. No instantiation was found in the inspected RoboKudo source; this is missing startup wiring, not evidence that the connector needs a rewrite.
- Use native synchronization to deliver accepted changes to the Coraplex planning world and Giskard controller world. Running all three through their separate real-deployment entry points normally gives three live software world instances. Replication is unnecessary between components sharing the exact same in-memory `World` instance, as Coraplex and its local Giskard executor do during simulated execution.
- `WorldSynchronizer` replicates accepted world changes and records per-publisher stream positions. It does not infer observation identity or establish a global order across competing writers.
- Giskard buffers incoming changes for safe control-loop boundaries, distinguishes structural and state changes, and exposes applied stream positions.
- Before constructing the next motion, ensure the required perception update has reached Coraplex and Giskard. The client's own publication position alone does not establish freshness of another publisher's observation.

### 5. Coordinate plans and attachment authority

- Before sensitive manipulation, let the plan request or await a sufficiently fresh accepted observation from the adapter's external inference cycle. Measure freshness from camera capture time, including transport and inference latency. Require the target to be visible within the fixed camera's field of view.
- While grasping or carrying an object, the gripper attachment and controller determine its pose; perception must not independently move that body or create competing writers.
- After placement within the camera's view, clear occlusion and request another observation before returning pose authority to perception and accepting the action result. Placement outside that view requires another source of verification.
- An attached object's pose may remain valid through controller state while the camera cannot see it.

The required sequence is: ensure visibility → capture a frame → run external Noctis → run external FoundationPose on that frame and its detections → write results through the single CAS adapter → transform once → resolve persistent instance → validate age and pose quality → wait for the required world update to be applied → plan/reach/grasp → use attachment authority while held → place within view → clear occlusion → reobserve through the same external pipeline. This is an integration contract, not an existing Coraplex API sequence.

## Implementation entry points

| File or component | Relevant work |
| --- | --- |
| `cognitive_robot_abstract_machine/robokudo/src/robokudo/annotators/external_perception.py` (proposed) | Single CAS adapter: read camera data, coordinate external workers, validate result/frame correspondence, and write detection and camera-relative pose annotations. |
| External Noctis worker (proposed; entry point to be selected in `noctis/`) | Segmentation in its own environment; return masks, ROIs, labels, and classification confidence through the request/result contract. |
| External FoundationPose worker (proposed; entry point to be selected) | Pose inference and optional tracking in its own environment; consume matched RGB/depth/intrinsics, masks, and CAD model identifiers and return camera-relative model poses. |
| [semantic_world_connector.py](cognitive_robot_abstract_machine/robokudo/src/robokudo/annotators/semantic_world_connector.py) | Reuse association, camera-to-world conversion, and identity-preserving update dispatch. Configure existing comparators; extend acceptance or association behavior only where adapter/configuration cannot meet the contract. |
| [world.py](cognitive_robot_abstract_machine/robokudo/src/robokudo/world.py) | Extend bounding-box-derived body updates to consume accepted FoundationPose poses without replacing visual geometry. Initialize CAD/collision geometry and semantics for newly perceived bodies using existing mesh support. |
| [belief_state.py](cognitive_robot_abstract_machine/robokudo/src/robokudo/types/belief_state.py) | Persistent belief, SDT body, and UUID association. |
| [foundation_pose_annotator.py](robokudo_foundation_pose/robokudo_foundation_pose/robokudo_foundation_pose/annotator/foundation_pose_annotator.py) | Reference for extracting external worker inference/tracking behavior and auditing required inputs, depth conversion, and extrinsics; not an in-process pipeline component in this deployment. |
| [cas.py](cognitive_robot_abstract_machine/robokudo/src/robokudo/cas.py) | Camera-intrinsic view naming and perception data views. |
| [world_synchronizer.py](cognitive_robot_abstract_machine/semantic_digital_twin/src/semantic_digital_twin/adapters/ros/world_synchronizer.py) | Existing native replication and per-publisher stream positions; instantiate/configure it in RoboKudo's application startup. |
| [world_synchronization.rst](cognitive_robot_abstract_machine/semantic_digital_twin/doc/world_synchronization.rst) | State-variable updates, atomic model-change replay, and full-world reloads. |
| [giskard.py](cognitive_robot_abstract_machine/giskardpy/src/giskardpy/middleware/ros2/giskard.py) | Controller world snapshot service and synchronization setup. |
| [world_updates.py](cognitive_robot_abstract_machine/giskardpy/src/giskardpy/middleware/ros2/world_updates.py) | Safe update application, structural/state changes, and applied positions. |
| [python_interface.py](cognitive_robot_abstract_machine/giskardpy/src/giskardpy/middleware/ros2/python_interface.py) | Client synchronization and waiting for required updates. |
| [demonstrations.py](cognitive_robot_abstract_machine/coraplex/src/coraplex/demonstrations.py) | Real-run planning world acquisition and synchronization setup. |
| [perception.py](cognitive_robot_abstract_machine/coraplex/src/coraplex/perception.py) and [misc.py](cognitive_robot_abstract_machine/coraplex/src/coraplex/robot_plans/motions/misc.py) | Existing plan/perception request integration points. |

The source inspected commits `12e9aa1e4` (cognitive_robot_abstract_machine), `1ef9a96` (robokudo_foundation_pose), and `ba76bbc` (noctis). Local implementation should govern implementation choices when it differs from online API documentation.
