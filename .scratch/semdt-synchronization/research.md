# SemDT synchronization prerequisites

Source inspection on 2026-10-20. Existing tests were read, not executed.

## Synchronization can be developed without FoundationPose

The existing connector tests construct an `ObjectHypothesis` with an ROI, mask, `PoseAnnotation`, and `BoundingBox3DAnnotation`, initialize a local SemDT world and camera transform, and exercise belief creation, association, and updates directly. No detector or inference worker is involved. The full connector `compute()` test also supplies a synthetic color image and checks that a belief and world-frame stamped pose are created. [Synthetic hypotheses](../../test/robokudo_test/test_semantic_world_connector.py#L24), [world fixture](../../test/robokudo_test/test_semantic_world_connector.py#L105), [association/update tests](../../test/robokudo_test/test_semantic_world_connector.py#L122), [compute test](../../test/robokudo_test/test_semantic_world_connector.py#L214).

There is also an existing test with actual recorded RGB-D input and geometric object detection:

```text
file_reader → ImagePreprocessorAnnotator → PointcloudCropAnnotator
→ PlaneAnnotator → PointCloudClusterExtractor → ClusterPoseBBAnnotator
→ SemanticDigitalTwinConnector
```

The test configures a static `map`/`camera` transform, processes two distinct CAS observations, checks that both observations update the same belief UUID, and checks pose, bounding box, stamped world-frame pose, body attachment, and visual box creation. This is a reusable baseline for perception-to-local-SemDT integration; it does not require Noctis or FoundationPose. Its crop is configured for the test fixture, so another recording may need different crop/plane/cluster settings. [Pipeline and transform configuration](../../test/robokudo_test/test_full_ae_execution.py#L91), [two-observation identity assertions](../../test/robokudo_test/test_full_ae_execution.py#L125), [annotation/body assertions](../../test/robokudo_test/test_full_ae_execution.py#L186).

The file-reader data helper downloads and verifies the fixture dataset on first use; cached data avoids that download. These tests still require the RoboKudo/ROS project environment: RoboKudo's test fixtures initialize ROS and provide a registered node. [Dataset helper](../../robokudo/src/robokudo/utils/data_downloader.py#L19), [ROS fixtures](../../test/robokudo_test/conftest.py#L108).

## Local belief updates and replicated worlds are different checks

The recorded-data test checks RoboKudo's local world. It does not instantiate `WorldSynchronizer` or assert a receiver world's state. A source search of `robokudo/src` found no `WorldSynchronizer`/`world_synchronizer` usage. This matches the missing application-startup wiring identified in `x.md`: obtain RoboKudo's runtime world, attach a synchronizer, and bootstrap a compatible receiver world. [Local test](../../test/robokudo_test/test_full_ae_execution.py#L91), [required startup wiring](../../x.md#L55).

Native SemDT synchronization already has model-free tests. `test_state_synchronization` creates two dummy worlds, attaches synchronizers, changes a state variable, and compares the receiver. `test_model_synchronization_body_only` creates a body and verifies the same body identifier exists at the receiver. Another test waits for the receiver's `has_applied()` acknowledgement of the publisher's stream position. These offer reusable patterns for testing transport and applied-state acknowledgement without camera inference. [Dummy world](../../test/semantic_digital_twin_test/test_ros/test_world_synchronizer.py#L95), [state replication](../../test/semantic_digital_twin_test/test_ros/test_world_synchronizer.py#L230), [body replication](../../test/semantic_digital_twin_test/test_ros/test_world_synchronizer.py#L505), [applied-position check](../../test/semantic_digital_twin_test/test_ros/test_world_synchronizer.py#L470).

The current implementation publishes model modifications and changed degree-of-freedom state through `WorldSynchronizer`; it does not run detection or estimate poses. [Synchronizer callbacks](../../semantic_digital_twin/src/semantic_digital_twin/adapters/ros/world_synchronizer.py#L480).

## Recommended sequence

The existing analysis engines are reusable starting points:

| Engine | Input and output | Work needed for synchronization testing |
| --- | --- | --- |
| [`other_analysis_engines/semdt_demo_from_storage.py`](../../robokudo/src/robokudo/descriptors/analysis_engines/other_analysis_engines/semdt_demo_from_storage.py#L28) | MongoDB RGB-D playback, geometric clusters, bounding-box poses, and the connector. | Supply a suitable recording with camera-to-world metadata and add native world replication. |
| [`other_analysis_engines/semdt_demo.py`](../../robokudo/src/robokudo/descriptors/analysis_engines/other_analysis_engines/semdt_demo.py#L24) | Kinect camera reader, geometric detection, and the connector. | Configure the reader and calibration for the actual camera, then add replication. |
| [`semdt_raytracer_demo.py`](../../robokudo/src/robokudo/descriptors/analysis_engines/semdt_raytracer_demo.py#L27) | Simulated RGB-D, geometric clusters, and bounding-box poses. | Add the connector and replication after resolving runtime-world lifetime. |

The simulated reader calls `set_world()` with a newly loaded descriptor world during each frame. A synchronizer attached to a previous world would therefore not automatically follow it. Keep a persistent belief world, keep simulation ground truth separate, and attach replication to the actual belief world before using this route for a multi-frame synchronization test. This recommendation follows from the [frame reader](../../robokudo/src/robokudo/io/semdt_raytracer_camera_interface.py#L68), [world loading](../../robokudo/src/robokudo/io/semdt_raytracer_camera_interface.py#L135), [descriptor construction](../../robokudo/src/robokudo/utils/module_loader.py#L230), and [world replacement](../../robokudo/src/robokudo/world.py#L112).

The geometric pose is derived from a bounding box; it does not establish the original CAD frame required by `x.md`. The current body update also depends on a bounding box and replaces existing visual geometry with a box. Pose-only updates that preserve CAD geometry therefore still need separate tests and implementation. [Geometric pose](../../robokudo/src/robokudo/annotators/cluster_pose_bb.py#L51), [body update](../../robokudo/src/robokudo/world.py#L270).

The [official engine catalog](https://robokudo.ai.uni-bremen.de/autoapi/robokudo/descriptors/analysis_engines/index.html) corroborates the reusable engine structure. Use local paths above: several demos are inside `other_analysis_engines` in this checkout. The loader builds a dotted import path, so a nested engine requires a name such as `other_analysis_engines.semdt_demo_from_storage`. [Module loader](../../robokudo/src/robokudo/utils/module_loader.py#L116).

`.venv-foundation-pose` is an existing Python virtual environment, not a perception stage. Synchronization needs an installed RoboKudo/SemDT and ROS environment; using this existing environment as the interpreter does not require invoking FoundationPose. Its configuration records Python 3.12.3 with system site packages enabled. [Environment configuration](../../.venv-foundation-pose/pyvenv.cfg), [existing runtime decision](../../docs/adr/0001-isolate-noctis-runtime.md).

1. Reuse the recorded file-reader pipeline for the first detection-to-world baseline. Add a focused test that connects this runtime world to a second SemDT world and checks matching body UUID and accepted pose after the receiver applies the published update.
2. Test `x.md`'s additional contracts with deterministic observations or mocked worker responses: camera-to-world conversion exactly once, camera-relative pose without a bounding box, CAD geometry preservation, repeated instance identity, invalid/stale/out-of-order observations, and receiver acknowledgement. These requirements are distinct from the current bounding-box baseline. [Pose-only body-update requirement](../../x.md#L34), [acceptance/identity requirements](../../x.md#L43), [replication requirement](../../x.md#L55).
3. Integrate real inference when validating the full deployment's segmentation, CAD-frame pose accuracy, latency, and worker protocol. A FoundationPose environment is necessary for executing that model, but is not a prerequisite for implementing or testing the synchronization layer. This follows from the model-free connector and native replication tests above.

The deployment scope needs an explicit choice: `x.md` proposes separate Noctis and FoundationPose workers behind one CAS adapter, while the existing ADR keeps Noctis in a worker and FoundationPose inside the RoboKudo Python 3.12 runtime. Reusing existing tests does not depend on resolving this difference; implementing the final inference pipeline does. [Proposed deployment](../../x.md#L5), [existing ADR](../../docs/adr/0001-isolate-noctis-runtime.md).
