# Configured local object beliefs

Set `SemanticDigitalTwinConnector.Descriptor().parameters.semantic_types` to a
mapping from classification labels to concrete `HasRootBody` semantic annotation
types. `None` preserves generic connector acceptance. An empty mapping enables
validation but accepts no model labels.

An application with an `AircraftModelCatalog` supplies the mapping from its
loaded definitions:

```python
descriptor = SemanticDigitalTwinConnector.Descriptor()
descriptor.parameters.semantic_types = {
    label: model.definition.semantic_type
    for label, model in catalog.models.items()
}
connector = SemanticDigitalTwinConnector(descriptor=descriptor)
```

To create mesh-bearing beliefs, also supply loaded model geometry:

```python
descriptor.parameters.model_geometry = {
    label: model for label, model in catalog.models.items()
}
```

Each value implements `robokudo.types.belief_creation.ModelGeometry.create_body`.
It returns a detached body with independent, loaded visual and collision shapes,
with metric scale and model-local offsets retained at the source mesh origin.
An enabled mapping must contain each observed model label; missing entries raise
`MissingModelGeometry`. The aircraft catalog implements this interface.

The generic connector does not import application types. The mapping validates
model acceptance independently of classification confidence or the connector's
association similarity threshold. Missing, empty, unknown, and conflicting
classification labels raise dataclass exceptions. Repeated classifications of
the same label are allowed.

Each hypothesis uses its newest supported `PoseAnnotation` or
`StampedPoseAnnotation`. Translations are metres, rotations are nonzero finite
xyzw quaternions, and the pose locates the source mesh origin. Plain poses are
camera-relative. Stamped poses must declare the CAS camera or world frame.

The CAS must declare an existing world frame and a capture time in
`data_timestamp` (integer nanoseconds), `CAMERA_INFO.header.stamp`, or both.
Supplied timestamps must agree, including a stamped pose's timestamp. Zero is a
valid capture time. CAS creation time and processing time are never substitutes.
Camera-info frame metadata, when supplied alongside a CAS camera frame, must
agree. Camera and world identifiers must differ.

Camera-relative poses require the declared camera frame and a finite rigid
`camera_to_world_transform` whose reference and child frames match the world and
camera bodies. Conversion uses `world_T_part = world_T_camera @ camera_T_part`.
A stamped world pose needs neither a camera frame nor a camera transform solely
for accepting that pose. Capture timestamps retain integer precision.

All configured hypotheses are validated and their world poses prepared before
any derived annotations or persistent beliefs change. A later invalid hypothesis
leaves an empty or populated belief world unchanged. Original annotations remain;
reprocessing a derived world pose neither transforms it again nor appends another
derived annotation. Empty observations retain existing object state.

`SemanticObservationAcceptance.prepare` returns a `PreparedObservation` containing
the original hypothesis, resolved semantic type, and captured world pose.
Configured creation prepares all required bodies and typed semantic annotations
before any persistent belief changes. Resource failures therefore leave the
persistent world unchanged, even after an earlier valid hypothesis. Bodies enter
the world with complete geometry in both collections. Each new belief retains
the body's UUID and receives one semantic annotation rooted at that body;
`IsPerceivable` annotations retain the classification label.

## Association and accepted updates

The connector uses the existing weighted comparators and one-to-one assignment.
Known different model labels cannot match, even when their geometric evidence
is similar. Classification confidence does not replace association similarity.
The descriptor's existing `confidence_threshold` determines whether a compatible
assignment updates a belief or creates a new one. Unmatched hypotheses also
create beliefs. Model labels identify kinds; detection identifiers identify
observations. Neither is a physical object's persistent UUID.

Configured mesh observations require a model label and supported pose, without
a per-detection mesh, bounding box, or image region. Optional image evidence
participates when available. Accepted matches update the existing body's world
pose and accepted hypothesis while preserving its belief, body, UUID, semantic
annotation, and visual and collision shapes. Shape-local offsets remain local
to the body, so translation and rotation move the complete model.

Absent objects retain their last accepted pose, capture time, geometry, and
identity. An empty observation changes no object state. Reappearance can update
the retained belief through the same association rules. Persistence lasts for
the running local world; it adds no expiry policy or durable storage.

With `model_geometry=None`, creation retains the existing bounding-box behavior;
with `semantic_types=None`, it retains generic untyped acceptance. Application
semantics stay outside the generic connector. See the application's
[aircraft configuration](../../robokudo_acrambly/doc/aircraft_models.md) and
[verification evidence](../../robokudo_acrambly/doc/aircraft_belief_verification.md).
