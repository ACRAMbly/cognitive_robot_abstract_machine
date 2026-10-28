# Configured observation validation

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
the original hypothesis, resolved semantic type, and captured world pose. This
is the extension point for subsequent configured body creation. This validation
ticket does not install model geometry or semantic annotations, change model-kind
association, or replace the existing bounding-box body-update path; those are
separate implementation tickets.
