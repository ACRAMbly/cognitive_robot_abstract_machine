# Aircraft semantic types and model resources

`robokudo_acrambly.aircraft.types` defines `AircraftPart` and the twelve concrete
part types from the connector inventory. They inherit `HasRootBody` and
`IsPerceivable`: instantiate an annotation with `root=body` and
`class_label=definition.label`, then register it through the world's existing
semantic annotation API. Queries for `AircraftPart` include every concrete kind;
queries for a concrete type select that kind. No grasp or assembly relationships
are defined.

`robokudo_acrambly.aircraft.models.AircraftModel` is the model inventory. Each
member's `value` holds its `AircraftLabel`, concrete semantic type, relative
resource path, and metres per mesh unit. The original `plane/*.stl` files are
already measured in metres and use scale 1.0.

Construct `AircraftModelCatalog(mesh_directory=...)` with the directory containing
the twelve STL files. The default is the complete inventory; the directory is
required and has no machine-specific default. Missing or unreadable files, or
paths naming a directory raise `MissingAircraftResource`, whose correction
identifies the required filename. Every resource is checked before any is loaded.

`catalog.models[label]` returns the loaded model and its definition. Call
`create_body(body_name)` on it to obtain a body with visual and collision geometry
already installed. Both collections use the original mesh, including all source
components and offsets. The body frame remains at the mesh origin. Each body,
collection, shape, mesh, and frame reference has independent mutable state.
Resources are eagerly loaded once per catalog; creating bodies does not reread
the files. Construct a new catalog to reload changed resources.

To replace a resource, use `dataclasses.replace` on its model definition, changing
`resource` and, if necessary, `metres_per_mesh_unit`. Pass the updated definitions
as the catalog's `definitions` tuple. Retain `label` and `semantic_type` so
observations continue to identify the same kind. The replacement supplies both
visual and collision geometry. Source coordinates are scaled but never recentered.

To create aircraft beliefs through `SemanticDigitalTwinConnector`, configure both
`descriptor.parameters.semantic_types` from each model's `definition.semantic_type`
and `descriptor.parameters.model_geometry` from the loaded catalog models:

```python
from pathlib import Path

from robokudo.annotators.semantic_world_connector import SemanticDigitalTwinConnector
from robokudo_acrambly.aircraft.models import AircraftModelCatalog

# Run from the checkout root, or pass your deployment's mesh directory.
catalog = AircraftModelCatalog(mesh_directory=Path("plane"))
descriptor = SemanticDigitalTwinConnector.Descriptor()
descriptor.parameters.semantic_types = {
    label: model.definition.semantic_type
    for label, model in catalog.models.items()
}
descriptor.parameters.model_geometry = {
    label: model for label, model in catalog.models.items()
}
connector = SemanticDigitalTwinConnector(descriptor=descriptor)
```

The connector prepares geometry before persistent changes, then registers one
body, belief, and correctly typed annotation for each new accepted object. Both
mesh collections are present at insertion. Observations supply their model label
and supported pose; no mesh or bounding box is required. See the generic
[observation contract](../../robokudo/docs/semantic_observations.md) for capture
metadata and frames.

Add `connector` to a RoboKudo `Pipeline` after the annotators that populate its
CAS with public `ObjectHypothesis`, `Classification`, and pose annotations. Set
the CAS world and camera metadata before the connector runs. The configured
labels are the `AircraftLabel` values in `AircraftModel`; the semantic-type and
geometry mappings must describe the same accepted labels.

On compatible accepted matches, the connector retains the existing belief,
body, UUID, semantic annotation, and shapes, updating the mesh-origin world pose
and accepted capture time. Different known models cannot match. Objects absent
from the CAS retain their last accepted state, including during an empty CAS;
reappearance uses the existing association rules.

The local acceptance test exercises every configured model through creation,
translation, rotation, partial absence, total absence, and reappearance with
fresh observations. It initializes a camera transform with rotation and
translation and changes detection identifiers. The test fixture owns the poses
and capture times; it is a test source rather than a production detector. See
[verification](aircraft_belief_verification.md) for commands and recorded results.
