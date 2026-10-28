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
required and has no machine-specific default. Missing or unreadable files, or paths naming a
directory raise `MissingAircraftResource`, whose correction identifies the
required filename. Every resource is checked before any is loaded.

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

This catalog prepares geometry without inserting bodies or observations into a
world. Connector validation, association, pose updates, and absence handling
belong to the subsequent implementation tickets.
