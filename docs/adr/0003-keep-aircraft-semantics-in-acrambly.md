# Keep aircraft semantics in ACRAMbly

The aircraft-part semantic types and model meshes live in `robokudo_acrambly`.
The generic RoboKudo connector takes a mapping from classification labels to
semantic annotation types, which the application supplies. In Coraplex, only the
demo plan script imports `robokudo_acrambly.aircraft.types`, so that synchronized
annotations can be deserialized, and the `coraplex` package stays independent of
it. This keeps the dependencies pointing from the application to RoboKudo and
Coraplex, never the other way round.
