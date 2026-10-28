# Keep aircraft semantics and models in ACRAMbly

Aircraft-part semantic types and model mesh definitions belong in
`robokudo_acrambly`. The generic RoboKudo connector accepts a configurable mapping
from classification labels to semantic annotation types supplied by the
application. This preserves the dependency direction from the application to
RoboKudo while allowing observations to identify a model and its pose without
repeating fixed mesh geometry.
