# ACRAMbly semantic annotations live in their own distribution

The aircraft and its parts, and the tetris blocks, are semantic annotations
specific to the ACRAMbly assembly tasks. They live in a new distribution,
`semantic_digital_twin_acrambly`, which depends on `semantic_digital_twin`,
together with the meshes of those objects. `semantic_digital_twin` is the generic
framework shared upstream, so assembly-toy concepts do not belong in it. A
subpackage of `robokudo_acrambly` would tie them to perception, although the
plans need them just as much.

## Consequences

- Perception labels its object hypotheses with these annotation classes
  instead of keeping its own list of part names, so `robokudo_acrambly` depends
  on `semantic_digital_twin_acrambly`.
- The distribution has no ORM interface yet: a world containing these
  annotations cannot be saved to a database until it gets a `generate_orm.py`.
