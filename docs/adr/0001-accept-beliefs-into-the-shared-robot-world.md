# Accept perceived beliefs into the shared robot world

RoboKudo fetches the robot's world from Giskard, makes it its own, and accepts
aircraft-part and Tetris-block beliefs directly into it. A `WorldSynchronizer`
publishes each new part and each accepted pose change to Giskard and Coraplex. Every process then
has one identity per part (the belief UUID) and one geometry, and the motion
controller gets collision geometry for every accepted part. The cost is that
RoboKudo becomes a writer in the robot's world, so a perception error reaches
motion planning.

The scripted aircraft engine keeps no separate local belief world: it always
runs against a world fetched from a world server, also for the synthetic target
and when inspecting perception on its own. The Tetris engine follows in a later
pull request.

## Considered Options

- **Keep a local belief world as a second mode, chosen by configuration.**
  Rejected to keep the code base clean: one startup path instead of two. The
  costs are accepted for now and handled later: every manual run needs a
  world server, and a broken world service blocks perception work.
- **Synchronize RoboKudo's separate belief world.** Not possible: sync messages
  identify bodies and connections by UUID, and the two worlds share no root.
- **Query RoboKudo per detection** (`RoboKudoPerception` → `Detection.apply_to`).
  The answer comes from the current frame's object hypotheses, not from beliefs,
  and its identifier is not the belief UUID. `apply_to` also moves the body in
  Coraplex, so two processes would write the same pose. Each query takes about
  10 s.

See `robokudo_acrambly/doc/coraplex_belief_consumption_discussion.md` and
`robokudo_acrambly/doc/coraplex_belief_consumption.md`.
