# Accept aircraft beliefs into the shared robot world

When RoboKudo runs alongside Coraplex, it fetches the robot's world, makes it its
own, and accepts aircraft-part beliefs directly into it. A `WorldSynchronizer`
publishes each new belief and each accepted pose change. Coraplex holds incoming
updates back and applies them at moments it chooses, so the world does not change
under a running plan. Running alone, RoboKudo keeps its separate local belief
world.

This gives every process one identity (the belief UUID) and one geometry per
part, and gives the motion controller collision geometry for every accepted part.
It makes RoboKudo a writer in the robot's world, so a perception error reaches
motion planning. The rejected alternatives were synchronizing a separate belief
world (impossible: sync messages identify entities by UUID, and the worlds share
no root) and querying RoboKudo per detection (answers from the current frame
rather than beliefs, and writes the same pose from two processes). See
`robokudo_acrambly/doc/coraplex_belief_consumption_discussion.md`.
