# Publish capture times beside the world, not in it

After each accepted capture, RoboKudo publishes a small message on its own
topic with the capture time, the belief UUIDs of the parts it accepted, and the
position of its `WorldSynchronizer` stream after their poses. A receiver trusts
the capture only once it has applied that stream position, so the message can
arrive in any order with the world updates. The capture time stays out of the
world model and out of the semantic types, and the message works the same for
any kind of perceived object.

## Considered Options

- **A capture-time field on the aircraft annotation**, written through a
  `@synchronized_attribute_modification` method. Rejected: every accepted
  capture would publish a model change, and Giskard aborts a running motion
  with `WorldModelModifiedDuringMotionError` when a model change is waiting.
- **A generic field in RoboKudo or Semantic Digital Twin.** Rejected: it
  changes packages outside `robokudo_acrambly` and has the same model-change
  problem.
