# 04: Publish and receive accepted captures

Status: done
Type: task
Blocked by: 02

Implementation permission: granted 2026-11-25.

## Outcome

After each accepted capture, RoboKudo publishes an accepted capture message with
the capture time, the accepted belief UUIDs and the stream position after their
poses. A receiver records the newest capture per part. Follow the
[specification](../spec.md#accepted-capture) and
[ADR 0004](../../../docs/adr/0004-publish-capture-times-beside-the-world.md).

## Acceptance criteria

- [ ] Test first: after one accepted capture, the receiver records its capture
  time and belief UUIDs, and the receiving world's `has_applied` on the recorded
  stream position is false while updates are held back and true after
  `apply_missed_messages()`.
- [ ] Test first: a part in view but not accepted is not listed; a part absent
  from a capture keeps its earlier capture in the receiver.
- [ ] Test first: the message round-trips through `DataclassJSONSerializer`.
- [ ] `AcceptedCapture` is a dataclass; the topic name is a `StrEnum` member; the
  transport is `std_msgs/String` with volatile QoS.
- [ ] The publishing annotator runs after the connector, finds the accepted
  beliefs as those whose latest hypothesis is one of the current CAS's object
  hypotheses, reads the stream position after their poses were published, and
  publishes nothing for a capture with no accepted part.
- [ ] The receiver keeps only the newest capture per part UUID.
- [ ] The capture time is not written into the world, and nothing changes in
  `robokudo/` or `semantic_digital_twin/`.
- [ ] Message, publisher and receiver live in `robokudo_acrambly`.

## Verification

Run the accepted capture tests and the aircraft acceptance tests in
`test/robokudo_acrambly_test` with `--orm-build never` and a memory cap, in one
pytest process with the `rclpy_node` fixture.

## Comments

- 2026-11-21: Specified in the planning session; no code written. How the
  annotator gets the pipeline's ROS node is to be settled during implementation
  without changing `robokudo/`.
