# 03: Take the aircraft world frame from the fetched root

Status: ready-for-agent
Type: task
Blocked by: 02

Implementation permission: not granted.

## Outcome

The aircraft world frame is the root of the fetched world, whatever its name
(`map` for the stand-in, `map2` for Tracy's Giskard). Follow the
[specification](../spec.md#world-frame).

## Acceptance criteria

- [ ] Test first: with a supplied world whose root is not named `map`, accepted
  poses are expressed in that root and no extra frame body appears.
- [ ] `world_frame` leaves `aircraft_frames.json`, the `AircraftFrames` data and
  its configuration loading; validation no longer checks it.
- [ ] The engine no longer adds a world frame body under the root.
- [ ] The camera frame and the scripted camera-to-world calibration stay.
- [ ] Test datasets such as `dataset/aircraft_frames_rotated.json` follow the new
  configuration shape.

## Verification

Run the aircraft frame, configuration, playback and acceptance tests in
`test/robokudo_acrambly_test` with `--orm-build never` and a memory cap.

## Comments

- 2026-11-21: Specified in the planning session; no code written.
