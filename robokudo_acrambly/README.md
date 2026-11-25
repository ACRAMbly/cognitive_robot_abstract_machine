# robokudo_acrambly

ACRAMbly's perception components for RoboKudo.

## Synthetic object hypotheses

`robokudo_acrambly.synthetic_observations` feeds a RoboKudo pipeline with
scripted observations: per pass, the object hypotheses that segmentation and 6D
pose estimation would leave in the CAS, specified by a scenario instead of
perceived from a camera. Each hypothesis carries a `Classification` (the part's
label) and a `PoseAnnotation` (its pose in the camera optical frame); the CAS
also gets a `CameraObservation` holding the pass's scheduled capture time.

Two scenarios are provided, each running 30 s at 10 Hz:

- `synthetic_tetris`: two tetris blocks.
- `synthetic_aircraft`: the twelve parts of the toy aircraft.

Run one with RoboKudo's launcher, for example:

```bash
python3 -m robokudo.scripts.main _ae synthetic_aircraft _ros_pkg robokudo_acrambly
```
