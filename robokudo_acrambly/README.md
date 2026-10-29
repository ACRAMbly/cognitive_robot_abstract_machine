# RoboKudo ACRAMbly

`robokudo_acrambly` owns ACRAMbly's custom RoboKudo annotators, analysis engines,
and example mesh resources. It depends on the workspace's existing `robokudo`
package and extends its public interfaces.

Install it with the repository's normal `uv sync --active` workflow after setting
up the ROS 2 environment. The workspace's `sensor_msgs` and other ROS dependencies
must remain available to Python.

Run the `scripted_aircraft` synthetic observation engine from the repository root
after the [ROS and configured-interpreter setup](doc/scripted_aircraft.md#setup-and-headless-execution):

```bash
export AIRCRAFT_FRAME_CONFIGURATION="$PWD/robokudo_acrambly/doc/aircraft_frames.json"
unset AIRCRAFT_MESH_DIRECTORY
python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae scripted_aircraft _headless _tickrate 20
```

This emits a finite sequence of minimal aircraft-part CAS observations. The
example file explicitly configures world/camera frames and a camera-to-world
pose; replace it with your application's calibration. Headless emission needs no
meshes. For optional native mesh verification, configure `AIRCRAFT_MESH_DIRECTORY`
explicitly (for example, `$PWD/plane`) and omit `_headless`.

- [Aircraft configuration, playback, CAS contract, and optional preview](doc/scripted_aircraft.md)
- [Aircraft model catalog and separate belief-consumer setup](doc/aircraft_models.md)

The independent `scripted_tetris` engine retains its own meshes, scenario,
image/geometry contract, and preview. Run it separately:

```bash
export TETRIS_MESH_DIRECTORY="$PWD/robokudo_acrambly/src/robokudo_acrambly/data"
python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae scripted_tetris _headless _tickrate 20
```

For an installed wheel, the mesh directory can be located with
`importlib.resources.files("robokudo_acrambly.data")`. Set `TETRIS_MESH_DIRECTORY`
to that directory, or to another directory containing the configured meshes.

- [Setup, configuration, and headless execution](doc/scripted_tetris.md)
- [Foreground masks](doc/scripted_tetris_masks.md)
- [Native visualization](doc/scripted_tetris_visualization.md)

Custom annotators live in `src/robokudo_acrambly/annotators`, pipeline descriptors
in `src/robokudo_acrambly/descriptors/analysis_engines`, and observation support
in `src/robokudo_acrambly/aircraft_observations` and
`src/robokudo_acrambly/scripted_observations`. The layout follows RoboKudo's
[extension package](https://robokudo.ai.uni-bremen.de/tutorials/create_your_own_package.html)
and [custom annotator](https://robokudo.ai.uni-bremen.de/tutorials/create_your_own_annotator.html)
guides, using this repository's Python workspace packaging.

## Colored-cube perception and ROS query client

After loading the same ROS 2 workspace and Python environment described in the
[setup guide](doc/scripted_tetris.md#setup-and-headless-execution), start the
query-driven colored-cube engine:

```bash
python -m robokudo.scripts.main _ros_pkg robokudo_acrambly _ae stacking_robokudo
```

This engine uses the existing Orbbec camera configuration and requires its camera
topics and transforms. It segments queried block colors and estimates bounding-box
poses from live camera data.

In a second terminal, load the same ROS 2 workspace and Python environment, then
run the standalone query client:

```bash
python -m robokudo_acrambly.scripts.robokudo_cram_integration
```

The client requests blue, red, and yellow blocks from `/robokudo/query` and prints
the detected poses grouped by color. Its existing size filtering and position
transformation remain applied. This client queries the live colored-cube engine;
the synthetic `scripted_tetris` engine supplies observations without a query
annotator.

The engine is imported from
`robokudo_acrambly.descriptors.analysis_engines.stacking_robokudo`; the query API
is imported from `robokudo_acrambly.scripts.robokudo_cram_integration`. Their former
module paths under `robokudo.descriptors.analysis_engines` have been removed.

## Tests

The package tests live in `test/robokudo_acrambly_test`. They reuse RoboKudo's ROS
and blackboard fixtures and run as a separate CI matrix entry.
