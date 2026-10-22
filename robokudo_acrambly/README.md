# RoboKudo ACRAMbly

`robokudo_acrambly` owns ACRAMbly's custom RoboKudo annotators, analysis engines,
and example mesh resources. It depends on the workspace's existing `robokudo`
package and extends its public interfaces.

Install it with the repository's normal `uv sync --active` workflow after setting
up the ROS 2 environment. The workspace's `sensor_msgs` and other ROS dependencies
must remain available to Python.

Run the supplied synthetic observation engine from the repository root:

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
in `src/robokudo_acrambly/scripted_observations`. The layout follows RoboKudo's
[extension package](https://robokudo.ai.uni-bremen.de/tutorials/create_your_own_package.html)
and [custom annotator](https://robokudo.ai.uni-bremen.de/tutorials/create_your_own_annotator.html)
guides, using this repository's Python workspace packaging.

The package tests live in `test/robokudo_acrambly_test`. They reuse RoboKudo's ROS
and blackboard fixtures and run as a separate CI matrix entry.
