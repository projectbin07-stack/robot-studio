# Prior art

Robot Studio is not the first tool that shows a robot description outside a
full ROS install. This page lists the tools we compared it with, what each
one does according to its own documentation or source (read on 2026-11-24,
links inline), and where Robot Studio differs. Where a page could not be
read, that is said rather than guessed.

## RViz / RViz2

- "A 3D visualizer for the Robot Operating System 2 (ROS 2) framework",
  installed with ROS 2. BSD licence
  ([README](https://github.com/ros2/rviz),
  [package.xml](https://github.com/ros2/rviz/blob/humble/rviz2/package.xml)).
- The RobotModel display draws the robot "in the correct pose (as defined by
  the current TF transforms)"; in Humble its Description Source is a topic
  or a file
  ([user guide source](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.rst),
  [robot_model_display.cpp](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp)).
  docs.ros.org itself was not reachable from our automated reader, so the
  tutorial sources on GitHub were read instead.
- RViz does not expand xacro and has no joint sliders: a launch file runs
  xacro, `robot_state_publisher` publishes TF, and `joint_state_publisher_gui`
  provides the sliders
  ([URDF tutorial](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.rst)).
- Meshes: `.stl` through its own loader, everything else (DAE, OBJ, ...)
  through Assimp. For COLLADA it sets
  `AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION` ("ASSIMP wants to change the
  orientation of the axis, but that's wrong for rviz") and keeps the
  `<unit meter>` scale
  ([assimp_loader.cpp](https://github.com/ros2/rviz/blob/humble/rviz_rendering/src/rviz_rendering/mesh_loader_helpers/assimp_loader.cpp),
  [mesh_loader.cpp](https://github.com/ros2/rviz/blob/humble/rviz_rendering/src/rviz_rendering/mesh_loader.cpp)).
  Robot Studio follows the same rule (see [decisions.md](decisions.md)).

RViz is the reference: it is what the robot's own stack shows. Robot Studio
is for the moments without a ROS machine at hand: reviewing a description
in a browser, on a phone, or in a pull request, with the URDF checks next to
the model.

## Foxglove

- The 3D panel loads URDF from a URL, a file path (desktop app only), a
  parameter or a topic; `package://` and `file://` URLs need the desktop app
  or a connection that can fetch assets. Joint control is by TF or joint
  states ([3D panel docs](https://docs.foxglove.dev/docs/visualization/panels/3d)).
- The panel docs say nothing about expanding xacro; the URDF marketing page
  says only that URDF "can be combined with Xacro"
  ([foxglove.dev/urdf](https://www.foxglove.dev/urdf)). We found no documented
  in-app xacro expansion.
- The open-source edition was discontinued in March 2024; the last
  open-source release was v1.87.0 (MPL-2.0) and future development is closed
  source ([announcement](https://discourse.openrobotics.org/t/foxglove-2-0-integrated-ui-new-pricing-and-open-source-changes/36583),
  [Foxglove blog](https://foxglove.dev/blog/foxglove-vs-foxglove-studio-two-years-on)).
  The `foxglove/studio` repository is archived.

## urdf-viz

- Rust viewer on kiss3d with binaries for macOS, Linux and Windows,
  Apache-2.0. Meshes: OBJ, STL, DAE (others with the `assimp` feature)
  ([README](https://github.com/openrr/urdf-viz)).
- xacro is expanded by running the external command: `rosrun xacro xacro`,
  falling back to `xacro`
  ([urdf-rs utils.rs](https://github.com/openrr/urdf-rs/blob/main/src/utils.rs)),
  so xacro input needs ROS or the xacro Python package installed.
- Joints move with keys or Ctrl+drag; it has position IK and an HTTP API. A
  wasm build has a live demo
  ([wasm README](https://github.com/openrr/urdf-viz/blob/main/examples/wasm/README.md)).

## Phobos

- Blender add-on and CLI from DFKI / University of Bremen for creating and
  editing models; imports and exports URDF, SDF and SMURF, exports STL, OBJ
  and DAE meshes, BSD-3-Clause
  ([README](https://github.com/dfki-ric/phobos)). The README does not mention
  xacro. Phobos is a modelling tool in Blender; Robot Studio's later builder
  milestone is much lighter and browser-only.

## Robot Web Tools (ros3djs) and urdf-loaders

- ros3djs (BSD) is built on three.js r89 with a patched ColladaLoader;
  `ROS3D.UrdfClient` reads the model from the ROS parameter server through
  rosbridge, so it needs a running ROS system
  ([README](https://github.com/RobotWebTools/ros3djs),
  [UrdfClient.js](https://github.com/RobotWebTools/ros3djs/blob/develop/src/urdf/UrdfClient.js)).
- gkjohnson's `urdf-loader` (Apache-2.0) is a three.js URDF loader and web
  component; its default mesh loader handles STL and DAE; it has no xacro
  support and points to the separate `xacro-parser`
  ([README](https://github.com/gkjohnson/urdf-loaders/blob/master/javascript/README.md)).
- `xacro-parser` (MIT) evaluates expressions with the `expr-eval` library and
  lists as not implemented: `^|default` pass-through, `xacro:call` and
  include namespaces; it states that arbitrary Python in `${}` cannot be
  supported ([README](https://github.com/gkjohnson/xacro-parser)).
  Robot Studio implements all three and evaluates a Python expression subset
  with Python's number semantics, checked against the real xacro (see
  [xacro.md](xacro.md)).

## Online viewers

- The urdf-loaders demo page accepts dropped URDF files or folders (Chrome
  only, per the page), with options for joint limits, collision geometry and
  the up axis; it does not mention xacro
  ([demo](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html)).
- Robot Viewer (viewer.robotsfan.com, Apache-2.0, three.js) lists URDF,
  "Xacro: ROS Xacro format with macro expansion and conditional logic
  support", MJCF and partial USD, all processed in the browser
  ([README](https://github.com/fan-ziqi/robot_viewer)). We did not test how
  complete its xacro support is.
- mymodelrobot.appspot.com was not checked.

## Where Robot Studio sits

|                             | Runs without ROS  | xacro expansion               | Checked against official xacro | URDF lint checks              | Meshes                        | Licence                  |
| --------------------------- | ----------------- | ----------------------------- | ------------------------------ | ----------------------------- | ----------------------------- | ------------------------ |
| RViz2                       | no                | via launch / xacro CLI        | is the reference stack         | no (urdfdom errors only)      | STL, DAE, OBJ, ...            | BSD                      |
| Foxglove                    | yes               | not documented                | n/a                            | no                            | glTF, STL, DAE, OBJ (markers) | closed source since 2024 |
| urdf-viz                    | yes, except xacro | runs external xacro           | uses it                        | no                            | OBJ, STL, DAE                 | Apache-2.0               |
| urdf-loaders + xacro-parser | yes               | partial (see above)           | not stated                     | no                            | STL, DAE                      | Apache-2.0 / MIT         |
| Robot Viewer                | yes               | yes (extent not tested by us) | not stated                     | no                            | per README                    | Apache-2.0               |
| Robot Studio 0.1            | yes               | yes, in the browser           | golden tests in CI             | 71 checks from urdf-inspector | STL, OBJ, DAE                 | MIT                      |

The "checked against official xacro" column for Robot Studio means: every
input in `tests/corpus` is expanded by the official `xacro` 2.1.1 Python
package in CI and compared with Robot Studio's output. It does not mean
every xacro file in the wild works; the unsupported features are listed in
[xacro.md](xacro.md).
