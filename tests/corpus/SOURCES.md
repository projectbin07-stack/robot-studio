# Corpus sources

The golden tests (`tests/xacro-golden.test.js`) expand every case in
[`cases.json`](cases.json) with Robot Studio and compare the result with the
output of the official `xacro` 2.1.1 Python package, stored in
[`golden/`](golden/) and regenerated in CI by `scripts/xacro_golden.py`.

## Written for this project (MIT, like the rest of the repository)

| Files | What they exercise |
|---|---|
| `features/expressions.xacro` | Number formatting and int/float semantics, strings, containers, logic, builtins, `math` |
| `features/scopes.xacro` | Lazy properties, computed names, `lazy_eval`, `default`, `remove`, `scope="parent"` / `"global"`, block properties, shadowing |
| `features/macros.xacro` | Parameters, `:=`/`=` defaults, `^` and `^\|` inheritance, `*` and `**` blocks, recursion, local macros, `xacro:call`, redefinition, typed parameters |
| `features/conditionals.xacro` | `xacro:if` / `xacro:unless` values, nesting, `xacro:element`, `xacro:attribute`, mixed text, CDATA, comments, `xacro:eval-comments` |
| `features/includes.xacro`, `features/inc/`, `features/pkgs/demo_parts/` | Relative includes, `$(find)`, `ns=` namespaces, glob includes, `optional` |
| `features/args.xacro` | `xacro:arg` defaults and overrides, `$(arg)`, `$(eval)`, `$(optenv)`, `xacro.arg()`, `xacro.tokenize()` |
| `features/errors/*.xacro` | Inputs the official xacro rejects; Robot Studio must reject them too |
| `features/lambda_expression.xacro` | A construct xacro accepts and Robot Studio deliberately does not (documented gap) |
| `examples/studio_mecanum/` | The bundled example robot, two argument sets |

## Third-party robot descriptions

Kept unmodified under `examples/third_party/` with their licence files, and
also shipped as examples in the web app with attribution.

| Package | Source | Commit | Licence | Files used |
|---|---|---|---|---|
| `turtlebot3_description` | https://github.com/ROBOTIS-GIT/turtlebot3 (branch `noetic`) | `4ae959ea6a52415c90bf752d2b76e3e28f8a87e2` | Apache-2.0 (`examples/third_party/turtlebot3_description/LICENSE`) | `package.xml`, `urdf/common_properties.xacro`, `urdf/turtlebot3_{burger,waffle,waffle_pi}.urdf.xacro` and `.gazebo.xacro`, the four Burger STL meshes |
| `urdf_tutorial` | https://github.com/ros/urdf_tutorial (branch `ros2`) | `050f1e47cfdb2c5f3eb0746bc15c57e6a870faef` | BSD-3-Clause (`examples/third_party/urdf_tutorial/LICENSE`) | `package.xml`, `urdf/08-macroed.urdf.xacro`, `meshes/l_finger*.dae` and their PNG textures |
| `ros2_control_demo_description`, `ros2_control_demo_example_1` | https://github.com/ros-controls/ros2_control_demos (branch `humble`) | `9f2199498571fa0ccb57a646a4691e0ab816f783` | Apache-2.0 (`examples/third_party/ros2_control_demos/LICENSE`) | `ros2_control_demo_description/package.xml`, `rrbot/urdf/rrbot_description.urdf.xacro`, `rrbot/urdf/rrbot.materials.xacro`; `example_1/package.xml`, `example_1/description/urdf/rrbot.urdf.xacro`, `example_1/description/ros2_control/rrbot.ros2_control.xacro` |

The TurtleBot3 Waffle and Waffle Pi descriptions are expanded in the golden
tests but their meshes are not included (the Waffle base STL alone is 16 MB),
so the web app shows them only through the Burger example.

`ros2_control_demo_example_1` installs its `description/` folder into its
share directory, so `$(find ros2_control_demo_example_1)` points at
`example_1/description` in `cases.json`, as it would in an installed
workspace.

## How `$(find)` is resolved for the goldens

ROS 2's xacro resolves `$(find pkg)` with `ament_index_python`, which is not
on PyPI. `scripts/xacro_golden.py` replaces that single lookup with the
package folders listed in `cases.json`; the rest of the official code runs
unmodified. The repository path is written as `@ROOT@` in the goldens so they
are identical on every machine.
