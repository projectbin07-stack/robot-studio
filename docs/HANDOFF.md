# Handoff

State of Robot Studio after milestone 3 (v0.3.0), for whoever builds
milestone 4. Read [decisions.md](decisions.md) first, then
[builder.md](builder.md) and [simulation.md](simulation.md).

## Status

- Milestone 1 scope is implemented: xacro expansion, model loading
  (file / folder / zip / examples), three.js view, live checks, panels,
  URL state, themes, phone layout, CLI.
- Milestone 2 scope is implemented: editable robot document with typed
  operations and undo/redo, templates (mecanum, differential, arm) with
  sensor mounts, properties panel, inertia from geometry, URDF / xacro /
  mecanum-kinematics TOML / ROS 2 package exports, autosave, CLI `new`,
  `export`, `templates`. Status of each acceptance criterion in
  [ROADMAP.md](ROADMAP.md#milestone-2-builder-v020-done).
- Milestone 3 scope is implemented: simulation of the open robot (mecanum
  or differential, detected from the URDF) with the drive mirrored from
  encoder-odometry-firmware, LiDAR / IMU / encoders / depth point cloud at
  the URDF's sensor frames, wheel odometry, scan matching and MCL from
  lidar-slam-lab, worlds with heights and an editor, live 3D layers, plots,
  HUD and teleop, a deterministic CLI recording. Status of each criterion in
  [ROADMAP.md](ROADMAP.md#milestone-3-simulation-v030-done).
- Validation: software tests; simulation (model output, labelled
  simulated); hardware none.
- Vendored: urdf-inspector **v0.1.1**, lidar-slam-lab **v0.1.0** (commits in
  `vendor/manifest.json`), three.js **0.187.0**.

## Module map

```
index.html, styles.css         page and styles (import map for three.js)
src/core/                      DOM-free, tested in Node, coverage-gated
  xacro/dom.js                 XML reader with ordered mixed content, CDATA, comments, lines
  xacro/pyexpr.js              tokenizer + parser for the Python expression subset
  xacro/pyvalue.js             Python values (BigInt ints), repr/str, arithmetic, formatting
  xacro/pyeval.js              evaluator, builtins, math, str/list/dict methods
  xacro/table.js               scoped symbol tables with lazy evaluation
  xacro/process.js             the xacro processor (directives, macros, includes, $() args)
  xacro/serialize.js           xacro-identical output; conversion to urdf-inspector's XmlElement
  xacro/index.js               public API
  vfs.js                       virtual file system, packages, package:// resolution, addZip
  zip.js                       zip reader (stored + deflate, CRC-32)
  paths.js                     forward-slash path helpers, glob
  model.js                     loadModel(): xacro -> urdf-inspector report -> findings with files, meshes
  scene.js                     what the 3D view draws: shapes, colours, poses, sliders, bounds
  examples.js                  bundled example catalogue and exampleFs()
  canonical.js                 canonical XML comparison for the golden tests
  zipwrite.js                  stored zip writer (package export)
  builder/document.js          RobotDoc (plain JSON), tree helpers, unique names, sensor guess
  builder/ops.js               typed edit operations (EditOp), validation, rename propagation
  builder/editor.js            Editor: history, undo/redo, modelFromDoc() -> loadModel()
  builder/inertia.js           inertial from a shape (urdf-inspector primitiveInertia + rotateTensor)
  builder/urdf.js              RobotDoc <-> URDF (the writer uses the xacro serializer)
  builder/templates.js         templates: params, document generator, xacro writer, validation
  builder/actions.js           add child / add sensor / collision from visual as one step
  builder/drive.js             mecanum TOML (urdf-inspector) and IK/FK from measured wheels
  builder/export.js            xacro export, ROS 2 package files and zip
  builder/storage.js           autosave format
  sim/se2.js                   SE(2) exp/log/compose (the firmware's odometry maths)
  sim/firmware.js              encoder-odometry-firmware mirrored: IK, saturation, velocity estimator, PI, watchdog, odometry
  sim/plant.js                 motor + gearbox per wheel, exact encoder edges (the firmware's Python plant)
  sim/robot.js                 simRobot(): drive kind, wheel geometry, sensor mounts from FK, Gazebo tags, footprint
  sim/world.js                 worlds with heights, bundled worlds, JSON and link encodings
  sim/imu.js                   IMU error model, imu-calibration-lab JSON and Gazebo noise import
  sim/depth.js                 depth camera point cloud
  sim/estimators.js            wheel odometry + scan matching + MCL over lidar-slam-lab, virtual scans
  sim/reference.js             lidar-slam-lab's own experiment through the wrapper (cross-check)
  sim/sim.js                   the fixed-step loop: createSim(), step()
  sim/routes.js                demo (autopilot), square, spin, none
  sim/settings.js              defaults, limits, normalizeSettings(), settingsDiff()
  sim/record.js                recording files and summary.json
  sim/session.js               the page's real-time session: fixed steps, teleop, plot histories
src/ui/                        browser only (not coverage-gated)
  app.js                       DOM wiring, panels, selection, URL updates, builder session
  builder.js                   template form and properties panel (DOM only)
  view3d.js                    three.js scene, lights, helpers, picking
  meshes.js                    STL/OBJ/DAE loading from the VFS, textures via blob URLs
  files.js                     drop / file input / folder input -> VirtualFs
  state.js                     URL <-> page state (DOM-free, tested)
  simui.js                     Simulate panel, HUD, plots, teleop, frame loop
  simview.js                   the simulation in 3D: world, LiDAR, map, trails, particles, cloud, frames
  plots.js                     canvas time plots
  worldedit.js                 top-down world editor
src/node/nodefs.js             real-file-system XacroFs for the CLI and tests
src/cli.js, bin/robot-studio.js  CLI
src/simcli.js                  the `sim` command
vendor/                        vendored code + manifest.json (never edit)
examples/studio_mecanum/       our illustrative model (4 xacro files, STL + OBJ)
examples/third_party/          TurtleBot3, urdf_tutorial, ros2_control_demos (unmodified, with licences)
tests/                         node:test suites; tests/corpus = golden corpus
tests/corpus/builder/          exported template files (xacro, URDF, TOML, reference speeds)
tests/builder-cases.js         which template cases are exported there
tests/corpus/sim/              simulator fixtures for CI cross-checks, golden digest, a test world
scripts/                       vendor.mjs, xacro_golden.py, browser-smoke.mjs, serve.js, generators,
                               builder-fixtures.js, mecanum_interop.py, ros2_package_check.sh,
                               sim-fixtures.js, firmware_interop.py, lsl-crosscheck.mjs
```

## How to run everything

```bash
npm ci
npm run check                      # format, typecheck, tests with coverage
npm run serve                      # http://127.0.0.1:8000/ (PORT=... to change)
node bin/robot-studio.js check examples/studio_mecanum/urdf/studio_mecanum.urdf.xacro

# goldens (needs Python; keep the venv out of git)
python -m venv .venv && .venv/Scripts/python -m pip install -r scripts/requirements-golden.txt   # bin/ on Linux
.venv/Scripts/python scripts/xacro_golden.py          # regenerate after adding a case
.venv/Scripts/python scripts/xacro_golden.py --check  # CI runs this on Linux; on Windows the 2 studio_mecanum goldens differ in 4 last digits (C library sin)
node scripts/golden-report.js                          # table for docs/xacro.md

# vendored code
node scripts/vendor.mjs            # re-vendor after changing a pin in scripts/vendor-lib.js
node scripts/vendor.mjs --verify   # what CI runs (needs network)

# browser smoke test / screenshots (Chrome or Edge)
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11 --screenshot shot.png
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --builder   # template, edit, Ctrl+Z, export, reload
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template arm --select-link link_3 --screenshot arm.png

# builder exports
node scripts/builder-fixtures.js           # rewrite tests/corpus/builder after changing a template or writer
node scripts/builder-fixtures.js --check   # what CI runs
# then regenerate the goldens on Linux (roller positions use sin/cos, last bit differs on Windows)
PYTHONPATH=../mecanum-kinematics/src python scripts/mecanum_interop.py   # CI installs mecanum-kinematics v0.1.0
node bin/robot-studio.js new mecanum -f package -o pkg.zip   # CI builds such zips with colcon in ros:humble-ros-base

# simulation
node bin/robot-studio.js sim mecanum --world office --route demo --seconds 60 --seed 1 --out run/
node scripts/sim-fixtures.js [--check]       # fixtures for the cross-checks and the golden digest
PYTHONPATH=../encoder-odometry-firmware/host/src python scripts/firmware_interop.py   # CI installs odobridge v0.1.0
node scripts/lsl-crosscheck.mjs --lsl ../lidar-slam-lab [--seeds 5] [--full-seeds 1]  # about 15 min locally
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --template mecanum --sim --sim-world lab --screenshot sim.png
node scripts/browser-smoke.mjs ... --headed --gpu   # visible window with the GPU, for frame rates
```

On this development machine: work only on D:, set
`TMP=/d/github-engineering-operations/.tmp`, `npm_config_cache` and
`PIP_CACHE_DIR` there, and use `PORT=8765` for the dev server if 8000 is
taken.

## Adding things

- A xacro feature: add a fixture to `tests/corpus/features/`, a case to
  `tests/corpus/cases.json`, run the golden generator, make the test pass.
- An example: put files under `examples/`, list them in
  `src/core/examples.js` (a test fails for unlisted or missing files), add
  the expected link count to `tests/model.test.js` and a line to the CI
  browser job.
- A new upstream module or file: add it to `SOURCES` in
  `scripts/vendor-lib.js` (lidar-slam-lab's files are the `LIDAR_SLAM` list),
  run `node scripts/vendor.mjs`, import from `vendor/<name>/...`.
- A world: add it to `WORLDS` in `src/core/sim/world.js` (the tests
  round-trip every bundled world).

## Milestone 4 starting points

- The recording format and its topic mapping are in
  [simulation.md](simulation.md#recording-format-and-the-ros-2-bag-mapping);
  `src/core/sim/record.js` writes it. A bag importer should produce the same
  streams so the estimators and views run on recorded data unchanged
  (`estimators.js` takes an odometry delta, a scan and a mount).
- `SimSession` (`src/core/sim/session.js`) is the shape a live session
  should have too: something that advances, a snapshot for the view, plot
  histories. `simview.js` draws from a session; a rosbridge session can
  feed the same layer.
- Teleop (`manual()` in `simui.js`) already produces a body twist with speed
  limits from the robot's wheels: the input side of `/cmd_vel`. Add the
  dead-man switch before sending anything to a real robot.
- The firmware's telemetry fields are in the recording, so a bag from the
  MentorPi (or odobridge's CSV) can be compared with a simulated run of the
  same commands.
- Headless browsers render with SwiftShader (10 to 14 fps); the browser job
  checks simulated seconds and errors, not frame rate. On Windows the smoke
  script ends every browser process started with its profile directory (Edge
  hands over to a new process that a plain kill leaves running).

## How the simulation hangs together

- The robot is the open model plus its `RobotDoc` (builder) or the model
  imported as a document (viewer): `simRobot(model, doc)`. Sensor tags and
  Gazebo extras come from the document; geometry from urdf-inspector's
  report. Simulation settings live in `doc.sim` (setSimulation operation)
  and in the link (`sim`, `w`, `r`, `ss`).
- The 3D view places the model's root frame with `view.setBase(matrix)` and
  turns the wheels with `view.setPose(q)`; `view.holdRender` lets the frame
  loop draw once per animation frame.
- Determinism: one seeded stream per noise source. Adding a random draw to
  an existing stream changes every later number and the golden digest in
  `tests/corpus/sim/golden-digest.json`; regenerate it with
  `node scripts/sim-fixtures.js` only after checking why it changed.
- Changing the drive or firmware mirror: regenerate the fixtures and run
  `scripts/firmware_interop.py` and `scripts/mecanum_interop.py`.

## Known issues

- Simulation: see [simulation.md](simulation.md#limitations) (2D motion only,
  one LiDAR, lidar-slam-lab's MCL motion model has no heading bias and reads
  a strafe as two rotations, scan matching has no loop closure).

- `xacro.load_yaml` is not implemented (no YAML reader); descriptions that
  need it stop with a clear error.
- Python comprehensions, lambda, f-strings and a few builtins are not
  supported in `${}` (list in [xacro.md](xacro.md)).
- Mesh formats: STL, OBJ (without `.mtl` materials) and DAE only; glTF and
  others show a placeholder and a finding.
- Floating and planar joints have no sliders (urdf-inspector's FK shows
  them at their origin).
- The camera is framed on rough bounds first and re-framed on the real
  bounding box once meshes have loaded; the helper sizes (axes, centre of
  mass marker) use the rough size.
- Large models: every mesh is loaded at once; no level of detail.
- The page has no automated accessibility check yet.
- Builder: no transform gizmo (numeric fields only); mimic and dynamics
  have no fields in the Properties panel (the operations support them);
  importing a xacro file imports its expansion (macros and properties are
  not kept); after a reload an imported robot's meshes are placeholders
  until its files are opened again; the differential template has no
  mecanum-kinematics export (that library models mecanum bases only);
  re-rendering the template form after a parameter edit moves keyboard
  focus out of it.
- urdf-inspector requests are listed in
  [upstream-requests.md](upstream-requests.md).
