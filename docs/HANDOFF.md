# Handoff

State of Robot Studio after milestone 2 (v0.2.0), for whoever builds
milestone 3. Read [decisions.md](decisions.md) first, then
[builder.md](builder.md).

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
- Validation: software tests only. Simulation: not applicable yet.
  Hardware: none.
- urdf-inspector is vendored at **v0.1.1** (commit in
  `vendor/manifest.json`), three.js at **0.187.0**.

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
src/ui/                        browser only (not coverage-gated)
  app.js                       DOM wiring, panels, selection, URL updates, builder session
  builder.js                   template form and properties panel (DOM only)
  view3d.js                    three.js scene, lights, helpers, picking
  meshes.js                    STL/OBJ/DAE loading from the VFS, textures via blob URLs
  files.js                     drop / file input / folder input -> VirtualFs
  state.js                     URL <-> page state (DOM-free, tested)
src/node/nodefs.js             real-file-system XacroFs for the CLI and tests
src/cli.js, bin/robot-studio.js  CLI
vendor/                        vendored code + manifest.json (never edit)
examples/studio_mecanum/       our illustrative model (4 xacro files, STL + OBJ)
examples/third_party/          TurtleBot3, urdf_tutorial, ros2_control_demos (unmodified, with licences)
tests/                         node:test suites; tests/corpus = golden corpus
tests/corpus/builder/          exported template files (xacro, URDF, TOML, reference speeds)
tests/builder-cases.js         which template cases are exported there
scripts/                       vendor.mjs, xacro_golden.py, browser-smoke.mjs, serve.js, generators,
                               builder-fixtures.js, mecanum_interop.py, ros2_package_check.sh
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
- A new upstream module (lidar-slam-lab in milestone 3): add a `git` entry
  to `SOURCES` in `scripts/vendor-lib.js` with the files needed, run
  `node scripts/vendor.mjs`, import from `vendor/lidar-slam-lab/src/...`.

## Milestone 3 starting points

- The robot to simulate is a `RobotDoc`; sensors are links with `sensor`
  set (`lidar2d`, `imu`, `camera`, `depth_camera`). Simulation
  parameters (LiDAR range, IMU noise) belong in new optional fields of the
  document, not in the URDF.
- Wheel geometry: `editor.model.report.context.wheels` (urdf-inspector's
  `measureWheels`), and `mecanumInverse` / `mecanumForward` in
  `src/core/builder/drive.js` (cross-checked against mecanum-kinematics in
  CI). A differential model with the same conventions is the next piece.
- Joint positions drive the view through `view.setPose(q)`; a simulation
  loop can update `state.q` and a base pose (the view has no world offset
  for the root yet: add one in `view3d.js`).
- lidar-slam-lab is vendored the same way as urdf-inspector (decision 2).

## Known issues

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
