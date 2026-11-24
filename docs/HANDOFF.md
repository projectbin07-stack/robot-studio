# Handoff

State of Robot Studio after milestone 1 (v0.1.0), for whoever builds
milestone 2. Read [decisions.md](decisions.md) first.

## Status

- Milestone 1 scope is implemented: xacro expansion, model loading
  (file / folder / zip / examples), three.js view, live checks, panels,
  URL state, themes, phone layout, CLI.
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
src/ui/                        browser only (not coverage-gated)
  app.js                       DOM wiring, panels, selection, URL updates
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
scripts/                       vendor.mjs, xacro_golden.py, browser-smoke.mjs, serve.js, generators
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
.venv/Scripts/python scripts/xacro_golden.py --check  # what CI runs
node scripts/golden-report.js                          # table for docs/xacro.md

# vendored code
node scripts/vendor.mjs            # re-vendor after changing a pin in scripts/vendor-lib.js
node scripts/vendor.mjs --verify   # what CI runs (needs network)

# browser smoke test / screenshots (Chrome or Edge)
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11 --screenshot shot.png
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
- urdf-inspector requests are listed in
  [upstream-requests.md](upstream-requests.md).
