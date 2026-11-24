# Design decisions

Short records of the choices that shape the code, with the reason and what
would change them.

## 1. Static browser app, ES modules, no build step

Same conventions as urdf-inspector: `index.html` loads `src/ui/app.js`
directly; types are JSDoc checked by `tsc --checkJs --strict`; Prettier;
`node:test` with coverage thresholds on `src/core/`. GitHub Pages serves
the repository files as they are. Everything that computes something lives
in `src/core/` (DOM-free, tested under Node); `src/ui/` only wires the DOM
and three.js.

## 2. Reuse by vendoring at pinned versions

urdf-inspector's URDF reader, 71+ checks, forward kinematics, inertia and
wheel code are used, not rewritten. They are copied by
`scripts/vendor.mjs` from the tag recorded in `scripts/vendor-lib.js`
(currently `v0.1.1`, commit in `vendor/manifest.json`) into
`vendor/urdf-inspector/src/core/`, with a SHA-256 per file. The CI job
`vendor` re-clones the tag and fails if any byte differs or the tag moved;
`tests/vendor.test.js` checks the manifest offline. Vendored files are
never edited: changes go to urdf-inspector first
([upstream-requests.md](upstream-requests.md)), then the pin moves.

Why not an npm dependency or a git submodule: urdf-inspector is not
published to npm and has no `exports` map; a submodule would not be served
by GitHub Pages without a build step and does not pin individual files.
Vendoring keeps the deployed site self-contained and the provenance
checkable.

lidar-slam-lab (later milestones) has a flat `src/` of DOM-free modules
(`geometry.js`, `lidar.js`, `world.js`, `grid.js`, ...); it will be added as
a third `SOURCES` entry vendored to `vendor/lidar-slam-lab/src/` with the
same tooling, no change to the scheme.

## 3. three.js is the one third-party runtime library

The 3D view must show real meshes (STL, OBJ and COLLADA with textures),
with lighting, shadows and picking good enough to compare with RViz.
urdf-inspector's hand-written canvas projection draws only primitives in
wireframe/flat shading; writing a mesh renderer, three mesh parsers
(COLLADA alone is a large format) and texture handling would be a project
of its own and less reliable than three.js, which is MIT-licensed, widely
used and has maintained loaders for exactly these formats.

Only what is used is vendored: `build/three.module.js`, `three.core.js`
and the addons `OrbitControls`, `STLLoader`, `OBJLoader`, `ColladaLoader`
(plus the `collada/` parser and composer and `TGALoader`, which it
imports), with three.js's `LICENSE`. Release `0.187.0`, fetched by
`npm pack` (registry integrity recorded in the manifest), resolved in the
page through an import map. Nothing is loaded from a CDN. `three` and
`@types/three` are also dev dependencies at the same version so `tsc`
type-checks `src/ui/` against them; Dependabot ignores both because a bump
means re-vendoring.

## 4. Our own XML document model for xacro

urdf-inspector's `xml.js` concatenates an element's character data and
drops comments. xacro needs children in document order with text, CDATA
and comments as separate nodes: blocks and conditionals move mixed content,
CDATA must not be evaluated (minidom keeps it as a separate node type),
comments survive into the output and `xacro:eval-comments` evaluates them.
`src/core/xacro/dom.js` is that reader (with line numbers and the XML rules
xacro's expat parser applies: line-end and attribute-value normalisation).
After expansion the tree is converted to urdf-inspector's `XmlElement`
shape (`toXmlElement`) so the vendored URDF reader and checks run
unchanged. An option for ordered mixed content in urdf-inspector's reader
is listed as an upstream request.

## 5. Expressions: a Python subset with Python's number semantics

xacro evaluates `${}` with Python's `eval`. A browser tool cannot, and must
not use JavaScript `eval`/`Function` on untrusted files either. The
evaluator (`pyexpr.js` parser, `pyeval.js` evaluator, `pyvalue.js` value
model) implements the expression grammar robot descriptions use, with
Python's semantics where they differ from JavaScript: ints are BigInts and
floats are numbers (so `6/3` prints `2.0` and `2**100` is exact), floor
division and modulo follow Python's signs, `round` is half-to-even on the
exact binary value, `str()` of a float is Python's shortest repr. Names
come only from xacro's documented symbol table; attribute access to names
starting with `_` and anything outside the grammar is rejected.

## 6. COLLADA up axis ignored, unit applied (as RViz)

three.js's ColladaLoader rotates `Z_UP` files by -90° about X (to its Y-up
convention) and scales by `<unit meter>`. RViz ignores the up axis
(`rviz_rendering/src/rviz_rendering/mesh_loader_helpers/assimp_loader.cpp`
sets `AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION`, with the comment
"ASSIMP wants to change the orientation of the axis, but that's wrong for
rviz") and keeps the unit scale. Robot Studio resets the loader's rotation
and keeps its scale, so meshes look as they do in RViz. The world is Z-up
(REP 103): `Object3D.DEFAULT_UP` and the camera's up vector are +Z and the
grid lies in the XY plane.

## 7. Files stay in the browser

Dropped files, folders and zips become an in-memory virtual file system
(`src/core/vfs.js`). Text files (`.xacro`, `.urdf`, `.xml`, `package.xml`,
`.yaml`, ...) are read up front because xacro includes are processed
synchronously, like the reference implementation; meshes and images are
read when the view needs them. Missing textures are replaced by a built-in
1×1 pixel instead of letting three.js request a URL, so the page never
makes network requests for model content. A link to a dropped model cannot
reproduce it and says so.

## 8. Zip reading with DecompressionStream

The zip reader (`src/core/zip.js`) parses the central directory and
inflates deflate entries with the platform's `DecompressionStream
("deflate-raw")`, available in current browsers and Node 22+. Stored and
deflate cover what zip tools write by default; ZIP64, encryption and other
methods are rejected with a message. It is tested against a zip written by
Python's `zipfile` and archives built in the test.

## 9. Golden tests against the official xacro, with `$(find)` substituted

The goldens are produced by the unmodified `xacro` 2.1.1 from PyPI. ROS 2's
xacro resolves `$(find)` with `ament_index_python`, which is not on PyPI;
the generator replaces that one function with the package map of each test
case. Everything else (parsing, evaluation, macro expansion, output) is the
official code.
