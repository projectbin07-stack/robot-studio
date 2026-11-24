# Upstream requests for urdf-inspector

Robot Studio vendors urdf-inspector's `src/core` unchanged (see
[decisions.md](decisions.md#2-reuse-by-vendoring-at-pinned-versions)).
These are changes that would let it reuse more and duplicate less. None of
them was made in urdf-inspector during this milestone.

| #   | Request                                                                                                                     | Why Robot Studio needs it                                                                                                     | Current workaround                                                                        |
| --- | --------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| 1   | `xml.js`: optional ordered mixed content (text, CDATA and comment nodes kept in order), alongside the current merged `text` | xacro moves mixed content and must not evaluate CDATA; Robot Studio carries its own reader (`src/core/xacro/dom.js`) for this | Own reader, converted to `XmlElement` after expansion                                     |
| 2   | An `exports` map in `package.json` (also raised in urdf-inspector's audit, #35)                                             | A stable public module list makes it clear which files are API and safe to vendor                                             | Vendoring by file path, pinned per tag                                                    |
| 3   | `Finding` with an optional `source` (file) next to `line`                                                                   | After xacro expansion an element's line belongs to one of several files                                                       | `src/core/model.js` maps each finding's element path to the file of its top-level element |
| 4   | `report.js`: let `checkText` take a function that formats a finding's location                                              | The CLI wants `file:line` instead of `(line N)`                                                                               | The CLI formats findings itself and reuses only `summaryLines`                            |
| 5   | `fk.js`: export joint frames in the root frame (parent pose × joint origin) for given positions                             | The 3D view draws joint axes at the joint frame                                                                               | `src/core/scene.js` composes it with the exported `compose` and `fromXyzRpy`              |
| 6   | Mesh checks: accept an optional "does this mesh exist" callback                                                             | Missing meshes are the most common problem with dropped packages                                                              | Robot Studio adds its own `mesh-missing` finding                                          |

Pin history: v0.1.0 at the start of milestone 1, moved to **v0.1.1**
(adds the `inertia-axes` check and `symEigen`, `principalAxisMismatch`;
nothing removed) before the v0.1.0 release of Robot Studio.
