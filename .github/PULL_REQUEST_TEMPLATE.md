## What this changes

<!-- One or two sentences. Link the issue if there is one. -->

## How it was verified

- [ ] `npm run check` passes (format, typecheck, tests with coverage)
- [ ] xacro changes: a fixture in `tests/corpus/` and its golden regenerated with `python scripts/xacro_golden.py`
- [ ] Vendored files untouched (or re-vendored with `node scripts/vendor.mjs` and the manifest updated)
- [ ] Checked in a browser if the app changed, `node scripts/browser-smoke.mjs` passes
- [ ] `CHANGELOG.md` updated
