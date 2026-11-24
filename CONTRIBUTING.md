# Contributing

Bug reports with a minimal input (and, for xacro, what the official `xacro`
prints for it), corrections, and pull requests are welcome.

## Setup

```bash
git clone https://github.com/projectbin07-stack/robot-studio.git
cd robot-studio
npm ci            # dev tools only: TypeScript (type checking), Prettier, Node and three.js types
npm run serve     # http://127.0.0.1:8000/
```

The site has no build step: `index.html` loads the ES modules in `src/` and
the vendored code in `vendor/` directly. Any static file server works.

## Checks

```bash
npm run check                      # format, typecheck, tests with coverage thresholds on src/core
node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11
python scripts/xacro_golden.py --check   # needs: pip install -r scripts/requirements-golden.txt
node scripts/vendor.mjs --verify         # needs network
```

CI runs all of these on Ubuntu and Windows with Node 22 and 24, plus a
headless Chrome job that loads every example.

## Guidelines

- `src/core/` stays DOM-free and tested; `src/ui/` only wires the DOM and
  three.js. Anything that computes a number belongs in `src/core/`.
- Never edit `vendor/`. Change urdf-inspector (or three.js's pin) upstream,
  then re-vendor with `node scripts/vendor.mjs`.
- xacro behaviour is defined by the official implementation: add a fixture
  to `tests/corpus/`, regenerate its golden with the official xacro, and
  make Robot Studio match it. Do not edit goldens by hand.
- No `eval`, `new Function` or network requests for model content. Model
  names and values go into the page as text, never as HTML.
- Example models: our own models are written for this project and labelled
  illustrative; third-party descriptions only with a licence that allows
  redistribution, unmodified, listed in `tests/corpus/SOURCES.md`.
- Keep simulation and hardware claims out of the docs unless they were run
  and recorded (see the validation tiers in the README).
- Types are JSDoc comments checked by `tsc --checkJs` in strict mode.
  Small, focused commits with imperative messages.
