// Minimal static file server for local development (ES modules do not load
// from file:// URLs). Serves the repository root on http://127.0.0.1:8000
// (PORT=... to change). GitHub Pages serves the same files in production.
// To use the page with a robot, `robot-studio studio` does the same and opens
// the browser (docs/ros2.md).

import { startServer } from "../src/node/serve.js";

const { url } = await startServer({ port: Number(process.env.PORT ?? 8000) });
console.log(`serving ${url}`);
