// A small static file server for the page (ES modules do not load from
// file:// URLs). Used by `robot-studio studio` (the one-command local launch
// for connecting to a robot) and by scripts/serve.js (development). It
// listens on 127.0.0.1 only: nothing on the network can reach it.

import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { extname, join, normalize, resolve, sep } from "node:path";
import { fileURLToPath } from "node:url";

/** The repository (or installed package) root: index.html, src/, vendor/, examples/. */
export const PAGE_ROOT = resolve(fileURLToPath(new URL("../..", import.meta.url)));

/** @type {Record<string, string>} */
export const TYPES = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json",
  ".png": "image/png",
  ".svg": "image/svg+xml",
  ".urdf": "application/xml; charset=utf-8",
  ".xacro": "application/xml; charset=utf-8",
  ".xml": "application/xml; charset=utf-8",
  ".stl": "model/stl",
  ".obj": "text/plain; charset=utf-8",
  ".dae": "model/vnd.collada+xml",
  ".jpg": "image/jpeg",
  ".zip": "application/zip",
  ".md": "text/markdown; charset=utf-8",
};

/**
 * Map a request path to a file under root, or null when it leaves the root.
 * @param {string} root
 * @param {string} urlPath the request's pathname (percent-encoded)
 * @returns {string | null}
 */
export function resolveRequest(root, urlPath) {
  let decoded;
  try {
    decoded = decodeURIComponent(urlPath);
  } catch {
    return null;
  }
  if (decoded.includes("\0")) return null;
  const rel = normalize(decoded).replace(/^[\\/]+/, "");
  const path = join(root, rel === "" || rel === "." ? "index.html" : rel);
  if (!path.startsWith(root + sep) && path !== root) return null;
  return path;
}

/**
 * Start serving `root` on 127.0.0.1. With `port` 0 the system picks a free
 * port; with `tries` > 1 the next ports are tried when one is taken.
 * @param {{ root?: string, port?: number, tries?: number }} [opts]
 * @returns {Promise<{ server: import("node:http").Server, port: number, url: string }>}
 */
export async function startServer(opts = {}) {
  const root = opts.root ?? PAGE_ROOT;
  const server = createServer(async (req, res) => {
    const url = new URL(req.url ?? "/", "http://localhost");
    const path = resolveRequest(root, url.pathname);
    if (path === null) {
      res.writeHead(403).end();
      return;
    }
    try {
      const body = await readFile(path);
      res.writeHead(200, {
        "content-type": TYPES[extname(path)] ?? "application/octet-stream",
        "cache-control": "no-cache",
      });
      res.end(body);
    } catch {
      res.writeHead(404).end("not found");
    }
  });
  const first = opts.port ?? 8000;
  const tries = Math.max(1, opts.tries ?? 1);
  for (let k = 0; k < tries; k++) {
    const port = first === 0 ? 0 : first + k;
    const ok = await new Promise((done) => {
      const onError = (/** @type {NodeJS.ErrnoException} */ e) => {
        server.off("listening", onListening);
        if (e.code === "EADDRINUSE" && k + 1 < tries) done(false);
        else done(e);
      };
      const onListening = () => {
        server.off("error", onError);
        done(true);
      };
      server.once("error", onError);
      server.once("listening", onListening);
      server.listen(port, "127.0.0.1");
    });
    if (ok === true) {
      const addr = server.address();
      const p = typeof addr === "object" && addr ? addr.port : port;
      return { server, port: p, url: `http://127.0.0.1:${p}/` };
    }
    if (ok !== false) throw ok;
  }
  throw new Error(`ports ${first} to ${first + tries - 1} are all in use`);
}
