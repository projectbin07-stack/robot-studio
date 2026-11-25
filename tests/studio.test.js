// The one-command local launch: a 127.0.0.1-only static server for the page.
import test from "node:test";
import assert from "node:assert/strict";
import { request } from "node:http";
import { join } from "node:path";
import { studioCommand, studioUrl } from "../src/studiocli.js";
import { PAGE_ROOT, resolveRequest, startServer } from "../src/node/serve.js";
import { main } from "../src/cli.js";

/** Raw GET (no client-side path normalisation). @param {string} url @param {string} path */
const get = (url, path) =>
  new Promise((done, fail) => {
    const u = new URL(url);
    const req = request({ host: u.hostname, port: u.port, path, method: "GET" }, (res) => {
      /** @type {Buffer[]} */
      const parts = [];
      res.on("data", (c) => parts.push(c));
      res.on("end", () =>
        done({
          status: res.statusCode,
          type: String(res.headers["content-type"]),
          body: Buffer.concat(parts).toString("utf8"),
        }),
      );
    });
    req.on("error", fail);
    req.end();
  });

test("studio URL: the page, with the rosbridge address checked and encoded", () => {
  assert.deepEqual(studioUrl("http://127.0.0.1:8000/", undefined), {
    ok: true,
    url: "http://127.0.0.1:8000/",
  });
  const r = studioUrl("http://127.0.0.1:8000/", "192.168.1.20");
  assert.ok(r.ok && r.url === "http://127.0.0.1:8000/?ros=ws%3A%2F%2F192.168.1.20%3A9090%2F");
  const bad = studioUrl("http://127.0.0.1:8000/", "http://robot");
  assert.ok(!bad.ok && /--ros: only ws/.test(bad.reason));
});

test("requests stay inside the page root", () => {
  assert.equal(resolveRequest(PAGE_ROOT, "/"), join(PAGE_ROOT, "index.html"));
  assert.equal(resolveRequest(PAGE_ROOT, "/src/cli.js"), join(PAGE_ROOT, "src", "cli.js"));
  // ".." cannot climb above the root: it is resolved inside it.
  assert.equal(resolveRequest(PAGE_ROOT, "/%2e%2e/%2e%2e/secret"), join(PAGE_ROOT, "secret"));
  assert.equal(resolveRequest(PAGE_ROOT, "/src/%2e%2e/%2e%2e/%2e%2e/x"), join(PAGE_ROOT, "x"));
  assert.equal(resolveRequest(PAGE_ROOT, "/a%00b"), null);
  assert.equal(resolveRequest(PAGE_ROOT, "/%E0%A4%A"), null);
});

test("studio serves the page on 127.0.0.1 and opens it with the address filled in", async () => {
  /** @type {string[]} */
  const opened = [];
  let out = "";
  const r = await studioCommand(
    ["--port", "0", "--ros", "ws://10.0.0.5:9090"],
    {
      out: (t) => (out += t),
      err: () => {},
    },
    { open: (u) => opened.push(u) },
  );
  assert.ok(typeof r === "object");
  if (typeof r !== "object") return;
  try {
    assert.match(r.url, /^http:\/\/127\.0\.0\.1:\d+\/\?ros=ws%3A%2F%2F10\.0\.0\.5%3A9090%2F$/);
    assert.deepEqual(opened, [r.url]);
    assert.match(out, /Robot Studio is served at http:\/\/127\.0\.0\.1/);
    const page = /** @type {any} */ (await get(r.url, "/"));
    assert.equal(page.status, 200);
    assert.match(page.type, /text\/html/);
    assert.match(page.body, /<title>Robot Studio<\/title>/);
    const js = /** @type {any} */ (await get(r.url, "/src/core/ros/url.js"));
    assert.equal(js.status, 200);
    assert.match(js.type, /text\/javascript/);
    assert.equal(
      /** @type {any} */ (await get(r.url, "/%2e%2e/%2e%2e/nothing-here.txt")).status,
      404,
    );
    assert.equal(/** @type {any} */ (await get(r.url, "/no-such-file.js")).status, 404);
  } finally {
    await r.close();
  }
});

test("studio: a taken port moves to the next one; usage errors", async () => {
  const a = await startServer({ port: 0 });
  const b = await startServer({ port: a.port, tries: 3 });
  try {
    assert.notEqual(a.port, b.port);
    await assert.rejects(startServer({ port: a.port, tries: 1 }), /EADDRINUSE/);
  } finally {
    await new Promise((d) => a.server.close(() => d(null)));
    await new Promise((d) => b.server.close(() => d(null)));
  }
  let err = "";
  const io = { out: () => {}, err: (/** @type {string} */ t) => (err += t) };
  assert.equal(await studioCommand(["--port", "70000"], io), 3);
  assert.equal(await studioCommand(["--ros", "ftp://x"], io), 3);
  assert.match(err, /--port must be/);
  assert.match(err, /--ros: only ws/);
  let help = "";
  assert.equal(await studioCommand(["--help"], { out: (t) => (help += t), err: () => {} }), 0);
  assert.match(help, /serve Robot Studio from this computer/);
  // Through the main CLI: a usage error is exit code 3.
  const code = await main(["studio", "--port", "x"], {
    out: () => {},
    err: () => {},
    writeFile: () => {},
    exists: () => false,
  });
  assert.equal(code, 3);
});
