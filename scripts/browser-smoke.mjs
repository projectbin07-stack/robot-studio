#!/usr/bin/env node
// Load the page in a headless Chromium-based browser (Chrome in CI, Edge
// locally) over the DevTools protocol, wait until the model is ready,
// check what the page shows, collect console errors and optionally save a
// screenshot. WebGL is rendered by SwiftShader, so no GPU is needed.
//
//   node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11 \
//     [--browser path] [--screenshot out.png] [--width 1440] [--height 900] [--mobile]
//
// Exit code 0 when the page reached "ready", showed the expected number of
// links, drew something with WebGL and logged no console errors.

import { spawn } from "node:child_process";
import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { parseArgs } from "node:util";

const { values } = parseArgs({
  options: {
    url: { type: "string", default: "http://127.0.0.1:8000/" },
    browser: { type: "string" },
    screenshot: { type: "string" },
    width: { type: "string", default: "1440" },
    height: { type: "string", default: "900" },
    mobile: { type: "boolean", default: false },
    "expect-links": { type: "string" },
    "expect-summary": { type: "string" },
    "user-data-dir": { type: "string" },
    upload: { type: "string" },
    timeout: { type: "string", default: "60000" },
  },
});

const candidates = [
  values.browser,
  process.env.BROWSER,
  "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
  "C:/Program Files/Microsoft/Edge/Application/msedge.exe",
  "/usr/bin/google-chrome",
  "/usr/bin/chromium-browser",
  "/usr/bin/chromium",
].filter((x) => typeof x === "string" && x !== "");
const browser = candidates.find((c) => existsSync(/** @type {string} */ (c)));
if (!browser) {
  console.error("no Chrome, Chromium or Edge found; pass --browser");
  process.exit(3);
}

const ownProfile = !values["user-data-dir"];
const profile = values["user-data-dir"] ?? mkdtempSync(join(process.env.TMP ?? tmpdir(), "smoke-"));
const args = [
  "--headless=new",
  "--use-angle=swiftshader",
  "--enable-unsafe-swiftshader",
  "--ignore-gpu-blocklist",
  "--no-first-run",
  "--no-default-browser-check",
  "--disable-extensions",
  "--hide-scrollbars",
  "--remote-debugging-port=0",
  `--user-data-dir=${profile}`,
  ...(process.platform === "linux" ? ["--no-sandbox"] : []),
  "about:blank",
];
const proc = spawn(/** @type {string} */ (browser), args, { stdio: "ignore" });

/** @param {number} ms */
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function devtoolsPort() {
  const file = join(profile, "DevToolsActivePort");
  for (let k = 0; k < 200; k++) {
    if (existsSync(file)) {
      const port = readFileSync(file, "utf8").split("\n")[0];
      if (port) return port;
    }
    await sleep(100);
  }
  throw new Error("the browser did not open a DevTools port");
}

let exitCode = 1;
try {
  const port = await devtoolsPort();
  const version = await (await fetch(`http://127.0.0.1:${port}/json/version`)).json();
  const ws = new WebSocket(version.webSocketDebuggerUrl);
  await new Promise((res, rej) => {
    ws.onopen = res;
    ws.onerror = rej;
  });
  let id = 0;
  /** @type {Map<number, { resolve: (v: any) => void, reject: (e: Error) => void }>} */
  const pending = new Map();
  /** @type {string[]} */
  const errors = [];
  ws.onmessage = (ev) => {
    const msg = JSON.parse(String(ev.data));
    if (msg.id !== undefined) {
      const p = pending.get(msg.id);
      pending.delete(msg.id);
      if (msg.error) p?.reject(new Error(msg.error.message));
      else p?.resolve(msg.result);
      return;
    }
    if (msg.method === "Runtime.consoleAPICalled" && msg.params.type === "error")
      errors.push(
        `console.error: ${msg.params.args.map((/** @type {any} */ a) => a.value ?? a.description).join(" ")}`,
      );
    if (msg.method === "Runtime.exceptionThrown")
      errors.push(
        `exception: ${msg.params.exceptionDetails.exception?.description ?? msg.params.exceptionDetails.text}`,
      );
    if (msg.method === "Log.entryAdded" && msg.params.entry.level === "error")
      errors.push(`log: ${msg.params.entry.text} ${msg.params.entry.url ?? ""}`);
  };
  /**
   * @param {string} method
   * @param {object} [params]
   * @param {string} [sessionId]
   * @returns {Promise<any>}
   */
  const send = (method, params = {}, sessionId) =>
    new Promise((resolve, reject) => {
      const n = ++id;
      pending.set(n, { resolve, reject });
      ws.send(JSON.stringify({ id: n, method, params, ...(sessionId ? { sessionId } : {}) }));
    });

  const { targetId } = await send("Target.createTarget", { url: "about:blank" });
  const { sessionId } = await send("Target.attachToTarget", { targetId, flatten: true });
  /** @param {string} m @param {object} [p] */
  const s = (m, p) => send(m, p, sessionId);
  await s("Page.enable");
  await s("Runtime.enable");
  await s("Log.enable");
  const width = Number(values.width);
  const height = Number(values.height);
  await s("Emulation.setDeviceMetricsOverride", {
    width,
    height,
    deviceScaleFactor: values.mobile ? 2 : 1,
    mobile: values.mobile,
  });
  await s("Page.navigate", { url: values.url });

  /** @param {string} expr */
  const evaluate = async (expr) => {
    const r = await s("Runtime.evaluate", {
      expression: expr,
      returnByValue: true,
      awaitPromise: true,
    });
    if (r.exceptionDetails)
      throw new Error(
        `page evaluation failed: ${r.exceptionDetails.exception?.description ?? r.exceptionDetails.text}`,
      );
    return r.result.value;
  };
  const deadline = Date.now() + Number(values.timeout);
  let st = "";
  if (values.upload) {
    // Wait for the default example, then open the given file through the file input.
    while (Date.now() < deadline && (await evaluate("document.body.dataset.state")) !== "ready")
      await sleep(250);
    await evaluate("document.body.dataset.state = 'uploading'");
    const { root } = await s("DOM.getDocument");
    const { nodeId } = await s("DOM.querySelector", { nodeId: root.nodeId, selector: "#file" });
    await s("DOM.setFileInputFiles", { nodeId, files: [resolve(values.upload)] });
    while (Date.now() < deadline && (await evaluate("document.body.dataset.state")) === "uploading")
      await sleep(100);
  }
  while (Date.now() < deadline) {
    st = await evaluate("document.body ? document.body.dataset.state : ''");
    if (st === "ready" || st === "error") break;
    await sleep(250);
  }
  await sleep(800); // let the last frame (textures, shadows) render
  const report = await evaluate(`(() => {
    const c = document.querySelector('#view canvas');
    let distinct = 0;
    if (c) {
      const t = document.createElement('canvas');
      t.width = 64; t.height = 64;
      const g = t.getContext('2d');
      g.drawImage(c, 0, 0, 64, 64);
      const d = g.getImageData(0, 0, 64, 64).data;
      const seen = new Set();
      for (let i = 0; i < d.length; i += 4) seen.add((d[i] >> 3) + ',' + (d[i+1] >> 3) + ',' + (d[i+2] >> 3));
      distinct = seen.size;
    }
    return {
      state: document.body.dataset.state,
      links: document.body.dataset.links,
      status: document.getElementById('status').textContent,
      summary: document.getElementById('summary').textContent,
      findings: document.querySelectorAll('#findings .finding').length,
      treeLinks: document.querySelectorAll('#tree .t-link').length,
      sliders: document.querySelectorAll('#joints input[type=range]').length,
      webglColours: distinct,
    };
  })()`);
  report.consoleErrors = errors;
  if (values.screenshot) {
    const shot = await s("Page.captureScreenshot", { format: "png", captureBeyondViewport: false });
    writeFileSync(values.screenshot, Buffer.from(shot.data, "base64"));
    report.screenshot = values.screenshot;
  }
  console.log(JSON.stringify(report, null, 2));
  const problems = [];
  if (report.state !== "ready") problems.push(`page state is '${report.state}', expected 'ready'`);
  if (values["expect-links"] && report.links !== values["expect-links"])
    problems.push(`page shows ${report.links} links, expected ${values["expect-links"]}`);
  if (values["expect-links"] && String(report.treeLinks) !== values["expect-links"])
    problems.push(`tree shows ${report.treeLinks} links, expected ${values["expect-links"]}`);
  if (values["expect-summary"] && !report.summary.includes(values["expect-summary"]))
    problems.push(`summary '${report.summary}' does not contain '${values["expect-summary"]}'`);
  if (report.webglColours < 12)
    problems.push(`the 3D view looks blank (${report.webglColours} colours)`);
  if (errors.length > 0) problems.push(`${errors.length} console error(s)`);
  for (const p of problems) console.error(`FAIL: ${p}`);
  exitCode = problems.length === 0 ? 0 : 1;
  ws.close();
} catch (e) {
  console.error(e);
} finally {
  proc.kill();
  await sleep(500);
  try {
    if (ownProfile) rmSync(profile, { recursive: true, force: true, maxRetries: 5 });
  } catch {
    // the browser may still hold files on Windows; the folder is in TMP
  }
  process.exit(exitCode);
}
