#!/usr/bin/env node
// Load the page in a headless Chromium-based browser (Chrome in CI, Edge
// locally) over the DevTools protocol, wait until the model is ready,
// check what the page shows, collect console errors and optionally save a
// screenshot. WebGL is rendered by SwiftShader, so no GPU is needed.
//
//   node scripts/browser-smoke.mjs --url http://127.0.0.1:8000/ --expect-links 11 \
//     [--browser path] [--screenshot out.png] [--width 1440] [--height 900] [--mobile]
//     [--builder]           create a mecanum robot from the template through the UI,
//                           edit a parameter, undo it with Ctrl+Z, export URDF and the
//                           ROS 2 package, reload and check the autosave is restored
//     [--edit-model]        click "Edit the open model" (import as an editable robot)
//     [--template id]       create a robot from that template through the UI
//     [--select-link name]  click a link in the tree before the screenshot
//     [--gpu]               render with the machine's GPU instead of SwiftShader
//     [--headed]            open a visible window (with --gpu: a normal browser frame rate)
//     [--sim]               open the simulation through the UI, drive the selected route
//                           (--sim-route, default demo; --sim-world, default office) for
//                           --sim-seconds of simulated time (default 12) at --sim-speed
//                           (default 2), then check the robot moved, the error measures
//                           filled in and the plots drew; reports fps and step time
//     [--sim-interact]      then strafe with the Q key, draw a box in the world editor,
//                           change a setting and download the recording
//
// Exit code 0 when the page reached "ready", showed the expected number of
// links, drew something with WebGL and logged no console errors.

import { spawn, spawnSync } from "node:child_process";
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
    builder: { type: "boolean", default: false },
    "select-link": { type: "string" },
    template: { type: "string" },
    "edit-model": { type: "boolean", default: false },
    sim: { type: "boolean", default: false },
    gpu: { type: "boolean", default: false },
    headed: { type: "boolean", default: false },
    "sim-route": { type: "string", default: "demo" },
    "sim-world": { type: "string", default: "office" },
    "sim-seconds": { type: "string", default: "12" },
    "sim-speed": { type: "string", default: "2" },
    "sim-layers": { type: "string", default: "" },
    "sim-interact": { type: "boolean", default: false },
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
  ...(values.headed ? [] : ["--headless=new"]),
  ...(values.gpu
    ? ["--enable-gpu", "--use-angle=default"]
    : ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"]),
  "--ignore-gpu-blocklist",
  "--no-first-run",
  "--no-default-browser-check",
  "--disable-extensions",
  "--hide-scrollbars",
  // Without these, a headless window can be treated as hidden and get only a
  // few animation frames per second.
  "--disable-renderer-backgrounding",
  "--disable-backgrounding-occluded-windows",
  "--disable-background-timer-throttling",
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
  // The new tab must be in front: a hidden tab gets no animation frames.
  await s("Page.bringToFront");
  await s("Emulation.setFocusEmulationEnabled", { enabled: true });
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
  /** @type {Record<string, unknown>} */
  const builder = {};
  /** @type {string[]} */
  const builderProblems = [];
  if (values.builder && st === "ready") {
    await runBuilder(s, evaluate, builder, builderProblems, profile, deadline);
  }
  if (values["edit-model"] && st === "ready") {
    await clickSelector(s, evaluate, "#btn-edit-model");
    while (Date.now() < deadline) {
      if (
        (await evaluate("document.body.classList.contains('editing')")) &&
        (await evaluate("document.body.dataset.state")) === "ready"
      )
        break;
      await sleep(100);
    }
    await sleep(500);
  }
  if (values.template && st === "ready") {
    // A new editing session starts at version 0, so also accept the switch to editing.
    const was = await evaluate(
      "[Number(document.body.dataset.version || 0), document.body.classList.contains('editing')]",
    );
    await evaluate(
      `(() => { const t = document.getElementById('template'); t.value = ${JSON.stringify(values.template)}; t.dispatchEvent(new Event('change')); })()`,
    );
    await clickSelector(s, evaluate, "#btn-create");
    while (Date.now() < deadline) {
      const [v, editing] = await evaluate(
        "[Number(document.body.dataset.version || 0), document.body.classList.contains('editing')]",
      );
      const changed = v !== was[0] || (editing && !was[1]);
      if (changed && (await evaluate("document.body.dataset.state")) === "ready") break;
      await sleep(100);
    }
  }
  /** @type {Record<string, unknown>} */
  const sim = {};
  /** @type {string[]} */
  const simProblems = [];
  if (values.sim && st === "ready") await runSim(s, evaluate, sim, simProblems, deadline);
  if (values["select-link"]) {
    await clickSelector(s, evaluate, `#tree .t-link[data-link="${values["select-link"]}"]`);
    await sleep(300);
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
  if (values.builder) report.builder = builder;
  if (values.sim) report.sim = sim;
  if (values.screenshot) {
    // On a phone the 3D view is at the top of the page; scenarios may have scrolled away from it.
    if (values.mobile) await evaluate("window.scrollTo(0, 0)");
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
  problems.push(...builderProblems, ...simProblems);
  for (const p of problems) console.error(`FAIL: ${p}`);
  exitCode = problems.length === 0 ? 0 : 1;
  ws.close();
} catch (e) {
  console.error(e);
} finally {
  proc.kill();
  if (process.platform === "win32") {
    // Edge on Windows hands over to a new browser process, so killing the one
    // we started leaves the browser, its renderers and its GPU process running
    // (a software-rendering GPU process keeps a core busy). End every process
    // started with this run's profile directory, and nothing else.
    const dir = profile.replace(/'/g, "''");
    spawnSync(
      "powershell",
      [
        "-NoProfile",
        "-Command",
        `Get-CimInstance Win32_Process -Filter "Name='msedge.exe' OR Name='chrome.exe'" | Where-Object { $_.CommandLine -and $_.CommandLine.Contains('${dir}') } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }`,
      ],
      { stdio: "ignore" },
    );
  }
  await sleep(500);
  try {
    if (ownProfile) rmSync(profile, { recursive: true, force: true, maxRetries: 5 });
  } catch {
    // the browser may still hold files on Windows; the folder is in TMP
  }
  process.exit(exitCode);
}

/**
 * Click the centre of the element matching a selector with real mouse events.
 * @param {(m: string, p?: object) => Promise<any>} s
 * @param {(expr: string) => Promise<any>} evaluate
 * @param {string} selector
 */
async function clickSelector(s, evaluate, selector) {
  const r = await evaluate(`(() => {
    const el = document.querySelector(${JSON.stringify(selector)});
    if (!el) return null;
    el.scrollIntoView({ block: "center" });
    const b = el.getBoundingClientRect();
    return { x: b.left + b.width / 2, y: b.top + b.height / 2 };
  })()`);
  if (!r) throw new Error(`nothing matches ${selector}`);
  for (const type of ["mousePressed", "mouseReleased"])
    await s("Input.dispatchMouseEvent", { type, x: r.x, y: r.y, button: "left", clickCount: 1 });
}

/**
 * Type into a text field like a user: click it, select its text, type, press Enter.
 * @param {(m: string, p?: object) => Promise<any>} s
 * @param {(expr: string) => Promise<any>} evaluate
 * @param {string} selector
 * @param {string} text
 */
async function typeInto(s, evaluate, selector, text) {
  await clickSelector(s, evaluate, selector);
  await evaluate(`document.querySelector(${JSON.stringify(selector)}).select()`);
  await s("Input.insertText", { text });
  await pressKey(s, "Enter", "Enter", 13, 0);
}

/**
 * @param {(m: string, p?: object) => Promise<any>} s
 * @param {string} key
 * @param {string} code
 * @param {number} keyCode
 * @param {number} modifiers 2 = Ctrl
 */
async function pressKey(s, key, code, keyCode, modifiers) {
  const base = {
    key,
    code,
    windowsVirtualKeyCode: keyCode,
    nativeVirtualKeyCode: keyCode,
    modifiers,
  };
  await s("Input.dispatchKeyEvent", { type: "rawKeyDown", ...base });
  if (key === "Enter") await s("Input.dispatchKeyEvent", { type: "char", text: "\r", ...base });
  await s("Input.dispatchKeyEvent", { type: "keyUp", ...base });
}

/**
 * The simulation scenario: open it through the UI, pick world and route,
 * drive, and read what the page reports.
 * @param {(m: string, p?: object) => Promise<any>} s
 * @param {(expr: string) => Promise<any>} evaluate
 * @param {Record<string, unknown>} out
 * @param {string[]} problems
 * @param {number} deadline
 */
async function runSim(s, evaluate, out, problems, deadline) {
  await clickSelector(s, evaluate, "#p-sim > summary");
  await clickSelector(s, evaluate, "#sim-open");
  const choose = (/** @type {string} */ id, /** @type {string} */ value) =>
    evaluate(
      `(() => { const e = document.getElementById(${JSON.stringify(id)}); e.value = ${JSON.stringify(value)}; e.dispatchEvent(new Event('change')); return e.value; })()`,
    );
  if (
    (await choose("sim-world", /** @type {string} */ (values["sim-world"]))) !== values["sim-world"]
  )
    problems.push(`no world '${values["sim-world"]}'`);
  await choose("sim-route", /** @type {string} */ (values["sim-route"]));
  await choose("sim-speed", /** @type {string} */ (values["sim-speed"]));
  for (const l of /** @type {string} */ (values["sim-layers"]).split(",").filter(Boolean)) {
    const [name, on] = l.split("=");
    await evaluate(
      `(() => { const e = document.querySelector('[data-layer="${name}"]'); e.checked = ${on !== "0"}; e.dispatchEvent(new Event('change')); })()`,
    );
  }
  const target = Number(values["sim-seconds"]);
  const t0 = Date.now();
  let t = 0;
  while (Date.now() < deadline) {
    t = Number((await evaluate("document.body.dataset.simT")) ?? 0);
    if (t >= target) break;
    await sleep(250);
  }
  const wall = (Date.now() - t0) / 1000;
  out.renderer = await evaluate(`(() => {
    const g = document.querySelector('#view canvas').getContext('webgl2');
    const x = g && g.getExtension('WEBGL_debug_renderer_info');
    return x ? g.getParameter(x.UNMASKED_RENDERER_WEBGL) : 'unknown';
  })()`);
  const d = await evaluate("({ ...document.body.dataset })");
  out.state = d.sim;
  out.simulatedSeconds = Number(d.simT);
  out.wallSeconds = Number(wall.toFixed(1));
  out.distance = Number(d.simDistance);
  out.fps = Number(d.simFps);
  out.stepMs = Number(d.simStepMs);
  out.frameMs = Number(d.simFrameMs);
  out.errors = d.simErrors ? JSON.parse(d.simErrors) : null;
  out.hud = await evaluate("document.getElementById('sim-hud').innerText");
  out.note = await evaluate("document.getElementById('sim-note').textContent");
  out.plotPixels = await evaluate(`(() => {
    const c = document.getElementById('plot-errors');
    const g = c.getContext('2d');
    const d = g.getImageData(0, 0, c.width, c.height).data;
    let n = 0;
    for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++;
    return n;
  })()`);
  out.url = await evaluate("location.search");
  if (d.sim !== "running") problems.push(`simulation state is '${d.sim}'`);
  if (!(Number(out.simulatedSeconds) >= target))
    problems.push(`only ${out.simulatedSeconds} s simulated`);
  if (!(Number(out.distance) > 1)) problems.push(`the robot moved only ${out.distance} m`);
  const e = /** @type {any} */ (out.errors);
  if (
    !e ||
    !["odom", "est", "mcl", "odomRms", "estRms", "mclRms"].every((k) => Number.isFinite(e[k]))
  )
    problems.push("the error measures did not fill in");
  if (!(Number(out.plotPixels) > 200)) problems.push("the error plot is empty");
  if (!String(out.url).includes("sim=1")) problems.push("the URL does not record the simulation");
  if (values["sim-interact"]) await interact(s, evaluate, out, problems);
}

/**
 * Teleop, world editing, a setting and the recording, through the UI.
 * @param {(m: string, p?: object) => Promise<any>} s
 * @param {(expr: string) => Promise<any>} evaluate
 * @param {Record<string, unknown>} out
 * @param {string[]} problems
 */
async function interact(s, evaluate, out, problems) {
  // 1. Hold Q (strafe left on a mecanum base) for 1.5 s with the focus on the page.
  await evaluate("document.activeElement && document.activeElement.blur()");
  const pose = () =>
    evaluate("(() => { const t = document.getElementById('hud-time').textContent; return t; })()");
  const before = Number(await evaluate("document.body.dataset.simDistance"));
  const key = { key: "q", code: "KeyQ", windowsVirtualKeyCode: 81, nativeVirtualKeyCode: 81 };
  await s("Input.dispatchKeyEvent", { type: "rawKeyDown", ...key });
  await sleep(700);
  out.teleopHud = await pose();
  await sleep(800);
  await s("Input.dispatchKeyEvent", { type: "keyUp", ...key });
  const after = Number(await evaluate("document.body.dataset.simDistance"));
  out.teleopDistance = Number((after - before).toFixed(3));
  if (!String(out.teleopHud).includes("manual")) problems.push("holding Q did not take over");
  if (!(after > before)) problems.push("the robot did not move while Q was held");

  // 2. Draw a box in the world editor.
  await clickSelector(s, evaluate, "#sim-world-editor > summary");
  await clickSelector(s, evaluate, '[data-tool="box"]');
  const r = await evaluate(`(() => {
    const c = document.getElementById('world-canvas');
    c.scrollIntoView({ block: 'center' });
    const b = c.getBoundingClientRect();
    return { x: b.left, y: b.top, w: b.width, h: b.height };
  })()`);
  const at = (/** @type {number} */ fx, /** @type {number} */ fy) => ({
    x: r.x + r.w * fx,
    y: r.y + r.h * fy,
  });
  const a = at(0.3, 0.3);
  const b = at(0.42, 0.45);
  await s("Input.dispatchMouseEvent", {
    type: "mousePressed",
    ...a,
    button: "left",
    clickCount: 1,
  });
  await s("Input.dispatchMouseEvent", { type: "mouseMoved", ...b, button: "left" });
  await s("Input.dispatchMouseEvent", {
    type: "mouseReleased",
    ...b,
    button: "left",
    clickCount: 1,
  });
  await sleep(500);
  out.worldAfterEdit = await evaluate("document.getElementById('sim-world').value");
  out.urlAfterEdit = await evaluate("location.search");
  if (out.worldAfterEdit !== "custom") problems.push("drawing a box did not edit the world");
  if (!/[?&]w=[^&]*~/.test(decodeURIComponent(String(out.urlAfterEdit))))
    problems.push("the edited world is not in the URL");

  // 3. Change a setting: 90 LiDAR beams.
  await clickSelector(s, evaluate, "#sim-settings > summary");
  await evaluate(`(() => {
    const e = document.getElementById('sf-lidar.beams');
    e.value = '90';
    e.dispatchEvent(new Event('change'));
  })()`);
  await sleep(500);
  out.noteAfterSetting = await evaluate("document.getElementById('sim-note').textContent");
  if (!String(out.noteAfterSetting).includes("90 beams"))
    problems.push("the LiDAR setting did not apply");

  // 4. Download the recording.
  await clickSelector(s, evaluate, "#sim-download");
  await sleep(500);
  out.download = await evaluate("document.body.dataset.lastExport");
  if (!String(out.download).endsWith("-simulation.zip")) problems.push("no recording download");
}

/**
 * The builder scenario: template -> edit -> undo -> export -> reload.
 * @param {(m: string, p?: object) => Promise<any>} s
 * @param {(expr: string) => Promise<any>} evaluate
 * @param {Record<string, unknown>} out
 * @param {string[]} problems
 * @param {string} profile
 * @param {number} deadline
 */
async function runBuilder(s, evaluate, out, problems, profile, deadline) {
  const version = () => evaluate("Number(document.body.dataset.version || 0)");
  /** @param {number} after */
  const waitVersion = async (after) => {
    while (Date.now() < deadline) {
      if ((await version()) > after && (await evaluate("document.body.dataset.state")) === "ready")
        return;
      await sleep(100);
    }
    throw new Error("the builder did not update");
  };
  const urdf = () => evaluate("document.getElementById('urdf').textContent");
  const downloads = join(profile, "downloads");
  await s("Browser.setDownloadBehavior", { behavior: "allow", downloadPath: downloads }).catch(
    () => {},
  );

  // 1. New robot from the mecanum template.
  await evaluate(
    "(() => { const t = document.getElementById('template'); t.value = 'mecanum'; t.dispatchEvent(new Event('change')); })()",
  );
  await clickSelector(s, evaluate, "#btn-create");
  await waitVersion(-1);
  const a = await urdf();
  out.created = {
    links: await evaluate("document.body.dataset.links"),
    summary: await evaluate("document.getElementById('summary').textContent"),
  };
  if (!/0 error\(s\), 0 warning\(s\)/.test(String(/** @type {any} */ (out.created).summary)))
    problems.push(`template robot is not clean: ${/** @type {any} */ (out.created).summary}`);
  if (!a.includes('<cylinder length="0.035" radius="0.04"/>'))
    problems.push("template URDF lacks the 0.04 m wheels");

  // 2. Edit one parameter in the template form.
  let v = await version();
  await typeInto(s, evaluate, '[data-param="wheel_radius"]', "0.05");
  await waitVersion(v);
  const b = await urdf();
  out.editMs = Number(await evaluate("document.body.dataset.editMs"));
  out.undoTitle = await evaluate("document.getElementById('btn-undo').title");
  if (b === a || !b.includes('<cylinder length="0.035" radius="0.05"/>'))
    problems.push("editing wheel_radius did not change the URDF");

  // 3. Undo with the keyboard shortcut (focus outside the form fields).
  await evaluate("document.activeElement && document.activeElement.blur()");
  v = await version();
  await pressKey(s, "z", "KeyZ", 90, 2);
  await waitVersion(v);
  const c = await urdf();
  out.undoRestored = c === a;
  if (c !== a) problems.push("Ctrl+Z did not restore the URDF");
  if ((await evaluate("document.querySelector('[data-param=\"wheel_radius\"]').value")) !== "0.04")
    problems.push("the form does not show the undone value");

  // 4. Select a wheel in the tree and edit its joint origin in the properties panel.
  await clickSelector(s, evaluate, '#tree .t-link[data-link="front_left_wheel"]');
  await sleep(200);
  v = await version();
  await typeInto(s, evaluate, '[data-key="joint.xyz.0"]', "0.11");
  await waitVersion(v);
  out.propertyEdit = (await urdf()).includes('xyz="0.11 0.125 0"');
  if (!out.propertyEdit)
    problems.push("editing the joint origin in the properties panel had no effect");
  out.findingsAfterEdit = await evaluate("document.getElementById('summary').textContent");
  v = await version();
  await pressKey(s, "z", "KeyZ", 90, 2);
  await waitVersion(v);

  // 5. Export the URDF and the ROS 2 package.
  for (const [btn, name] of [
    ["#btn-export-urdf", "mecanum_bot.urdf"],
    ["#btn-export-package", "mecanum_bot_description.zip"],
  ]) {
    await clickSelector(s, evaluate, btn);
    const t0 = Date.now();
    while (Date.now() - t0 < 5000 && (await evaluate("document.body.dataset.lastExport")) !== name)
      await sleep(100);
    if ((await evaluate("document.body.dataset.lastExport")) !== name)
      problems.push(`export ${name} did not happen`);
    const file = join(downloads, name);
    const t1 = Date.now();
    while (Date.now() - t1 < 5000 && !existsSync(file)) await sleep(100);
    if (existsSync(file)) {
      const bytes = readFileSync(file);
      /** @type {any} */ (out)[name] = bytes.length;
      if (name.endsWith(".urdf") && bytes.toString("utf8") !== a)
        problems.push("the downloaded URDF differs from the shown one");
      if (name.endsWith(".zip") && bytes.readUInt32LE(0) !== 0x04034b50)
        problems.push("the package is not a zip");
    } else /** @type {any} */ (out)[name] = "not saved by this browser (export ran)";
  }
  out.exportNote = await evaluate("document.getElementById('export-note').textContent");

  // 6. Reload: the robot comes back from this browser's storage.
  const url = await evaluate("location.origin + location.pathname + '?s=builder'");
  await s("Page.navigate", { url });
  await sleep(500);
  while (Date.now() < deadline && (await evaluate("document.body.dataset.state")) !== "ready")
    await sleep(150);
  const d = await urdf();
  out.restoredAfterReload = d === a;
  if (d !== a) problems.push("the autosaved robot was not restored after a reload");
}
