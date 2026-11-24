// The Simulate panel, the HUD, the live plots, teleop (keyboard, gamepad,
// on-screen sticks) and the frame loop. The simulation itself is
// src/core/sim (DOM-free, tested in Node); this module only wires it to the
// page. Everything shown is labelled simulated.

import { SimSession } from "../core/sim/session.js";
import {
  WORLDS,
  decodeWorld,
  encodeWorld,
  parseSimWorld,
  serializeSimWorld,
} from "../core/sim/world.js";
import { ROUTES, routeSpeeds } from "../core/sim/routes.js";
import { normalizeSettings, settingsDiff } from "../core/sim/settings.js";
import { imuModelFromCalibration } from "../core/sim/imu.js";
import { writeZip } from "../core/zipwrite.js";
import { SimLayer, TRAIL_COLORS } from "./simview.js";
import { Plot } from "./plots.js";
import { WorldEditor } from "./worldedit.js";

/** @typedef {import("./view3d.js").RobotView} RobotView */
/** @typedef {import("./state.js").PageState} PageState */
/** @typedef {import("../core/model.js").LoadedModel} LoadedModel */
/** @typedef {import("../core/builder/document.js").RobotDoc} RobotDoc */
/** @typedef {import("../core/builder/document.js").SimDoc} SimDoc */
/** @typedef {import("../core/sim/world.js").SimWorld} SimWorld */
/** @typedef {import("../core/sim/firmware.js").Twist} Twist */

/** @param {string} id */
const $ = (id) => /** @type {HTMLElement} */ (document.getElementById(id));
/** @param {string} id */
const input = (id) => /** @type {HTMLInputElement} */ (document.getElementById(id));

/**
 * @typedef {object} SimHost
 * @property {RobotView} view
 * @property {PageState} state
 * @property {() => void} saveUrl
 * @property {() => { model: LoadedModel, doc: RobotDoc | null, label: string } | null} robot the open robot
 * @property {() => SimDoc | null} docSim simulation stored in the edited document, if any
 * @property {(sim: SimDoc) => void} storeDocSim keep the simulation in the edited document (builder)
 */

/** Settings shown in the panel: [key, label] or [group title]. */
const FIELDS = /** @type {([string] | [string, string])[]} */ ([
  ["LiDAR (first link tagged 2D LiDAR)"],
  ["lidar.useUrdf", "take samples, range, noise, rate from a URDF <gazebo> tag"],
  ["lidar.beams", "beams"],
  ["lidar.minRange", "min range (m)"],
  ["lidar.maxRange", "max range (m)"],
  ["lidar.rangeNoise", "range noise std (m)"],
  ["lidar.dropout", "dropout probability"],
  ["lidar.rateHz", "rate (Hz)"],
  ["IMU (first link tagged IMU)"],
  ["imu.useUrdf", "use URDF <gazebo> IMU noise"],
  ["imu.rateHz", "rate (Hz)"],
  ["Drive and encoders"],
  ["drive.ideal", "ideal wheels (no motor model, no slip)"],
  ["drive.encoderPpr", "encoder lines per motor rev"],
  ["drive.gearRatio", "gear ratio"],
  ["drive.maxWheelSpeed", "max wheel speed, rad/s (0 = URDF limit)"],
  ["drive.slip", "random wheel slip"],
  ["drive.slipStd", "slip std (fraction)"],
  ["drive.radiusErrorLeft", "left wheel radius error (fraction)"],
  ["drive.radiusErrorRight", "right wheel radius error (fraction)"],
  ["Estimators"],
  ["estimators.scanMatching", "scan matching"],
  ["estimators.particles", "MCL particles (0 = off)"],
  ["estimators.mclGlobal", "MCL starts with no idea where it is"],
  ["Depth camera"],
  ["depth.enabled", "point cloud (first link tagged depth camera)"],
  ["Run"],
  ["seed", "random seed"],
]);

/**
 * @param {any} obj
 * @param {string} key dotted path
 */
const getPath = (obj, key) => key.split(".").reduce((o, k) => (o == null ? undefined : o[k]), obj);
/**
 * @param {any} obj
 * @param {string} key
 * @param {unknown} v
 */
function setPath(obj, key, v) {
  const ks = key.split(".");
  let o = obj;
  for (const k of ks.slice(0, -1)) o = o[k] ??= {};
  o[ks[ks.length - 1]] = v;
}

/** @param {SimHost} host */
export function initSim(host) {
  const { view, state } = host;
  const layer = new SimLayer(view);
  /** @type {SimSession | null} */
  let session = null;
  let running = true;
  let speed = 1;
  let follow = true;
  /** @type {SimWorld} */
  let world = /** @type {SimWorld} */ (decodeWorldSafe(state.world));
  let routeId = ROUTES.some((r) => r.id === state.route) ? state.route : "demo";
  /** @type {Record<string, unknown>} */
  let given = { ...state.simSettings };
  let lastFrame = 0;
  let frames = 0;
  let fpsT0 = 0;
  let fps = 0;
  let plotT = 0;
  /** Main-thread time of one frame (simulation steps, scene update, draw calls), ms, averaged. */
  let frameMs = 0;
  /** @type {number | null} */
  let raf = null;
  /** The panel has been opened (state.sim may be set by a link before that). */
  let opened = false;

  // ------------------------------------------------------------ controls
  const worldSel = /** @type {HTMLSelectElement} */ ($("sim-world"));
  const routeSel = /** @type {HTMLSelectElement} */ ($("sim-route"));
  for (const r of ROUTES) routeSel.append(new Option(r.title, r.id));

  /** @param {string} text */
  const note = (text) => ($("sim-note").textContent = text);

  /** @param {string} text */
  function decodeWorldSafe(text) {
    try {
      return decodeWorld(text);
    } catch (e) {
      setTimeout(() =>
        note(
          `The world in the link could not be read (${e instanceof Error ? e.message : String(e)}); showing the office.`,
        ),
      );
      return /** @type {SimWorld} */ (decodeWorld("office"));
    }
  }

  function renderWorldSelect() {
    worldSel.replaceChildren(...WORLDS.map((w) => new Option(w.title, w.id)));
    if (world.id === "custom") worldSel.append(new Option(`Edited: ${world.name}`, "custom"));
    worldSel.value = world.id;
  }

  const editor = new WorldEditor(
    /** @type {HTMLCanvasElement} */ ($("world-canvas")),
    (w, what) => {
      world = w;
      renderWorldSelect();
      persist();
      restart(`world edited (${what})`);
    },
  );
  for (const b of document.querySelectorAll("[data-tool]"))
    b.addEventListener("click", () => {
      editor.tool = /** @type {any} */ (/** @type {HTMLElement} */ (b).dataset.tool);
      for (const x of document.querySelectorAll("[data-tool]"))
        x.setAttribute("aria-pressed", String(x === b));
    });
  for (const [id, key] of /** @type {const} */ ([
    ["box-h", "boxHeight"],
    ["cyl-h", "cylHeight"],
  ]))
    input(id).addEventListener("change", () => {
      const v = Number(input(id).value);
      if (v > 0 && v <= 20) editor[key] = v;
      else input(id).value = String(editor[key]);
    });
  $("sim-world-editor").addEventListener("toggle", () => editor.draw());
  $("world-clear-route").addEventListener("click", () => {
    world = { ...structuredClone(world), route: [], id: "custom" };
    editor.setWorld(world);
    renderWorldSelect();
    persist();
    restart("route cleared");
  });
  $("world-save").addEventListener("click", () => {
    const a = document.createElement("a");
    a.href = URL.createObjectURL(
      new Blob([serializeSimWorld(world)], { type: "application/json" }),
    );
    a.download = `${world.id === "custom" ? "world" : world.id}.json`;
    a.click();
  });
  input("world-load").addEventListener("change", async () => {
    const f = input("world-load").files?.[0];
    if (!f) return;
    try {
      world = parseSimWorld(await f.text());
      if (world.id !== "custom" && encodeWorld(world) !== world.id) world.id = "custom";
      editor.setWorld(world);
      renderWorldSelect();
      persist();
      restart(`world loaded from ${f.name}`);
    } catch (e) {
      note(`${f.name}: ${e instanceof Error ? e.message : String(e)}`);
    }
    input("world-load").value = "";
  });
  worldSel.addEventListener("change", () => {
    if (worldSel.value === "custom") return;
    world = /** @type {SimWorld} */ (decodeWorld(worldSel.value));
    editor.setWorld(world);
    renderWorldSelect();
    persist();
    restart(`world: ${world.name}`);
  });
  routeSel.addEventListener("change", () => {
    routeId = routeSel.value;
    persist();
    restart(`route: ${routeSel.selectedOptions[0]?.textContent ?? routeId}`);
  });
  $("sim-run").addEventListener("click", () => {
    running = !running;
    $("sim-run").textContent = running ? "Pause" : "Run";
    lastFrame = 0;
  });
  $("sim-reset").addEventListener("click", () => restart("reset"));
  $("sim-speed").addEventListener(
    "change",
    () => (speed = Number(/** @type {HTMLSelectElement} */ ($("sim-speed")).value)),
  );
  $("sim-open").addEventListener("click", () => open());
  $("sim-close").addEventListener("click", () => close());
  $("sim-download").addEventListener("click", () => {
    if (!session) return;
    const r = host.robot();
    const files = session.files(r?.label ?? "robot");
    const zip = writeZip(
      Object.entries(files).map(([path, data]) => ({ path: `simulation/${path}`, data })),
    );
    const a = document.createElement("a");
    a.href = URL.createObjectURL(
      new Blob([/** @type {BlobPart} */ (zip)], { type: "application/zip" }),
    );
    a.download = `${session.robot.name}-simulation.zip`;
    a.click();
    note(
      `Recording of ${session.sim.t.toFixed(1)} simulated seconds (${Object.keys(files).length} files, ${zip.length} bytes). Simulation output; layout in docs/simulation.md.`,
    );
    document.body.dataset.lastExport = a.download;
  });
  input("imu-cal").addEventListener("change", async () => {
    const f = input("imu-cal").files?.[0];
    if (!f) return;
    try {
      const cal = JSON.parse(await f.text());
      const m = imuModelFromCalibration(cal);
      setPath(given, "imu.calibration", cal);
      $("imu-cal-note").textContent = `Using ${f.name}: ${m.source}.`;
      persist();
      restart("IMU calibration loaded");
    } catch (e) {
      $("imu-cal-note").textContent = `${f.name}: ${e instanceof Error ? e.message : String(e)}`;
    }
  });
  for (const el of document.querySelectorAll("[data-layer]"))
    el.addEventListener("change", () => {
      const k = /** @type {HTMLElement} */ (el).dataset.layer ?? "";
      const on = /** @type {HTMLInputElement} */ (el).checked;
      if (k === "follow") follow = on;
      else layer.setOptions({ [k]: on });
      if (k === "tf") {
        state.frames = on || state.frames;
        view.applyOptions({ frames: on });
      }
    });

  function renderFields() {
    let settings;
    try {
      settings = normalizeSettings(given);
    } catch {
      settings = normalizeSettings({});
    }
    const host2 = $("sim-fields");
    host2.className = "sim-fields";
    host2.replaceChildren();
    for (const f of FIELDS) {
      if (f.length === 1) {
        const d = document.createElement("div");
        d.className = "group";
        d.textContent = f[0];
        host2.append(d);
        continue;
      }
      const [key, label] = f;
      const v = getPath(settings, key);
      const l = document.createElement("label");
      l.textContent = label;
      l.htmlFor = `sf-${key}`;
      const inp = document.createElement("input");
      inp.id = `sf-${key}`;
      inp.dataset.setting = key;
      if (typeof v === "boolean") {
        inp.type = "checkbox";
        inp.checked = v;
      } else {
        inp.type = "text";
        inp.value = String(v);
        inp.inputMode = "decimal";
      }
      inp.addEventListener("change", () => {
        const next = structuredClone(given);
        setPath(next, key, inp.type === "checkbox" ? inp.checked : Number(inp.value));
        try {
          normalizeSettings(next);
        } catch (e) {
          note(e instanceof Error ? e.message : String(e));
          renderFields();
          return;
        }
        given = settingsDiff(normalizeSettings(next));
        persist();
        restart(`${label} = ${inp.type === "checkbox" ? inp.checked : inp.value}`);
      });
      host2.append(l, inp);
    }
  }

  /** Keep world, route and settings in the URL and (when editing) in the document. */
  function persist() {
    state.world = encodeWorld(world);
    state.route = routeId;
    state.simSettings = given;
    host.saveUrl();
    host.storeDocSim({ world: state.world, route: routeId, settings: given });
  }

  // ------------------------------------------------------------ session
  /** @param {string} why */
  function restart(why) {
    if (!opened) return;
    build(why);
  }

  /** @param {string} why */
  function build(why) {
    const r = host.robot();
    session = null;
    if (!r) {
      note("Open or build a robot first.");
      return;
    }
    let settings;
    try {
      settings = normalizeSettings(given);
    } catch (e) {
      note(`settings: ${e instanceof Error ? e.message : String(e)}; using the defaults`);
      given = {};
      settings = normalizeSettings({});
    }
    try {
      session = new SimSession(r.model, r.doc, world, routeId, settings, {
        clock: () => performance.now(),
      });
    } catch (e) {
      note(`This robot cannot be simulated: ${e instanceof Error ? e.message : String(e)}.`);
      layer.setVisible(false);
      showChrome(false);
      document.body.dataset.sim = "unavailable";
      return;
    }
    layer.setVisible(true);
    layer.clear();
    layer.setWorld(world, session.sim.lidar?.sensor.height ?? 0);
    editor.setWorld(world);
    showChrome(true);
    const s = session.sim;
    const parts = [
      `${session.robot.drive} drive, ${session.robot.wheels.length} wheels, max ${s.controller.config.maxWheelSpeed.toFixed(1)} rad/s`,
      s.lidar
        ? `LiDAR ${s.lidar.sensor.link} (${s.lidar.config.beams} beams, ${s.lidar.rateHz.toFixed(0)} Hz, ${s.lidar.source})`
        : "no LiDAR",
      s.imu ? `IMU ${s.imu.sensor.link} (${s.imu.source})` : "no IMU",
      `encoders ${s.ticksPerRev} counts/rev`,
    ];
    note(
      `${why ? `${why}. ` : ""}${parts.join("; ")}.${[...session.robotNotes, ...s.notes].map((n) => ` Note: ${n}.`).join("")}`,
    );
    const b = s.truth;
    // Look down at the robot from behind, with several metres of the world in view.
    view.frame([b.x, b.y, 0], 4);
    view.camera.position.set(b.x - Math.cos(b.theta) * 4.2, b.y - Math.sin(b.theta) * 4.2, 7.5);
    view.camera.near = 0.02;
    view.camera.far = 200;
    view.camera.updateProjectionMatrix();
    view.controls.update();
    view.setFloor({ x0: 0, y0: 0, x1: world.width, y1: world.height });
    document.body.dataset.sim = "running";
    lastFrame = 0;
    layer.update(session);
  }

  /** @param {boolean} on */
  function showChrome(on) {
    $("sim-controls").hidden = !state.sim;
    $("sim-open").hidden = state.sim;
    $("sim-hud").hidden = !on;
    $("sim-layers").hidden = !on;
    $("sticks").hidden = !on;
    $("p-plots").hidden = !on;
    document.body.classList.toggle("simulating", on);
  }

  function open() {
    state.sim = true;
    opened = true;
    const stored = host.docSim();
    if (stored) {
      world = decodeWorldSafe(stored.world);
      routeId = stored.route;
      given = { ...stored.settings };
    }
    renderWorldSelect();
    routeSel.value = routeId;
    renderFields();
    /** @type {HTMLDetailsElement} */ ($("p-sim")).open = true;
    host.saveUrl();
    build("");
    if (raf === null) raf = requestAnimationFrame(frame);
  }

  function close() {
    state.sim = false;
    opened = false;
    session = null;
    layer.setVisible(false);
    showChrome(false);
    host.saveUrl();
    delete document.body.dataset.sim;
    if (raf !== null) cancelAnimationFrame(raf);
    raf = null;
    view.setPose(state.q);
  }

  // ------------------------------------------------------------ teleop
  /** @type {Set<string>} */
  const keys = new Set();
  const KEYS = new Set([
    "w",
    "a",
    "s",
    "d",
    "q",
    "e",
    " ",
    "arrowup",
    "arrowdown",
    "arrowleft",
    "arrowright",
  ]);
  window.addEventListener("keydown", (e) => {
    if (!session || e.ctrlKey || e.metaKey || e.altKey) return;
    const t = /** @type {HTMLElement | null} */ (e.target);
    if (t && (t.tagName === "INPUT" || t.tagName === "SELECT" || t.tagName === "TEXTAREA")) return;
    const k = e.key.toLowerCase();
    if (!KEYS.has(k)) return;
    keys.add(k);
    e.preventDefault();
  });
  window.addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
  window.addEventListener("blur", () => keys.clear());
  /** @type {Record<string, { x: number, y: number }>} */
  const sticks = { "stick-move": { x: 0, y: 0 }, "stick-turn": { x: 0, y: 0 } };
  for (const id of Object.keys(sticks)) {
    const el = $(id);
    const knob = /** @type {HTMLElement} */ (el.firstElementChild);
    /** @param {PointerEvent} e */
    const move = (e) => {
      const r = el.getBoundingClientRect();
      const rx = r.width / 2;
      const ry = r.height / 2;
      const x = Math.max(-1, Math.min(1, (e.clientX - r.left - rx) / rx));
      const y = Math.max(-1, Math.min(1, (e.clientY - r.top - ry) / ry));
      sticks[id] = { x, y: id === "stick-turn" ? 0 : y };
      knob.style.transform = `translate(${x * rx * 0.6}px, ${sticks[id].y * ry * 0.6}px)`;
    };
    el.addEventListener("pointerdown", (e) => {
      el.setPointerCapture(e.pointerId);
      move(e);
      e.preventDefault();
    });
    el.addEventListener("pointermove", (e) => {
      if (el.hasPointerCapture(e.pointerId)) move(e);
    });
    const release = () => {
      sticks[id] = { x: 0, y: 0 };
      knob.style.transform = "";
    };
    el.addEventListener("pointerup", release);
    el.addEventListener("pointercancel", release);
  }

  /**
   * The manual command from every input, or null when nothing is held.
   * @returns {Twist | null}
   */
  function manual() {
    if (!session) return null;
    const sp = routeSpeeds(session.robot);
    const mec = session.robot.drive === "mecanum";
    let fx = 0;
    let fy = 0;
    let fw = 0;
    const k = (/** @type {string} */ a, /** @type {string} */ b) =>
      keys.has(a) || keys.has(b) ? 1 : 0;
    fx += k("w", "arrowup") - k("s", "arrowdown");
    fw += k("a", "arrowleft") - k("d", "arrowright");
    if (mec) fy += (keys.has("q") ? 1 : 0) - (keys.has("e") ? 1 : 0);
    fx += -sticks["stick-move"].y;
    if (mec) fy += -sticks["stick-move"].x;
    else fw += -sticks["stick-move"].x;
    fw += -sticks["stick-turn"].x;
    const pads = typeof navigator.getGamepads === "function" ? navigator.getGamepads() : [];
    for (const pad of pads) {
      if (!pad) continue;
      const dz = (/** @type {number} */ v) => (Math.abs(v) < 0.15 ? 0 : v);
      fx += -dz(pad.axes[1] ?? 0);
      if (mec) fy += -dz(pad.axes[0] ?? 0);
      else fw += -dz(pad.axes[0] ?? 0);
      fw += -dz(pad.axes[2] ?? 0);
    }
    const c = (/** @type {number} */ v) => Math.max(-1, Math.min(1, v));
    if (keys.has(" ")) return { vx: 0, vy: 0, wz: 0 };
    if (fx === 0 && fy === 0 && fw === 0) return null;
    return { vx: c(fx) * sp.v, vy: c(fy) * sp.v, wz: c(fw) * sp.w };
  }

  // ------------------------------------------------------------ frame loop
  const plots = {
    wheels: new Plot(/** @type {HTMLCanvasElement} */ ($("plot-wheels")), "Wheel speed", "rad/s", [
      ...["FL", "FR", "RL", "RR"].map((l, i) => ({
        label: l,
        index: 4 + i,
        color: ["#2a78d6", "#eb6a1f", "#2e9d5b", "#9b5cff"][i],
      })),
      ...[0, 1, 2, 3].map((i) => ({
        label: i === 0 ? "target" : "",
        index: i,
        color: ["#2a78d6", "#eb6a1f", "#2e9d5b", "#9b5cff"][i],
        dashed: true,
      })),
    ]),
    twist: new Plot(/** @type {HTMLCanvasElement} */ ($("plot-twist")), "Twist", "m/s, rad/s", [
      { label: "vx", index: 3, color: "#2a78d6" },
      { label: "vy", index: 4, color: "#2e9d5b" },
      { label: "wz", index: 5, color: "#eb6a1f" },
      { label: "commanded", index: 0, color: "#2a78d6", dashed: true },
      { label: "", index: 1, color: "#2e9d5b", dashed: true },
      { label: "", index: 2, color: "#eb6a1f", dashed: true },
    ]),
    imu: new Plot(/** @type {HTMLCanvasElement} */ ($("plot-imu")), "IMU", "m/s², rad/s", [
      { label: "ax", index: 0, color: "#2a78d6" },
      { label: "ay", index: 1, color: "#2e9d5b" },
      { label: "gz", index: 5, color: "#eb6a1f" },
    ]),
    errors: new Plot(
      /** @type {HTMLCanvasElement} */ ($("plot-errors")),
      "Error vs truth",
      "m",
      [
        { label: "wheel odom", index: 0, color: "#d8453b" },
        { label: "+ scan match", index: 1, color: "#2a78d6" },
        { label: "MCL", index: 2, color: "#9b5cff" },
      ],
      { min: 0 },
    ),
  };

  /** @param {number} now */
  function frame(now) {
    raf = requestAnimationFrame(frame);
    if (!session) return;
    const dt = lastFrame === 0 ? 0 : Math.min(0.25, (now - lastFrame) / 1000);
    lastFrame = now;
    frames++;
    if (now - fpsT0 >= 1000) {
      fps = (frames * 1000) / (now - fpsT0);
      frames = 0;
      fpsT0 = now;
    }
    const work0 = performance.now();
    session.manual = manual();
    const before = { ...session.sim.truth };
    if (running) session.advance(dt * speed);
    view.holdRender = true;
    layer.update(session);
    const s = session.sim;
    if (follow) {
      const dx = s.truth.x - before.x;
      const dy = s.truth.y - before.y;
      if (dx !== 0 || dy !== 0) {
        view.camera.position.x += dx;
        view.camera.position.y += dy;
        view.controls.target.x += dx;
        view.controls.target.y += dy;
        view.controls.update();
      }
    }
    view.holdRender = false;
    view.render();
    const work = performance.now() - work0;
    frameMs = frameMs === 0 ? work : 0.9 * frameMs + 0.1 * work;
    if (now - plotT > 100) {
      plotT = now;
      hud();
      for (const [k, p] of Object.entries(plots))
        p.draw(session.history[/** @type {keyof SimSession["history"]} */ (k)], s.t, 20);
      editor.setRobot(s.truth, session.robot.footprint.radius);
    }
  }

  function hud() {
    if (!session) return;
    const s = session.sim;
    const e = session.snapshot().errors;
    const routeNote = session.manual
      ? "manual"
      : session.routeDone
        ? "route finished"
        : session.route.progress();
    $("hud-time").textContent =
      `t = ${s.t.toFixed(1)} s · ${s.distance.toFixed(2)} m · ${routeNote}`;
    const rows = [["", "now", "RMS"]];
    /** @type {[string, number, number, number][]} */
    const data = e
      ? [
          ["wheel odometry", TRAIL_COLORS.odom, e.odom, e.odomRms],
          .../** @type {[string, number, number, number][]} */ (
            s.settings.estimators.scanMatching
              ? [["+ scan matching", TRAIL_COLORS.est, e.est, e.estRms]]
              : []
          ),
          .../** @type {[string, number, number, number][]} */ (
            s.est?.pf ? [["MCL", TRAIL_COLORS.mcl, e.mcl, e.mclRms]] : []
          ),
        ]
      : [];
    const table = $("hud-errors");
    table.replaceChildren();
    const head = document.createElement("tr");
    for (const t of rows[0]) {
      const th = document.createElement("th");
      th.textContent = t;
      head.append(th);
    }
    table.append(head);
    for (const [label, color, now, rms] of data) {
      const tr = document.createElement("tr");
      const td0 = document.createElement("td");
      const sw = document.createElement("span");
      sw.className = "sw";
      sw.style.background = `#${color.toString(16).padStart(6, "0")}`;
      td0.append(sw, label);
      tr.append(td0);
      for (const v of [now, rms]) {
        const td = document.createElement("td");
        td.textContent = `${v.toFixed(3)} m`;
        tr.append(td);
      }
      table.append(tr);
    }
    if (!e) {
      const tr = document.createElement("tr");
      const td = document.createElement("td");
      td.textContent = "no LiDAR: no estimators";
      tr.append(td);
      table.append(tr);
    }
    $("hud-perf").textContent =
      `${fps.toFixed(0)} fps · frame ${frameMs.toFixed(1)} ms · ${session.stepMs.toFixed(2)} ms per 10 ms step (measured in this browser)`;
    const d = document.body.dataset;
    d.simT = s.t.toFixed(2);
    d.simDistance = s.distance.toFixed(3);
    d.simFps = fps.toFixed(1);
    d.simStepMs = session.stepMs.toFixed(3);
    d.simFrameMs = frameMs.toFixed(2);
    d.simErrors = e
      ? JSON.stringify({
          odom: e.odom,
          est: e.est,
          mcl: e.mcl,
          odomRms: e.odomRms,
          estRms: e.estRms,
          mclRms: e.mclRms,
        })
      : "";
  }

  return {
    /** The robot changed (opened, edited, undone): rebuild a running simulation. */
    robotChanged() {
      if (!opened) return;
      // An undo or redo may have changed the simulation stored in the document.
      const stored = host.docSim();
      if (
        stored &&
        (stored.world !== encodeWorld(world) ||
          stored.route !== routeId ||
          JSON.stringify(stored.settings) !== JSON.stringify(given))
      ) {
        world = decodeWorldSafe(stored.world);
        routeId = stored.route;
        given = { ...stored.settings };
        renderWorldSelect();
        routeSel.value = routeId;
        renderFields();
        state.world = stored.world;
        state.route = routeId;
        state.simSettings = given;
        host.saveUrl();
      }
      build("robot changed");
    },
    /** Open the simulation if the link asked for it. */
    openFromLink() {
      if (state.sim && !opened) open();
    },
    get session() {
      return session;
    },
  };
}
