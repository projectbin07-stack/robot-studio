// The ROS 2 panel: a live robot over rosbridge (labelled LIVE), driving it
// through the dead-man switch (a separate, explicit opt-in, a visible
// "driving a real robot" banner), and replaying recordings (labelled REPLAY,
// "simulated data" when the recording says so) with the comparison against
// the simulator. The logic is in src/core/ros (DOM-free, tested in Node);
// this module wires it to the page and to the simulation's 3D layer, HUD,
// plots and teleop. Message content only ever reaches the page as text.

import { checkRosbridgeUrl } from "../core/ros/url.js";
import { RosbridgeClient } from "../core/ros/rosbridge.js";
import { LiveSession } from "../core/ros/live.js";
import { CmdVelGate, disarmOnDisconnect } from "../core/ros/deadman.js";
import { twistMsg } from "../core/ros/msgs.js";
import { parseRecording } from "../core/ros/recording.js";
import { ReplaySession } from "../core/ros/replay.js";
import { compareRecordings, runSchedule, scheduleOf } from "../core/ros/compare.js";
import { simRobot } from "../core/sim/robot.js";
import { listZip, readZipEntry } from "../core/zip.js";
import { SimLayer, TRAIL_COLORS } from "./simview.js";
import { Plot } from "./plots.js";

/** @typedef {import("./view3d.js").RobotView} RobotView */
/** @typedef {import("./state.js").PageState} PageState */
/** @typedef {import("../core/model.js").LoadedModel} LoadedModel */
/** @typedef {import("../core/builder/document.js").RobotDoc} RobotDoc */
/** @typedef {import("../core/sim/robot.js").SimRobot} SimRobot */
/** @typedef {import("../core/ros/recording.js").Recording} Recording */

/** @param {string} id */
const $ = (id) => /** @type {HTMLElement} */ (document.getElementById(id));

/**
 * @typedef {object} RosHost
 * @property {RobotView} view
 * @property {PageState} state
 * @property {() => void} saveUrl
 * @property {() => { model: LoadedModel, doc: RobotDoc | null, label: string } | null} robot
 * @property {(urdf: string, label: string) => void} openUrdf open a URDF text as the model
 * @property {{ close: () => void, axes: (mec: boolean) => { fx: number, fy: number, fw: number } | null,
 *   setExternalTeleop: (fn: () => boolean) => void, showSticks: (on: boolean) => void, releaseAll: () => void }} sim
 */

/** @param {RosHost} host */
export function initRos(host) {
  const { view, state } = host;
  const layer = new SimLayer(view);
  /** @type {"live" | "replay" | null} */
  let mode = null;
  /** @type {RosbridgeClient | null} */
  let client = null;
  /** @type {LiveSession | null} */
  let live = null;
  /** @type {CmdVelGate | null} */
  let gate = null;
  /** @type {ReplaySession | null} */
  let replay = null;
  /** @type {Recording | null} */
  let recording = null;
  /** @type {SimRobot | null} */
  let robot = null;
  let running = true;
  /** @type {number | null} */
  let raf = null;
  let last = 0;
  let plotAt = 0;
  let topicsAt = 0;
  let frames = 0;
  let fpsT0 = 0;
  let fps = 0;
  /** @type {ReturnType<typeof setInterval> | null} */
  let tickTimer = null;
  /** @type {{ x: number, y: number } | null} */
  let prevBase = null;

  const urlInput = /** @type {HTMLInputElement} */ ($("ros-url"));
  urlInput.value = state.ros;
  const setStatus = (/** @type {string} */ t) => ($("ros-status").textContent = t);
  const setNote = (/** @type {string} */ t) => ($("ros-replay-note").textContent = t);
  const d = document.body.dataset;

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
      "Error vs simulated truth (none on a robot)",
      "m",
      [
        { label: "wheel odom", index: 0, color: "#d8453b" },
        { label: "+ scan match", index: 1, color: "#2a78d6" },
        { label: "MCL", index: 2, color: "#9b5cff" },
      ],
      { min: 0 },
    ),
  };

  /** The open model as a simulation robot (for the wheels, the LiDAR mount and the speed limit). */
  function readRobot() {
    const r = host.robot();
    if (!r) return null;
    const s = simRobot(r.model, r.doc);
    return s.ok ? s.robot : null;
  }

  /** @param {string} kind @param {string} text @param {"live" | "replay" | ""} cls */
  function badge(kind, text, cls) {
    for (const id of ["hud-badge", "plots-badge"]) {
      const el = $(id);
      el.textContent = id === "hud-badge" ? text : kind;
      el.classList.toggle("live", cls === "live");
      el.classList.toggle("replay", cls === "replay");
    }
  }

  /** @param {boolean} on */
  function chrome(on) {
    $("sim-hud").hidden = !on;
    $("p-plots").hidden = !on;
    $("ros-close").hidden = !on;
    document.body.classList.toggle("simulating", on);
  }

  function start() {
    host.sim.close();
    prevBase = null;
    layer.setVisible(true);
    layer.clear();
    chrome(true);
    last = 0;
    if (raf === null) raf = requestAnimationFrame(frame);
  }

  /** Leave live / replay mode (the simulation's labels come back). */
  function stop() {
    if (gate) gate.disarm("closed");
    mode = null;
    replay = null;
    recording = null;
    layer.setVisible(false);
    chrome(false);
    badge("simulated", "Simulated", "");
    $("drive-banner").hidden = true;
    host.sim.showSticks(false);
    if (raf !== null) cancelAnimationFrame(raf);
    raf = null;
    delete d.ros;
    delete d.replay;
  }

  /** Frame the camera on a point, as the simulation does. @param {number} x @param {number} y @param {number} th */
  function aim(x, y, th) {
    view.frame([x, y, 0], 4);
    view.camera.position.set(x - Math.cos(th) * 4.2, y - Math.sin(th) * 4.2, 7.5);
    view.camera.near = 0.02;
    view.camera.far = 200;
    view.camera.updateProjectionMatrix();
    view.controls.update();
  }

  // ------------------------------------------------------------ live
  $("ros-connect").addEventListener("click", () => {
    const check = checkRosbridgeUrl(urlInput.value, location.protocol);
    if (!check.ok) {
      setStatus(`Not connected: ${check.reason}.`);
      return;
    }
    disconnect();
    urlInput.value = check.url;
    state.ros = check.url;
    host.saveUrl();
    robot = readRobot();
    client = new RosbridgeClient({
      url: check.url,
      WebSocket: /** @type {any} */ (window.WebSocket),
    });
    const c = client;
    const warn = check.warnings.map((w) => ` Warning: ${w}.`).join("");
    c.onState((st, detail) => {
      d.ros = st;
      if (st === "open") {
        setStatus(`LIVE: connected to ${check.host}. Showing the robot's own data.`);
        $("ros-live").hidden = false;
      } else if (st === "reconnecting")
        setStatus(
          `Connection lost (${detail}); reconnecting to ${check.host}. Driving is off.${warn}`,
        );
      else if (st === "connecting") setStatus(`Connecting to ${check.host}${warn}`);
      else if (st === "closed") setStatus(`Not connected (${detail}).${warn}`);
      updateDrive();
    });
    c.onLog((level, text) => {
      if (level === "error" || level === "warn" || level === "warning")
        $("ros-status").textContent += ` ${text}.`;
    });
    live = new LiveSession(c, { robot, now: () => performance.now() });
    live.onDescription(() => {
      /** @type {HTMLButtonElement} */ ($("ros-load-urdf")).disabled = false;
    });
    $("ros-connect").hidden = true;
    $("ros-disconnect").hidden = false;

    mode = "live";
    badge("live", "LIVE: real robot", "live");
    layer.clearWorld(10);
    aim(0, 0, 0);
    start();
    makeGate();
    c.connect();
  });

  function disconnect() {
    if (gate) gate.disarm("disconnected");
    if (live) live.dispose();
    if (client) client.close();
    live = null;
    client = null;
    gate = null;
    $("ros-connect").hidden = false;
    $("ros-disconnect").hidden = true;
    $("ros-live").hidden = true;
    if (mode === "live") stop();
  }
  $("ros-disconnect").addEventListener("click", () => {
    disconnect();
    setStatus("Disconnected.");
  });

  $("ros-load-urdf").addEventListener("click", async () => {
    if (!live) return;
    let text = live.robotDescription;
    if (text === null) {
      try {
        text = await live.requestDescription();
      } catch (e) {
        setStatus(
          `No robot_description: not on /robot_description and rosapi said: ${e instanceof Error ? e.message : String(e)}.`,
        );
        return;
      }
    }
    if (text !== null) host.openUrdf(text, "robot_description from the robot");
  });

  /** A new model was opened (a robot_description, a file): re-read the robot. */
  function robotChanged() {
    robot = readRobot();
    if (live) {
      live.robot = robot;
      makeGate();
    }
  }

  function makeGate() {
    if (gate) gate.disarm("the robot changed");
    gate = null;
    if (!client || !robot) {
      updateDrive();
      return;
    }
    const c = client;
    gate = new CmdVelGate({
      geometry: robot.geometry,
      maxWheelSpeed: robot.maxWheelSpeed,
      publish: (t) => {
        live?.commanded(t);
        return c.isOpen ? c.publish("/cmd_vel", twistMsg(t)) : false;
      },
      now: () => performance.now(),
    });
    disarmOnDisconnect(gate, c);
    gate.onChange(() => updateDrive());
    updateDrive();
  }

  function updateDrive() {
    const g = gate;
    const armed = !!g && g.state !== "disarmed";
    const banner = $("drive-banner");
    banner.hidden = !armed;
    banner.classList.toggle("armed", armed && g?.state !== "driving");
    banner.replaceChildren(
      g?.state === "driving" ? "DRIVING A REAL ROBOT" : "READY TO DRIVE A REAL ROBOT",
      Object.assign(document.createElement("small"), {
        textContent: g
          ? `/cmd_vel, at most ${Math.round(g.fraction * 100)} % of the wheel limit (${g.wheelLimit.toFixed(1)} rad/s); let go to stop`
          : "",
      }),
    );
    $("ros-arm").hidden = armed;
    $("ros-disarm").hidden = !armed;
    /** @type {HTMLButtonElement} */ ($("ros-arm")).disabled = !g || !client?.isOpen;
    host.sim.showSticks(armed);
    $("ros-drive-state").textContent = !robot
      ? "Driving needs the robot's model (its wheels and their speed limit): open it or use its robot_description."
      : !g
        ? ""
        : `${g.state === "disarmed" ? "Not driving" : g.state === "driving" ? "Driving" : "Armed"}: ${g.reason}. Sent ${g.sent} command(s).`;
    d.rosDrive = g ? g.state : "none";
    d.rosSent = String(g?.sent ?? 0);
  }

  $("ros-arm").addEventListener("click", () => {
    if (!gate || !client?.isOpen) return;
    const confirmed = /** @type {HTMLInputElement} */ ($("ros-drive-confirm")).checked;
    if (!confirmed) {
      $("ros-drive-state").textContent =
        "Tick the confirmation first: commands go to a real robot.";
      return;
    }
    try {
      client.advertise("/cmd_vel", "geometry_msgs/msg/Twist");
      gate.arm({
        confirmed,
        fraction: Number(/** @type {HTMLSelectElement} */ ($("ros-speed")).value),
      });
    } catch (e) {
      $("ros-drive-state").textContent = e instanceof Error ? e.message : String(e);
      return;
    }
    // The checkbox is cleared: each arming needs its own confirmation.
    /** @type {HTMLInputElement} */ ($("ros-drive-confirm")).checked = false;
    if (tickTimer === null) tickTimer = setInterval(() => gate?.tick(), 50);
    updateDrive();
  });
  $("ros-disarm").addEventListener("click", () => gate?.disarm("stopped by the user"));
  host.sim.setExternalTeleop(() => !!gate && gate.state !== "disarmed");

  /** The teleop command at the robot's full speed; the gate scales it down. */
  function teleop() {
    if (!robot) return null;
    const a = host.sim.axes(robot.drive === "mecanum");
    if (!a) return null;
    const g = robot.geometry;
    const v = g.wheelRadius * robot.maxWheelSpeed;
    const w = v / (g.halfWidth + g.halfLength || g.halfWidth);
    return { vx: a.fx * v, vy: a.fy * v, wz: a.fw * w };
  }
  // Release and focus: stop at once, not at the next animation frame.
  window.addEventListener("keyup", () => gate?.update(teleop()));
  window.addEventListener("pointerup", () => setTimeout(() => gate?.update(teleop())));
  window.addEventListener("blur", () => {
    host.sim.releaseAll();
    gate?.blur();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState === "hidden") {
      host.sim.releaseAll();
      gate?.hidden();
    }
  });
  window.addEventListener("pagehide", () => gate?.disarm("the page was closed"));

  // ------------------------------------------------------------ replay
  /** @param {FileList} list */
  async function readFiles(list) {
    /** @type {Record<string, string>} */
    const files = {};
    const dec = new TextDecoder();
    for (const f of Array.from(list)) {
      if (f.name.toLowerCase().endsWith(".zip")) {
        const bytes = new Uint8Array(await f.arrayBuffer());
        for (const e of listZip(bytes)) {
          const name = e.name.split("/").pop() ?? "";
          if (/\.(csv|jsonl|json)$/.test(name))
            files[name] = dec.decode(await readZipEntry(bytes, e));
        }
      } else files[f.name] = await f.text();
    }
    return files;
  }

  /** @type {HTMLInputElement} */ ($("ros-replay-file")).addEventListener("change", async (e) => {
    const input = /** @type {HTMLInputElement} */ (e.target);
    if (!input.files?.length) return;
    try {
      const rec = parseRecording(await readFiles(input.files));
      if (mode === "live") disconnect();
      recording = rec;
      openReplay();
    } catch (err) {
      setNote(`Could not read the recording: ${err instanceof Error ? err.message : String(err)}.`);
    }
    input.value = "";
  });

  function openReplay() {
    const rec = /** @type {Recording} */ (recording);
    robot = readRobot();
    replay = new ReplaySession(rec, { wheelJoints: robot?.wheels.map((w) => w.joint) ?? [] });
    mode = "replay";
    running = true;
    $("ros-replay-run").textContent = "Pause";
    badge("replay", rec.simulated ? "Replay: SIMULATED data" : "Replay: recorded data", "replay");
    if (rec.world) layer.setWorld(rec.world, replay.track.opts.lidarHeight);
    else layer.clearWorld(15);
    const s = rec.world?.start ?? { x: 0, y: 0, theta: 0 };
    aim(s.x, s.y, s.theta);
    start();
    $("ros-replay-controls").hidden = false;
    $("ros-compare-row").hidden = false;
    $("ros-compare-out").hidden = true;
    const name = rec.summary?.robot?.name;
    setNote(
      `${rec.simulated ? "Simulated recording (the file says so)" : "Recorded data"}: ${(replay.end - replay.start).toFixed(1)} s, ${rec.scans?.items.length ?? 0} scans. The robot is drawn with the open model${name && robot && name !== robot.name ? ` (${robot.name}; the recording is of ${String(name).slice(0, 80)})` : ""}.${rec.notes.length ? ` Note: ${rec.notes.join("; ")}.` : ""}`,
    );
  }
  $("ros-replay-run").addEventListener("click", () => {
    running = !running;
    $("ros-replay-run").textContent = running ? "Pause" : "Run";
  });
  $("ros-replay-restart").addEventListener("click", () => {
    if (recording) openReplay();
  });
  $("ros-compare").addEventListener("click", () => {
    const rec = recording;
    const r = host.robot();
    if (!rec || !r) return;
    const sched = scheduleOf(rec);
    if (sched.t.length === 0) {
      setNote("The recording has no commanded twist (cmd_vel): nothing to run on the simulator.");
      return;
    }
    setNote(
      `Simulating ${sched.t[sched.t.length - 1].toFixed(1)} s of the recording's commands with ${r.label}...`,
    );
    setTimeout(() => {
      try {
        const seed = typeof rec.summary?.seed === "number" ? rec.summary.seed : 1;
        const run = runSchedule({ model: r.model, doc: r.doc }, sched, rec, { seed });
        const cmp = compareRecordings(rec, parseRecording(run.files));
        renderCompare(cmp, rec.simulated);
      } catch (e) {
        setNote(`Comparison failed: ${e instanceof Error ? e.message : String(e)}.`);
      }
    });
  });

  /**
   * @param {ReturnType<typeof compareRecordings>} cmp
   * @param {boolean} simulated
   */
  function renderCompare(cmp, simulated) {
    const t = $("ros-compare-out");
    t.hidden = false;
    t.replaceChildren();
    const row = (/** @type {string[]} */ cells, head = false) => {
      const tr = document.createElement("tr");
      for (const c of cells) {
        const el = document.createElement(head ? "th" : "td");
        el.textContent = c;
        tr.append(el);
      }
      t.append(tr);
    };
    const f = (/** @type {number | null} */ v) => (v === null ? "-" : v.toPrecision(3));
    row(["field", "RMS diff", "max diff", "recording", "simulation"], true);
    for (const r of cmp.fields)
      row([r.field.replace("wheel_state.", ""), f(r.rms), f(r.max), f(r.finalA), f(r.finalB)]);
    setNote(
      `Simulation vs ${simulated ? "a SIMULATED recording (not a robot)" : "the recording"}: the recording's command schedule run on the simulator, compared through the firmware's ODOMETRY and WHEEL_STATE fields at ${cmp.samples} samples over ${cmp.seconds.toFixed(1)} s.${cmp.notes.length ? ` ${cmp.notes.join("; ")}.` : ""}`,
    );
    d.compareRows = String(cmp.fields.length);
  }
  $("ros-close").addEventListener("click", () => {
    if (mode === "live") disconnect();
    stop();
    $("ros-replay-controls").hidden = true;
    $("ros-compare-row").hidden = true;
  });
  $("sim-open").addEventListener("click", () => {
    if (mode === "live") disconnect();
    if (mode) stop();
  });

  // ------------------------------------------------------------ frame loop
  /** @param {number} now */
  function frame(now) {
    raf = requestAnimationFrame(frame);
    const dt = last === 0 ? 0 : Math.min(0.25, (now - last) / 1000);
    last = now;
    frames++;
    if (now - fpsT0 >= 1000) {
      fps = (frames * 1000) / (now - fpsT0);
      frames = 0;
      fpsT0 = now;
    }
    const src = mode === "live" ? live : mode === "replay" ? replay : null;
    if (!src) return;
    if (mode === "live" && gate) gate.update(teleop());
    if (mode === "replay" && replay && running)
      replay.advance(dt * Number(/** @type {HTMLSelectElement} */ ($("ros-replay-speed")).value));
    const f = src.frame();
    // Follow the robot, as the simulation does (not across a jump: a restart, a looped fixture).
    if (prevBase) {
      const dx = f.base.x - prevBase.x;
      const dy = f.base.y - prevBase.y;
      if ((dx !== 0 || dy !== 0) && Math.hypot(dx, dy) < 2) {
        view.camera.position.x += dx;
        view.camera.position.y += dy;
        view.controls.target.x += dx;
        view.controls.target.y += dy;
        view.controls.update();
      }
    }
    prevBase = { x: f.base.x, y: f.base.y };
    view.holdRender = true;
    layer.draw(
      f,
      robot ?? { baseHeight: 0, rootInBase: { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], p: [0, 0, 0] } },
    );
    view.holdRender = false;
    view.render();
    if (now - plotAt > 100) {
      plotAt = now;
      for (const [k, p] of Object.entries(plots))
        p.draw(src.history[/** @type {keyof typeof plots} */ (k)], f.t, 20);
      hud(f);
    }
    if (mode === "live" && live && now - topicsAt > 500) {
      topicsAt = now;
      topicsTable(live);
      updateDrive();
    }
  }

  /** @param {import("../core/ros/track.js").ViewFrame} f */
  function hud(f) {
    const table = $("hud-errors");
    table.replaceChildren();
    /** @param {string} label @param {number | null} color @param {string} value */
    const line = (label, color, value) => {
      const tr = document.createElement("tr");
      const td0 = document.createElement("td");
      if (color !== null) {
        const sw = document.createElement("span");
        sw.className = "sw";
        sw.style.background = `#${color.toString(16).padStart(6, "0")}`;
        td0.append(sw);
      }
      td0.append(label);
      const td = document.createElement("td");
      td.textContent = value;
      tr.append(td0, td);
      table.append(tr);
    };
    if (mode === "live" && live) {
      $("hud-time").textContent =
        `t = ${f.t.toFixed(1)} s · ${client?.state ?? ""} · base ${live.baseFrame()}`;
      const o = f.trails.odom;
      const e = f.trails.est;
      line(
        "wheel odometry (robot)",
        TRAIL_COLORS.odom,
        o ? `${o.x.toFixed(2)}, ${o.y.toFixed(2)} m` : "no /odom yet",
      );
      line(
        "+ scan matching (studio)",
        TRAIL_COLORS.est,
        e && o ? `${Math.hypot(e.x - o.x, e.y - o.y).toFixed(3)} m from odometry` : "-",
      );
      line("LiDAR mount", null, live.mountFrom);
      line("no ground truth on a robot: no error measures", null, "");
      d.rosScans = String(live.topics["/scan"]?.count ?? 0);
      d.rosT = f.t.toFixed(2);
    } else if (replay) {
      $("hud-time").textContent =
        `t = ${(replay.time - replay.start).toFixed(1)} / ${(replay.end - replay.start).toFixed(1)} s${replay.done ? " · end" : ""}`;
      const er = replay.track.errors();
      if (er) {
        line(
          "wheel odometry",
          TRAIL_COLORS.odom,
          `${er.odom.toFixed(3)} m (RMS ${er.odomRms.toFixed(3)})`,
        );
        line(
          "+ scan matching",
          TRAIL_COLORS.est,
          `${er.est.toFixed(3)} m (RMS ${er.estRms.toFixed(3)})`,
        );
        if (replay.track.est?.pf)
          line("MCL", TRAIL_COLORS.mcl, `${er.mcl.toFixed(3)} m (RMS ${er.mclRms.toFixed(3)})`);
        line("errors vs the recording's simulated truth", null, "");
      } else line("no ground truth in this recording: no error measures", null, "");
      d.replayT = (replay.time - replay.start).toFixed(2);
      d.replayDone = String(replay.done);
      d.replayScans = String(replay.track.scans);
    }
    $("hud-perf").textContent = `${fps.toFixed(0)} fps (measured in this browser)`;
  }

  /** @param {LiveSession} s */
  function topicsTable(s) {
    const t = $("ros-topics");
    t.replaceChildren();
    const head = document.createElement("tr");
    for (const h of ["topic", "Hz", "msgs", "age", "rejected"]) {
      const th = document.createElement("th");
      th.textContent = h;
      head.append(th);
    }
    t.append(head);
    for (const r of s.topicSummary()) {
      const tr = document.createElement("tr");
      if (r.age === null || r.age > 2) tr.className = "stale";
      if (r.rejected > 0) tr.className = "bad";
      if (r.error) tr.title = r.error;
      for (const c of [
        r.topic,
        r.hz.toFixed(1),
        String(r.count),
        r.age === null ? "-" : `${r.age.toFixed(1)} s`,
        String(r.rejected),
      ]) {
        const td = document.createElement("td");
        td.textContent = c;
        tr.append(td);
      }
      t.append(tr);
    }
  }

  return { robotChanged };
}
