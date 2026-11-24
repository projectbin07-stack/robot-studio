// A rosbridge v2 protocol client (JSON over WebSocket), written from the
// protocol specification (rosbridge_suite ROSBRIDGE_PROTOCOL.md): the
// operations subscribe / unsubscribe, advertise / unadvertise, publish,
// call_service and their replies publish, service_response, status, and
// fragment reassembly. Every incoming message is untrusted: it is size
// checked before parsing, parsed without evaluation (json.js), and each
// operation's fields are type checked; anything else is counted and
// dropped. After a lost connection it reconnects with exponential backoff
// and subscribes / advertises again. The WebSocket constructor and the
// timers are injected, so Node tests drive it against the fake server.

import { parseRosJson } from "./json.js";

/** @typedef {"idle" | "connecting" | "open" | "reconnecting" | "closed"} ConnState */

/**
 * The part of the WebSocket interface used (browser and Node 22+ both have it).
 * @typedef {object} SocketLike
 * @property {number} readyState
 * @property {(data: string) => void} send
 * @property {(code?: number, reason?: string) => void} close
 * @property {((ev: any) => void) | null} onopen
 * @property {((ev: { data: unknown }) => void) | null} onmessage
 * @property {((ev: any) => void) | null} onclose
 * @property {((ev: any) => void) | null} onerror
 */

/**
 * @typedef {object} ClientOptions
 * @property {string} url a checked ws:// or wss:// URL (url.js)
 * @property {new (url: string) => SocketLike} WebSocket
 * @property {number} [maxMessageBytes] larger messages are dropped unparsed (default 8 MiB)
 * @property {number} [maxFragments] fragments per message (default 4096)
 * @property {number} [reconnectMs] first retry delay (default 500 ms), doubling
 * @property {number} [maxReconnectMs] longest retry delay (default 8 s)
 * @property {boolean} [reconnect] default true
 * @property {(fn: () => void, ms: number) => unknown} [setTimeout]
 * @property {(h: unknown) => void} [clearTimeout]
 */

const TOPIC = /^\/?[A-Za-z_~][A-Za-z0-9_~/]{0,254}$/;

/** @param {string} name */
export const validName = (name) => TOPIC.test(name) && !name.includes("//");

export class RosbridgeClient {
  /** @param {ClientOptions} opts */
  constructor(opts) {
    this.opts = opts;
    this.maxBytes = opts.maxMessageBytes ?? 8 * 1024 * 1024;
    this.maxFragments = opts.maxFragments ?? 4096;
    this.setTimeout = opts.setTimeout ?? ((fn, ms) => setTimeout(fn, ms));
    this.clearTimeout = opts.clearTimeout ?? ((h) => clearTimeout(/** @type {any} */ (h)));
    /** @type {ConnState} */
    this.state = "idle";
    /** @type {SocketLike | null} */
    this.ws = null;
    /** @type {Map<string, { type: string, options: Record<string, number>, handlers: Set<(msg: unknown) => void> }>} */
    this.subs = new Map();
    /** @type {Map<string, string>} topic -> type */
    this.adverts = new Map();
    /** @type {Map<string, { resolve: (v: unknown) => void, reject: (e: Error) => void, timer: unknown }>} */
    this.calls = new Map();
    /** @type {Map<string, { total: number, parts: (string | undefined)[], have: number, size: number }>} */
    this.fragments = new Map();
    this.nextId = 1;
    this.attempt = 0;
    this.wantOpen = false;
    /** @type {unknown} */
    this.retryTimer = null;
    this.stats = {
      received: 0,
      bytes: 0,
      sent: 0,
      connects: 0,
      dropped: { oversized: 0, malformed: 0, invalid: 0, unknownOp: 0, binary: 0, unsubscribed: 0 },
    };
    /** @type {Set<(state: ConnState, detail: string) => void>} */
    this.stateListeners = new Set();
    /** @type {Set<(level: string, text: string) => void>} */
    this.logListeners = new Set();
  }

  /** @param {(state: ConnState, detail: string) => void} fn */
  onState(fn) {
    this.stateListeners.add(fn);
    return () => this.stateListeners.delete(fn);
  }

  /** @param {(level: string, text: string) => void} fn */
  onLog(fn) {
    this.logListeners.add(fn);
    return () => this.logListeners.delete(fn);
  }

  /**
   * @param {ConnState} s
   * @param {string} [detail]
   */
  setState(s, detail = "") {
    this.state = s;
    for (const fn of this.stateListeners) fn(s, detail);
  }

  /**
   * @param {string} level
   * @param {string} text
   */
  log(level, text) {
    for (const fn of this.logListeners) fn(level, text);
  }

  connect() {
    if (this.state === "open" || this.state === "connecting") return;
    this.wantOpen = true;
    this.open();
  }

  open() {
    this.setState(this.attempt === 0 ? "connecting" : "reconnecting", this.opts.url);
    let ws;
    try {
      ws = new this.opts.WebSocket(this.opts.url);
    } catch (e) {
      this.log(
        "error",
        `cannot open ${this.opts.url}: ${e instanceof Error ? e.message : String(e)}`,
      );
      this.scheduleRetry();
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      if (this.ws !== ws) return;
      this.attempt = 0;
      this.stats.connects++;
      this.setState("open", this.opts.url);
      for (const [topic, type] of this.adverts) this.sendOp({ op: "advertise", topic, type });
      for (const [topic, s] of this.subs) this.sendSubscribe(topic, s.type, s.options);
    };
    ws.onmessage = (ev) => {
      if (this.ws === ws) this.receive(ev.data);
    };
    ws.onerror = () => {
      if (this.ws === ws) this.log("error", `connection error (${this.opts.url})`);
    };
    ws.onclose = (ev) => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.fragments.clear();
      for (const [id, c] of this.calls) {
        this.clearTimeout(c.timer);
        c.reject(new Error("connection closed"));
        this.calls.delete(id);
      }
      const why =
        ev && typeof ev.code === "number"
          ? `closed (${ev.code}${ev.reason ? `: ${String(ev.reason).slice(0, 120)}` : ""})`
          : "closed";
      if (this.wantOpen && this.opts.reconnect !== false) {
        this.setState("reconnecting", why);
        this.scheduleRetry();
      } else this.setState("closed", why);
    };
  }

  scheduleRetry() {
    if (!this.wantOpen || this.opts.reconnect === false) {
      this.setState("closed", "not reconnecting");
      return;
    }
    const base = this.opts.reconnectMs ?? 500;
    const delay = Math.min(this.opts.maxReconnectMs ?? 8000, base * 2 ** this.attempt);
    this.attempt++;
    this.retryTimer = this.setTimeout(() => {
      this.retryTimer = null;
      if (this.wantOpen) this.open();
    }, delay);
  }

  /** Close for good (no reconnect). */
  close() {
    this.wantOpen = false;
    if (this.retryTimer !== null) this.clearTimeout(this.retryTimer);
    this.retryTimer = null;
    const ws = this.ws;
    this.ws = null;
    if (ws) {
      ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null;
      try {
        ws.close(1000, "closed by the user");
      } catch {
        // already closed
      }
    }
    for (const c of this.calls.values()) {
      this.clearTimeout(c.timer);
      c.reject(new Error("connection closed"));
    }
    this.calls.clear();
    this.setState("closed", "closed by the user");
  }

  get isOpen() {
    return this.state === "open" && this.ws !== null && this.ws.readyState === 1;
  }

  /**
   * @param {Record<string, unknown>} msg
   * @returns {boolean} sent
   */
  sendOp(msg) {
    if (!this.ws || this.ws.readyState !== 1) return false;
    this.ws.send(JSON.stringify(msg));
    this.stats.sent++;
    return true;
  }

  /**
   * @param {string} topic
   * @param {string} type
   * @param {Record<string, number>} options
   */
  sendSubscribe(topic, type, options) {
    this.sendOp({ op: "subscribe", id: `subscribe:${topic}`, topic, type, ...options });
  }

  /**
   * Subscribe; returns the function that removes this handler.
   * @param {string} topic
   * @param {string} type e.g. "sensor_msgs/msg/LaserScan"
   * @param {(msg: unknown) => void} handler
   * @param {{ throttle_rate?: number, queue_length?: number }} [options]
   */
  subscribe(topic, type, handler, options = {}) {
    if (!validName(topic)) throw new Error(`not a valid topic name: ${topic}`);
    let s = this.subs.get(topic);
    if (!s) {
      /** @type {Record<string, number>} */
      const o = {};
      if (options.throttle_rate !== undefined) o.throttle_rate = options.throttle_rate;
      if (options.queue_length !== undefined) o.queue_length = options.queue_length;
      s = { type, options: o, handlers: new Set() };
      this.subs.set(topic, s);
      this.sendSubscribe(topic, type, o);
    }
    s.handlers.add(handler);
    return () => {
      const cur = this.subs.get(topic);
      if (!cur) return;
      cur.handlers.delete(handler);
      if (cur.handlers.size === 0) {
        this.subs.delete(topic);
        this.sendOp({ op: "unsubscribe", id: `subscribe:${topic}`, topic });
      }
    };
  }

  /**
   * @param {string} topic
   * @param {string} type
   */
  advertise(topic, type) {
    if (!validName(topic)) throw new Error(`not a valid topic name: ${topic}`);
    this.adverts.set(topic, type);
    this.sendOp({ op: "advertise", id: `advertise:${topic}`, topic, type });
  }

  /** @param {string} topic */
  unadvertise(topic) {
    if (this.adverts.delete(topic))
      this.sendOp({ op: "unadvertise", id: `advertise:${topic}`, topic });
  }

  /**
   * Publish on an advertised topic. Returns false when not connected.
   * @param {string} topic
   * @param {unknown} msg
   */
  publish(topic, msg) {
    if (!this.adverts.has(topic)) throw new Error(`advertise ${topic} before publishing`);
    return this.sendOp({ op: "publish", topic, msg });
  }

  /**
   * @param {string} service
   * @param {Record<string, unknown>} args
   * @param {{ timeoutMs?: number, type?: string }} [opts]
   * @returns {Promise<unknown>} the response's values
   */
  callService(service, args, opts = {}) {
    if (!validName(service))
      return Promise.reject(new Error(`not a valid service name: ${service}`));
    const id = `call_service:${service}:${this.nextId++}`;
    return new Promise((resolve, reject) => {
      const msg = {
        op: "call_service",
        id,
        service,
        args,
        ...(opts.type ? { type: opts.type } : {}),
      };
      if (!this.sendOp(msg)) {
        reject(new Error("not connected"));
        return;
      }
      const timer = this.setTimeout(() => {
        this.calls.delete(id);
        reject(new Error(`${service}: no response within ${opts.timeoutMs ?? 3000} ms`));
      }, opts.timeoutMs ?? 3000);
      this.calls.set(id, { resolve, reject, timer });
    });
  }

  /** @param {unknown} data */
  receive(data) {
    this.stats.received++;
    if (typeof data !== "string") {
      // Binary frames carry BSON / CBOR / PNG, which are never requested.
      this.stats.dropped.binary++;
      return;
    }
    this.stats.bytes += data.length;
    if (data.length > this.maxBytes) {
      this.stats.dropped.oversized++;
      this.log("warn", `dropped a ${data.length}-byte message (limit ${this.maxBytes})`);
      return;
    }
    let m;
    try {
      m = parseRosJson(data);
    } catch (e) {
      this.stats.dropped.malformed++;
      this.log(
        "warn",
        `dropped a malformed message: ${e instanceof Error ? e.message : String(e)}`,
      );
      return;
    }
    this.dispatch(m);
  }

  /** @param {unknown} m */
  dispatch(m) {
    if (m === null || typeof m !== "object" || Array.isArray(m)) {
      this.stats.dropped.invalid++;
      return;
    }
    const o = /** @type {Record<string, unknown>} */ (m);
    switch (o.op) {
      case "publish": {
        if (typeof o.topic !== "string" || o.msg === null || typeof o.msg !== "object") {
          this.stats.dropped.invalid++;
          return;
        }
        const s = this.subs.get(o.topic);
        if (!s) {
          this.stats.dropped.unsubscribed++;
          return;
        }
        for (const h of [...s.handlers]) {
          try {
            h(o.msg);
          } catch (e) {
            this.stats.dropped.invalid++;
            this.log("warn", `${o.topic}: ${e instanceof Error ? e.message : String(e)}`);
          }
        }
        return;
      }
      case "service_response": {
        const c = typeof o.id === "string" ? this.calls.get(o.id) : undefined;
        if (!c) {
          this.stats.dropped.invalid++;
          return;
        }
        this.calls.delete(/** @type {string} */ (o.id));
        this.clearTimeout(c.timer);
        if (o.result === false)
          c.reject(
            new Error(
              `service call failed: ${typeof o.values === "string" ? o.values.slice(0, 200) : "no reason given"}`,
            ),
          );
        else c.resolve(o.values);
        return;
      }
      case "status": {
        const level = typeof o.level === "string" ? o.level.slice(0, 16) : "info";
        const text = typeof o.msg === "string" ? o.msg.slice(0, 500) : "";
        this.log(level, `rosbridge: ${text}`);
        return;
      }
      case "fragment":
        this.fragment(o);
        return;
      default:
        this.stats.dropped.unknownOp++;
    }
  }

  /** @param {Record<string, unknown>} o */
  fragment(o) {
    const { id, data, num, total } = o;
    if (
      typeof id !== "string" ||
      typeof data !== "string" ||
      !Number.isInteger(num) ||
      !Number.isInteger(total) ||
      /** @type {number} */ (total) < 1 ||
      /** @type {number} */ (total) > this.maxFragments ||
      /** @type {number} */ (num) < 0 ||
      /** @type {number} */ (num) >= /** @type {number} */ (total)
    ) {
      this.stats.dropped.invalid++;
      return;
    }
    const n = /** @type {number} */ (num);
    const t = /** @type {number} */ (total);
    let f = this.fragments.get(id);
    if (!f) {
      if (this.fragments.size >= 16) {
        this.stats.dropped.oversized++;
        return;
      }
      f = { total: t, parts: new Array(t), have: 0, size: 0 };
      this.fragments.set(id, f);
    }
    if (f.total !== t) {
      this.fragments.delete(id);
      this.stats.dropped.invalid++;
      return;
    }
    if (f.parts[n] === undefined) {
      f.parts[n] = data;
      f.have++;
      f.size += data.length;
    }
    if (f.size > this.maxBytes) {
      this.fragments.delete(id);
      this.stats.dropped.oversized++;
      return;
    }
    if (f.have === f.total) {
      this.fragments.delete(id);
      this.receiveWhole(f.parts.join(""));
    }
  }

  /** @param {string} text a reassembled message */
  receiveWhole(text) {
    let m;
    try {
      m = parseRosJson(text, { maxBytes: this.maxBytes });
    } catch {
      this.stats.dropped.malformed++;
      return;
    }
    this.dispatch(m);
  }
}
