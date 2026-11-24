#!/usr/bin/env node
// A fake rosbridge server for tests and the CI browser job: an RFC 6455
// WebSocket server (handshake, masked client frames, fragmentation,
// ping / pong, close) written on node:http, speaking the rosbridge v2
// operations the studio uses. It replays message fixtures (one rosbridge
// "publish" per line with a time offset) for the topics a client has
// subscribed to, answers rosapi/get_param with a robot_description, records
// everything clients send (advertise, publish on /cmd_vel with arrival
// times), and can misbehave on request: malformed JSON, bare NaN /
// Infinity tokens, an oversized message, an unknown op, a binary frame,
// dropping the connection.
//
//   node scripts/fake-rosbridge.mjs --port 9090 --fixture tests/fixtures/rosbridge/sim_mecanum.jsonl
//     [--description tests/fixtures/rosbridge/robot_description.urdf] [--log cmd_vel.jsonl]
//
// The fixtures are simulator output (scripts/rosbridge-fixtures.js), in the
// JSON form rosbridge_server sends; nothing here came from a robot.

import { createServer } from "node:http";
import { createHash } from "node:crypto";
import { readFileSync, appendFileSync } from "node:fs";
import { fileURLToPath } from "node:url";

const GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/**
 * @typedef {object} FakeOptions
 * @property {number} [port] 0 = any free port
 * @property {{ dt: number, topic: string, line: string }[]} [fixture] replayed in a loop
 * @property {string} [description] robot_description for /robot_description and rosapi/get_param
 * @property {number} [speed] replay speed factor (default 1)
 * @property {(entry: { t: number, op: string, topic?: string, msg?: unknown }) => void} [onClientMessage]
 */

/**
 * A connected client, from the server's side.
 */
class Conn {
  /**
   * @param {import("node:net").Socket} socket
   * @param {FakeServer} server
   */
  constructor(socket, server) {
    this.socket = socket;
    this.server = server;
    /** @type {Set<string>} */
    this.subs = new Set();
    /** @type {Map<string, string>} */
    this.adverts = new Map();
    this.buf = Buffer.alloc(0);
    /** @type {Buffer[]} */
    this.partial = [];
    this.closed = false;
    socket.on("data", (d) => this.onData(d));
    socket.on("close", () => this.end());
    socket.on("error", () => this.end());
  }

  end() {
    if (this.closed) return;
    this.closed = true;
    this.server.conns.delete(this);
  }

  /** @param {Buffer} d */
  onData(d) {
    this.buf = Buffer.concat([this.buf, d]);
    for (;;) {
      if (this.buf.length < 2) return;
      const b0 = this.buf[0];
      const b1 = this.buf[1];
      const fin = (b0 & 0x80) !== 0;
      const op = b0 & 0x0f;
      const masked = (b1 & 0x80) !== 0;
      let len = b1 & 0x7f;
      let p = 2;
      if (len === 126) {
        if (this.buf.length < 4) return;
        len = this.buf.readUInt16BE(2);
        p = 4;
      } else if (len === 127) {
        if (this.buf.length < 10) return;
        len = Number(this.buf.readBigUInt64BE(2));
        p = 10;
      }
      if (!masked) {
        // RFC 6455 5.1: a server must close on an unmasked client frame.
        this.close(1002);
        return;
      }
      if (this.buf.length < p + 4 + len) return;
      const mask = this.buf.subarray(p, p + 4);
      const payload = Buffer.from(this.buf.subarray(p + 4, p + 4 + len));
      for (let i = 0; i < payload.length; i++) payload[i] ^= mask[i & 3];
      this.buf = this.buf.subarray(p + 4 + len);
      if (op === 0x8) {
        this.close(1000);
        return;
      }
      if (op === 0x9) {
        this.frame(0xa, payload);
        continue;
      }
      if (op === 0xa) continue;
      if (op === 0x1 || op === 0x0) {
        this.partial.push(payload);
        if (fin) {
          const text = Buffer.concat(this.partial).toString("utf8");
          this.partial = [];
          this.server.onClientText(this, text);
        }
      }
    }
  }

  /**
   * @param {number} op
   * @param {Buffer} payload
   */
  frame(op, payload) {
    if (this.closed) return;
    const n = payload.length;
    const head =
      n < 126 ? Buffer.from([0x80 | op, n]) : n < 65536 ? Buffer.alloc(4) : Buffer.alloc(10);
    if (n >= 126 && n < 65536) {
      head[0] = 0x80 | op;
      head[1] = 126;
      head.writeUInt16BE(n, 2);
    } else if (n >= 65536) {
      head[0] = 0x80 | op;
      head[1] = 127;
      head.writeBigUInt64BE(BigInt(n), 2);
    }
    this.socket.write(Buffer.concat([head, payload]));
  }

  /** @param {string} text */
  send(text) {
    this.frame(0x1, Buffer.from(text, "utf8"));
  }

  /** @param {number} code */
  close(code) {
    if (this.closed) return;
    const b = Buffer.alloc(2);
    b.writeUInt16BE(code, 0);
    this.frame(0x8, b);
    this.socket.end();
    this.end();
  }

  /** Drop the TCP connection without a close frame (a lost Wi-Fi link). */
  drop() {
    this.socket.destroy();
    this.end();
  }
}

export class FakeServer {
  /** @param {FakeOptions} opts */
  constructor(opts = {}) {
    this.opts = opts;
    /** @type {Set<Conn>} */
    this.conns = new Set();
    /** @type {{ t: number, op: string, topic?: string, msg?: unknown, raw: Record<string, unknown> }[]} */
    this.received = [];
    this.connections = 0;
    this.t0 = performance.now();
    this.http = createServer((_req, res) => {
      res.writeHead(426, { "content-type": "text/plain" });
      res.end("fake rosbridge: WebSocket only\n");
    });
    this.http.on("upgrade", (req, socket) => {
      const key = req.headers["sec-websocket-key"];
      if (typeof key !== "string" || req.headers.upgrade?.toLowerCase() !== "websocket") {
        socket.end("HTTP/1.1 400 Bad Request\r\n\r\n");
        return;
      }
      const accept = createHash("sha1")
        .update(key + GUID)
        .digest("base64");
      socket.write(
        "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" +
          `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
      );
      const conn = new Conn(/** @type {import("node:net").Socket} */ (socket), this);
      this.conns.add(conn);
      this.connections++;
    });
    /** @type {ReturnType<typeof setTimeout> | null} */
    this.timer = null;
    this.cursor = 0;
    this.loopStart = 0;
  }

  /** @returns {Promise<number>} the port */
  listen() {
    return new Promise((resolve) => {
      this.http.listen(this.opts.port ?? 0, "127.0.0.1", () => {
        const addr = this.http.address();
        resolve(typeof addr === "object" && addr ? addr.port : 0);
      });
    });
  }

  async close() {
    if (this.timer) clearTimeout(this.timer);
    this.timer = null;
    for (const c of [...this.conns]) c.drop();
    await new Promise((r) => this.http.close(() => r(undefined)));
  }

  now() {
    return performance.now() - this.t0;
  }

  /**
   * @param {Conn} conn
   * @param {string} text
   */
  onClientText(conn, text) {
    /** @type {any} */
    let m;
    try {
      m = JSON.parse(text);
    } catch {
      conn.send(
        JSON.stringify({ op: "status", level: "error", msg: "could not parse the message" }),
      );
      return;
    }
    const entry = { t: this.now(), op: String(m.op), topic: m.topic, msg: m.msg, raw: m };
    this.received.push(entry);
    this.opts.onClientMessage?.(entry);
    if (m.op === "subscribe" && typeof m.topic === "string") {
      conn.subs.add(m.topic);
      if (m.topic === "/robot_description" && this.opts.description !== undefined)
        conn.send(
          JSON.stringify({
            op: "publish",
            topic: "/robot_description",
            msg: { data: this.opts.description },
          }),
        );
      this.startReplay();
    } else if (m.op === "unsubscribe") conn.subs.delete(m.topic);
    else if (m.op === "advertise") conn.adverts.set(m.topic, m.type);
    else if (m.op === "unadvertise") conn.adverts.delete(m.topic);
    else if (m.op === "publish" && !conn.adverts.has(m.topic))
      conn.send(
        JSON.stringify({
          op: "status",
          level: "warning",
          msg: `publish on ${m.topic} without advertise`,
        }),
      );
    else if (m.op === "call_service") {
      if (m.service === "/rosapi/get_param" && this.opts.description !== undefined)
        conn.send(
          JSON.stringify({
            op: "service_response",
            id: m.id,
            service: m.service,
            result: true,
            // rosapi returns the parameter's value JSON-encoded.
            values: { value: JSON.stringify(this.opts.description), successful: true },
          }),
        );
      else
        conn.send(
          JSON.stringify({
            op: "service_response",
            id: m.id,
            service: m.service,
            result: false,
            values: "unknown service",
          }),
        );
    }
  }

  /** Send the fixture's messages for the subscribed topics, in a loop, at their own pace. */
  startReplay() {
    const fx = this.opts.fixture;
    if (this.timer || !fx || fx.length === 0) return;
    const speed = this.opts.speed ?? 1;
    this.cursor = 0;
    this.loopStart = this.now();
    const run = () => {
      const now = this.now();
      const loopLen = fx[fx.length - 1].dt + 0.05;
      while (
        this.cursor < fx.length &&
        this.loopStart + (fx[this.cursor].dt * 1000) / speed <= now
      ) {
        const f = fx[this.cursor++];
        for (const c of this.conns) if (c.subs.has(f.topic)) c.send(f.line);
      }
      if (this.cursor >= fx.length) {
        this.cursor = 0;
        this.loopStart += (loopLen * 1000) / speed;
      }
      this.timer = setTimeout(run, 5);
    };
    run();
  }

  /** @param {string} text */
  broadcast(text) {
    for (const c of this.conns) c.send(text);
  }

  /** @param {Buffer} data */
  broadcastBinary(data) {
    for (const c of this.conns) c.frame(0x2, data);
  }

  /** Drop every connection (the client should reconnect). */
  dropAll() {
    for (const c of [...this.conns]) c.drop();
  }

  /** Messages clients published on a topic, with arrival times (ms). */
  published(topic = "/cmd_vel") {
    return this.received.filter((r) => r.op === "publish" && r.topic === topic);
  }
}

/**
 * A fixture file: one JSON object per line, {"dt": s, "topic": ..., "line": "<the rosbridge message text>"}.
 * @param {string} path
 */
export function loadFixture(path) {
  return readFileSync(path, "utf8")
    .split("\n")
    .filter((l) => l.trim())
    .map((l) => /** @type {{ dt: number, topic: string, line: string }} */ (JSON.parse(l)));
}

if (process.argv[1] && fileURLToPath(import.meta.url) === process.argv[1]) {
  const { parseArgs } = await import("node:util");
  const { values } = parseArgs({
    options: {
      port: { type: "string", default: "9090" },
      fixture: { type: "string" },
      description: { type: "string" },
      log: { type: "string" },
    },
  });
  const logPath = values.log;
  const server = new FakeServer({
    port: Number(values.port),
    fixture: values.fixture ? loadFixture(values.fixture) : [],
    description: values.description ? readFileSync(values.description, "utf8") : undefined,
    onClientMessage: (e) => {
      if (logPath)
        appendFileSync(
          logPath,
          JSON.stringify({ t: Number(e.t.toFixed(1)), op: e.op, topic: e.topic, msg: e.msg }) +
            "\n",
        );
    },
  });
  const port = await server.listen();
  console.log(`fake rosbridge (simulated fixture data) on ws://127.0.0.1:${port}`);
  const stop = async () => {
    await server.close();
    process.exit(0);
  };
  process.on("SIGINT", stop);
  process.on("SIGTERM", stop);
}
