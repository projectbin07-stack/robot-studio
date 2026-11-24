// Test-only readers for what src/core/ros writes: an MCAP record walker that
// checks every CRC the writer fills in, and a CDR decoder driven by the same
// definitions. The real cross-checks are rosbag-inspect and ROS 2's own
// rosbag2 / rclpy in CI; these let the unit tests look inside the files.

import { crc32 } from "../src/core/zip.js";
import { PRIMITIVES, msgFields } from "../src/core/ros/msgdefs.js";
import { OP } from "../src/core/ros/mcap.js";

const td = new TextDecoder();

/** @typedef {{ messageCount: bigint, schemaCount: number, channelCount: number, attachmentCount: number, metadataCount: number, chunkCount: number, start: bigint, end: bigint }} McapStats */

/**
 * @param {Uint8Array} buf
 */
export function readMcap(buf) {
  const v = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  const magic = [0x89, 0x4d, 0x43, 0x41, 0x50, 0x30, 0x0d, 0x0a];
  for (let i = 0; i < 8; i++) {
    if (buf[i] !== magic[i]) throw new Error("bad leading magic");
    if (buf[buf.length - 8 + i] !== magic[i]) throw new Error("bad trailing magic");
  }
  /** @type {{ op: number, start: number, content: Uint8Array }[]} */
  const records = [];
  let p = 8;
  while (p < buf.length - 8) {
    const op = buf[p];
    const len = Number(v.getBigUint64(p + 1, true));
    records.push({ op, start: p, content: buf.subarray(p + 9, p + 9 + len) });
    p += 9 + len;
  }
  if (p !== buf.length - 8) throw new Error("records overrun the file");
  /** @param {Uint8Array} c */
  const reader = (c) => {
    const d = new DataView(c.buffer, c.byteOffset, c.byteLength);
    let o = 0;
    return {
      u8: () => c[o++],
      u16: () => ((o += 2), d.getUint16(o - 2, true)),
      u32: () => ((o += 4), d.getUint32(o - 4, true)),
      u64: () => ((o += 8), d.getBigUint64(o - 8, true)),
      str: () => {
        const n = d.getUint32(o, true);
        o += 4 + n;
        return td.decode(c.subarray(o - n, o));
      },
      bytes: (/** @type {number} */ n) => ((o += n), c.subarray(o - n, o)),
      rest: () => c.subarray(o),
      get pos() {
        return o;
      },
    };
  };
  /** @type {Map<number, { name: string, encoding: string, data: string }>} */
  const schemas = new Map();
  /** @type {Map<number, { topic: string, schemaId: number, encoding: string, metadata: Record<string, string> }>} */
  const channels = new Map();
  /** @type {{ channel: number, sequence: number, logTime: bigint, data: Uint8Array }[]} */
  const messages = [];
  /** @type {{ name: string, data: Record<string, string> }[]} */
  const metadata = [];
  /** @type {McapStats | null} */
  let stats = null;
  let chunks = 0;
  let chunkIndexes = 0;
  let dataEnd = -1;
  /** @param {ReturnType<typeof reader>} r */
  const map = (r) => {
    const n = r.u32();
    const end = r.pos + n;
    /** @type {Record<string, string>} */
    const m = {};
    while (r.pos < end) {
      const k = r.str();
      m[k] = r.str();
    }
    return m;
  };
  /** @param {{ op: number, content: Uint8Array }} rec @param {boolean} inSummary */
  const handle = (rec, inSummary) => {
    const r = reader(rec.content);
    if (rec.op === OP.schema) {
      const id = r.u16();
      schemas.set(id, { name: r.str(), encoding: r.str(), data: td.decode(r.bytes(r.u32())) });
    } else if (rec.op === OP.channel) {
      const id = r.u16();
      channels.set(id, { schemaId: r.u16(), topic: r.str(), encoding: r.str(), metadata: map(r) });
    } else if (rec.op === OP.message && !inSummary) {
      messages.push({
        channel: r.u16(),
        sequence: r.u32(),
        logTime: r.u64(),
        data: (r.u64(), r.rest()),
      });
    } else if (rec.op === OP.chunk) {
      chunks++;
      r.u64();
      r.u64();
      const size = Number(r.u64());
      const crc = r.u32();
      if (r.str() !== "") throw new Error("compressed chunk");
      const data = r.bytes(Number(r.u64()));
      if (data.length !== size || crc32(data) !== crc) throw new Error("chunk CRC or size wrong");
      let q = 0;
      const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
      while (q < data.length) {
        const op = data[q];
        const len = Number(dv.getBigUint64(q + 1, true));
        handle({ op, content: data.subarray(q + 9, q + 9 + len) }, false);
        q += 9 + len;
      }
    } else if (rec.op === OP.metadata) metadata.push({ name: r.str(), data: map(r) });
    else if (rec.op === OP.statistics) {
      stats = {
        messageCount: r.u64(),
        schemaCount: r.u16(),
        channelCount: r.u32(),
        attachmentCount: r.u32(),
        metadataCount: r.u32(),
        chunkCount: r.u32(),
        start: r.u64(),
        end: r.u64(),
      };
    } else if (rec.op === OP.chunkIndex) chunkIndexes++;
  };
  let inSummary = false;
  for (const rec of records) {
    if (rec.op === OP.dataEnd) {
      const want = reader(rec.content).u32();
      if (crc32(buf.subarray(0, rec.start)) !== want) throw new Error("data section CRC wrong");
      dataEnd = rec.start;
      inSummary = true;
      continue;
    }
    if (rec.op === OP.footer) {
      const r = reader(rec.content);
      const summaryStart = Number(r.u64());
      r.u64();
      const crc = r.u32();
      if (crc32(buf.subarray(summaryStart, rec.start + 9 + 16)) !== crc)
        throw new Error("summary CRC wrong");
      continue;
    }
    if (inSummary && (rec.op === OP.schema || rec.op === OP.channel)) continue;
    handle(rec, inSummary);
  }
  if (dataEnd < 0) throw new Error("no Data End record");
  const header = reader(records[0].content);
  return {
    profile: header.str(),
    library: header.str(),
    schemas,
    channels,
    messages,
    metadata,
    stats: /** @type {McapStats | null} */ (stats),
    chunks,
    chunkIndexes,
  };
}

/**
 * Decode XCDR1 little-endian CDR (what encodeCdr writes).
 * @param {string} type
 * @param {Uint8Array} data
 * @returns {any}
 */
export function decodeCdr(type, data) {
  if (data[0] !== 0 || data[1] !== 1) throw new Error("not CDR_LE");
  const v = new DataView(data.buffer, data.byteOffset, data.byteLength);
  let p = 4;
  /** @param {number} a */
  const align = (a) => {
    const off = (p - 4) % a;
    if (off) p += a - off;
  };
  /** @param {string} t */
  const prim = (t) => {
    const size = /** @type {Record<string, number>} */ (PRIMITIVES)[t];
    align(size);
    const at = p;
    p += size;
    if (t === "float64") return v.getFloat64(at, true);
    if (t === "float32") return v.getFloat32(at, true);
    if (t === "int32") return v.getInt32(at, true);
    if (t === "uint32") return v.getUint32(at, true);
    if (t === "int64") return Number(v.getBigInt64(at, true));
    if (t === "uint64") return Number(v.getBigUint64(at, true));
    if (t === "int16") return v.getInt16(at, true);
    if (t === "uint16") return v.getUint16(at, true);
    if (t === "int8") return v.getInt8(at);
    if (t === "bool") return v.getUint8(at) !== 0;
    return v.getUint8(at);
  };
  /** @param {string} t @returns {any} */
  const one = (t) => {
    if (t === "string") {
      const n = /** @type {number} */ (prim("uint32"));
      const s = td.decode(data.subarray(p, p + n - 1));
      if (data[p + n - 1] !== 0) throw new Error("string without NUL");
      p += n;
      return s;
    }
    if (Object.hasOwn(PRIMITIVES, t)) return prim(t);
    /** @type {Record<string, any>} */
    const o = {};
    for (const f of msgFields(t)) {
      if (f.array === "") o[f.name] = one(f.type);
      else {
        const n = /** @type {number} */ (f.array === "fixed" ? f.length : prim("uint32"));
        o[f.name] = Array.from({ length: n }, () => one(f.type));
      }
    }
    return o;
  };
  const out = one(type);
  if (data.length - p > 3) throw new Error(`${data.length - p} bytes left after ${type}`);
  return out;
}
