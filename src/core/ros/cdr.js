// CDR serialization of ROS 2 messages (XCDR1, little endian), driven by the
// definitions in msgdefs.js: the 4-byte encapsulation header 00 01 00 00,
// then each field aligned to its own size (at most 8), counted from the
// first byte after the header. Strings are a uint32 length that counts the
// terminating NUL, the UTF-8 bytes and the NUL; sequences a uint32 element
// count and the elements; fixed arrays only the elements. This is what
// rmw_fastrtps and rmw_cyclonedds write for these (final, non-XTypes) types.

import { PRIMITIVES, msgFields } from "./msgdefs.js";

/** @typedef {import("./msgdefs.js").MsgField} MsgField */

const utf8 = new TextEncoder();

class Writer {
  constructor() {
    this.buf = new Uint8Array(256);
    this.view = new DataView(this.buf.buffer);
    this.pos = 4; // after the encapsulation header
    this.buf.set([0, 1, 0, 0]);
  }
  /** @param {number} n */
  ensure(n) {
    if (this.pos + n <= this.buf.length) return;
    let size = this.buf.length * 2;
    while (size < this.pos + n) size *= 2;
    const next = new Uint8Array(size);
    next.set(this.buf);
    this.buf = next;
    this.view = new DataView(next.buffer);
  }
  /** @param {number} a */
  align(a) {
    const off = (this.pos - 4) % a;
    if (off) {
      const pad = a - off;
      this.ensure(pad);
      this.pos += pad; // the buffer starts zeroed and only grows: padding stays 0
    }
  }
  /**
   * @param {string} type
   * @param {unknown} v
   */
  primitive(type, v) {
    const size = /** @type {Record<string, number>} */ (PRIMITIVES)[type];
    this.align(size);
    this.ensure(size);
    const d = this.view;
    const p = this.pos;
    if (type === "float64") d.setFloat64(p, num(v), true);
    else if (type === "float32") d.setFloat32(p, num(v), true);
    else if (type === "int64") d.setBigInt64(p, BigInt(intOf(v)), true);
    else if (type === "uint64") d.setBigUint64(p, BigInt(intOf(v)), true);
    else if (type === "int32") d.setInt32(p, intOf(v), true);
    else if (type === "uint32") d.setUint32(p, intOf(v), true);
    else if (type === "int16") d.setInt16(p, intOf(v), true);
    else if (type === "uint16") d.setUint16(p, intOf(v), true);
    else if (type === "int8") d.setInt8(p, intOf(v));
    else if (type === "bool") d.setUint8(p, v ? 1 : 0);
    else d.setUint8(p, intOf(v)); // uint8, byte, char
    this.pos += size;
  }
  /** @param {unknown} v */
  string(v) {
    const bytes = utf8.encode(typeof v === "string" ? v : "");
    this.primitive("uint32", bytes.length + 1);
    this.ensure(bytes.length + 1);
    this.buf.set(bytes, this.pos);
    this.pos += bytes.length + 1;
  }
  /**
   * @param {string} type
   * @param {unknown} v
   */
  single(type, v) {
    if (type === "string") this.string(v);
    else if (Object.hasOwn(PRIMITIVES, type)) this.primitive(type, v);
    else this.message(type, v);
  }
  /**
   * @param {string} type
   * @param {unknown} value
   */
  message(type, value) {
    const obj = /** @type {Record<string, unknown>} */ (
      value && typeof value === "object" ? value : {}
    );
    for (const f of msgFields(type)) this.field(f, obj[f.name]);
  }
  /**
   * @param {MsgField} f
   * @param {unknown} v
   */
  field(f, v) {
    if (f.array === "") return this.single(f.type, v);
    const items = /** @type {ArrayLike<unknown>} */ (
      v && typeof v === "object" && "length" in v ? v : []
    );
    if (f.array === "fixed") {
      const n = /** @type {number} */ (f.length);
      if (items.length !== n)
        throw new Error(`${f.name}: ${f.type}[${n}] needs ${n} values, got ${items.length}`);
    } else this.primitive("uint32", items.length);
    for (let i = 0; i < items.length; i++) this.single(f.type, items[i]);
  }
}

/** @param {unknown} v */
function num(v) {
  if (v === undefined) return 0;
  if (typeof v !== "number") throw new Error(`expected a number, got ${typeof v}`);
  return v;
}

/** @param {unknown} v */
function intOf(v) {
  const n = num(v);
  if (!Number.isInteger(n)) throw new Error(`expected an integer, got ${n}`);
  return n;
}

/**
 * Serialize a message. Missing fields are written as zeros, "" and empty
 * sequences; a value of the wrong kind (a string for a number, 1.5 for an
 * integer, a fixed array of the wrong length) is an error.
 * @param {string} type full name, e.g. "sensor_msgs/msg/LaserScan"
 * @param {unknown} value
 * @returns {Uint8Array}
 */
export function encodeCdr(type, value) {
  const w = new Writer();
  w.message(type, value);
  return w.buf.slice(0, w.pos);
}
