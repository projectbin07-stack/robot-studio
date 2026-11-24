// An MCAP writer (https://mcap.dev/spec), the layout rosbag2's MCAP storage
// plugin reads: magic, Header (profile "ros2"), the schemas and channels,
// then the messages in uncompressed chunks of about 1 MiB, each followed by
// its Message Index records; Metadata records; Data End with the CRC-32 of
// the data section; a summary section (schemas, channels, statistics,
// metadata and chunk indexes, with Summary Offset records) and the Footer
// with the summary CRC-32. Messages must be added in log-time order.

import { crc32 } from "../zip.js";

const MAGIC = Uint8Array.from([0x89, 0x4d, 0x43, 0x41, 0x50, 0x30, 0x0d, 0x0a]);
export const OP = Object.freeze({
  header: 0x01,
  footer: 0x02,
  schema: 0x03,
  channel: 0x04,
  message: 0x05,
  chunk: 0x06,
  messageIndex: 0x07,
  chunkIndex: 0x08,
  statistics: 0x0b,
  metadata: 0x0c,
  metadataIndex: 0x0d,
  summaryOffset: 0x0e,
  dataEnd: 0x0f,
});
const CHUNK_TARGET = 1 << 20;
const utf8 = new TextEncoder();

/** Little-endian byte builder. */
class Bytes {
  constructor(size = 1024) {
    this.buf = new Uint8Array(size);
    this.view = new DataView(this.buf.buffer);
    this.length = 0;
  }
  /** @param {number} n */
  ensure(n) {
    if (this.length + n <= this.buf.length) return;
    let size = this.buf.length * 2;
    while (size < this.length + n) size *= 2;
    const next = new Uint8Array(size);
    next.set(this.buf.subarray(0, this.length));
    this.buf = next;
    this.view = new DataView(next.buffer);
  }
  /** @param {number} v */
  u8(v) {
    this.ensure(1);
    this.buf[this.length++] = v;
  }
  /** @param {number} v */
  u16(v) {
    this.ensure(2);
    this.view.setUint16(this.length, v, true);
    this.length += 2;
  }
  /** @param {number} v */
  u32(v) {
    this.ensure(4);
    this.view.setUint32(this.length, v >>> 0, true);
    this.length += 4;
  }
  /** @param {number | bigint} v */
  u64(v) {
    this.ensure(8);
    this.view.setBigUint64(this.length, BigInt(v), true);
    this.length += 8;
  }
  /** @param {Uint8Array} b */
  raw(b) {
    this.ensure(b.length);
    this.buf.set(b, this.length);
    this.length += b.length;
  }
  /** @param {string} s */
  str(s) {
    const b = utf8.encode(s);
    this.u32(b.length);
    this.raw(b);
  }
  /** @param {Record<string, string>} m */
  strMap(m) {
    const inner = new Bytes(64);
    for (const [k, v] of Object.entries(m).sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0))) {
      inner.str(k);
      inner.str(v);
    }
    this.u32(inner.length);
    this.raw(inner.bytes());
  }
  bytes() {
    return this.buf.subarray(0, this.length);
  }
  /**
   * Append a record: opcode, u64 content length, content.
   * @param {number} op
   * @param {Bytes} content
   */
  record(op, content) {
    this.u8(op);
    this.u64(content.length);
    this.raw(content.bytes());
  }
}

/**
 * @typedef {object} McapChannelInfo
 * @property {number} id
 * @property {number} schemaId
 * @property {string} topic
 * @property {string} type
 * @property {Record<string, string>} metadata
 */

export class McapWriter {
  /** @param {{ library: string, chunkSize?: number }} opts */
  constructor(opts) {
    this.out = new Bytes(1 << 16);
    this.out.raw(MAGIC);
    const h = new Bytes();
    h.str("ros2");
    h.str(opts.library);
    this.out.record(OP.header, h);
    this.chunkSize = opts.chunkSize ?? CHUNK_TARGET;
    /** @type {Map<string, { id: number, record: Bytes }>} */
    this.schemas = new Map();
    /** @type {McapChannelInfo[]} */
    this.channels = [];
    /** @type {Bytes[]} */
    this.channelRecords = [];
    /** @type {Map<number, number>} */
    this.counts = new Map();
    /** @type {{ name: string, data: Record<string, string> }[]} */
    this.metadata = [];
    /** @type {Bytes[]} */
    this.chunkIndexes = [];
    this.messageCount = 0;
    /** @type {bigint | null} */
    this.start = null;
    this.end = 0n;
    this.lastTime = 0n;
    this.chunk = new Bytes(this.chunkSize + 4096);
    /** @type {Map<number, Bytes>} message index entries of the open chunk, per channel */
    this.chunkIndex = new Map();
    /** @type {bigint | null} */
    this.chunkStart = null;
    this.chunkEnd = 0n;
    this.finished = false;
  }

  resetChunk() {
    this.chunk = new Bytes(this.chunkSize + 4096);
    this.chunkIndex = new Map();
    this.chunkStart = null;
    this.chunkEnd = 0n;
  }

  /**
   * Register a channel (and its schema, once per type).
   * @param {string} topic
   * @param {string} type full name "pkg/msg/Type"
   * @param {string} schemaText ros2msg definition bundle
   * @param {Record<string, string>} [metadata] e.g. offered_qos_profiles
   * @returns {number} channel id
   */
  addChannel(topic, type, schemaText, metadata = {}) {
    if (this.messageCount > 0) throw new Error("add every channel before the first message");
    let schema = this.schemas.get(type);
    if (!schema) {
      const s = new Bytes();
      s.u16(this.schemas.size + 1);
      s.str(type);
      s.str("ros2msg");
      const data = utf8.encode(schemaText);
      s.u32(data.length);
      s.raw(data);
      schema = { id: this.schemas.size + 1, record: s };
      this.schemas.set(type, schema);
      this.out.record(OP.schema, s);
    }
    const id = this.channels.length;
    const c = new Bytes();
    c.u16(id);
    c.u16(schema.id);
    c.str(topic);
    c.str("cdr");
    c.strMap(metadata);
    this.out.record(OP.channel, c);
    this.channels.push({ id, schemaId: schema.id, topic, type, metadata });
    this.channelRecords.push(c);
    this.counts.set(id, 0);
    return id;
  }

  /**
   * @param {number} channel
   * @param {bigint} logTime ns
   * @param {Uint8Array} data CDR
   */
  addMessage(channel, logTime, data) {
    if (!this.counts.has(channel)) throw new Error(`unknown channel ${channel}`);
    if (logTime < this.lastTime) throw new Error("messages must be added in log-time order");
    this.lastTime = logTime;
    const seq = /** @type {number} */ (this.counts.get(channel));
    const m = new Bytes(data.length + 32);
    m.u16(channel);
    m.u32(seq);
    m.u64(logTime);
    m.u64(logTime);
    m.raw(data);
    let idx = this.chunkIndex.get(channel);
    if (!idx) {
      idx = new Bytes(256);
      this.chunkIndex.set(channel, idx);
    }
    idx.u64(logTime);
    idx.u64(this.chunk.length);
    this.chunk.record(OP.message, m);
    if (this.chunkStart === null) this.chunkStart = logTime;
    this.chunkEnd = logTime;
    if (this.start === null) this.start = logTime;
    this.end = logTime;
    this.counts.set(channel, seq + 1);
    this.messageCount++;
    if (this.chunk.length >= this.chunkSize) this.flushChunk();
  }

  /**
   * A Metadata record (name and string pairs), written after the messages.
   * @param {string} name
   * @param {Record<string, string>} data
   */
  addMetadata(name, data) {
    this.metadata.push({ name, data });
  }

  flushChunk() {
    if (this.chunkStart === null) return;
    const records = this.chunk.bytes();
    const chunkStartOffset = this.out.length;
    const c = new Bytes(records.length + 64);
    c.u64(this.chunkStart);
    c.u64(this.chunkEnd);
    c.u64(records.length);
    c.u32(crc32(records));
    c.str("");
    c.u64(records.length);
    c.raw(records);
    this.out.record(OP.chunk, c);
    const chunkLength = this.out.length - chunkStartOffset;
    const indexStart = this.out.length;
    /** @type {[number, number][]} */
    const offsets = [];
    for (const [ch, entries] of [...this.chunkIndex].sort((a, b) => a[0] - b[0])) {
      offsets.push([ch, this.out.length]);
      const mi = new Bytes(entries.length + 16);
      mi.u16(ch);
      mi.u32(entries.length);
      mi.raw(entries.bytes());
      this.out.record(OP.messageIndex, mi);
    }
    const ci = new Bytes(128);
    ci.u64(this.chunkStart);
    ci.u64(this.chunkEnd);
    ci.u64(chunkStartOffset);
    ci.u64(chunkLength);
    ci.u32(offsets.length * 10);
    for (const [ch, off] of offsets) {
      ci.u16(ch);
      ci.u64(off);
    }
    ci.u64(this.out.length - indexStart);
    ci.str("");
    ci.u64(records.length);
    ci.u64(records.length);
    this.chunkIndexes.push(ci);
    this.resetChunk();
  }

  /** @returns {Uint8Array} the whole file */
  finish() {
    if (this.finished) throw new Error("finish() was already called");
    this.finished = true;
    this.flushChunk();
    /** @type {Bytes[]} */
    const metadataIndexes = [];
    for (const m of this.metadata) {
      const offset = this.out.length;
      const r = new Bytes();
      r.str(m.name);
      r.strMap(m.data);
      this.out.record(OP.metadata, r);
      const mi = new Bytes();
      mi.u64(offset);
      mi.u64(this.out.length - offset);
      mi.str(m.name);
      metadataIndexes.push(mi);
    }
    const de = new Bytes();
    de.u32(crc32(this.out.bytes()));
    this.out.record(OP.dataEnd, de);

    const summaryStart = this.out.length;
    /** @type {[number, number, number][]} group opcode, start, length */
    const groups = [];
    /**
     * @param {number} op
     * @param {Bytes[]} records
     */
    const group = (op, records) => {
      if (records.length === 0) return;
      const start = this.out.length;
      for (const r of records) this.out.record(op, r);
      groups.push([op, start, this.out.length - start]);
    };
    group(
      OP.schema,
      [...this.schemas.values()].map((s) => s.record),
    );
    group(OP.channel, this.channelRecords);
    const st = new Bytes();
    st.u64(this.messageCount);
    st.u16(this.schemas.size);
    st.u32(this.channels.length);
    st.u32(0);
    st.u32(this.metadata.length);
    st.u32(this.chunkIndexes.length);
    st.u64(this.start ?? 0n);
    st.u64(this.end);
    st.u32(this.counts.size * 10);
    for (const [ch, n] of this.counts) {
      st.u16(ch);
      st.u64(n);
    }
    group(OP.statistics, [st]);
    group(OP.metadataIndex, metadataIndexes);
    group(OP.chunkIndex, this.chunkIndexes);
    const summaryOffsetStart = this.out.length;
    for (const [op, start, length] of groups) {
      const so = new Bytes();
      so.u8(op);
      so.u64(start);
      so.u64(length);
      this.out.record(OP.summaryOffset, so);
    }
    // Footer: the summary CRC covers the summary section through summary_offset_start.
    this.out.u8(OP.footer);
    this.out.u64(20);
    this.out.u64(summaryStart);
    this.out.u64(summaryOffsetStart);
    const crc = crc32(this.out.bytes().subarray(summaryStart));
    this.out.u32(crc);
    this.out.raw(MAGIC);
    return this.out.bytes().slice();
  }
}
