// Minimal ZIP reader for a robot description package dropped as a .zip.
// Reads the central directory (APPNOTE.TXT 6.3, sections 4.3.12 to 4.3.16)
// and supports the two methods every zip tool writes: 0 (stored) and 8
// (deflate, inflated with the platform's DecompressionStream
// "deflate-raw"). CRC-32 is verified. ZIP64, encryption and multi-disk
// archives are rejected with a clear message.

export class ZipError extends Error {}

/** @type {Uint32Array | null} */
let crcTable = null;

/** CRC-32 (IEEE 802.3, as zip uses). @param {Uint8Array} data */
export function crc32(data) {
  if (!crcTable) {
    crcTable = new Uint32Array(256);
    for (let n = 0; n < 256; n++) {
      let c = n;
      for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
      crcTable[n] = c >>> 0;
    }
  }
  let crc = 0xffffffff;
  for (let i = 0; i < data.length; i++) crc = crcTable[(crc ^ data[i]) & 0xff] ^ (crc >>> 8);
  return (crc ^ 0xffffffff) >>> 0;
}

/**
 * @typedef {object} ZipEntry
 * @property {string} name path inside the archive, forward slashes
 * @property {number} method 0 stored, 8 deflate
 * @property {number} compressedSize
 * @property {number} size uncompressed size
 * @property {number} crc
 * @property {number} offset of the local file header
 * @property {boolean} directory
 */

/**
 * List the entries of a zip archive.
 * @param {Uint8Array} buf
 * @returns {ZipEntry[]}
 */
export function listZip(buf) {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  // End of central directory: 22 bytes plus a comment of up to 65535 bytes.
  let eocd = -1;
  for (let i = buf.length - 22; i >= Math.max(0, buf.length - 22 - 65535); i--) {
    if (dv.getUint32(i, true) === 0x06054b50) {
      eocd = i;
      break;
    }
  }
  if (eocd < 0) throw new ZipError("not a zip file (no end-of-central-directory record)");
  const disk = dv.getUint16(eocd + 4, true);
  const count = dv.getUint16(eocd + 10, true);
  const cdSize = dv.getUint32(eocd + 12, true);
  const cdOffset = dv.getUint32(eocd + 16, true);
  if (disk !== 0) throw new ZipError("multi-disk zip archives are not supported");
  if (count === 0xffff || cdOffset === 0xffffffff || cdSize === 0xffffffff)
    throw new ZipError("ZIP64 archives are not supported; re-create the zip without ZIP64");
  if (cdOffset + cdSize > eocd) throw new ZipError("corrupt zip: central directory out of range");
  /** @type {ZipEntry[]} */
  const out = [];
  let p = cdOffset;
  const dec = new TextDecoder();
  for (let k = 0; k < count; k++) {
    if (p + 46 > buf.length || dv.getUint32(p, true) !== 0x02014b50)
      throw new ZipError(`corrupt zip: bad central directory entry ${k + 1}`);
    const flags = dv.getUint16(p + 8, true);
    const method = dv.getUint16(p + 10, true);
    const crc = dv.getUint32(p + 16, true);
    const compressedSize = dv.getUint32(p + 20, true);
    const size = dv.getUint32(p + 24, true);
    const nameLen = dv.getUint16(p + 28, true);
    const extraLen = dv.getUint16(p + 30, true);
    const commentLen = dv.getUint16(p + 32, true);
    const offset = dv.getUint32(p + 42, true);
    const name = dec.decode(buf.subarray(p + 46, p + 46 + nameLen)).replace(/\\/g, "/");
    if (flags & 1) throw new ZipError(`${name}: encrypted zip entries are not supported`);
    out.push({
      name,
      method,
      compressedSize,
      size,
      crc,
      offset,
      directory: name.endsWith("/"),
    });
    p += 46 + nameLen + extraLen + commentLen;
  }
  return out;
}

/**
 * Read one entry's bytes.
 * @param {Uint8Array} buf
 * @param {ZipEntry} e
 * @returns {Promise<Uint8Array>}
 */
export async function readZipEntry(buf, e) {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  if (dv.getUint32(e.offset, true) !== 0x04034b50)
    throw new ZipError(`corrupt zip: bad local header for ${e.name}`);
  const nameLen = dv.getUint16(e.offset + 26, true);
  const extraLen = dv.getUint16(e.offset + 28, true);
  const start = e.offset + 30 + nameLen + extraLen;
  if (start + e.compressedSize > buf.length)
    throw new ZipError(`corrupt zip: ${e.name} is truncated`);
  const raw = buf.subarray(start, start + e.compressedSize);
  let data;
  if (e.method === 0) data = raw.slice();
  else if (e.method === 8) data = await inflateRaw(raw);
  else
    throw new ZipError(
      `${e.name}: compression method ${e.method} is not supported (only stored and deflate)`,
    );
  if (data.length !== e.size) throw new ZipError(`${e.name}: size ${data.length} != ${e.size}`);
  if (crc32(data) !== e.crc) throw new ZipError(`${e.name}: CRC-32 mismatch, the zip is corrupt`);
  return data;
}

/**
 * Inflate raw deflate data with the Compression Streams API.
 * @param {Uint8Array} raw
 * @returns {Promise<Uint8Array>}
 */
export async function inflateRaw(raw) {
  const ds = new DecompressionStream("deflate-raw");
  const stream = new Blob([/** @type {BlobPart} */ (raw)]).stream().pipeThrough(ds);
  try {
    return new Uint8Array(await new Response(stream).arrayBuffer());
  } catch (err) {
    throw new ZipError(
      `invalid deflate data (${err instanceof Error ? err.message : String(err)})`,
    );
  }
}
