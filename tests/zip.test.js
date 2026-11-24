// The zip reader: an archive written by Python's zipfile, archives built
// here with stored and deflated entries, and every rejection path.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { deflateRawSync } from "node:zlib";
import { ZipError, crc32, inflateRaw, listZip, readZipEntry } from "../src/core/zip.js";
import { VirtualFs, addZip } from "../src/core/vfs.js";
import { loadModel } from "../src/core/model.js";

/**
 * Build a zip with a minimal writer (local headers + central directory).
 * @param {{ name: string, data: Uint8Array, method?: 0 | 8, flags?: number, crc?: number }[]} files
 * @param {{ comment?: string, zip64?: boolean, disk?: number }} [opts]
 */
function makeZip(files, opts = {}) {
  /** @type {Buffer[]} */
  const parts = [];
  /** @type {Buffer[]} */
  const central = [];
  let offset = 0;
  for (const f of files) {
    const method = f.method ?? 8;
    const body = method === 8 ? deflateRawSync(f.data) : Buffer.from(f.data);
    const name = Buffer.from(f.name);
    const crc = f.crc ?? crc32(f.data);
    const local = Buffer.alloc(30);
    local.writeUInt32LE(0x04034b50, 0);
    local.writeUInt16LE(20, 4);
    local.writeUInt16LE(f.flags ?? 0, 6);
    local.writeUInt16LE(method, 8);
    local.writeUInt32LE(crc, 14);
    local.writeUInt32LE(body.length, 18);
    local.writeUInt32LE(f.data.length, 22);
    local.writeUInt16LE(name.length, 26);
    const cd = Buffer.alloc(46);
    cd.writeUInt32LE(0x02014b50, 0);
    cd.writeUInt16LE(f.flags ?? 0, 8);
    cd.writeUInt16LE(method, 10);
    cd.writeUInt32LE(crc, 16);
    cd.writeUInt32LE(body.length, 20);
    cd.writeUInt32LE(f.data.length, 24);
    cd.writeUInt16LE(name.length, 28);
    cd.writeUInt32LE(offset, 42);
    central.push(cd, name);
    parts.push(local, name, body);
    offset += 30 + name.length + body.length;
  }
  const cdBuf = Buffer.concat(central);
  const comment = Buffer.from(opts.comment ?? "");
  const end = Buffer.alloc(22);
  end.writeUInt32LE(0x06054b50, 0);
  end.writeUInt16LE(opts.disk ?? 0, 4);
  end.writeUInt16LE(opts.zip64 ? 0xffff : files.length, 8);
  end.writeUInt16LE(opts.zip64 ? 0xffff : files.length, 10);
  end.writeUInt32LE(cdBuf.length, 12);
  end.writeUInt32LE(offset, 16);
  end.writeUInt16LE(comment.length, 20);
  return new Uint8Array(Buffer.concat([...parts, cdBuf, end, comment]));
}

const enc = (/** @type {string} */ s) => new TextEncoder().encode(s);
const dec = (/** @type {Uint8Array} */ b) => new TextDecoder().decode(b);

test("CRC-32 matches the standard check value", () => {
  assert.equal(crc32(enc("123456789")), 0xcbf43926);
  assert.equal(crc32(new Uint8Array(0)), 0);
});

test("reads an archive written by Python's zipfile (stored, deflate, directory)", async () => {
  const buf = new Uint8Array(readFileSync(new URL("fixtures/description.zip", import.meta.url)));
  const entries = listZip(buf);
  assert.deepEqual(
    entries.map((e) => [e.name, e.method, e.directory]),
    [
      ["zip_demo/", 0, true],
      ["zip_demo/package.xml", 8, false],
      ["zip_demo/urdf/robot.urdf.xacro", 8, false],
      ["zip_demo/meshes/cube.stl", 0, false],
    ],
  );
  const pkg = dec(await readZipEntry(buf, entries[1]));
  assert.match(pkg, /<name>zip_demo<\/name>/);
  const stl = dec(await readZipEntry(buf, entries[3]));
  assert.match(stl, /^solid cube/);
});

test("a dropped zip becomes a file system: package found, xacro expanded, mesh resolved", async () => {
  const buf = new Uint8Array(readFileSync(new URL("fixtures/description.zip", import.meta.url)));
  const fs = new VirtualFs();
  assert.equal(await addZip(fs, buf), 3);
  assert.equal(fs.findPackage("zip_demo"), "/zip_demo");
  assert.deepEqual(fs.candidates(), ["/zip_demo/urdf/robot.urdf.xacro"]);
  const m = loadModel(fs, "/zip_demo/urdf/robot.urdf.xacro");
  assert.equal(m.error, null);
  assert.equal(m.meshes[0].resolved, "/zip_demo/meshes/cube.stl");
  const bytes = await fs.readBinary("/zip_demo/meshes/cube.stl");
  assert.match(dec(/** @type {Uint8Array} */ (bytes)), /endsolid/);
});

test("archives built here: comment, both methods, non-ASCII names", async () => {
  const data = enc("x".repeat(1000));
  const buf = makeZip(
    [
      { name: "a/stored.txt", data: enc("plain"), method: 0 },
      { name: "a/über.urdf", data },
    ],
    { comment: "made by a test" },
  );
  const [a, b] = listZip(buf);
  assert.equal(dec(await readZipEntry(buf, a)), "plain");
  assert.equal(b.name, "a/über.urdf");
  assert.ok(b.compressedSize < 100);
  assert.equal((await readZipEntry(buf, b)).length, 1000);
});

test("rejects what it cannot read, with a reason", async () => {
  assert.throws(() => listZip(enc("not a zip at all, just text")), /not a zip file/);
  assert.throws(() => listZip(makeZip([{ name: "a", data: enc("x") }], { zip64: true })), /ZIP64/);
  assert.throws(() => listZip(makeZip([{ name: "a", data: enc("x") }], { disk: 1 })), /multi-disk/);
  assert.throws(() => listZip(makeZip([{ name: "a", data: enc("x"), flags: 1 }])), /encrypted/);
  const badCrc = makeZip([{ name: "a", data: enc("x"), crc: 1 }]);
  await assert.rejects(readZipEntry(badCrc, listZip(badCrc)[0]), /CRC-32 mismatch/);
  const lzma = makeZip([{ name: "a", data: enc("x"), method: 0 }]);
  const e = { ...listZip(lzma)[0], method: 14 };
  await assert.rejects(readZipEntry(lzma, e), /method 14 is not supported/);
  await assert.rejects(readZipEntry(lzma, { ...listZip(lzma)[0], size: 5 }), /size 1 != 5/);
  await assert.rejects(readZipEntry(lzma, { ...listZip(lzma)[0], offset: 3 }), /bad local header/);
  await assert.rejects(
    readZipEntry(lzma, { ...listZip(lzma)[0], compressedSize: 9999 }),
    /truncated/,
  );
  const broken = makeZip([{ name: "a", data: enc("x") }]);
  const cdStart = broken.length - 22 - 47;
  broken[cdStart] = 0;
  assert.throws(() => listZip(broken), /bad central directory entry/);
  const outOfRange = makeZip([{ name: "a", data: enc("x") }]);
  new DataView(outOfRange.buffer).setUint32(outOfRange.length - 6, 999999, true);
  assert.throws(() => listZip(outOfRange), /out of range/);
  await assert.rejects(inflateRaw(new Uint8Array([0xff, 0xff, 0xff])), ZipError);
});
