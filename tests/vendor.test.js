// The vendored copies match vendor/manifest.json byte for byte, and the
// checker notices a modified, missing or extra file. (CI's vendor job also
// re-fetches each upstream version; see scripts/vendor.mjs --verify.)

import { test } from "node:test";
import assert from "node:assert/strict";
import { cpSync, mkdtempSync, readFileSync, rmSync, writeFileSync, unlinkSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { gzipSync } from "node:zlib";
import {
  SOURCES,
  compareWithUpstream,
  readTarGz,
  sha256,
  verifyAgainstManifest,
} from "../scripts/vendor-lib.js";

const root = fileURLToPath(new URL("..", import.meta.url));
/** @type {import("../scripts/vendor-lib.js").Manifest} */
const manifest = JSON.parse(readFileSync(join(root, "vendor/manifest.json"), "utf8"));

test("the vendored files match the manifest and the pinned sources", () => {
  assert.deepEqual(verifyAgainstManifest(manifest, root), []);
  const ui = manifest.sources.find((s) => s.name === "urdf-inspector");
  assert.equal(ui?.tag, SOURCES[0].kind === "git" ? SOURCES[0].tag : "");
  assert.match(ui?.commit ?? "", /^[0-9a-f]{40}$/);
  const lsl = manifest.sources.find((s) => s.name === "lidar-slam-lab");
  assert.equal(lsl?.tag, SOURCES[1].kind === "git" ? SOURCES[1].tag : "");
  assert.match(lsl?.commit ?? "", /^[0-9a-f]{40}$/);
  assert.ok(lsl?.files.some((f) => f.path === "src/scanmatch.js"));
  const three = manifest.sources.find((s) => s.name === "three");
  assert.equal(three?.version, "0.187.0");
  assert.match(three?.integrity ?? "", /^sha512-/);
  for (const s of manifest.sources)
    assert.ok(
      s.files.some((f) => f.path === "LICENSE"),
      `${s.name} keeps its licence`,
    );
  assert.deepEqual(
    SOURCES.map((s) => s.files.length),
    manifest.sources.map((s) => s.files.length),
  );
});

test("a modified, missing or extra vendored file is reported", () => {
  const tmp = mkdtempSync(join(process.env.TMP ?? tmpdir(), "vendor-test-"));
  try {
    cpSync(join(root, "vendor"), join(tmp, "vendor"), { recursive: true });
    writeFileSync(join(tmp, "vendor/urdf-inspector/src/core/fk.js"), "// edited\n", { flag: "a" });
    unlinkSync(join(tmp, "vendor/three/LICENSE"));
    writeFileSync(join(tmp, "vendor/three/extra.js"), "x");
    const problems = verifyAgainstManifest(manifest, tmp);
    assert.equal(problems.length, 3, problems.join("\n"));
    assert.match(problems.join("\n"), /fk\.js: sha256 .* != manifest/);
    assert.match(problems.join("\n"), /three\/LICENSE: missing/);
    assert.match(problems.join("\n"), /extra\.js: not in the manifest/);
  } finally {
    rmSync(tmp, { recursive: true, force: true });
  }
});

test("upstream comparison and the tar reader", () => {
  const entry = /** @type {import("../scripts/vendor-lib.js").ManifestEntry} */ (
    manifest.sources[0]
  );
  /** @type {Map<string, Uint8Array>} */
  const upstream = new Map(
    entry.files.map((f) => [f.path, readFileSync(join(root, entry.dest, f.path))]),
  );
  assert.deepEqual(compareWithUpstream(entry, upstream, root), []);
  upstream.set("src/core/xml.js", new TextEncoder().encode("changed upstream"));
  upstream.delete("LICENSE");
  const problems = compareWithUpstream(entry, upstream, root);
  assert.equal(problems.length, 3, problems.join("\n"));

  // A two-file ustar archive, the second with a name prefix.
  const block = (/** @type {string} */ name, /** @type {string} */ body, prefix = "") => {
    const h = Buffer.alloc(512);
    h.write(name, 0);
    h.write(body.length.toString(8).padStart(11, "0"), 124);
    h.write("0", 156);
    h.write("ustar", 257);
    h.write(prefix, 345);
    const data = Buffer.alloc(Math.ceil(body.length / 512) * 512);
    data.write(body);
    return Buffer.concat([h, data]);
  };
  const tgz = gzipSync(
    Buffer.concat([
      block("package/a.txt", "hello"),
      block("b.txt", "x".repeat(600), "package/deep"),
      Buffer.alloc(1024),
    ]),
  );
  const files = readTarGz(tgz);
  assert.equal(new TextDecoder().decode(files.get("package/a.txt")), "hello");
  assert.equal(files.get("package/deep/b.txt")?.length, 600);
  assert.equal(sha256("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
});
