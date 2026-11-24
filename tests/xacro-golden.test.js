// Every corpus input must expand exactly like the official xacro (canonical
// comparison; see src/core/canonical.js), or fail where it fails.

import { test } from "node:test";
import assert from "node:assert/strict";
import { loadCases, runCase } from "./golden.js";

for (const c of loadCases()) {
  test(`golden: ${c.id}`, () => {
    const r = runCase(c);
    assert.ok(r.pass, `${c.id}: ${r.detail}`);
    // Successful expansions are byte-identical to xacro's own output, or
    // differ only in the last bits of numbers computed with sin/cos.
    if (r.expected === "urdf" && !c.unsupported)
      assert.ok(r.byteIdentical || r.maxUlps > 0, `${c.id} differs in layout only`);
  });
}

test("the corpus covers every feature fixture and at least two third-party descriptions", () => {
  const cases = loadCases();
  const third = new Set(
    cases
      .filter((c) => c.entry.startsWith("examples/third_party/"))
      .map((c) => c.entry.split("/")[2]),
  );
  assert.ok(third.size >= 2, [...third].join(", "));
  assert.ok(cases.filter((c) => c.id.startsWith("error-")).length >= 15);
});
