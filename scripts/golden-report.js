// Print the golden comparison as a Markdown table (used for docs/xacro.md).
import { loadCases, runCase } from "../tests/golden.js";

const rows = loadCases().map(runCase);
console.log("| case | official xacro | Robot Studio | result |");
console.log("|---|---|---|---|");
for (const r of rows) {
  const result = r.case.unsupported
    ? "known gap: rejected with a clear error"
    : r.expected === "error"
      ? r.pass
        ? "both fail"
        : "MISMATCH"
      : r.pass
        ? r.byteIdentical
          ? `identical (${r.elements} elements, byte for byte)`
          : `canonically equal (${r.elements} elements)`
        : "MISMATCH";
  console.log(`| ${r.case.id} | ${r.expected} | ${r.actual} | ${result} |`);
}
const pass = rows.filter((r) => r.pass).length;
console.log(`\n${pass} of ${rows.length} cases pass.`);
process.exitCode = pass === rows.length ? 0 : 1;
