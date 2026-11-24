#!/usr/bin/env node
import { readFileSync, writeFileSync } from "node:fs";
import { main } from "../src/cli.js";

process.exitCode = await main(process.argv.slice(2), {
  out: (text) => process.stdout.write(text),
  err: (text) => process.stderr.write(text),
  writeFile: (path, data) => writeFileSync(path, data),
  exists: (path) => {
    try {
      readFileSync(path);
      return true;
    } catch {
      return false;
    }
  },
});
