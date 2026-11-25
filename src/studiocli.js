// `robot-studio studio`: the one-command local launch. It serves the page
// from this computer (http://127.0.0.1:PORT/) and opens the browser on it,
// with the rosbridge address filled in when --ros is given. A page served
// from this computer can open ws:// to a robot on the local network: it is
// not a public https page, so neither mixed-content blocking nor the
// browser's local network access prompt applies (docs/ros2.md).
//
//   npx github:projectbin07-stack/robot-studio studio --ros ws://192.168.1.20:9090
//   node bin/robot-studio.js studio [--port 8000] [--ros URL] [--no-open]

import { spawn } from "node:child_process";
import { parseArgs } from "node:util";
import { checkRosbridgeUrl } from "./core/ros/url.js";
import { startServer } from "./node/serve.js";

export const STUDIO_HELP = `robot-studio studio: serve Robot Studio from this computer and open it

Usage:
  robot-studio studio [--ros ws://robot:9090] [--port 8000] [--no-open]

  --ros URL     fill in the rosbridge address (the page still connects only
                when you click Connect)
  --port N      first port to try (default 8000; the next 20 are tried if taken)
  --no-open     print the address instead of opening the browser

The server listens on 127.0.0.1 only and serves the page's own files. Stop it
with Ctrl+C.
`;

/**
 * The address to open: the page, with ?ros= when given.
 * @param {string} base
 * @param {string | undefined} ros
 * @returns {{ ok: true, url: string } | { ok: false, reason: string }}
 */
export function studioUrl(base, ros) {
  if (ros === undefined) return { ok: true, url: base };
  const c = checkRosbridgeUrl(ros, "http:");
  if (!c.ok) return { ok: false, reason: `--ros: ${c.reason}` };
  return { ok: true, url: `${base}?ros=${encodeURIComponent(c.url)}` };
}

/**
 * Open a URL in the default browser (detached).
 * @param {string} url
 */
export function openBrowser(url) {
  const [cmd, args] =
    process.platform === "win32"
      ? ["rundll32", ["url.dll,FileProtocolHandler", url]]
      : process.platform === "darwin"
        ? ["open", [url]]
        : ["xdg-open", [url]];
  const p = spawn(cmd, args, { detached: true, stdio: "ignore" });
  p.on("error", () => {});
  p.unref();
}

/**
 * @param {string[]} argv
 * @param {{ out: (t: string) => void, err: (t: string) => void }} io
 * @param {{ open?: (url: string) => void, root?: string }} [deps] for tests
 * @returns {Promise<number | { url: string, close: () => Promise<void> }>} an exit code, or
 *   the running server (the process stays up until Ctrl+C)
 */
export async function studioCommand(argv, io, deps = {}) {
  const { values } = parseArgs({
    args: argv,
    options: {
      ros: { type: "string" },
      port: { type: "string", default: "8000" },
      "no-open": { type: "boolean", default: false },
      help: { type: "boolean", short: "h" },
    },
  });
  if (values.help) {
    io.out(STUDIO_HELP);
    return 0;
  }
  const port = Number(values.port);
  if (!Number.isInteger(port) || port < 0 || port > 65535) {
    io.err("robot-studio studio: --port must be 0 to 65535\n");
    return 3;
  }
  const check = studioUrl("http://127.0.0.1/", values.ros);
  if (!check.ok) {
    io.err(`robot-studio studio: ${check.reason}\n`);
    return 3;
  }
  const { server, url } = await startServer({ port, tries: port === 0 ? 1 : 20, root: deps.root });
  const page = /** @type {{ ok: true, url: string }} */ (studioUrl(url, values.ros)).url;
  io.out(`Robot Studio is served at ${page}\n`);
  io.out(
    "Only this computer can reach it (127.0.0.1). Connect to the robot from the ROS 2 panel; Ctrl+C stops the server.\n",
  );
  if (!values["no-open"]) (deps.open ?? openBrowser)(page);
  return {
    url: page,
    close: () => new Promise((done) => server.close(() => done())),
  };
}
