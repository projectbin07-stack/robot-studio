// The rosbridge address the user types (or a link pre-fills; it is never
// connected to without a click). Only ws: and wss: URLs, no credentials,
// a host, a sane port; "robot.local" or "192.168.1.20:9090" are completed to
// ws://...:9090 (rosbridge_server's default port).

export const DEFAULT_PORT = 9090;
const MAX_LENGTH = 512;

/**
 * @typedef {{ ok: true, url: string, host: string, warnings: string[] } | { ok: false, reason: string }} UrlCheck
 */

/** @param {string} host */
export const isLoopback = (host) =>
  host === "localhost" ||
  host.endsWith(".localhost") ||
  /^127(?:\.\d{1,3}){3}$/.test(host) ||
  host === "[::1]";

/**
 * @param {string} input
 * @param {string} [pageProtocol] the page's location.protocol ("https:" warns about ws:)
 * @returns {UrlCheck}
 */
export function checkRosbridgeUrl(input, pageProtocol = "") {
  const text = input.trim();
  if (!text)
    return { ok: false, reason: "enter the rosbridge address, e.g. ws://192.168.1.20:9090" };
  if (text.length > MAX_LENGTH)
    return { ok: false, reason: `longer than ${MAX_LENGTH} characters` };
  // A control character or space inside is never part of a host name.
  if (/[\s\u0000-\u001f\u007f]/.test(text))
    return { ok: false, reason: "contains spaces or control characters" };
  const hasScheme = /^[a-zA-Z][a-zA-Z0-9+.-]*:\/\//.test(text);
  let u;
  try {
    u = new URL(hasScheme ? text : `ws://${text}`);
  } catch {
    return { ok: false, reason: "not a valid address" };
  }
  if (u.protocol !== "ws:" && u.protocol !== "wss:")
    return { ok: false, reason: `only ws:// and wss:// addresses (got ${u.protocol}//)` };
  if (u.username || u.password)
    return { ok: false, reason: "no user name or password in the address" };
  if (!u.hostname) return { ok: false, reason: "no host" };
  if (u.hash) return { ok: false, reason: "no #fragment in the address" };
  if (!hasScheme && !u.port) u.port = String(DEFAULT_PORT);
  /** @type {string[]} */
  const warnings = [];
  if (pageProtocol === "https:" && u.protocol === "ws:" && !isLoopback(u.hostname))
    warnings.push(
      "this page is served over https, so the browser blocks unencrypted ws:// connections to other machines (mixed content); open the studio from a local server (npm run serve) or use wss:// (docs/ros2.md)",
    );
  return { ok: true, url: u.toString(), host: u.host, warnings };
}
