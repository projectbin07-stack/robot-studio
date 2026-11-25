// The rosbridge address the user types (or a link pre-fills; it is never
// connected to without a click). Only ws: and wss: URLs, no credentials,
// a host, a sane port; "robot.local" or "192.168.1.20:9090" are completed to
// ws://...:9090 (rosbridge_server's default port).

export const DEFAULT_PORT = 9090;
const MAX_LENGTH = 512;

/**
 * @typedef {{ ok: true, url: string, host: string, warnings: string[] } | { ok: false, reason: string }} UrlCheck
 */

/** The one-command local launch (docs/ros2.md): serves this page from 127.0.0.1. */
export const LOCAL_LAUNCH = "npx github:projectbin07-stack/robot-studio studio";

/** @param {string} host */
export const isLoopback = (host) =>
  host === "localhost" ||
  host.endsWith(".localhost") ||
  /^127(?:\.\d{1,3}){3}$/.test(host) ||
  host === "[::1]";

/**
 * A host a browser recognises as on the local network before resolving it:
 * a private or link-local IPv4 / IPv6 literal, or a .local (mDNS) name.
 * Chromium's Local Network Access exempts ws:// to these (and loopback) from
 * mixed-content blocking and asks the user instead; a DNS name that resolves
 * to a LAN address is not exempt.
 * @param {string} host URL hostname (IPv6 in brackets)
 */
export function isLocalNetworkHost(host) {
  const h = host.toLowerCase();
  if (h.endsWith(".local")) return true;
  const m = /^(\d{1,3})\.(\d{1,3})\.\d{1,3}\.\d{1,3}$/.exec(h);
  if (m) {
    const a = Number(m[1]);
    const b = Number(m[2]);
    return (
      a === 10 ||
      (a === 172 && b >= 16 && b <= 31) ||
      (a === 192 && b === 168) ||
      (a === 169 && b === 254)
    );
  }
  return /^\[(f[cd][0-9a-f]{0,2}|fe[89ab][0-9a-f]?):/.test(h);
}

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
  // What browsers do with a public https page opening ws:// (docs/ros2.md,
  // "Reaching a robot from the live site"): Chrome and Edge ask for local
  // network access for loopback, private-address and .local targets (tested
  // with Chrome 157 and Edge 158); other names are mixed content.
  if (
    pageProtocol === "https:" &&
    u.protocol === "ws:" &&
    (isLoopback(u.hostname) || isLocalNetworkHost(u.hostname))
  )
    warnings.push(
      `the browser will ask to allow access to devices on your local network: allow it (Chrome and Edge 147 or later; Firefox allows only this computer, Safari none). If it is blocked, start the studio on your computer with ${LOCAL_LAUNCH} and connect from there (docs/ros2.md)`,
    );
  else if (pageProtocol === "https:" && u.protocol === "ws:")
    warnings.push(
      `this page is served over https, so the browser blocks unencrypted ws:// to a host name (mixed content); use the robot's IP address, wss://, or start the studio on your computer with ${LOCAL_LAUNCH} (docs/ros2.md)`,
    );
  return { ok: true, url: u.toString(), host: u.host, warnings };
}
