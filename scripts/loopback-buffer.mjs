#!/usr/bin/env node
// How much data a loopback TCP connection holds when the reader does not
// keep up, before Node's own write queue (socket.writableLength) grows past
// 64 KiB, the point at which scripts/fake-rosbridge.mjs starts dropping
// replayed data for a slow client. Everything below that point sits in the
// operating system's socket buffers, ahead of any later message (a service
// reply in the live-view test). CI prints it on Linux and Windows; see the
// HANDOFF note on the Linux-only live-view test.
import net from "node:net";

const chunk = Buffer.alloc(16384, 120);
const server = net.createServer((s) => {
  let written = 0;
  const t = setInterval(() => {
    for (let i = 0; i < 16; i++) {
      s.write(chunk);
      written += chunk.length;
    }
    if (s.writableLength > 65536 || written > 256 * 1024 * 1024) {
      const kib = Math.round((written - s.writableLength) / 1024);
      console.log(
        `${process.platform}: ${kib} KiB held by the OS on a loopback connection whose reader is paused, before Node queued more than 64 KiB`,
      );
      clearInterval(t);
      s.destroy();
      server.close();
      client.destroy();
    }
  }, 20);
});
/** @type {net.Socket} */
let client;
server.listen(0, "127.0.0.1", () => {
  const addr = /** @type {net.AddressInfo} */ (server.address());
  client = net.connect(addr.port, "127.0.0.1");
  client.pause();
});
