#!/usr/bin/env node
// Whoop Pilot's local server, one process for everything on this Mac: the web app, the goggles video
// (goggles-helper.mjs), the flight recorder (recorder.mjs) and read-only access to the user's SiteSpec and
// Spacial captures (house-files.mjs).
//
// It answers only http://localhost:8790. The app's settings, API key, houses and calibration live in that
// origin's storage (localStorage, OPFS), so it never moves to another port: if 8790 is taken by another
// program it says so and quits.
//
// Same origin only: no CORS headers at all; requests from other sites (Origin, Referer, Sec-Fetch-Site) and
// for other host names (DNS rebinding) get 403. Every POST and every video stream needs the per-launch
// token, which GET /api/session hands only to this origin's own scripts.
//
//   node whoop.mjs [--wifi] [--peer IP[:port]] [--local-port N] [--no-open]
//
// Routes: /app/... and the rest of the project (static), /whoop-pilot.json (marker), /api/session,
// /goggles/status, /goggles/stream (GET, token), /rec/start|stop|telemetry|command (POST JSON, token), /rec/status,
// /rec/list, /rec/<id>/<file> (token), /house-files/ (projects), /house-files/<project>/<path> (file, or a
// JSON listing for a folder).
import http from "node:http";
import net from "node:net";
import fs from "node:fs";
import fsp from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { createHash, randomBytes, timingSafeEqual } from "node:crypto";
import { execFile } from "node:child_process";
import { fileURLToPath } from "node:url";
import { GogglesHelper, PORT as GOGGLES_PORT } from "./goggles-helper.mjs";
import { Recorder, httpError } from "./recorder.mjs";
import { HouseFiles, HOUSE_ROOTS } from "./house-files.mjs";

export const PORT = Number(process.env.WHOOP_TEST_PORT) || 8790; // WHOOP_TEST_PORT: tests only
const ROOT = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const DATA = process.env.WHOOP_TEST_DATA || (process.env.WHOOP_TEST_PORT ? path.join(os.tmpdir(), "whoop-pilot-test") : path.join(os.homedir(), "Library", "Application Support", "WhoopPilot"));
const API = /^\/(api|goggles|rec|house-files)(\/|$)/;
const LIMIT = { json: 1 << 20, lines: 32 << 20 };
const MIME = {
  ".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8", ".json": "application/json", ".map": "application/json", ".jsonl": "application/x-ndjson",
  ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".gif": "image/gif", ".webp": "image/webp", ".ico": "image/x-icon",
  ".wasm": "application/wasm", ".h264": "video/h264", ".mp4": "video/mp4", ".webm": "video/webm",
  ".glb": "model/gltf-binary", ".gltf": "model/gltf+json", ".woff2": "font/woff2", ".woff": "font/woff", ".ttf": "font/ttf",
  ".txt": "text/plain; charset=utf-8", ".md": "text/markdown; charset=utf-8", ".lua": "text/plain; charset=utf-8", ".csv": "text/csv; charset=utf-8",
  ".pdf": "application/pdf",
};
const log = (...a) => console.log(new Date().toISOString().slice(11, 23), ...a);
const okSeg = (s) => s && !s.startsWith(".") && !/[/\\\0]/.test(s) && s !== "node_modules";
const inside = (dir, p) => p === dir || p.startsWith(dir + path.sep);
// A top-level page load. Once the app's service worker (sw.js: fetch(e.request)) controls the page, Chrome
// sends its navigations with Sec-Fetch-Mode navigate but Sec-Fetch-Dest empty. Page scripts can't send
// either form; frames (iframe, object, embed) have their own destinations.
const navigation = (req) => req.method === "GET" && (req.headers["sec-fetch-dest"] === "document" || (req.headers["sec-fetch-mode"] === "navigate" && req.headers["sec-fetch-dest"] === "empty"));
const POSTS = { "/rec/start": "start", "/rec/stop": "stop", "/rec/telemetry": "telemetry", "/rec/command": "commands" };
const decode = (segs) => {
  try {
    return segs.map(decodeURIComponent);
  } catch {
    return null;
  }
};

// Identifies the app code that's being served (sizes and times of app/), for recordings.
function appBuild() {
  const h = createHash("sha256");
  const walk = (dir) => {
    for (const e of fs.readdirSync(dir, { withFileTypes: true }).sort((a, b) => a.name.localeCompare(b.name))) {
      if (e.name.startsWith(".")) continue;
      const p = path.join(dir, e.name);
      if (e.isDirectory()) walk(p);
      else if (e.isFile()) {
        const st = fs.statSync(p);
        h.update(`${path.relative(ROOT, p)}:${st.size}:${Math.round(st.mtimeMs)}\n`);
      }
    }
  };
  walk(path.join(ROOT, "app"));
  return h.digest("hex").slice(0, 12);
}

// Whether something already accepts connections there. Binding alone can't tell: Node sets SO_REUSEADDR,
// with which macOS lets 127.0.0.1:port bind on top of another program's *:port and take loopback from it.
const answers = (host, port) =>
  new Promise((resolve) => {
    const s = net.connect({ host, port, timeout: 1000 }, () => (s.destroy(), resolve(true)));
    s.on("error", () => resolve(false)).on("timeout", () => (s.destroy(), resolve(false)));
  });

function json(res, status, obj) {
  const b = Buffer.from(JSON.stringify(obj));
  res.writeHead(status, { "Content-Type": "application/json", "Content-Length": b.length });
  res.end(b);
}

async function readJson(req, limit) {
  const chunks = [];
  let n = 0;
  for await (const c of req) {
    if ((n += c.length) > limit) throw httpError(413, "Request too large.");
    chunks.push(c);
  }
  const s = Buffer.concat(chunks).toString("utf8").trim();
  try {
    return s ? JSON.parse(s) : {};
  } catch {
    throw httpError(400, "Expected JSON.");
  }
}

// One file, with HEAD and single byte ranges (a 76 MB splat, video replay).
function sendFile(req, res, file, st) {
  const size = st.size;
  let start = 0;
  let end = size - 1;
  const range = /^bytes=(\d*)-(\d*)$/.exec(req.headers.range || "");
  if (range && (range[1] || range[2])) {
    start = range[1] ? Number(range[1]) : Math.max(0, size - Number(range[2]));
    end = range[1] && range[2] ? Math.min(Number(range[2]), size - 1) : size - 1;
    if (start > end || start >= size) {
      res.writeHead(416, { "Content-Range": `bytes */${size}` });
      return res.end();
    }
  }
  const headers = { "Content-Type": MIME[path.extname(file).toLowerCase()] || "application/octet-stream", "Content-Length": end - start + 1, "Accept-Ranges": "bytes" };
  if (range) headers["Content-Range"] = `bytes ${start}-${end}/${size}`;
  res.writeHead(range ? 206 : 200, headers);
  if (req.method === "HEAD" || !size) return res.end();
  fs.createReadStream(file, { start, end }).on("error", () => res.destroy()).pipe(res);
}

export async function startWhoop({ port = PORT, root = ROOT, dataDir = DATA, houseRoots = HOUSE_ROOTS, goggles = {}, ipv6 = true, log: say = log } = {}) {
  const token = randomBytes(24).toString("hex");
  const tokenBuf = Buffer.from(token);
  const tokenOk = (t) => typeof t === "string" && Buffer.byteLength(t) === tokenBuf.length && timingSafeEqual(Buffer.from(t), tokenBuf);
  const rootReal = fs.realpathSync(root);
  const helper = new GogglesHelper({ ...goggles, log: say });
  const houses = new HouseFiles(houseRoots);
  let origins = new Set();
  let hosts = new Set();
  const recorder = new Recorder({ dir: path.join(dataDir, "recordings"), log: say, context: () => ({ server: { build: appBuild(), port }, goggles: helper.status() }) });
  helper.on("unit", (u) => recorder.unit(u));
  let lastDeny = 0;

  const originOf = (u) => {
    try {
      return new URL(u).origin;
    } catch {
      return "invalid";
    }
  };

  // null when the request may proceed, else why not. Top-level navigations from elsewhere may open the
  // app's pages (they run in this origin then); everything else must come from this origin.
  function gate(req, pathname) {
    const h = req.headers;
    if (!hosts.has(h.host)) return "host";
    if (navigation(req) && !API.test(pathname)) return null;
    if (h.origin !== undefined && !origins.has(h.origin)) return "origin";
    if (h.referer && !origins.has(originOf(h.referer))) return "referer";
    const site = h["sec-fetch-site"];
    if (site && site !== "same-origin" && site !== "none") return "site";
    return null;
  }

  function deny(req, res, why) {
    if (Date.now() - lastDeny > 5000) say(`Refused ${req.method} ${req.url.split("?")[0]} (${why}${req.headers.origin ? ` from ${req.headers.origin}` : ""}).`);
    lastDeny = Date.now();
    json(res, 403, { error: why === "token" ? "Missing or stale session token: reload the app." : `Whoop Pilot only answers its own page at http://localhost:${port}.`, why });
  }

  async function staticFile(req, res, pathname, query) {
    const segs = decode(pathname.split("/").slice(1));
    const dirWanted = segs?.at(-1) === "";
    if (!segs || !segs.slice(0, dirWanted ? -1 : undefined).every(okSeg)) return json(res, 404, { error: "Not found." });
    const p = await fsp.realpath(path.join(rootReal, ...segs)).catch(() => null);
    const st = p && inside(rootReal, p) && (await fsp.stat(p).catch(() => null));
    if (!st) return json(res, 404, { error: "Not found." });
    if (st.isDirectory()) {
      if (!dirWanted) {
        res.writeHead(301, { Location: pathname + "/" + query });
        return res.end();
      }
      const index = path.join(p, "index.html");
      const ist = await fsp.stat(index).catch(() => null);
      return ist?.isFile() ? sendFile(req, res, index, ist) : json(res, 404, { error: "Not found." });
    }
    sendFile(req, res, p, st);
  }

  async function route(req, res) {
    const q = req.url.indexOf("?");
    const pathname = q < 0 ? req.url : req.url.slice(0, q);
    const query = q < 0 ? "" : req.url.slice(q);
    const params = new URLSearchParams(query);
    for (const [k, v] of Object.entries({
      "Cache-Control": "no-store", "X-Content-Type-Options": "nosniff", "Cross-Origin-Resource-Policy": "same-origin",
      "Cross-Origin-Opener-Policy": "same-origin", "Cross-Origin-Embedder-Policy": "credentialless", // cross-origin isolated: WASM threads for the vision and localization workers
      "X-Frame-Options": "DENY", "Content-Security-Policy": "frame-ancestors 'none'",
    })) res.setHeader(k, v);
    const why = gate(req, pathname);
    if (why) return deny(req, res, why);
    const h = req.headers;
    if (navigation(req) && h.host.startsWith("127.0.0.1:")) {
      res.writeHead(302, { Location: `http://localhost:${port}${req.url}` }); // the app's data lives in the localhost origin
      return res.end();
    }
    let m;
    if (pathname === "/whoop-pilot.json") return json(res, 200, { app: "whoop-pilot", server: "whoop", goggles: helper.transport });
    if (pathname === "/" || pathname === "/index.html") {
      res.writeHead(302, { Location: "/app/" });
      return res.end();
    }
    if (pathname === "/api/session") {
      const site = h["sec-fetch-site"];
      if (!(site === "same-origin" || (!site && origins.has(h.origin)))) return deny(req, res, "session");
      return json(res, 200, { app: "whoop-pilot", token, build: appBuild(), goggles: helper.transport, recording: recorder.recording, features: ["goggles", "recorder", "house-files"] });
    }
    if ((m = /^\/goggles\/(?:([0-9a-f]+)\/)?stream$/.exec(pathname))) {
      if (!tokenOk(m[1] ?? h["x-whoop-token"] ?? params.get("token"))) return deny(req, res, "token");
      if (req.method !== "GET") {
        res.setHeader("Allow", "GET");
        return json(res, 405, { error: "Method not allowed." });
      }
      return helper.attach(req, res);
    }
    if (req.method === "POST") {
      if (!tokenOk(h["x-whoop-token"])) return deny(req, res, "token");
      const kind = POSTS[pathname];
      if (!kind) return json(res, 404, { error: "No such endpoint." });
      const lines = kind === "telemetry" || kind === "commands";
      const body = await readJson(req, lines ? LIMIT.lines : LIMIT.json);
      if (typeof body !== "object" || body === null || (!lines && Array.isArray(body))) throw httpError(400, lines ? "Expected a JSON object or an array." : "Expected a JSON object.");
      if (kind === "start") return json(res, 200, await recorder.start(body));
      if (kind === "stop") return json(res, 200, await recorder.stop(typeof body.reason === "string" ? body.reason.slice(0, 200) : "stopped"));
      return json(res, 200, recorder.lines(kind, body));
    }
    if (req.method !== "GET" && req.method !== "HEAD") {
      res.setHeader("Allow", "GET, HEAD, POST");
      return json(res, 405, { error: "Method not allowed." });
    }
    if (pathname === "/goggles" || pathname === "/goggles/" || pathname === "/goggles/status") return json(res, 200, helper.status());
    if (pathname === "/rec/status") return json(res, 200, recorder.status());
    if (pathname === "/rec/list") return json(res, 200, { recordings: await recorder.list() });
    if ((m = /^\/rec\/([^/]+)\/([^/]+)$/.exec(pathname))) {
      if (!tokenOk(h["x-whoop-token"] ?? params.get("token"))) return deny(req, res, "token");
      const [id, name] = decode([m[1], m[2]]) || [];
      const file = id && (await recorder.file(id, name));
      const st = file && (await fsp.stat(file).catch(() => null));
      return st?.isFile() ? sendFile(req, res, file, st) : json(res, 404, { error: "No such recording file." });
    }
    if (pathname === "/house-files" || pathname === "/house-files/") return json(res, 200, { projects: await houses.list() });
    if (pathname.startsWith("/house-files/")) {
      const segs = decode(pathname.slice("/house-files/".length).split("/").filter(Boolean));
      const id = segs?.shift();
      const r = id && (await houses.resolve(id, segs));
      if (!r) return json(res, 404, { error: "Not found in your capture folders." });
      if (r.stat.isDirectory()) return json(res, 200, { project: id, kind: r.project.kind, path: segs.join("/"), entries: await houses.dir(r.file, r.project.dir) });
      return sendFile(req, res, r.file, r.stat);
    }
    if (API.test(pathname)) return json(res, 404, { error: "No such endpoint." });
    return staticFile(req, res, pathname, query);
  }

  const handler = (req, res) =>
    route(req, res).catch((e) => {
      if (!e.status) say(`${req.method} ${req.url.split("?")[0]}: ${e.stack || e.message}`);
      if (res.headersSent) res.destroy();
      else json(res, e.status || 500, { error: e.message });
    });

  const listen = (server, p, host) =>
    new Promise((resolve, reject) => {
      server.once("error", reject);
      server.listen(p, host, () => {
        server.off("error", reject);
        resolve(server.address().port);
      });
    });
  for (const host of port ? (ipv6 ? ["127.0.0.1", "::1"] : ["127.0.0.1"]) : []) {
    if (await answers(host, port)) throw Object.assign(new Error(`${host} port ${port} is taken`), { code: "EADDRINUSE", host });
  }
  const servers = [http.createServer(handler)];
  port = await listen(servers[0], port, "127.0.0.1");
  // Chrome tries localhost on ::1 first: hold it too, so nothing else can answer there.
  if (ipv6) {
    const v6 = http.createServer(handler);
    try {
      await listen(v6, port, "::1");
      servers.push(v6);
    } catch (e) {
      if (e.code === "EADDRINUSE") {
        servers[0].close();
        throw Object.assign(new Error(`[::1]:${port} is taken`), { code: "EADDRINUSE", host: "::1" });
      }
    }
  }
  origins = new Set([`http://localhost:${port}`, `http://127.0.0.1:${port}`]);
  hosts = new Set([`localhost:${port}`, `127.0.0.1:${port}`]);
  helper.start();
  const close = async () => {
    helper.stop();
    if (recorder.recording || recorder.starting) await recorder.stop("server stopped").catch(() => {});
    await Promise.all(servers.map((s) => new Promise((r) => (s.close(r), s.closeAllConnections()))));
  };
  return { port, url: `http://localhost:${port}/app/`, token, helper, recorder, houses, close };
}

function openBrowser(url) {
  if (process.platform !== "darwin") return execFile("xdg-open", [url], () => {});
  execFile("open", ["-a", "Google Chrome", url], (e) => e && execFile("open", ["-a", "Microsoft Edge", url], (e2) => e2 && execFile("open", [url], () => {})));
}

async function marker(host, port) {
  const r = await fetch(`http://${host}:${port}/whoop-pilot.json`, { signal: AbortSignal.timeout(1500) }).catch(() => null);
  const j = r?.ok ? await r.json().catch(() => null) : null;
  return j?.app === "whoop-pilot" ? j : null;
}

const portOwner = (port) =>
  new Promise((resolve) =>
    execFile("lsof", ["-nP", `-iTCP:${port}`, "-sTCP:LISTEN", "-Fpc"], (e, out) => {
      const pid = /^p(\d+)/m.exec(out || "")?.[1];
      const cmd = /^c(.+)/m.exec(out || "")?.[1];
      resolve(pid ? `${cmd || "a program"} (process ${pid})` : "another program");
    }),
  );

async function main() {
  const opts = { wifi: false, peer: null, localPort: GOGGLES_PORT };
  let open = !process.env.WHOOP_NO_OPEN && !process.env.WHOOP_TEST_PORT;
  for (let i = 2; i < process.argv.length; i++) {
    const a = process.argv[i];
    if (a === "--wifi") opts.wifi = true;
    else if (a === "--usb") opts.wifi = false;
    else if (a === "--peer") opts.peer = process.argv[++i];
    else if (a === "--local-port") opts.localPort = Number(process.argv[++i]);
    else if (a === "--no-open") open = false;
    else {
      console.log("usage: node whoop.mjs [--wifi] [--peer IP[:port]] [--local-port N] [--no-open]");
      process.exit(a === "-h" || a === "--help" ? 0 : 2);
    }
  }
  const url = `http://localhost:${PORT}/app/`;
  let w;
  try {
    w = await startWhoop({ goggles: opts });
  } catch (e) {
    if (e.code !== "EADDRINUSE") throw e;
    const host = e.host === "::1" ? "[::1]" : "127.0.0.1";
    const ours = await marker(host, PORT);
    if (ours?.server === "whoop") {
      log(`Whoop Pilot is already running: ${url}`);
      if (ours.goggles && ours.goggles !== (opts.wifi ? "wifi" : "usb")) log(`It reads the goggles over ${ours.goggles === "wifi" ? "Wi-Fi" : "USB"}. To switch, close its window and start again${opts.wifi ? " with --wifi" : ""}.`);
      if (open) openBrowser(url);
      process.exit(0);
    }
    if (ours) log(`The simulator-only Whoop Pilot server (from a start.command window without Node.js) has port ${PORT}. Close that window, then start again for goggles video and the recorder.`);
    else log(`Port ${PORT} is used by ${await portOwner(PORT)}. Whoop Pilot keeps your settings, API key and house data under http://localhost:${PORT}, so it can't use another port. Quit that program, then start again.`);
    process.exit(1);
  }
  log(`Whoop Pilot: ${url}  (goggles over ${opts.wifi ? "Wi-Fi" : "USB"}; flight recordings go to ${path.join(DATA, "recordings").replace(os.homedir(), "~")})`);
  log("Keep this window open while you fly. Close it (or press Ctrl+C) to stop.");
  if (open) openBrowser(url);
  let closing = false;
  const quit = async () => {
    if (closing) process.exit(0);
    closing = true;
    await w.close();
    process.exit(0);
  };
  for (const sig of ["SIGINT", "SIGTERM", "SIGHUP"]) process.on(sig, quit);
}

if (process.argv[1] && fs.realpathSync(process.argv[1]) === fileURLToPath(import.meta.url)) main();
