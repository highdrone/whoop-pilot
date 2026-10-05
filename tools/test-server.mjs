// Whoop Pilot's server (whoop.mjs): the app and its marker, same-origin enforcement (Host, Origin, Referer,
// Sec-Fetch-Site, no CORS), the session token on POSTs and video, read-only capture files (listing, ranges,
// traversal and symlink escapes), the goggles stream from a fake Goggles 3, the flight recorder's files and
// caps, and the CLI's one-port rule (WHOOP_TEST_PORT). Servers run on free ports in 8800-8899.
import assert from "node:assert/strict";
import http from "node:http";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";
import { FakeGoggles } from "./fake-goggles.mjs";
import { startWhoop } from "./whoop.mjs";
import { Recorder } from "./recorder.mjs";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), "whoop-server-test-"));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const noGoggles = { devices: async () => [] };

async function until(test, ms, what) {
  const t0 = Date.now();
  while (!(await test())) {
    if (Date.now() - t0 > ms) throw new Error(`timed out waiting for ${what}`);
    await sleep(10);
  }
}

async function freePort() {
  for (let p = 8800 + Math.floor(Math.random() * 100), n = 0; n < 100; n++, p = 8800 + ((p - 8800 + 1) % 100)) {
    const ok = await new Promise((r) => {
      const s = http.createServer().once("error", () => r(false));
      s.listen(p, "127.0.0.1", () => s.close(() => r(true)));
    });
    if (ok) return p;
  }
  throw new Error("no free port in 8800-8899");
}

// Raw HTTP, so Host, Origin and Sec-Fetch-* are exactly what a test says (fetch would add or forbid some).
// A 5 s deadline: a stream that should have been refused never ends.
function req(port, p, { method = "GET", headers = {}, body } = {}) {
  return new Promise((resolve, reject) => {
    const r = http.request({ host: "127.0.0.1", port, path: p, method, headers: { Host: `localhost:${port}`, ...headers } }, (res) => {
      const chunks = [];
      res.on("data", (c) => chunks.push(c));
      res.on("end", () => {
        clearTimeout(deadline);
        const b = Buffer.concat(chunks);
        let json = null;
        try {
          json = JSON.parse(b.toString());
        } catch {}
        resolve({ status: res.statusCode, headers: res.headers, body: b, json });
      });
    });
    const deadline = setTimeout(() => r.destroy(new Error(`no complete answer in 5 s: ${method} ${p} (status ${r.res?.statusCode})`)), 5000);
    r.on("error", reject);
    r.end(body === undefined ? undefined : typeof body === "string" ? body : JSON.stringify(body));
  });
}

const same = (port) => ({ "Sec-Fetch-Site": "same-origin", Referer: `http://localhost:${port}/app/` });
const post = (w, p, body, token = w.token) => req(w.port, p, { method: "POST", body, headers: { ...same(w.port), Origin: `http://localhost:${w.port}`, "Content-Type": "application/json", ...(token && { "X-Whoop-Token": token }) } });

// Frames of the goggles stream: [type u8][length u32le][payload].
function streamFrames(port, p, headers, onFrame) {
  const r = http.get({ host: "127.0.0.1", port, path: p, headers: { Host: `localhost:${port}`, ...headers } }, (res) => {
    let buf = Buffer.alloc(0);
    if (res.statusCode !== 200) return onFrame({ status: res.statusCode });
    res.on("data", (c) => {
      buf = Buffer.concat([buf, c]);
      while (buf.length >= 5 && buf.length >= 5 + buf.readUInt32LE(1)) {
        const len = buf.readUInt32LE(1);
        onFrame({ type: buf[0], payload: buf.subarray(5, 5 + len) });
        buf = buf.subarray(5 + len);
      }
    });
  });
  r.on("error", () => {});
  return () => r.destroy();
}

function makeUnits(n) {
  return Array.from({ length: n }, (_, i) => {
    const u = new Uint8Array(1500 + ((i * 7919) % 9000));
    u.set([0, 0, 0, 1, 0x41, 0x9a, i & 255], 0);
    for (let k = 7; k < u.length; k++) u[k] = ((k * 31 + i) % 251) + 1;
    return u;
  });
}

// A SiteSpec and a Spacial root, a symlink out of a project, a project that is a symlink out, a dotfile.
function fakeRoots() {
  const base = path.join(TMP, "houses");
  const outside = path.join(TMP, "outside");
  fs.mkdirSync(outside, { recursive: true });
  fs.writeFileSync(path.join(outside, "secret.txt"), "not yours");
  const project = (root, id, files, info) => {
    const d = path.join(base, root, id);
    for (const [rel, data] of Object.entries(files)) {
      fs.mkdirSync(path.dirname(path.join(d, rel)), { recursive: true });
      fs.writeFileSync(path.join(d, rel), data);
    }
    if (info) fs.writeFileSync(path.join(d, "project.json"), JSON.stringify(info));
    return d;
  };
  const splat = Buffer.from(Array.from({ length: 5000 }, (_, i) => i % 256));
  const a = project("SiteSpec Projects", "house-a-1234", { "outputs/splat.spz": splat, "outputs/scene.json": "{}", "outputs/thumbnail.jpg": "jpg", "outputs/plans/house-a-rooms.json": "[]", "outputs/plans/house-a-orthophoto.png": "png", "site.json": "{}", "work/room/rooms.auto.json": "{}", ".hidden": "x" }, { name: "House A", created: 1790000000, updated: 1790800000 });
  project("Spacial Projects", "room-b-5678", { "outputs/splat.spz": "spz", "outputs/scene.json": "{}" }, { name: "Room B", created: 1790900000 });
  project("Spacial Projects", "no-splat-9999", { "notes.txt": "x" }, { created: 1780000000 });
  fs.symlinkSync(path.join(outside, "secret.txt"), path.join(a, "outputs", "leak.txt"));
  fs.symlinkSync(outside, path.join(base, "SiteSpec Projects", "escape"));
  return { roots: [{ kind: "sitespec", dir: path.join(base, "SiteSpec Projects") }, { kind: "spacial", dir: path.join(base, "Spacial Projects") }], splat };
}

let w; // the shared in-process server for most tests
const logged = []; // what w logged
const tests = {
  async "serves the app: / -> /app/, index, MIME types, no-store, marker"() {
    const { port } = w;
    const root = await req(port, "/");
    assert.equal(root.status, 302);
    assert.equal(root.headers.location, "/app/");
    assert.equal((await req(port, "/app")).headers.location, "/app/");
    const index = await req(port, "/app/");
    assert.equal(index.status, 200);
    assert.match(index.headers["content-type"], /^text\/html/);
    assert.match(index.body.toString(), /<html/i);
    assert.equal(index.headers["cache-control"], "no-store");
    for (const [p, type] of [["/app/js/main.js", /^text\/javascript/], ["/app/manifest.json", /^application\/json/], ["/app/icon.svg", /^image\/svg\+xml/], ["/app/icon-192.png", /^image\/png/], ["/radio/SCRIPTS/MIXES/aibrg.lua", /^text\/plain/]]) {
      const r = await req(port, p, { headers: same(port) });
      assert.equal(r.status, 200, p);
      assert.match(r.headers["content-type"], type, p);
    }
    const m = await req(port, "/whoop-pilot.json");
    assert.deepEqual([m.status, m.json.app, m.json.server], [200, "whoop-pilot", "whoop"]);
    const head = await req(port, "/app/js/main.js", { method: "HEAD" });
    assert.equal(head.status, 200);
    assert.equal(head.body.length, 0);
    assert.equal(Number(head.headers["content-length"]), fs.statSync(path.join(HERE, "../app/js/main.js")).size);
  },

  async "no CORS anywhere, isolation and framing headers on every response"() {
    const { port } = w;
    for (const [p, method, headers] of [["/app/", "GET", {}], ["/api/session", "GET", same(port)], ["/goggles/status", "GET", {}], ["/house-files/", "GET", {}], ["/rec/start", "OPTIONS", { Origin: "http://evil.example", "Access-Control-Request-Method": "POST" }], ["/app/", "GET", { Origin: "http://evil.example" }]]) {
      const r = await req(port, p, { method, headers });
      assert.ok(!Object.keys(r.headers).some((h) => h.startsWith("access-control-")), `${method} ${p} has CORS headers`);
      assert.equal(r.headers["cross-origin-resource-policy"], "same-origin", p);
      assert.equal(r.headers["x-frame-options"], "DENY", p);
      assert.equal(r.headers["x-content-type-options"], "nosniff", p);
    }
    const app = await req(port, "/app/");
    assert.deepEqual([app.headers["cross-origin-opener-policy"], app.headers["cross-origin-embedder-policy"]], ["same-origin", "credentialless"]);
  },

  async "other sites, other host names and sandboxed pages get 403"() {
    const { port } = w;
    const cases = {
      "foreign Origin": { Origin: "http://evil.example" },
      "same-site but other port": { Origin: "http://localhost:3000" },
      "null Origin (sandbox, file://)": { Origin: "null" },
      "foreign Referer": { Referer: "https://evil.example/page" },
      "Sec-Fetch-Site cross-site": { "Sec-Fetch-Site": "cross-site" },
      "Sec-Fetch-Site same-site": { "Sec-Fetch-Site": "same-site" },
      "DNS rebinding Host": { Host: `evil.example:${port}` },
      "Host on another port": { Host: "localhost:8790" },
    };
    for (const [what, headers] of Object.entries(cases)) {
      for (const p of ["/app/js/main.js", "/whoop-pilot.json", "/goggles/status", "/house-files/", "/rec/list"]) {
        assert.equal((await req(port, p, { headers })).status, 403, `${what}: ${p}`);
      }
    }
    for (const origin of [`http://localhost:${port}`, `http://127.0.0.1:${port}`]) assert.equal((await req(port, "/app/js/main.js", { headers: { Origin: origin, Referer: `${origin}/app/`, "Sec-Fetch-Site": "same-origin" } })).status, 200, origin);
    const post403 = await req(port, "/rec/start", { method: "POST", headers: { Origin: "http://evil.example", "X-Whoop-Token": w.token }, body: {} });
    assert.equal(post403.status, 403, "even with the token");
    assert.equal(w.recorder.recording, false);
  },

  async "a link from another site may open the app's page, never an API route"() {
    const { port } = w;
    const link = { "Sec-Fetch-Site": "cross-site", "Sec-Fetch-Mode": "navigate", "Sec-Fetch-Dest": "document", Referer: "https://elsewhere.example/" };
    // Measured in Chrome 154: once sw.js controls the app, its fetch(e.request) keeps mode navigate and sends dest empty.
    const viaWorker = { ...link, "Sec-Fetch-Dest": "empty" };
    const viaWorkerInit = { ...link, "Sec-Fetch-Mode": "same-origin" }; // a worker that rebuilds the request with an init
    for (const [what, nav] of Object.entries({ link, viaWorker, viaWorkerInit })) {
      assert.equal((await req(port, "/app/", { headers: nav })).status, 200, what);
      for (const p of ["/api/session", "/house-files/", "/goggles/status", "/rec/list"]) assert.equal((await req(port, p, { headers: nav })).status, 403, `${what}: ${p}`);
      const ip = await req(port, "/app/", { headers: { ...nav, Host: `127.0.0.1:${port}` } });
      assert.deepEqual([ip.status, ip.headers.location], [302, `http://localhost:${port}/app/`], `${what}: 127.0.0.1 navigations go to the localhost origin, where the data is`);
    }
    for (const dest of ["iframe", "frame", "object", "embed"]) assert.equal((await req(port, "/app/", { headers: { ...link, "Sec-Fetch-Dest": dest } })).status, 403, `framing: ${dest}`);
    for (const mode of ["cors", "no-cors", "same-origin"]) assert.equal((await req(port, "/app/js/main.js", { headers: { ...viaWorker, "Sec-Fetch-Mode": mode } })).status, 403, `a script on another site: ${mode}`);
    assert.equal((await req(port, "/app/", { method: "HEAD", headers: link })).status, 403, "navigations are GET");
  },

  async "the session token goes only to this origin's own scripts"() {
    const { port } = w;
    const ok = await req(port, "/api/session", { headers: same(port) });
    assert.equal(ok.status, 200);
    assert.equal(ok.json.token, w.token);
    assert.ok(/^[0-9a-f]{48}$/.test(w.token));
    assert.equal((await req(port, "/api/session", { headers: { Origin: `http://localhost:${port}` } })).json.token, w.token, "no Sec-Fetch-Site, matching Origin");
    for (const headers of [{}, { "Sec-Fetch-Site": "none" }, { "Sec-Fetch-Site": "cross-site" }, { Origin: "http://evil.example" }]) {
      const r = await req(port, "/api/session", { headers });
      assert.equal(r.status, 403, JSON.stringify(headers));
      assert.ok(!r.body.toString().includes(w.token));
    }
  },

  async "every POST needs the token"() {
    for (const token of [null, "", "0".repeat(48), w.token.slice(0, -1) + (w.token.endsWith("0") ? "1" : "0"), w.token + "00"]) {
      for (const p of ["/rec/start", "/rec/stop", "/rec/telemetry", "/rec/command", "/app/", "/nowhere"]) assert.equal((await post(w, p, {}, token)).status, 403, `${p} with ${JSON.stringify(token)}`);
    }
    assert.equal(w.recorder.recording, false);
    assert.equal((await post(w, "/nowhere", {})).status, 404);
    assert.equal((await post(w, "/rec/telemetry", [{ a: 1 }])).status, 409, "not recording");
    assert.equal((await post(w, "/rec/start", "{nope")).status, 400);
    assert.equal((await req(w.port, "/app/", { method: "DELETE", headers: same(w.port) })).status, 405);
  },

  async "POST bodies must be JSON objects (arrays for telemetry and commands): 400, never a crash"() {
    for (const body of ["null", "[]", "[{}]", '"stop"', "5", "true"]) {
      for (const p of ["/rec/start", "/rec/stop"]) {
        const r = await post(w, p, body);
        assert.equal(r.status, 400, `${p} ${body}: ${r.body}`);
      }
    }
    for (const body of ["null", '"x"', "5"]) for (const p of ["/rec/telemetry", "/rec/command"]) assert.equal((await post(w, p, body)).status, 400, `${p} ${body}`);
    assert.equal(w.recorder.recording, false);
    assert.ok(!logged.some((l) => /Error|\n\s+at /.test(l)), logged.join("\n"));
    const started = await post(w, "/rec/start", { label: "body checks" });
    assert.equal(started.status, 200);
    assert.equal((await post(w, "/rec/telemetry", [])).status, 200, "an empty batch keeps the recording alive");
    assert.equal((await post(w, "/rec/stop", { reason: 42 })).json.stopReason, "stopped");
  },

  async "the video stream answers GET only"() {
    for (const method of ["HEAD", "POST", "PUT"]) {
      const r = await req(w.port, `/goggles/${w.token}/stream`, { method, headers: same(w.port), body: method === "HEAD" ? undefined : "{}" });
      assert.deepEqual([r.status, r.headers.allow], [405, "GET"], method);
    }
    assert.equal((await req(w.port, "/goggles/status")).json.clients, 0, "nothing left attached");
  },

  async "static files: no traversal, dotfiles or node_modules"() {
    const { port } = w;
    for (const p of ["/../README.md", "/app/../../etc/passwd", "/%2e%2e/%2e%2e/etc/passwd", "/app/%2e%2e/%2e%2e/etc/hosts", "/app%2F..%2F.gitignore", "/.gitignore", "/tools/node_modules/usb/package.json", "/app/%00.js", "/app/%zz"]) {
      const r = await req(port, p, { headers: same(port) });
      assert.equal(r.status, 404, `${p}: ${r.status}`);
    }
    assert.equal((await req(port, "/README.md", { headers: same(port) })).status, 200);
  },

  async "capture files: listing newest first, files, folders, ranges; no traversal, dotfiles or symlink escapes"() {
    const { roots, splat } = fakeRoots();
    const h = await startWhoop({ port: await freePort(), dataDir: path.join(TMP, "data-houses"), houseRoots: roots, goggles: noGoggles, log: () => {} });
    try {
      const { port } = h;
      const list = (await req(port, "/house-files/", { headers: same(port) })).json.projects;
      assert.deepEqual(list.map((p) => [p.id, p.kind, p.name]), [["room-b-5678", "spacial", "Room B"], ["house-a-1234", "sitespec", "House A"], ["no-splat-9999", "spacial", "no-splat-9999"]]);
      const a = list[1];
      assert.equal(a.date, new Date(1790800000e3).toISOString());
      assert.deepEqual(a.splat, { path: "outputs/splat.spz", size: splat.length, url: "/house-files/house-a-1234/outputs/splat.spz" });
      assert.equal(a.thumbnail, "/house-files/house-a-1234/outputs/thumbnail.jpg");
      assert.deepEqual(a.files, { splat: "outputs/splat.spz", scene: "outputs/scene.json", site: "site.json", rooms: "outputs/plans/house-a-rooms.json", roomsAuto: "work/room/rooms.auto.json", orthophoto: "outputs/plans/house-a-orthophoto.png", orthophotoMeta: null, thumbnail: "outputs/thumbnail.jpg" });
      assert.deepEqual([a.ready, list[2].ready, list[2].splat, list[2].thumbnail], [true, false, null, null]);
      assert.ok(!list.some((p) => p.id === "escape"), "a project symlinked out of the root isn't listed");
      const f = await req(port, a.splat.url, { headers: same(port) });
      assert.deepEqual([f.status, f.headers["content-type"]], [200, "application/octet-stream"]);
      assert.ok(f.body.equals(splat));
      const part = await req(port, a.splat.url, { headers: { ...same(port), Range: "bytes=100-199" } });
      assert.deepEqual([part.status, part.headers["content-range"], part.body.length], [206, `bytes 100-199/${splat.length}`, 100]);
      assert.ok(part.body.equals(splat.subarray(100, 200)));
      const tail = await req(port, a.splat.url, { headers: { ...same(port), Range: "bytes=-10" } });
      assert.ok(tail.body.equals(splat.subarray(-10)));
      assert.equal((await req(port, a.splat.url, { headers: { ...same(port), Range: "bytes=999999-" } })).status, 416);
      const dir = (await req(port, "/house-files/house-a-1234/outputs/plans/", { headers: same(port) })).json;
      assert.deepEqual(dir.entries.map((e) => [e.name, e.kind, e.size]), [["house-a-orthophoto.png", "file", 3], ["house-a-rooms.json", "file", 2]]);
      const top = (await req(port, "/house-files/house-a-1234", { headers: same(port) })).json.entries.map((e) => e.name);
      assert.deepEqual((await req(port, "/house-files/house-a-1234//", { headers: same(port) })).json.entries.map((e) => e.name), top);
      assert.deepEqual(top, ["outputs", "project.json", "site.json", "work"], "no dotfiles in listings");
      const outputs = (await req(port, "/house-files/house-a-1234/outputs/", { headers: same(port) })).json.entries.map((e) => e.name);
      assert.ok(outputs.includes("splat.spz") && !outputs.includes("leak.txt"), "no symlinks out of the project in listings");
      for (const p of [
        "/house-files/house-a-1234/outputs/leak.txt", // symlink to a file outside
        "/house-files/escape/secret.txt", // project symlinked outside
        "/house-files/house-a-1234/.hidden",
        "/house-files/house-a-1234/../room-b-5678/outputs/scene.json",
        "/house-files/house-a-1234/%2e%2e/%2e%2e/%2e%2e/outside/secret.txt",
        "/house-files/house-a-1234/outputs%2F..%2F..%2F..%2Foutside%2Fsecret.txt",
        "/house-files/..%2Foutside/secret.txt",
        "/house-files/%2e%2e/outside/secret.txt",
        "/house-files/house-a-1234/outputs/missing.bin",
        "/house-files/nope/x",
      ]) {
        const r = await req(port, p, { headers: same(port) });
        assert.equal(r.status, 404, p);
        assert.ok(!r.body.toString().includes("not yours"), p);
      }
    } finally {
      await h.close();
    }
  },

  async "goggles video streams same-origin with the token, and the recorder saves it with telemetry and commands"() {
    const units = makeUnits(90);
    const fake = new FakeGoggles({ units, fps: 120 });
    const data = path.join(TMP, "data-goggles");
    const g = await startWhoop({ port: await freePort(), dataDir: data, houseRoots: [], goggles: { devices: async () => [fake] }, log: () => {} });
    const stops = [];
    try {
      const { port, token } = g;
      const origin = { Origin: `http://localhost:${port}`, "Sec-Fetch-Site": "same-origin" };
      for (const [p, headers] of [["/goggles/stream", origin], [`/goggles/${"0".repeat(48)}/stream`, origin], [`/goggles/${token}/stream`, { Origin: "http://evil.example" }], [`/goggles/${token}/stream`, { "Sec-Fetch-Site": "cross-site" }]]) {
        assert.equal((await req(port, p, { headers })).status, 403, `${p} ${JSON.stringify(headers)}`);
      }
      const got = [];
      let status = null;
      stops.push(streamFrames(port, `/goggles/${token}/stream`, origin, (f) => (f.type === 2 ? (status ??= JSON.parse(f.payload)) : f.type === 1 && got.push(Buffer.from(f.payload)))));
      const viaQuery = [];
      stops.push(streamFrames(port, `/goggles/stream?token=${token}`, origin, (f) => f.type === 1 && viaQuery.push(f.payload)));
      await until(() => got.length >= 20 && viaQuery.length >= 5, 8000, "video through the stream");
      assert.equal(status.app, "whoop-pilot-goggles");
      const first = units.findIndex((u) => Buffer.from(u).equals(got[0]));
      assert.ok(first >= 0, "a fake unit");
      got.forEach((u, i) => assert.ok(u.equals(Buffer.from(units[(first + i) % units.length])), `unit ${i} in order and intact`));
      const st = (await req(port, "/goggles/status", { headers: origin })).json;
      assert.deepEqual([st.state, st.clients, st.transport], ["live", 2, "usb"]);
      assert.match(st.device, /2ca3:0020 on USB/);

      const started = await post(g, "/rec/start", { label: "Kitchen pass #1", house: "house-a", lens: { uptilt: 20, fov: 159 }, app: { build: "test" }, note: "hand-carried" });
      assert.equal(started.status, 200, started.body.toString());
      assert.match(started.json.id, /^\d{4}-\d\d-\d\dT\d\d-\d\d-\d\d-kitchen-pass-1$/);
      assert.equal((await post(g, "/rec/start", { label: "again" })).status, 409);
      const t0 = Date.now();
      assert.deepEqual((await post(g, "/rec/telemetry", [{ t: t0, alt: 0.5, yaw: 1 }, { t: t0 + 33, alt: 0.6, yaw: 1.1 }])).json, { recording: true, id: started.json.id, written: 2 });
      await until(() => g.recorder.rec?.video.units >= 30, 8000, "recorded units");
      await post(g, "/rec/telemetry", [{ alt: 0.7 }]);
      await post(g, "/rec/command", [{ t: t0, cmd: "go_to", args: { place: "kitchen" } }]);
      assert.equal((await req(port, "/rec/status", { headers: origin })).json.recording, true);
      const stopped = await post(g, "/rec/stop", {});
      assert.equal(stopped.status, 200);
      assert.equal(stopped.json.stopReason, "stopped");
      assert.equal((await post(g, "/rec/stop", {})).status, 409);
      assert.equal((await post(g, "/rec/telemetry", [{ late: 1 }])).status, 409);

      const dir = path.join(data, "recordings", started.json.id);
      const video = fs.readFileSync(path.join(dir, "video.h264"));
      const index = fs.readFileSync(path.join(dir, "video.jsonl"), "utf8").trim().split("\n").map(JSON.parse);
      assert.ok(index.length >= 30);
      let off = 0;
      const k = units.findIndex((u) => Buffer.from(u).equals(video.subarray(0, u.length)));
      assert.ok(k >= 0, "the file starts with a whole unit");
      index.forEach((e, i) => {
        assert.deepEqual([e.off, e.bytes], [off, units[(k + i) % units.length].length], `index ${i}`);
        assert.ok(video.subarray(off, off + e.bytes).equals(Buffer.from(units[(k + i) % units.length])), `unit ${i} as received`);
        assert.ok(e.t >= t0 - 5000 && e.t <= Date.now() && (!i || e.t >= index[i - 1].t), `receive time ${i}`);
        off += e.bytes;
      });
      assert.equal(off, video.length);
      const tel = fs.readFileSync(path.join(dir, "telemetry.jsonl"), "utf8").trim().split("\n").map(JSON.parse);
      assert.deepEqual(tel.slice(0, 2), [{ t: t0, alt: 0.5, yaw: 1 }, { t: t0 + 33, alt: 0.6, yaw: 1.1 }]);
      assert.equal(tel[2].alt, 0.7);
      assert.ok(tel[2].t >= t0, "t added when missing");
      assert.deepEqual(fs.readFileSync(path.join(dir, "commands.jsonl"), "utf8").trim().split("\n").map(JSON.parse), [{ t: t0, cmd: "go_to", args: { place: "kitchen" } }]);
      const meta = JSON.parse(fs.readFileSync(path.join(dir, "meta.json"), "utf8"));
      assert.deepEqual([meta.label, meta.house, meta.lens, meta.app, meta.note, meta.stopReason], ["Kitchen pass #1", "house-a", { uptilt: 20, fov: 159 }, { build: "test" }, "hand-carried", "stopped"]);
      assert.deepEqual([meta.video.units, meta.video.bytes, meta.telemetry, meta.commands], [index.length, video.length, 3, 1]);
      assert.ok(meta.ended && meta.durationMs > 0 && meta.server.build && meta.goggles.start.state === "live");
      assert.ok(!fs.readdirSync(dir).some((f) => f.endsWith(".tmp")));

      const list = (await req(port, "/rec/list", { headers: origin })).json.recordings;
      assert.equal(list[0].id, started.json.id);
      assert.equal(list[0].files["video.h264"], video.length);
      const metaUrl = `/rec/${started.json.id}/meta.json`;
      assert.equal((await req(port, metaUrl, { headers: origin })).status, 403, "recorded files need the token");
      assert.equal((await req(port, metaUrl, { headers: { ...origin, "X-Whoop-Token": token } })).json.id, started.json.id);
      const part = await req(port, `/rec/${started.json.id}/video.h264?token=${token}`, { headers: { ...origin, Range: "bytes=0-99" } });
      assert.ok(part.body.equals(video.subarray(0, 100)));
      for (const p of [`/rec/..%2F..%2Fx/meta.json`, `/rec/${started.json.id}/..%2Fmeta.json`, `/rec/${started.json.id}/.secret`]) assert.equal((await req(port, `${p}?token=${token}`, { headers: origin })).status, 404, p);
    } finally {
      stops.forEach((s) => s());
      await g.close();
      fake.stopStreaming();
    }
  },

  async "without the usb package the app is still served and the goggles status says why"() {
    const broken = async () => {
      throw Object.assign(new Error(`The "usb" package isn't installed or didn't build (x). Run: cd tools && npm install`), { fatal: true });
    };
    const b = await startWhoop({ port: await freePort(), dataDir: path.join(TMP, "data-nousb"), houseRoots: [], goggles: { devices: broken }, log: () => {} });
    try {
      await until(async () => (await req(b.port, "/goggles/status")).json.stats.fatal, 3000, "the fatal status");
      const st = (await req(b.port, "/goggles/status")).json;
      assert.deepEqual([st.state, st.stats.error.startsWith('The "usb" package')], ["no-goggles", true]);
      await sleep(2500);
      assert.equal((await req(b.port, "/goggles/status")).json.stats.fatal, true, "stays, no retry loop");
      assert.equal((await req(b.port, "/app/")).status, 200);
    } finally {
      await b.close();
    }
  },

  async "recorder stops itself at the size cap, the time cap, when the page goes quiet and when the disk fills"() {
    const dir = path.join(TMP, "caps");
    const unit = new Uint8Array(10000);
    const r = new Recorder({ dir, maxBytes: 55000, log: () => {} });
    await r.start({ label: "size" });
    for (let i = 0; i < 10; i++) r.unit(unit);
    await until(() => r.last, 2000, "size stop");
    assert.equal(r.last.stopReason, "size cap");
    assert.equal(r.last.video.units, 6);
    const t = new Recorder({ dir, maxMs: 300, log: () => {} });
    await t.start({ label: "time" });
    await until(() => t.last, 3000, "time stop");
    assert.equal(t.last.stopReason, "time cap");
    const q = new Recorder({ dir, idleMs: 300, log: () => {} });
    await q.start({ label: "quiet" });
    await until(() => q.last, 3000, "idle stop");
    assert.match(q.last.stopReason, /nothing from the page/);
    assert.equal(fs.readdirSync(dir).length, 3);
    for (const d of fs.readdirSync(dir)) assert.ok(JSON.parse(fs.readFileSync(path.join(dir, d, "meta.json"), "utf8")).ended, d);
    let full = false;
    const gone = new Recorder({ dir, free: async () => (full ? 2 ** 29 : 50 * 2 ** 30), log: () => {} });
    await gone.start({ label: "disk" });
    await sleep(1100);
    assert.equal(gone.recording, true, "plenty of disk");
    full = true; // something else fills the disk mid-flight
    await until(() => gone.last, 3000, "low-disk stop");
    assert.equal(gone.last.stopReason, "disk nearly full");
    assert.equal(JSON.parse(fs.readFileSync(path.join(dir, gone.last.id, "meta.json"), "utf8")).stopReason, "disk nearly full");
    await assert.rejects(new Recorder({ dir, free: async () => 2 ** 30 + 1e6 }).start({}), { status: 507 });
    const twin = path.join(TMP, "twins");
    const [a, b] = [new Recorder({ dir: twin }), new Recorder({ dir: twin })];
    const ids = [(await a.start({ label: "same" })).id, (await b.start({ label: "same" })).id];
    await Promise.all([a.stop(), b.stop()]);
    assert.notEqual(ids[0], ids[1]);
    if (ids[0].slice(0, 19) === ids[1].slice(0, 19)) assert.ok(ids[1].endsWith("-same-2"), ids[1]);
  },

  async "recorder: a stop that arrives while the recording is starting stops it"() {
    const r = new Recorder({ dir: path.join(TMP, "race") });
    const [started, stopped] = await Promise.all([r.start({ label: "quick" }), r.stop("abort")]);
    assert.deepEqual([started.recording, stopped.recording, stopped.id, stopped.stopReason, r.recording], [true, false, started.id, "abort", false]);
    const low = new Recorder({ dir: path.join(TMP, "race"), free: async () => 0 });
    const [failed, nothing] = await Promise.allSettled([low.start({}), low.stop()]);
    assert.deepEqual([failed.reason?.status, nothing.reason?.status, low.recording], [507, 409, false]);
    const slow = (rec) => {
      const free = rec.free;
      rec.free = async (d) => (await sleep(200), free(d));
      return () => (rec.free = free);
    };
    const restore = slow(w.recorder);
    try {
      const a = post(w, "/rec/start", { label: "race" });
      await sleep(60);
      assert.ok(w.recorder.starting, "the stop below arrives while the start is under way");
      const b = await post(w, "/rec/stop", { reason: "double click" });
      assert.deepEqual([(await a).status, b.status, b.json.stopReason, w.recorder.recording], [200, 200, "double click", false]);
    } finally {
      restore();
    }
    const h = await startWhoop({ port: await freePort(), dataDir: path.join(TMP, "data-close"), houseRoots: [], goggles: noGoggles, log: () => {} });
    slow(h.recorder);
    const pending = post(h, "/rec/start", { label: "closing" }).catch(() => null);
    await sleep(60);
    await h.close();
    await pending;
    assert.equal(h.recorder.recording, false);
    const [id] = fs.readdirSync(path.join(TMP, "data-close", "recordings"));
    assert.equal(JSON.parse(fs.readFileSync(path.join(TMP, "data-close", "recordings", id, "meta.json"), "utf8")).stopReason, "server stopped");
  },

  async "CLI: one port only; tells our server from others (WHOOP_TEST_PORT)"() {
    const run = (port, ms = 8000) => {
      const child = spawn(process.execPath, [path.join(HERE, "whoop.mjs")], { env: { ...process.env, WHOOP_TEST_PORT: String(port), WHOOP_TEST_DATA: path.join(TMP, "data-cli") }, stdio: ["ignore", "pipe", "pipe"] });
      const r = { child, out: "" };
      child.stdout.on("data", (d) => (r.out += d));
      child.stderr.on("data", (d) => (r.out += d));
      const t = setTimeout(() => child.kill("SIGKILL"), ms);
      r.done = new Promise((resolve) => child.on("exit", (code) => (clearTimeout(t), resolve(code))));
      return r;
    };
    const taken = async (handler, check, host = "127.0.0.1") => {
      const port = await freePort();
      const s = http.createServer(handler);
      await new Promise((r) => s.listen(port, host, r));
      const r = run(port);
      const code = await r.done;
      s.close();
      assert.equal(code, 1, `${host ?? "*"}: ${r.out}`);
      assert.match(r.out, check(port));
    };
    const other = (port) => new RegExp(`Port ${port} is used by node \\(process ${process.pid}\\).*can't use another port`);
    await taken((q, res) => res.end("hello"), other);
    await taken((q, res) => res.end("hello"), other, undefined); // *:port (dual stack): macOS would let 127.0.0.1 bind on top
    await taken((q, res) => res.end("hello"), other, "0.0.0.0");
    await taken((q, res) => res.end(JSON.stringify({ app: "whoop-pilot", server: "python" })), () => /simulator-only/);
    const wild = http.createServer().listen(await freePort());
    await new Promise((r) => wild.once("listening", r));
    await assert.rejects(startWhoop({ port: wild.address().port, dataDir: path.join(TMP, "data-wild"), houseRoots: [], goggles: noGoggles, log: () => {} }), { code: "EADDRINUSE" });
    wild.close();
    const port = await freePort();
    const server = run(port, 20000);
    await until(() => /Whoop Pilot: http:\/\/localhost:\d+\/app\//.test(server.out), 8000, "the server's start line");
    assert.equal((await req(port, "/whoop-pilot.json")).json.server, "whoop");
    const again = run(port);
    assert.equal(await again.done, 0, again.out);
    assert.match(again.out, /already running/);
    server.child.kill("SIGTERM");
    assert.equal(await server.done, 0, server.out);
  },
};

w = await startWhoop({ port: await freePort(), dataDir: path.join(TMP, "data"), houseRoots: [], goggles: noGoggles, log: (m) => logged.push(m) });
let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    await fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    const where = e.stack?.split("\n").find((l) => l.includes("test-server.mjs")) || "";
    console.log(`FAIL  ${name}\n      ${e.message.split("\n").join("\n      ")}\n      ${where.trim()}`);
  }
}
await w.close();
fs.rmSync(TMP, { recursive: true, force: true });
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
