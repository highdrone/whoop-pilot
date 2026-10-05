// Runs radio/SCRIPTS/MIXES/aibrg.lua in a Lua VM (fengari) with a mocked EdgeTX API and
// checks it against the app's protocol code. Usage: cd tools && npm install && npm test
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import fengari from "fengari";
import { encodeCommand, decodeTelemetry, MASK } from "../app/js/protocol.js";

const { lua, lauxlib, lualib, to_luastring, to_jsstring } = fengari;
const here = path.dirname(fileURLToPath(import.meta.url));
const SCRIPT = readFileSync(path.join(here, "../radio/SCRIPTS/MIXES/aibrg.lua"));

const ALL = MASK.roll | MASK.pitch | MASK.thr | MASK.yaw;
const COPILOT = MASK.roll | MASK.pitch | MASK.yaw;

function makeRadio() {
  const L = lauxlib.luaL_newstate();
  lualib.luaL_openlibs(L);
  const env = {
    time: 0, // EdgeTX getTime(): 10 ms ticks
    rxBuf: "",
    tx: [],
    tones: 0,
    haptics: 0,
    sensors: { RxBt: 4.1, RQly: 100, "1RSS": -42, Ptch: 0.012, Roll: -0.02, Yaw: 1.5, FM: "STAB" },
  };
  const def = (name, fn) => {
    lua.lua_pushjsfunction(L, fn);
    lua.lua_setglobal(L, to_luastring(name));
  };
  lua.lua_pushinteger(L, 1);
  lua.lua_setglobal(L, to_luastring("SOURCE"));
  lua.lua_pushinteger(L, 0);
  lua.lua_setglobal(L, to_luastring("PLAY_NOW"));
  def("getTime", (L) => (lua.lua_pushinteger(L, env.time), 1));
  // EdgeTX semantics: returns bytes up to and including the first newline, or the rest of the FIFO.
  def("serialRead", (L) => {
    const i = env.rxBuf.search(/[\r\n]/);
    const n = i === -1 ? env.rxBuf.length : i + 1;
    const s = env.rxBuf.slice(0, n);
    env.rxBuf = env.rxBuf.slice(n);
    lua.lua_pushstring(L, to_luastring(s));
    return 1;
  });
  def("serialWrite", (L) => (env.tx.push(to_jsstring(lua.lua_tostring(L, 1))), 0));
  def("getValue", (L) => {
    const v = env.sensors[to_jsstring(lua.lua_tostring(L, 1))];
    if (typeof v === "number") lua.lua_pushnumber(L, v);
    else if (typeof v === "string") lua.lua_pushstring(L, to_luastring(v));
    else lua.lua_pushnil(L);
    return 1;
  });
  def("playTone", () => (env.tones++, 0));
  def("playHaptic", () => (env.haptics++, 0));

  if (lauxlib.luaL_loadbuffer(L, SCRIPT, null, to_luastring("aibrg")) !== lua.LUA_OK) {
    throw new Error(to_jsstring(lua.lua_tostring(L, -1)));
  }
  lua.lua_call(L, 0, 1);
  lua.lua_getfield(L, -1, to_luastring("input"));
  const nInputs = lua.lua_rawlen(L, -1);
  lua.lua_pop(L, 1);
  lua.lua_getfield(L, -1, to_luastring("output"));
  const nOutputs = lua.lua_rawlen(L, -1);
  lua.lua_pop(L, 1);
  const ref = lauxlib.luaL_ref(L, lua.LUA_REGISTRYINDEX);

  // One mixer-script cycle (~30 ms on a real radio). Sticks in -1024..1024.
  function tick({ ail = 0, ele = 0, thr = -1024, rud = 0, sw = -1024, ptt = -1024 } = {}, ticks = 3) {
    env.time += ticks;
    lua.lua_rawgeti(L, lua.LUA_REGISTRYINDEX, ref);
    lua.lua_getfield(L, -1, to_luastring("run"));
    for (const v of [ail, ele, thr, rud, sw, ptt]) lua.lua_pushinteger(L, v);
    if (lua.lua_pcall(L, 6, 5, 0) !== lua.LUA_OK) throw new Error(to_jsstring(lua.lua_tostring(L, -1)));
    const out = [];
    for (let i = -5; i <= -1; i++) out.push(lua.lua_tonumber(L, i));
    lua.lua_pop(L, 6);
    const [r, p, t, y, mode] = out;
    return { r, p, t, y, mode };
  }

  let seq = 0;
  function send(c) {
    env.rxBuf += encodeCommand({ seq: ++seq, roll: 0, pitch: 0, thr: 0, yaw: 0, hover: 0.4, ...c });
  }
  return { env, tick, send, nInputs, nOutputs };
}

const raw = (v) => Math.round(v * 1024);
const rawThr = (t) => Math.round((t * 2 - 1) * 1024);
const ON = 1024;

const tests = {
  "declares 6 inputs and 5 outputs (EdgeTX max is 6 each)"() {
    const r = makeRadio();
    assert.equal(r.nInputs, 6);
    assert.equal(r.nOutputs, 5);
  },

  "passes sticks through while the AI switch is off, and reports telemetry"() {
    const r = makeRadio();
    r.send({ mask: ALL, roll: 0.5, thr: 0.6 });
    const o = r.tick({ ail: 100, ele: -50, thr: -900, rud: 20 });
    assert.deepEqual([o.r, o.p, o.t, o.y, o.mode], [100, -50, -900, 20, -1024]);
    for (let i = 0; i < 5; i++) r.tick();
    const t = decodeTelemetry(r.env.tx.at(-1));
    assert.ok(t, "telemetry line decodes");
    assert.equal(t.engaged, false);
    assert.equal(t.vbat, 4.1);
    assert.equal(t.lq, 100);
    assert.equal(t.rssi, -42);
    assert.equal(t.att.yaw, 1.5);
    assert.equal(t.att.pitch, 0.012);
    assert.equal(t.fm, "STAB");
  },

  "sends telemetry at about 10 Hz"() {
    const r = makeRadio();
    for (let i = 0; i < 34; i++) r.tick(); // ~1 s
    assert.ok(r.env.tx.length >= 9 && r.env.tx.length <= 12, `got ${r.env.tx.length}`);
  },

  "applies app commands on all four axes when engaged with a full mask"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    r.send({ mask: ALL, roll: 0.25, pitch: -0.1, thr: 0.45, yaw: 0.05 });
    const o = r.tick({ sw: ON });
    assert.deepEqual([o.r, o.p, o.t, o.y, o.mode], [raw(0.25), raw(-0.1), rawThr(0.45), raw(0.05), 1024]);
    for (let i = 0; i < 4; i++) r.tick({ sw: ON });
    const t = decodeTelemetry(r.env.tx.at(-1));
    assert.equal(t.engaged, true);
    assert.equal(t.linkOk, true);
    assert.equal(t.seq, 1);
  },

  "co-pilot mask leaves throttle to the pilot"() {
    const r = makeRadio();
    r.tick({ sw: ON, thr: -200 });
    r.send({ mask: COPILOT, roll: 0.2, thr: 0.9 });
    const o = r.tick({ sw: ON, thr: -200 });
    assert.equal(o.r, raw(0.2));
    assert.equal(o.t, -200);
  },

  "ignores lines with a bad checksum or garbage"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    r.env.rxBuf += "$C,5,15,500,0,0,0,0*00\nhello\n$C,5,15,500\n";
    const o = r.tick({ sw: ON, ail: 7 });
    assert.equal(o.r, 7);
    r.tick({ sw: ON });
    r.tick({ sw: ON });
    assert.equal(decodeTelemetry(r.env.tx.at(-1)).rxBad, 3);
  },

  "reassembles a line split across reads and uses the latest command"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    const line = encodeCommand({ seq: 9, mask: ALL, roll: 0.3, pitch: 0, thr: 0.5, yaw: 0, hover: 0.4 });
    r.env.rxBuf += line.slice(0, 10);
    r.tick({ sw: ON });
    r.env.rxBuf += line.slice(10);
    r.send({ mask: ALL, roll: -0.3, thr: 0.5 });
    const o = r.tick({ sw: ON });
    assert.equal(o.r, raw(-0.3));
  },

  "pilot stick override hands that axis back until re-engaged"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    r.send({ mask: ALL, roll: 0.2, pitch: 0.1, thr: 0.5 });
    let o = r.tick({ sw: ON, ail: 400 });
    assert.equal(o.r, 400, "roll follows the pilot while overridden");
    assert.equal(o.p, raw(0.1), "other axes stay with the app");
    r.send({ mask: ALL, roll: 0.2, pitch: 0.1, thr: 0.5 });
    o = r.tick({ sw: ON, ail: 0 });
    assert.equal(o.r, 0, "override is latched after the stick recenters");
    r.tick({ sw: -1024 });
    r.tick({ sw: ON });
    r.send({ mask: ALL, roll: 0.2, pitch: 0.1, thr: 0.5 });
    o = r.tick({ sw: ON });
    assert.equal(o.r, raw(0.2), "re-engaging clears the override");
  },

  "throttle override is relative to the stick position when engaged"() {
    const r = makeRadio();
    r.tick({ sw: ON, thr: -300 });
    r.send({ mask: ALL, thr: 0.5 });
    let o = r.tick({ sw: ON, thr: -200 });
    assert.equal(o.t, rawThr(0.5), "small throttle movement does not override");
    r.send({ mask: ALL, thr: 0.5 });
    o = r.tick({ sw: ON, thr: 0 });
    assert.equal(o.t, 0, "moving throttle 25%+ takes over");
  },

  "failsafe: link loss while flying throttle descends, then idles, and stays latched"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    r.send({ mask: ALL, roll: 0.2, thr: 0.5, hover: 0.42 });
    r.tick({ sw: ON });
    // no packets for 0.33 s
    let o;
    for (let i = 0; i < 11; i++) o = r.tick({ sw: ON, ail: 30 });
    const expected = Math.min(rawThr(0.5), rawThr(0.42)) - 160;
    assert.equal(o.t, expected, "descends below the hover estimate");
    assert.equal(o.r, 30, "attitude returns to the pilot");
    assert.ok(r.env.haptics >= 1, "radio vibrates");
    for (let i = 0; i < 105; i++) o = r.tick({ sw: ON });
    assert.ok(o.t < expected && o.t > -1024, `ramping down, got ${o.t}`);
    for (let i = 0; i < 60; i++) o = r.tick({ sw: ON });
    assert.equal(o.t, -1024, "idle after the descent");
    r.send({ mask: ALL, roll: 0.2, thr: 0.5 });
    o = r.tick({ sw: ON });
    assert.equal(o.t, -1024, "failsafe stays latched when packets return");
    r.tick({ sw: -1024 });
    r.tick({ sw: ON });
    r.send({ mask: ALL, roll: 0.2, thr: 0.5 });
    o = r.tick({ sw: ON });
    assert.equal(o.t, rawThr(0.5), "switch off/on clears failsafe");
  },

  "failsafe with the app's throttle at idle stays at idle"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    r.send({ mask: ALL, thr: 0 });
    r.tick({ sw: ON });
    let o;
    for (let i = 0; i < 12; i++) o = r.tick({ sw: ON });
    assert.equal(o.t, -1024);
  },

  "engaging before the app connects keeps the pilot in charge, then hands over"() {
    const r = makeRadio();
    let o;
    for (let i = 0; i < 20; i++) o = r.tick({ sw: ON, ail: 5, thr: -1000 });
    assert.equal(o.r, 5);
    assert.equal(o.t, -1000);
    const t = decodeTelemetry(r.env.tx.at(-1));
    assert.equal(t.failsafe, false, "no failsafe without ever having a link");
    assert.equal(t.linkOk, false);
    r.send({ mask: ALL, roll: 0.1, thr: 0 });
    o = r.tick({ sw: ON, thr: -1000 });
    assert.equal(o.r, raw(0.1), "app takes over once packets arrive");
  },

  "an idle app (mask 0) never moves anything"() {
    const r = makeRadio();
    r.tick({ sw: ON });
    r.send({ mask: 0, roll: 0.5, pitch: 0.5, thr: 0.9, yaw: 0.5 });
    const o = r.tick({ sw: ON, ail: 1, ele: 2, thr: -1000, rud: 3 });
    assert.deepEqual([o.r, o.p, o.t, o.y], [1, 2, -1000, 3]);
  },

  "flight mode text with Betaflight's disarmed '*' or failsafe '!' still decodes"() {
    for (const fm of ["STAB*", "ACRO*", "!FS!", "WAIT*"]) {
      const r = makeRadio();
      r.env.sensors = { ...r.env.sensors, FM: fm };
      for (let i = 0; i < 12; i++) r.tick();
      const t = decodeTelemetry(r.env.tx.at(-1));
      assert.ok(t, `line with FM ${fm} decodes: ${r.env.tx.at(-1)}`);
      assert.equal(t.fm, fm);
    }
  },

  "survives missing sensors and a missing serial port"() {
    const r = makeRadio();
    r.env.sensors = {};
    for (let i = 0; i < 5; i++) r.tick();
    const t = decodeTelemetry(r.env.tx.at(-1));
    assert.equal(t.vbat, 0);
    assert.equal(t.fm, "");
  },
};

let failed = 0;
for (const [name, fn] of Object.entries(tests)) {
  try {
    fn();
    console.log(`  ok  ${name}`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name}\n      ${e.message}`);
  }
}
console.log(failed ? `\n${failed} failed` : `\nall ${Object.keys(tests).length} passed`);
process.exit(failed ? 1 : 0);
