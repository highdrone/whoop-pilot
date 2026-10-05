// The house the app flies, as one object (docs/HOME-DRONE.md, "Wave C as wired"). For the active house and mode it owns
// the 3D voxel map, the twins (the simulator's camera, which main.js's simulator world makes; the real drone's localization
// twin, made here), calib.json, vision localization (nav/splatloc.js), live depth (vision/depth.js) into avoidance, change
// detection and the voxels, the flight memory and the house edits that memory makes. One localizer, safety layer, mission
// runner, vision localizer, live depth and change detector serve the page's life and move from house to house; the
// per-house parts (voxels, twins, calib, memory) are let go on a house switch, and the memory, the localization twin and
// live depth follow the simulator/real switch, so nothing outlives the house or world it was made for.
// Live depth always feeds avoidance. Change detection, the voxels and "what the camera saw" take only video one rule
// (nav/changes.js changeTrust) trusts: the simulator's, or the real camera once calibrated and checked on the pad, with a
// fix under a second old and a sure pose.
// The simulator's flights change the 3D map only until the next real-mode use: they are never saved as the house's.
// "Ready to fly?" items from here (readiness(), merged into missions.preflightCheck so every way a mission starts refuses
// alike): the vision side's (nav/readiness.js, wired to live depth and the 3D map), the page's radio blocks
// (pageChecks()) on the real drone, and in "Rehearse the real flight" the real drone's vision rules.
// mods: { LOC, SAF, MIS, SPL, DEP, CHG (optional modules or null), HouseMemory, Twin, buildVoxels, voxelCentres, houseFrame,
// coverageReport }. store() -> Promise<house/store.js store>. edit(id, fn) applies a house edit (main.js's editHouse).
// Events: "log" { level, text }, "progress" { key, text, value } (text null: done), "built", "sync", "vox", "memory",
// "changes" ({ open, blocking }), "house" (memory edited it), "map" { house, map } (rebuilt), "calib", "twin" (the real
// drone's twin goes: let go of it), "takeoff", "landed" { freeAdded }, "flight" (a memory flight record, ended: its report),
// "ready" (the vision checklist changed), "pad" (the pad check's result: the pose may have moved).
import { Emitter } from "../util.js";
import { safetyWanted, padReset } from "./autonomy.js";

export const SESSION = { freshFix: 1000, refreshEvery: 5000 };

// A programming error's text (a TypeError and the like) is no news for the user: the plain line, the rest to the console.
const DEV_ERROR = /TypeError|ReferenceError|RangeError|Cannot read prop|is not a function|is not defined|of (undefined|null)\b/;
export const plainError = (text, plain) => (DEV_ERROR.test(text ?? "") ? (console.warn(text), plain) : text);

export class HomeSession extends Emitter {
  constructor({ ctl, perception, settings, store, mods, edit = null, alerts = null, tools = null, inspector = null, sim = () => null, pageChecks = () => [] }) {
    super();
    Object.assign(this, { ctl, perception, settings, store, mods, edit, alerts, tools, inspector, sim, pageChecks });
    Object.assign(this, { house: null, map: null, ortho: null, vox: null, twin: null, locTwin: null, calib: null, memory: null });
    Object.assign(this, { localizer: null, safety: null, missions: null, splat: null, depth: null, changes: null });
    Object.assign(this, { gen: 0, locGen: 0, locLoading: null, locFailed: null, memoryKey: null, memoryOffs: [], building: null, dbRun: null });
    Object.assign(this, { rebuildPending: false, voxPending: null, simTouched: false, flying: false, freeAtTakeoff: null, cov: null, voxWhy: "", depthReady: false });
    Object.assign(this, { readyAt: -Infinity, readyRefresh: null, trust: null, trustFor: null, pausedSaid: new Set() });
    this.offs = [ctl.on("tick", () => this.tick())];
  }

  mode() {
    return this.settings.get("mode") === "real" ? "real" : "sim";
  }
  // The real drone, or the simulator flying this house (not its demo apartment).
  inHouse() {
    return !!this.house && (this.mode() === "real" || this.sim()?.world?.kind === "house");
  }
  // The twin live depth and change detection render the expected view with: the simulator's, or the real drone's.
  depthTwin() {
    return !this.inHouse() ? null : this.mode() === "sim" ? this.twin : this.locTwin;
  }
  log(level, text) {
    this.emit("log", { level, text });
  }
  progress(key, text, value = null) {
    this.emit("progress", { key, text, value });
  }
  make(what, fn) {
    try {
      return fn() || null;
    } catch (e) {
      console.error(e);
      this.log("error", `${what} didn't start: ${e.message}`);
      return null;
    }
  }

  // ---------------------------------------------------------------- the house

  // A house switch (null: none). The per-house parts go first; the page's parts move to the new house; its calib.json and
  // its 3D map load (the map in the background: flights use the 2.5D map meanwhile). -> Promise of both.
  setHouse(house, map, ortho = null) {
    const gen = ++this.gen;
    this.missions?.stop?.("the house changed");
    this.dropLocTwin();
    this.setVox(null);
    Object.assign(this, { house, map, ortho, calib: null, rebuildPending: false, voxPending: null, simTouched: false, locFailed: null, cov: null });
    this.pausedSaid.clear();
    if (house) this.build();
    this.applyCalib();
    this.sync();
    return house ? Promise.all([this.loadCalib(gen), this.load3D()]) : Promise.resolve();
  }

  build() {
    const { house, map, ctl, settings, perception, mods: M } = this;
    if (this.localizer) this.localizer.setMap(map, house);
    else if (M.LOC) {
      this.localizer = this.make("The position estimate", () => new M.LOC.Localizer({ ctl, map, house, settings }));
      this.localizer?.on("pose", () => this.syncSafety()); // a reset on the pad (or a fix) can attach the safety layer
      this.localizer?.on("conflict", (c) => this.conflictSaid(c));
    }
    if (this.safety) this.safety.setMap(map, house);
    else if (this.localizer && M.SAF) {
      this.safety = this.make("The safety layer", () => new M.SAF.Safety({ map, localizer: this.localizer, settings, house, alerts: this.alerts })); // acts only as ctl.safety
      this.safety?.on("trip", ({ reason }) => this.log("error", `Safety: ${reason}.`));
    }
    if (!this.splat && this.localizer && M.SPL) {
      this.splat = this.make("Vision localization", () => new M.SPL.SplatLocalizer({ localizer: this.localizer, ctl, settings, house, map }));
      if (this.splat) {
        this.splat.enabled = false;
        this.splat.on("status", (s) => this.splatSaid(s));
        this.splat.attach(perception);
        const off = this.splat.checklist?.()?.on?.("change", () => this.emit("ready"));
        if (off) this.offs.push(off);
      }
    }
    this.splat?.setHouse(house, map);
    this.makeTrust();
    if (this.changes) this.changes.setMap(map, house);
    else if (this.safety && M.CHG)
      this.changes = this.make("Change detection", () => new M.CHG.ChangeDetector({ map, house, memory: this.memory, avoid: this.safety.avoid,
        render: (pose, view) => this.depthTwin()?.pixels(pose, view), trust: this.trust }));
    if (!this.depth && this.localizer && M.DEP) {
      this.depth = this.make("Live depth", () => new M.DEP.LiveDepth({ perception, localizer: this.localizer, ctl, settings, twin: this.depthTwin(), calib: this.calib,
        osd: this.splat?.osd ?? null }));
      this.depth?.on("depth", (f) => this.depthFrame(f));
      this.depth?.on("status", ({ text, why }) => {
        if (this.ctl.safety === this.safety) this.safety?.avoid.setDepthStatus(why ?? "");
        if (text && why !== "stopped") this.log("info", text);
      });
      this.depth?.on("error", (m) => this.log("error", plainError(m, "Live depth hit a problem: the 3D map's speed limits still apply.")));
    }
    // the vision checklist sees live depth's model, the 3D map and how to build it
    this.splat?.checklist?.({ depth: this.depth, has3D: () => !!this.vox, prepare3D: (o) => this.load3D(o) });
    if (this.missions) this.missions.setHouse({ house, map, memory: this.memory, splat: this.splat, depth: this.depth });
    else if (this.localizer && M.MIS) {
      this.missions = this.make("Missions", () => new M.MIS.MissionRunner({ ctl, map, house, localizer: this.localizer, perception, alerts: this.alerts, settings,
        memory: this.memory, splat: this.splat, depth: this.depth, readiness: () => this.readiness() }));
      this.missions?.on("done", () => this.rebuildPending && this.rebuildWhenLanded());
    }
    this.emit("built", this);
  }

  // Vision's status: a download is one line that ends on its last words (100%), the rest log lines in plain words.
  splatSaid(s) {
    if (s.progress == null) return s.text && this.log(s.level === "warn" ? "error" : "info", plainError(s.text, "Vision localization hit a problem and is trying again."));
    const done = s.progress >= 1;
    if (done && this.visionDone === s.text) return; // its 100% said again: one line is enough
    this.visionDone = done ? s.text : null;
    this.progress("vision", s.text, Math.min(1, s.progress));
    if (done) this.progress("vision", null);
  }

  // The one rule for what video may change (nav/changes.js changeTrust): change detection's, the voxels' and "what the
  // camera saw"'s; made again if vision or the localizer is replaced. Without change detection only the simulator's exact
  // position is trusted.
  makeTrust() {
    const { settings, splat, localizer, mods: M } = this;
    if (this.trust && this.trustFor?.splat === splat && this.trustFor?.localizer === localizer) return;
    this.trustFor = { splat, localizer };
    this.trust = M.CHG?.changeTrust ? M.CHG.changeTrust({ settings, splat, localizer, calib: () => this.calib })
      : () => (this.mode() === "sim" && !splat?.enabled ? { ok: true, why: "" } : { ok: false, why: "change detection isn't part of this version" });
    this.changes?.setTrust?.(this.trust);
  }

  // A disagreement between the camera and the motion estimate, said at most every 20 s, by how much and in what.
  conflictSaid(c, now = performance.now()) {
    if (now - (this.conflictAt ?? -Infinity) < 20000) return;
    this.conflictAt = now;
    const p = this.localizer.pose(), turn = Number.isFinite(c?.yaw) ? Math.abs(Math.atan2(Math.sin(c.yaw - p.yaw), Math.cos(c.yaw - p.yaw))) * (180 / Math.PI) : 0;
    const by = (c?.apart ?? 0) >= 0.05 ? `by ${c.apart.toFixed(2)} m` : turn >= 1 ? `by ${Math.round(turn)}° in heading` : "slightly";
    this.log("error", `The camera and the drone's own motion estimate disagree ${by}: it goes on with wider margins until a view from another spot settles it.`);
  }

  // ---------------------------------------------------------------- after every switch

  // Mode, simulator world, twin or a setting changed: who drives the position, what vision and live depth use, the
  // memory for this house and world, and what Claude's tools may fly.
  sync() {
    const on = this.inHouse(), real = this.mode() === "real", s = this.settings, L = this.localizer, sp = this.splat;
    const vision = !!sp && on && (real ? s.get("locVision") !== false : s.get("simLoc") === "vision");
    if (L) {
      const sim = this.sim(), simHouse = on && !real;
      L.setTruth(simHouse ? () => ({ x: sim.drone.x, y: sim.drone.y, z: sim.drone.z, yaw: sim.drone.yaw }) : null);
      L.setSource(simHouse && !vision ? "truth" : s.get("locSource") || "fused");
    }
    if (sp) {
      sp.enabled = vision;
      sp.setTwin(vision ? (real ? this.locTwin : this.twin) : null);
    }
    if (L) L.expectVision = vision; // a flight whose vision never locks on counts its stale and lost clocks from take-off
    if (on && real && !this.locTwin && (vision || s.get("avoid") !== false)) this.loadLocTwin();
    if (vision && sp?.twin) this.ensureDb();
    if (real) this.refreshReadiness();
    if (real && this.simTouched) this.cleanVox();
    this.syncSafety();
    this.depth?.setTwin(this.depthTwin());
    const wantDepth = on && s.get("avoid") !== false && !!this.depthTwin();
    if (this.depth && wantDepth && !this.depth.estimator) this.startDepth();
    else if (this.depth && !wantDepth) {
      if (this.depth.estimator) (this.depth.stop(), (this.depthReady = false));
      if (on) this.safety?.avoid.setDepthStatus(!this.depthTwin() ? "no 3D twin of the house is loaded yet" : "live depth is off in Settings");
    }
    this.syncMemory();
    const { house, map, missions, memory, splat } = this;
    this.tools?.setHouse(on && missions ? { house, map, missions, localizer: L, save: this.edit && ((h) => this.edit(h.id, () => {})), memory, vox: this.vox ?? null, splat } : {});
    this.emit("sync");
  }

  // Every pose (and every switch): the safety layer is the controller's filter only in the house, with a position.
  syncSafety() {
    const { ctl, safety, localizer } = this;
    ctl.safety = safety && safetyWanted({ inHouse: this.inHouse(), sim: this.mode() === "sim", localizer }) ? safety : null;
  }

  startDepth() {
    this.depthStart ??= this.depth.start((l, n) => this.progress("depth", `Downloading the depth model (once): ${Math.round((100 * l) / n)}%`, l / n))
      .then(() => (this.progress("depth", null), (this.depthReady = !!this.depth?.estimator), this.refreshReadiness({ force: true })))
      .catch((e) => {
        this.progress("depth", null);
        this.safety?.avoid.setDepthStatus(e.message);
        this.log("error", `Live depth is off (${e.message}); the 3D map's speed limits still apply.`);
      })
      .finally(() => (this.depthStart = null));
  }

  // The simulator's twin camera (main.js makes it with the simulator's world; null while it loads or in the demo).
  setSimTwin(twin) {
    this.twin = twin;
    this.sync();
  }

  // ---------------------------------------------------------------- the real drone's localization twin and database

  async loadLocTwin() {
    const house = this.house, M = this.mods;
    if (!house?.splatFile || !M.Twin || this.locLoading === house || this.locFailed === house) return;
    const gen = ++this.locGen;
    this.locLoading = house;
    try {
      const bytes = await (await this.store()).readFile(house.id, house.splatFile);
      if (!bytes) throw new Error("the scan file is missing");
      const twin = await M.Twin.create({ splat: bytes, house });
      if (gen !== this.locGen || house !== this.house || this.mode() !== "real") return void twin.dispose?.();
      this.locTwin = twin;
      this.sync();
    } catch (e) {
      if (gen === this.locGen) {
        this.locFailed = house;
        this.log("error", `The 3D scan didn't load for the real drone (no vision position or live depth): ${e.message}`);
      }
    } finally {
      if (this.locLoading === house) this.locLoading = null;
    }
  }

  dropLocTwin() {
    this.locGen++;
    const t = this.locTwin;
    this.locTwin = null;
    this.locLoading = null;
    if (!t) return;
    if (this.splat?.twin === t) this.splat.setTwin(null);
    this.depth?.setTwin(null);
    this.emit("twin", null); // the 3D view and calibration let go of it before it goes
    t.dispose?.();
  }

  // The relocalization database for this house and lens: about 30 s on the ground, paused while flying, kept in OPFS.
  ensureDb() {
    const sp = this.splat;
    if (!sp?.ensureDb || this.dbRun || sp.db?.meta?.key === sp.dbKey?.()) return this.dbRun;
    const text = ({ done = 0, total = 0, paused }) => `Building the position database: ${done} of ${total || "?"} views${paused ? " (paused while flying)" : ""}`, twin = sp.twin;
    const why = (e) => (sp.twin !== twin ? "the 3D scan was swapped meanwhile, so it is built again from the new one"
      : plainError(e.message, "the vision worker hit a problem (Settings → House → Build the position database tries again)"));
    this.dbRun = sp.ensureDb({ onProgress: (p) => this.progress("db", text(p), p.total ? p.done / p.total : null) })
      .then((db) => (this.progress("db", null), this.emit("sync"), db))
      .catch((e) => (this.progress("db", null), this.log(sp.twin !== twin ? "info" : "error", `The position database wasn't built: ${why(e)}.`), null))
      .finally(() => (this.dbRun = null));
    return this.dbRun;
  }

  // More "Ready to fly?" items, merged by id into missions.preflightCheck() (so a mission from a button, a voice command
  // or Claude refuses on the same blocks): the vision side's (model downloads, the position database, calibration, the
  // pad check), the page's radio blocks on the real drone (ANGLE mode, the radio script: missions can't see them), and in
  // a rehearsal the real drone's vision rules (its items block, a fix under a second old, the simulator's scan camera).
  readiness() {
    let items = [];
    try {
      items = this.splat?.readiness?.({ mode: this.mode() }) ?? [];
    } catch {}
    if (this.rehearsing()) items = [...items.map((it) => (it.ok ? it : { ...it, level: "block" })), ...this.rehearsalChecks()];
    return this.mode() === "real" ? [...items, ...(this.pageChecks() ?? [])] : items;
  }

  // "Rehearse the real flight": the simulator in this house, its position from its own camera.
  rehearsing() {
    return this.mode() === "sim" && this.settings.get("simRehearse") === true && this.inHouse() && !!this.splat?.enabled;
  }

  rehearsalChecks() {
    const q = this.localizer?.fixQuality?.(), fresh = q?.visionAge < SESSION.freshFix, out = [];
    if (!this.sim()?.twin) out.push({ id: "sim-camera", ok: false, level: "block", when: "before",
      text: "The rehearsal needs the simulator's 3D-scan camera, which isn't running: without it the drone can't find itself from its camera." });
    out.push({ id: "fix", ok: fresh, level: fresh ? "info" : "block", when: "battery", text: fresh ? `Vision fixes: ${q.rate.toFixed(1)} a second.`
      : "No position fix from the camera in the last second, so the real drone wouldn't start either: let it see the room (not a blank wall), or wait a moment." });
    return out;
  }

  // What the vision checklist knows of the downloads and the stored position database (Cache Storage, OPFS): asked at most
  // every SESSION.refreshEvery ms while the list shows, and when live depth has loaded.
  refreshReadiness({ force = false } = {}) {
    const r = this.splat?.checklist?.(), now = performance.now();
    if (!r?.refresh || this.readyRefresh || (!force && now - this.readyAt < SESSION.refreshEvery)) return;
    this.readyAt = now;
    this.readyRefresh = r.refresh().catch(() => null).finally(() => (this.readyRefresh = null));
  }

  // ---------------------------------------------------------------- the 3D map

  setVox(vox) {
    this.vox = vox;
    this.map?.setVoxels(vox);
    this.cov = null;
    this.emit("vox", vox);
  }

  // The stored 3D map, else one built (11-35 s) from the scan's centres and a twin of it (for space only renders vouch for)
  // and stored. Never while flying: it waits for the landing. -> the VoxelMap, or null (the 2.5D map then).
  load3D({ rebuild = false } = {}) {
    const { house, map, mods: M } = this, gen = this.gen;
    if (!house || !map || !M.buildVoxels) return Promise.resolve(null);
    if (this.ctl.isFlying()) {
      this.voxPending = { rebuild };
      this.log("info", "I'll build the 3D map after landing.");
      return Promise.resolve(null);
    }
    if (this.building?.gen === gen) return this.building.p;
    const step = (text, value = null) => this.progress("map3d", text, value), live = () => gen === this.gen;
    const p = (async () => {
      try {
        const s = await this.store();
        let vox = rebuild ? null : await s.loadVoxels(house);
        if (vox) await vox.refresh?.();
        else {
          step("Building the 3D map: reading the scan…", 0);
          const bytes = house.splatFile && (await s.readFile(house.id, house.splatFile));
          if (!live()) return null;
          const centres = bytes ? await M.voxelCentres(bytes, house.splatFile, M.houseFrame({ f: house.frame.f, Yf: house.frame.Yf })) : null;
          let twin = null;
          try {
            twin = bytes && M.Twin ? await M.Twin.create({ splat: new Blob([bytes]), house }) : null;
          } catch {
            this.log("info", "The 3D map is built without renders of the scan, so more of the house stays unknown.");
          }
          try {
            if (!live()) return null;
            const text = (q) => (/^3D map/.test(q.text ?? "") ? q.text : `Building the 3D map: ${q.text ?? q.phase ?? "working"}`);
            vox = await M.buildVoxels({ house, map, centres, twin, onProgress: (q) => step(text(q), q.total ? q.done / q.total : null) });
          } finally {
            await twin?.dispose?.();
          }
          if (!live()) return null;
          await s.saveVoxels(house, vox).catch(() => this.log("error", "Couldn't save the 3D map (storage full?): it will be built again next time."));
        }
        if (!live()) return null;
        step(null);
        const usable = vox.built?.usable !== false;
        this.voxWhy = usable ? "" : vox.built.why;
        if (!usable) this.log("error", `The 3D map can't tell free space (${vox.built.why}), so the drone flies on the floor-plan map.`);
        this.simTouched = false;
        this.setVox(usable ? vox : null);
        this.sync();
        return this.vox;
      } catch (e) {
        if (!live()) return null;
        step(null);
        this.voxWhy = e.message;
        this.setVox(null);
        this.log("error", `3D map unavailable, so the drone flies on the floor-plan map (${e.message}).`);
        return null;
      } finally {
        if (this.building?.gen === gen) this.building = null;
      }
    })();
    this.building = { gen, p };
    return p;
  }

  // The simulator's flights changed the 3D map: the stored one again (they are not the house's).
  cleanVox() {
    if (!this.simTouched) return Promise.resolve(this.vox);
    this.simTouched = false;
    return this.load3D();
  }

  saveVox() {
    const { house, vox } = this;
    if (!house || !vox || this.simTouched) return Promise.resolve();
    return this.store().then((s) => s.saveVoxels(house, vox)).catch((e) => this.log("error", `Couldn't save the 3D map: ${e.message}`));
  }

  // coverageReport for the current 3D map: made again when the map changed (and the last one is older than maxAge ms),
  // never in the air (it takes a few hundred ms of the control loop's thread).
  coverage({ maxAge = 0 } = {}) {
    const { vox, map, house } = this, now = performance.now();
    if (!vox || !map || !this.mods.coverageReport) return null;
    if (this.cov?.vox === vox && (this.cov.version === vox.version || now - this.cov.at < maxAge || this.ctl.isFlying())) return this.cov.report;
    if (this.ctl.isFlying()) return null;
    try {
      this.cov = { vox, version: vox.version, at: now, report: this.mods.coverageReport({ house, map, vox }) };
      return this.cov.report;
    } catch (e) {
      this.log("error", `The coverage report failed: ${e.message}`);
      return null;
    }
  }

  // ---------------------------------------------------------------- live depth, trusted or not

  // Whether video may change what the house is (changes, voxels, what the camera saw), by makeTrust()'s rule: the
  // simulator's with its exact position; while vision drives the position (the real drone, a rehearsal), a fix less than a
  // second old and a sure pose, and on the real drone the camera calibrated and checked on the pad too. -> "" or why not.
  untrusted(f) {
    return this.trust?.(f)?.why ?? "";
  }

  depthFrame(f) {
    const { safety, ctl, vox } = this;
    if (!safety || ctl.safety !== safety) return;
    safety.avoid.ingest(f);
    const why = this.untrusted(f);
    if (why) return this.paused(why);
    this.changes?.ingest(f);
    if (!vox) return;
    if (this.mode() === "sim") this.simTouched = true;
    const mapDepth = this.mods.DEP?.mapDepth;
    vox.integrate(f.pose, { ...f, depth: mapDepth ? mapDepth(f) : f.depth }, f.lens, { t: f.t });
    vox.markSeen(f.pose, f.lens, Date.now(), { width: f.width, height: f.height, quality: { age: performance.now() - f.t, luma: this.perception.latest?.luma } });
  }

  // Video the rule doesn't trust changes nothing: the memory's flight report says why "what the camera saw" wasn't
  // measured, the log says each reason once (again after each take-off or house switch).
  paused(why) {
    this.memory?.seenPaused?.(why);
    if (this.pausedSaid.has(why)) return;
    this.pausedSaid.add(why);
    this.log("info", `Change detection paused: ${why}.`);
  }

  // ---------------------------------------------------------------- calibration

  async loadCalib(gen) {
    try {
      const s = await this.store();
      if (this.splat) this.splat.store = s;
      const b = await s.readFile(this.house.id, "calib.json");
      if (gen !== this.gen) return;
      this.calib = b ? JSON.parse(new TextDecoder().decode(b)) : null;
    } catch (e) {
      if (gen === this.gen) this.log("error", `The camera calibration didn't load: ${e.message}`);
    }
    if (gen === this.gen) this.applyCalib();
  }

  applyCalib() {
    const c = this.calib;
    this.splat?.setCalib(c);
    this.depth?.setCalib(c);
    this.perception.geometry = { ...this.perception.geometry, aspect: c?.width && c?.height ? c.width / c.height : undefined };
    this.emit("calib", c);
  }

  async saveCalib(calib) {
    const house = this.house;
    if (!house) return;
    await (await this.store()).writeFile(house.id, "calib.json", JSON.stringify(calib));
    if (house !== this.house) return;
    this.calib = calib;
    this.applyCalib();
    this.refreshReadiness({ force: true });
  }

  // The pad check can run: the real drone's vision position on, its video showing.
  canPadCheck() {
    const src = this.perception.source;
    return !!this.splat?.enabled && this.mode() === "real" && !!src?.ready?.() && src.kind !== "sim";
  }

  // "The drone is on its home pad": the position there (ground only), then, with the real drone's video, the camera's
  // check against the scan from the pad (its status event says how it went; "pad" with the result: it may re-seat the
  // pose on the camera's). -> "" or why not.
  pad() {
    const why = padReset({ ctl: this.ctl, missions: this.missions, localizer: this.localizer, pad: this.house?.home });
    if (!why && this.canPadCheck())
      this.splat.verifyOnPad?.().then((r) => this.emit("pad", r ?? null), (e) => this.log("error", `The pad check failed: ${plainError(e.message, "it hit a problem: press the pad button again")}`));
    return why ?? "";
  }

  // ---------------------------------------------------------------- memory

  async syncMemory() {
    const world = this.mode(), key = this.inHouse() ? `${this.house.id}|${world}` : null, M = this.mods.HouseMemory, s = this.settings;
    if (key !== this.memoryKey) {
      this.memoryKey = key;
      this.memoryOffs.forEach((f) => f());
      this.memoryOffs = [];
      this.memory?.setHouse({});
      this.memory = null;
      this.attachMemory();
      this.emit("memory", null);
      if (!key || !M) return;
      let m = null;
      try {
        m = await M.open(this.house.id, { world, keepDays: s.get("memoryDays"), keepFlights: s.get("memoryFlights") });
      } catch (e) {
        if (key === this.memoryKey) this.log("error", `The flight memory didn't open: ${e.message}`);
      }
      if (key !== this.memoryKey || !m) return;
      this.memory = m;
      if (!m.stored) this.log("info", "The flight memory can't be saved in this browser, so it lasts until the page closes.");
      this.memoryOffs = [
        m.on("house", (e) => this.houseChanged(e)),
        m.on("error", (msg) => this.log("error", msg)),
        m.on("change", () => this.emit("changes", this.openChanges())),
        m.on("annotate", () => this.emit("changes", this.openChanges())),
        m.on("flight", (f) => f?.t1 != null && this.emit("flight", f)),
      ];
      this.emit("memory", m);
    }
    this.attachMemory();
  }

  attachMemory() {
    const m = this.memory;
    m?.setHouse({ house: this.house, map: this.map, vox: this.vox ?? null, save: this.mode() === "real" ? (h) => this.store().then((st) => st.saveHouse(h)) : null });
    this.alerts?.setMemory?.(m);
    this.inspector?.setMemory?.(m);
    this.changes?.setMemory?.(m);
    if (this.house) this.missions?.setHouse({ house: this.house, map: this.map, memory: m, splat: this.splat, depth: this.depth });
  }

  // Suspected changes: { open, blocking } (memory.openChanges: blocking = the drone keeps clear of it now).
  openChanges() {
    const m = this.memory;
    if (m?.openChanges) return m.openChanges();
    const open = m?.changes?.filter((c) => c.status === "suspected").length ?? 0;
    return { open, blocking: open };
  }

  async houseChanged({ rebuild, saved } = {}) {
    await saved;
    this.emit("house", this.house);
    this.saveVox();
    if (rebuild) this.rebuildWhenLanded();
  }

  // A door changed since the map was built: build the HomeMap again from the house, after landing.
  rebuildWhenLanded() {
    if (this.ctl.isFlying() || this.missions?.busy) this.rebuildPending = true;
    else return this.rebuildMap();
  }

  async rebuildMap() {
    this.rebuildPending = false;
    const gen = this.gen, house = this.house;
    if (!house) return;
    const map = await (await this.store()).loadMap(house);
    if (gen !== this.gen) return;
    map.setVoxels(this.vox ?? null);
    this.map = map;
    this.localizer?.setMap(map, house);
    this.safety?.setMap(map, house);
    this.changes?.setMap(map, house);
    this.splat?.setHouse(house, map);
    this.attachMemory();
    this.cov = null;
    this.emit("map", { house, map });
    this.sync();
  }

  // ---------------------------------------------------------------- take-off and landing

  tick() {
    const flying = this.ctl.isFlying();
    if (flying === this.flying) return;
    this.flying = flying;
    if (!flying) return this.landed();
    this.freeAtTakeoff = this.vox?.counts?.().free ?? null;
    this.pausedSaid.clear();
    this.emit("takeoff");
  }

  // What flights alone found occupied is forgotten (voxels.js forgetFlight), what four frames agreed is free stays; the map
  // is stored (not the simulator's), and work that waited for the landing runs.
  landed() {
    const vox = this.vox;
    let freeAdded = null;
    if (vox?.forgetFlight) {
      vox.forgetFlight();
      if (this.freeAtTakeoff != null) freeAdded = (vox.counts().free - this.freeAtTakeoff) * vox.res ** 3;
      if (freeAdded > 0.01 && !this.simTouched) this.log("info", `This flight added about ${freeAdded.toFixed(2)} m³ of known free space to the 3D map.`);
      this.saveVox();
      this.cov = null;
    }
    this.freeAtTakeoff = null;
    if (this.rebuildPending) this.rebuildWhenLanded();
    if (this.voxPending) {
      const o = this.voxPending;
      this.voxPending = null;
      this.load3D(o);
    }
    this.emit("landed", { freeAdded });
  }

  dispose() {
    this.offs.forEach((f) => f());
    this.memoryOffs.forEach((f) => f());
    this.memory?.setHouse({});
    this.dropLocTwin();
    this.missions?.dispose?.();
    this.splat?.dispose?.();
    this.depth?.stop?.();
    this.changes?.dispose?.();
    this.localizer?.dispose?.();
  }
}
