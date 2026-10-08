// Settings → House: the captures the server can read (/house-files/, newest first) with "Use this capture", a folder
// picker for captures elsewhere, the houses stored in this browser (house/store.js: make one active, rename its rooms,
// delete it), and for the active one its home pad, your landmarks and no-fly zones, the ones Claude's survey added (each
// removable), and what the import checked.
// app: { store() -> Promise<store>, active() -> id, use(id), edit(id, fn(house)), session() -> Promise<session|null> }.
import { Emitter } from "../util.js";
import { importCapture, dirSource } from "../house/import.js";
import { h, fmtBytes, fmtDate, confirmButton, nextPaint } from "./dom.js";

const drop = (list, pred) => {
  const i = list.findIndex(pred);
  if (i >= 0) list.splice(i, 1);
};
const SPLAT = /(^|\/)(splat\.(spz|splat|ply)|splat-centres\.bin)$/;

// A capture served by tools/whoop.mjs at /house-files/<project>/..., in house/import.js's source shape. Big files
// stream with onProgress({ path, loaded, total }). The importer probes for optional files: folder listings (fetched
// once each) answer for the ones a capture doesn't have, so nothing is asked for that isn't there.
export function houseSource(project, onProgress = () => {}, fetchImpl = (u) => fetch(u)) {
  const url = (path, dir) => "/house-files/" + [project.id, ...path.split("/").filter(Boolean)].map(encodeURIComponent).join("/") + (dir ? "/" : "");
  const listings = new Map();
  const entries = (dir) => {
    if (!listings.has(dir)) listings.set(dir, fetchImpl(url(dir, true)).then((r) => (r.ok ? r.json() : null)).then((j) => j?.entries ?? null, () => null));
    return listings.get(dir);
  };
  // false when a listing on the way says the path isn't there (as a file, or as a folder for dir); true otherwise.
  const has = async (path, dir = false) => {
    const parts = path.split("/").filter(Boolean);
    for (let i = 0; i < parts.length; i++) {
      const list = await entries(parts.slice(0, i).join("/"));
      if (!list) return true;
      const e = list.find((x) => x.name === parts[i]);
      if (!e || (e.kind && e.kind !== (i < parts.length - 1 || dir ? "dir" : "file"))) return false;
    }
    return true;
  };
  return {
    name: project.id,
    async read(path) {
      if (!(await has(path))) return null;
      const r = await fetchImpl(url(path)).catch(() => null);
      if (!r?.ok) return null;
      const total = Number(r.headers.get("content-length")) || 0;
      if (!r.body || total < 1e6) return r.arrayBuffer();
      const out = new Uint8Array(total), reader = r.body.getReader();
      let loaded = 0;
      for (let step = 0; ; ) {
        const { done, value } = await reader.read();
        if (done) break;
        out.set(value, loaded);
        loaded += value.length;
        if (loaded >= step || loaded === total) {
          onProgress({ path, loaded, total });
          step = loaded + total / 100;
        }
      }
      return out.buffer;
    },
    list: async (path) => ((await has(path, true)) ? (await entries(path.split("/").filter(Boolean).join("/")))?.map((e) => e.name) ?? null : null),
  };
}

// What the import found, in plain words.
export function checkSummary(house) {
  const out = [], c = house.checks ?? {}, f = house.frame?.f ?? 1;
  if (f !== 1) out.push(`Distances use your tape-measure checks (they ${f < 1 ? "shrink" : "stretch"} the scan by ${Math.abs((f - 1) * 100).toFixed(1)}%).`);
  else if (house.source?.kind === "sitespec") out.push("No tape-measure correction was found: distances rely on the AprilTags' printed size.");
  if (c.scale?.scale) out.push(`The scan's walls and the floor plan agree to ${Math.abs((c.scale.scale - 1) * 100).toFixed(1)}% (${c.scale.inliers} walls compared).`);
  const tags = house.tags?.length ?? 0;
  if (tags) out.push(`${tags} AprilTag${tags === 1 ? "" : "s"} in the scan${house.home?.source === "tag 0" ? "; tag 0 is the home pad" : ""}.`);
  const doors = house.doors ?? [];
  if (doors.length) out.push(`${doors.filter((d) => d.passable).length} of ${doors.length} doorways are wide and tall enough to fly through.`);
  return out;
}

export class HousePanel extends Emitter {
  constructor(root, { app }) {
    super();
    this.root = root;
    this.app = app;
    this.busy = false;
    this.open = new Set(); // houses whose room list is showing
    const $ = (s) => root.querySelector(s);
    Object.assign(this, { captures: $("#captureList"), houses: $("#houseList"), box: $("#importBox"), bar: $("#importBar"), text: $("#importText"), warnings: $("#importWarnings") });
    const pick = $("#btnPickFolder");
    if (!window.showDirectoryPicker) Object.assign(pick, { disabled: true, title: "This browser can't open folders. Use Chrome." });
    pick.addEventListener("click", () => this.pickFolder());
  }

  async refresh() {
    await Promise.all([this.listCaptures(), this.listHouses()]);
  }

  async listCaptures() {
    const box = this.captures;
    const session = await this.app.session();
    if (!session) {
      box.replaceChildren(h("p", { class: "note" }, "Start Whoop Pilot with start.command (it needs Node.js) to see your SpaceBunny and Spacial captures here, or choose a capture folder below."));
      return;
    }
    const projects = await fetch("/house-files/").then((r) => r.json()).then((j) => j.projects ?? [], () => null);
    if (!projects?.length) {
      const none = "No captures yet in your SpaceBunny, SiteSpec or Spacial Projects folders or Whoop Pilot's captures folder.";
      box.replaceChildren(h("p", { class: "note" }, projects ? none : "Couldn't list your captures."));
      return;
    }
    const stored = new Set((await this.stored()).map((x) => x.name));
    box.replaceChildren(...projects.map((p) => h("article", { class: "capture" },
      p.thumbnail ? h("img", { src: p.thumbnail, alt: "", loading: "lazy", width: 72, height: 54 }) : h("div", { class: "thumb" }),
      h("div", { class: "meta" },
        h("strong", {}, p.name),
        h("small", {}, [fmtDate(p.date), p.splat ? `3D scan ${fmtBytes(p.splat.size)}` : "no 3D scan yet", p.app ?? (p.kind === "spacial" ? "Spacial" : "SiteSpec")].join(" · ")),
        stored.has(p.name) && h("small", { class: "badge" }, "In this browser"),
        !p.ready && h("small", { class: "warn-text" }, `Not finished: open it in ${p.app === "Spacial" ? "Spacial" : "SpaceBunny"} until it has a 3D scan.`),
      ),
      h("button", { type: "button", class: "btn", disabled: !p.ready || this.busy, onclick: () => this.importFrom(houseSource(p, (e) => this.progress(e)), p.name, p.splat?.size) },
        stored.has(p.name) ? "Import again" : "Use this capture"),
    )));
  }

  async stored() {
    try {
      return await (await this.app.store()).listHouses();
    } catch {
      return [];
    }
  }

  async pickFolder() {
    let dir;
    try {
      dir = await window.showDirectoryPicker({ id: "whoop-capture", mode: "read" });
    } catch (e) {
      if (e.name !== "AbortError") this.stage(`Couldn't open that folder: ${e.message}`, "error");
      return;
    }
    await this.importFrom(dirSource(dir), dir.name);
  }

  progress({ path, loaded, total }) {
    if (!SPLAT.test(path)) return;
    this.stage(`Copying the 3D scan: ${fmtBytes(loaded)} of ${fmtBytes(total)}`, "busy", loaded / total);
  }

  stage(text, state = "busy", value = null) {
    this.box.hidden = false;
    this.box.dataset.state = state;
    this.text.textContent = text;
    this.bar.hidden = state !== "busy";
    if (value == null) this.bar.removeAttribute("value");
    else this.bar.value = value;
  }

  async importFrom(src, name) {
    if (this.busy) return;
    this.busy = true;
    this.warnings.replaceChildren();
    this.root.querySelectorAll(".capture button").forEach((b) => (b.disabled = true));
    let building = false;
    const source = {
      ...src,
      read: async (path) => {
        if (!building) this.stage(SPLAT.test(path) ? "Copying the 3D scan…" : "Reading the floor plan and the capture's notes…");
        const bytes = await src.read(path);
        if (bytes && SPLAT.test(path) && !building) {
          building = true;
          this.stage("Building the flight map from the 3D scan (a few seconds)…");
          await nextPaint();
        }
        return bytes;
      },
    };
    try {
      this.stage(`Opening “${name}”…`);
      const result = await importCapture(source, {});
      this.stage("Saving the house in this browser…");
      await nextPaint();
      const store = await this.app.store();
      await store.saveImport(result);
      const { house, warnings } = result;
      this.stage(`Imported “${house.name}”: ${house.rooms.length} room${house.rooms.length === 1 ? "" : "s"}, ${house.doors.length} doorways.${warnings.length ? " Please check:" : ""}`, "done");
      this.warnings.replaceChildren(...warnings.map((w) => h("li", {}, w.text)), ...checkSummary(house).map((t) => h("li", { class: "ok" }, t)));
      await this.app.use(house.id);
    } catch (e) {
      console.error(e);
      this.stage(`The import didn't work: ${e.message}`, "error");
    } finally {
      this.busy = false;
      await this.refresh();
    }
  }

  async listHouses() {
    const list = await this.stored(), active = this.app.active();
    if (!list.length) {
      this.houses.replaceChildren(h("p", { class: "note" }, "No house yet. Use a capture above; the house and its 3D scan are then kept in this browser."));
      return;
    }
    const store = await this.app.store();
    const rows = [];
    for (const x of list) {
      const on = x.id === active, house = on || this.open.has(x.id) ? await store.loadHouse(x.id) : null;
      rows.push(h("article", { class: `house${on ? " active" : ""}` },
        h("div", { class: "house-h" },
          h("div", { class: "meta" }, h("strong", {}, x.name), h("small", {}, `${x.rooms} room${x.rooms === 1 ? "" : "s"} · saved ${fmtDate(x.saved)}`)),
          on ? h("span", { class: "badge on" }, "Active") : h("button", { type: "button", class: "btn", onclick: () => this.use(x.id) }, "Use"),
          h("button", { type: "button", class: "btn ghost", "aria-expanded": String(!!house), onclick: () => this.toggle(x.id) }, house ? "Close" : "Rooms…"),
          confirmButton("Delete", "Delete for good?", () => this.remove(x.id)),
        ),
        house && this.details(house, on),
      ));
    }
    if (!store.persisted)
      rows.push(h("p", { class: "note" }, "Chrome may clear stored houses when the disk runs low. Installing Whoop Pilot as an app (Chrome menu → Cast, save and "
        + "share) makes that much less likely."));
    this.houses.replaceChildren(...rows);
  }

  details(house, active) {
    const edit = (fn) => this.app.edit(house.id, fn).then(() => this.listHouses());
    const mine = (x) => x.source === "user", claude = (x) => x.source === "claude";
    const home = house.home;
    // one source's landmarks and no-fly zones, each with Remove
    const marks = (of, none) => {
      const lms = house.landmarks.filter(of), kos = house.keepouts.filter(of);
      return h("ul", { class: "marks" },
        lms.map((l) => h("li", {}, l.name, h("button", { type: "button", class: "btn ghost small", "aria-label": `Remove ${l.name}`,
          onclick: () => edit((hs) => drop(hs.landmarks, (q) => of(q) && q.name === l.name && q.x === l.x && q.y === l.y)) }, "Remove"))),
        kos.map((k, i) => {
          const name = of === claude ? `No-fly: ${(k.kind ?? "zone").replace(/-/g, " ")}${k.why ? ` (${k.why})` : ""}` : `No-fly zone ${i + 1}`;
          return h("li", {}, name, h("button", { type: "button", class: "btn ghost small", "aria-label": `Remove ${name}`,
            onclick: () => edit((hs) => drop(hs.keepouts, (q) => q === hs.keepouts.filter(of)[i])) }, "Remove"));
        }),
        !lms.length && !kos.length && h("li", { class: "muted" }, none));
    };
    const fromClaude = house.landmarks.some(claude) || house.keepouts.some(claude);
    return h("div", { class: "house-body" },
      h("div", { class: "label" }, "Rooms"),
      h("div", { class: "rooms" }, house.rooms.map((r) => h("label", {}, h("span", { class: "muted" },
        { user: "Your name", claude: "Claude's name" }[r.nameSource] ?? "From the capture"),
        h("input", { value: r.name, spellcheck: false, "aria-label": `Name of ${r.name}`, onchange: (e) => {
          const name = e.target.value.trim();
          if (name && name !== r.name) edit((hs) => Object.assign(hs.rooms.find((q) => q.id === r.id), { name, nameSource: "user" }));
        } })))),
      h("small", { class: "note" }, "Claude and the mission buttons use these names: “go to the kitchen”."),
      active && h("div", { class: "label" }, "Home pad"),
      active && h("p", { class: "note" }, home
        ? `${{ user: "Set by you", "tag 0": "On AprilTag 0" }[home.source] ?? "Where the capture started"} at ${home.x.toFixed(2)}, ${home.y.toFixed(2)} m. `
          + "Move it with the map’s Home tool."
        : "Not set: use the map’s Home tool. Missions start and end there."),
      active && h("div", { class: "label" }, "Your landmarks and no-fly zones"),
      active && marks(mine, "None yet: add them with the map's Landmark and No-fly tools."),
      active && fromClaude && h("div", { class: "label" }, "From Claude's survey"),
      active && fromClaude && marks(claude, ""),
      active && house.warnings?.length > 0 && h("details", {},
        h("summary", {}, `${house.warnings.length} note${house.warnings.length === 1 ? "" : "s"} from the import`),
        h("ul", {}, house.warnings.map((w) => h("li", {}, w)))),
      active && h("ul", { class: "checks-list" }, checkSummary(house).map((t) => h("li", {}, t))),
    );
  }

  toggle(id) {
    if (this.open.has(id)) this.open.delete(id);
    else this.open.add(id);
    this.listHouses();
  }

  async use(id) {
    await this.app.use(id);
    await this.listHouses();
  }

  async remove(id) {
    if (this.app.active() === id) await this.app.use("");
    await (await this.app.store()).deleteHouse(id);
    this.open.delete(id);
    await this.refresh();
  }
}
