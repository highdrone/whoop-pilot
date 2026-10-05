// "Every inch": how much of the house the 3D map knows, for Settings → House (docs/HOME-DRONE.md, Wave C; the PANELS
// contract). house/coverage.js's report in plain words: the score, why unknown space matters (the drone never flies
// through it), a row per room, what to rescan with the scanner and at what height ("Show on map" emits "show"), every gap,
// and what the drone's own flights added: free space its live depth found and how much of each room its camera has seen
// (vox.markSeen), with when. In the air the report may not be worked out (coverageReport() null): it says so, and works
// it out again on landing (flying()).
//   const cov = new CoveragePanel(el, { house, map, vox, coverageReport, flying });  cov.on("show", ({ x, y, z, text }) => …)
import { Emitter } from "../util.js";
import { put, watchFlying, plainWords } from "./changecard.js";
import { h } from "./dom.js";
import { FREE, FLAG, bandMask } from "../house/voxels.js";
import { coverageReport as report } from "../house/coverage.js";
import { when } from "../memory/memory.js";

const BAND = [0.3, 1.6];
const m3 = (v) => (v >= 10 ? `${Math.round(v)} m³` : v >= 0.1 ? `${v.toFixed(1)} m³` : `${Math.round(v * 1000)} litres`);
const plural = (n, w, ws = `${w}s`) => `${n} ${n === 1 ? w : ws}`;

export function scoreWords(score) {
  return score >= 90 ? "Good: nearly all of it is known." : score >= 75 ? "Mostly known: a few gaps worth a rescan." : score >= 50 ? "Patchy: rescan the gaps below before flying there." : "Poor: most of the flying space is unknown, so the drone can't go there.";
}

// The report's rooms as table rows, worst first: { id, name, known, unknown, unknownM3, flyable, gaps, words, level }. known
// and unknown are shares of the room's open space at flying height (free or unknown, furniture left out), as the score is.
export function coverageRows(report) {
  return report.rooms.map((r) => {
    const open = r.knownFreePct + r.unknownPct, unknown = open > 0 ? Math.round((100 * r.unknownPct) / open) : 0, known = open > 0 ? 100 - unknown : 0;
    const level = unknown <= 10 ? "ok" : unknown <= 30 ? "warn" : "bad", gaps = r.gaps.length;
    const words = unknown <= 10 ? (gaps ? `Well covered: ${plural(gaps, "gap")} worth a rescan (optional)` : "Well covered")
      : gaps ? `${plural(gaps, "gap")} to rescan` : r.hiddenM3 > r.unknownM3 / 2 ? "Unknown space is closed off (inside furniture)" : "Some unknown space";
    return { id: r.id, name: r.name, known, unknown, unknownM3: m3(r.unknownM3), flyable: m3(r.flyableM3), gaps: r.gaps.length, twinOnly: Math.round(r.twinOnlyPct ?? 0), words, level };
  }).sort((a, b) => b.unknown - a.unknown);
}

// The rescans, one per thing to do (the same words at several spots are one), most gaps first: { text, spots, gaps }.
export function rescans(suggestions) {
  const by = new Map();
  for (const s of suggestions) {
    const g = by.get(s.text) ?? by.set(s.text, { text: s.text, spots: [], gaps: 0 }).get(s.text);
    g.spots.push({ x: s.x, y: s.y, height: s.height, room: s.room });
    g.gaps += s.gaps ?? 1;
  }
  return [...by.values()].sort((a, b) => b.gaps - a.gaps);
}

// What flights added, per room, at the drone's heights: free space only flights found (live depth, kept after four frames
// agreed), the share of the room's known free space the drone's camera has seen (vox.markSeen), and when it last looked.
export function flightAdditions(vox, map, band = BAND) {
  const { nx, ny, res } = vox, layer = nx * ny, mask = bandMask(vox, map, band), v3 = res ** 3;
  const rooms = map.rooms.map((r) => ({ id: r.id, name: r.name, free: 0, flight: 0, seen: 0, last: 0 }));
  const roomOf = new Int16Array(layer).fill(-1);
  for (let r = 0; r < ny; r++)
    for (let c = 0; c < nx; c++) {
      const k = map.idx(vox.x0 + (c + 0.5) * res, vox.y0 + (r + 0.5) * res);
      if (k >= 0) roomOf[r * nx + c] = map.room[k];
    }
  const seen = vox.seen;
  for (let i = 0; i < vox.n; i++) {
    if (!mask[i] || vox.st[i] !== FREE) continue;
    const R = rooms[roomOf[i % layer]];
    if (!R) continue;
    R.free++;
    if (vox.flags[i] & FLAG.FLIGHT && !(vox.flags[i] & (FLAG.CARVED | FLAG.TWIN))) R.flight++;
    if (seen?.[i] > 0) (R.seen++, seen[i] > R.last && (R.last = seen[i]));
  }
  const out = rooms.map((R) => ({ id: R.id, name: R.name, flightM3: +(R.flight * v3).toFixed(3), seenPct: R.free ? Math.round((100 * R.seen) / R.free) : 0,
    lastSeen: R.last ? vox.epoch + R.last - 1 : null }));
  return { rooms: out, flown: out.some((r) => r.flightM3 > 0 || r.seenPct > 0), flightM3: +out.reduce((a, r) => a + r.flightM3, 0).toFixed(3) };
}

export class CoveragePanel extends Emitter {
  constructor(el, { house = null, map = null, vox = null, coverageReport = report, now = Date.now, flying = () => false } = {}) {
    super();
    Object.assign(this, { el, coverageReport, now, flying, report: null, adds: null, seq: 0 });
    el.classList.add("cov");
    this.off = watchFlying(flying, (air) => !air && this.refresh()); // landed: what the flight added
    this.setHouse(house, map, vox);
  }

  setHouse(house, map, vox) {
    Object.assign(this, { house, map, vox, report: null, adds: null });
    this.refresh();
  }

  async refresh() {
    const seq = ++this.seq, { house, map, vox } = this;
    if (!house || !map) return put(this.el, h("p", { class: "note" }, "Load a house to see how much of it the 3D map knows."));
    if (!vox) return put(this.el, h("p", { class: "note" }, "The 3D map isn't built yet. Until it is, the drone flies on the floor plan alone, and slowly."));
    put(this.el, h("p", { class: "note cov-busy" }, "Working out what the 3D map knows…"));
    await new Promise((r) => setTimeout(r, 30)); // "Working out…" shows first (not a frame wait: hidden pages get none)
    try {
      const report = this.coverageReport({ house, map, vox });
      if (seq !== this.seq) return;
      if (!report) return put(this.el, h("p", { class: "note" }, this.flying() ? "The coverage is worked out on the ground: it shows after landing." : "The coverage isn't worked out yet: press Check again in a moment."),
        !this.flying() && h("div", { class: "row" }, h("button", { type: "button", class: "btn small", onclick: () => this.refresh() }, "Check again")));
      const adds = flightAdditions(vox, map, report.band ?? BAND);
      Object.assign(this, { report, adds });
      this.render();
    } catch (e) {
      if (seq === this.seq) put(this.el, h("p", { class: "note warn-text" }, `Couldn't work out the coverage: ${plainWords(e, "the 3D map didn't load properly")}. Import the capture again if it keeps happening.`),
        h("div", { class: "row" }, h("button", { type: "button", class: "btn small", onclick: () => this.refresh() }, "Check again")));
    }
  }

  // "show" { x, y, z, text, room, spots? }: the first spot to scan from (z: the scanner's height there), and all of them.
  show(p) {
    const z = (this.map.floorAt(p.x, p.y) ?? 0) + (p.height ?? 1.0);
    this.emit("show", { x: p.x, y: p.y, z, text: p.text, room: p.room, ...(p.spots && { spots: p.spots }) });
  }

  render() {
    const { report, adds } = this, rows = coverageRows(report), names = new Map(report.rooms.map((r) => [r.id, r.name]));
    const showBtn = (p) => h("button", { type: "button", class: "btn ghost small", onclick: () => this.show(p) }, "Show on map");
    const todo = rescans(report.suggestions), item = (t) => h("li", {},
      h("span", {}, t.text, h("small", {}, ` (${t.gaps} gap${t.gaps === 1 ? "" : "s"}${t.spots.length > 1 ? `, ${t.spots.length} spots` : ""})`)), showBtn({ ...t.spots[0], text: t.text, spots: t.spots }));
    put(this.el,
      h("div", { class: "cov-score", "data-level": report.note ? "bad" : report.score >= 90 ? "ok" : report.score >= 75 ? "warn" : "bad" },
        h("b", {}, report.note ? "–" : `${report.score}%`),
        h("span", {}, report.note ? "The 3D map can't tell free space" : "of the space the drone can fly in (0.3 to 1.6 m up) is known", h("small", {}, report.note ?? scoreWords(report.score)))),
      h("p", { class: "cov-why" }, "The drone only flies through space the 3D scan (or an earlier flight's camera) has seen. Space nobody has seen counts as a wall, so gaps make rooms smaller or unreachable. Rescan a gap with the scanner app, then import the capture again."),
      h("div", { class: "cov-table", role: "table", "aria-label": "Coverage per room" },
        h("div", { class: "cov-tr cov-th", role: "row" }, ["Room", "Known", "Unknown", "What to do"].map((t) => h("span", { role: "columnheader" }, t))),
        rows.map((r) => h("div", { class: "cov-tr", role: "row", "data-level": r.level },
          h("span", { role: "cell", class: "cov-name" }, r.name),
          h("span", { role: "cell" }, h("span", { class: "cov-bar", style: `--known:${r.known}%;--unknown:${r.unknown}%`, "aria-hidden": "true" }), `${r.known}% known`),
          h("span", { role: "cell" }, `${r.unknown}% unknown (${r.unknownM3})`),
          h("span", { role: "cell", class: "cov-do" }, r.words)))),
      h("p", { class: "hint cov-legend" }, h("span", { class: "cov-sw", "data-k": "known", "aria-hidden": "true" }), "known open space, ", h("span", { class: "cov-sw", "data-k": "unknown", "aria-hidden": "true" }),
        "unknown (counts as a wall): shares of each room's open space 0.3 to 1.6 m up, furniture left out."),
      h("h4", {}, report.score >= 90 ? "What to rescan (optional)" : "What to rescan"),
      todo.length ? h("ol", { class: "cov-sugg" }, todo.slice(0, 4).map(item)) : h("p", { class: "note" }, report.note ? "Import the capture again with its 3D scan and capture path." : "Nothing worth a rescan."),
      todo.length > 4 && h("details", { class: "cov-gaps" }, h("summary", {}, `${todo.length - 4} more`), h("ol", { class: "cov-sugg", start: 5 }, todo.slice(4).map(item))),
      report.rooms.some((r) => r.gaps.length) && h("details", { class: "cov-gaps" }, h("summary", {}, `Every gap (${report.rooms.reduce((a, r) => a + r.gaps.length, 0)})`),
        h("ul", {}, report.rooms.flatMap((r) => r.gaps.map((g) => h("li", {}, h("span", {}, g.text, h("small", {}, ` ${g.size} m² across, ${g.heightBand} up`)),
          h("button", { type: "button", class: "btn ghost small", onclick: () => this.emit("show", { x: g.x, y: g.y, z: g.z, text: g.text, room: r.id }) }, "Show")))))),
      h("h4", {}, "What flights added"),
      adds.flown ? h("ul", { class: "cov-flights" }, adds.rooms.filter((r) => r.flightM3 > 0 || r.seenPct > 0).map((r) => h("li", {},
        h("strong", {}, names.get(r.id) ?? r.name), ": ", `the camera has seen ${r.seenPct}% of it${r.lastSeen ? `, last ${when(r.lastSeen, this.now())}` : ""}`,
        r.flightM3 > 0 ? `; flights found ${m3(r.flightM3)} more free space` : "")))
        : h("p", { class: "note" }, "No flight has added to the 3D map yet. With live depth on and the camera calibrated, each flight fills in space the scan missed and marks what its camera looked at."),
      h("div", { class: "row" }, h("button", { type: "button", class: "btn small", onclick: () => this.refresh() }, "Check again")));
  }

  dispose() {
    this.seq++;
    this.off();
    put(this.el);
    this.el.classList.remove("cov");
  }
}
