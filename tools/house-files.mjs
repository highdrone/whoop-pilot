// Read-only access to the user's SiteSpec and Spacial captures, so the app can import the newest one
// without a folder picker. Only files inside a project folder of one of the roots are reachable: names
// are checked segment by segment (no "..", no dotfiles) and the real path, symlinks resolved, must stay
// inside the project's real folder.
import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";

export const HOUSE_ROOTS = [
  { kind: "sitespec", dir: path.join(os.homedir(), "SiteSpec Projects") },
  { kind: "spacial", dir: path.join(os.homedir(), "Spacial Projects") },
];

const SPLATS = ["outputs/splat.spz", "outputs/splat.splat", "outputs/splat.ply"];
const okName = (s) => s && s !== "." && s !== ".." && !s.startsWith(".") && !/[/\\\0]/.test(s);
const inside = (dir, p) => p === dir || p.startsWith(dir + path.sep);
const real = (p) => fs.realpath(p).catch(() => null);
const stat = (p) => fs.stat(p).catch(() => null);
const url = (id, rel) => "/house-files/" + [id, ...rel.split("/")].map(encodeURIComponent).join("/");

export class HouseFiles {
  constructor(roots = HOUSE_ROOTS) {
    this.roots = roots;
  }

  // The project folder's real path, or null. SiteSpec first when both roots have the name.
  async project(id) {
    if (!okName(id)) return null;
    for (const { kind, dir } of this.roots) {
      const root = await real(dir);
      const p = root && (await real(path.join(root, id)));
      if (p && inside(root, p) && p !== root && (await stat(p))?.isDirectory()) return { id, kind, dir: p };
    }
    return null;
  }

  // segs: decoded path segments inside the project. Returns { file, stat } or null (missing or not allowed).
  async resolve(id, segs) {
    const proj = await this.project(id);
    if (!proj || !segs.every(okName)) return null;
    const p = await real(path.join(proj.dir, ...segs));
    if (!p || !inside(proj.dir, p)) return null;
    const st = await stat(p);
    return st && { file: p, stat: st, project: proj };
  }

  // A folder's entries (what resolve() would serve: no dotfiles, no symlinks out of the project).
  async dir(file, projectDir) {
    const entries = await fs.readdir(file, { withFileTypes: true });
    const out = [];
    for (const e of entries) {
      if (!okName(e.name)) continue;
      const p = path.join(file, e.name);
      if (e.isSymbolicLink() && !inside(projectDir, (await real(p)) ?? "")) continue;
      const st = await stat(p);
      if (st) out.push({ name: e.name, kind: st.isDirectory() ? "dir" : "file", size: st.isDirectory() ? null : st.size, mtime: st.mtime.toISOString() });
    }
    return out.sort((a, b) => a.name.localeCompare(b.name));
  }

  async describe({ id, kind, dir }) {
    const has = async (rel) => ((await stat(path.join(dir, rel)))?.isFile() ? rel : null);
    const project = await fs.readFile(path.join(dir, "project.json"), "utf8").then(JSON.parse).catch(() => ({}));
    const plans = await fs.readdir(path.join(dir, "outputs/plans")).catch(() => []);
    const plan = (re) => {
      const n = plans.find((n) => re.test(n));
      return n ? `outputs/plans/${n}` : null;
    };
    let splat = null;
    for (const rel of SPLATS) {
      const st = await stat(path.join(dir, rel));
      if (st?.isFile()) {
        splat = { path: rel, size: st.size, url: url(id, rel) };
        break;
      }
    }
    const files = {
      splat: splat?.path ?? null,
      scene: await has("outputs/scene.json"),
      site: await has("site.json"),
      rooms: plan(/-rooms\.json$/),
      roomsAuto: await has("work/room/rooms.auto.json"),
      orthophoto: plan(/-orthophoto\.png$/),
      orthophotoMeta: plan(/-orthophoto\.json$/),
      thumbnail: await has("outputs/thumbnail.jpg"),
    };
    const secs = project.updated ?? project.created;
    const date = Number.isFinite(secs) ? new Date(secs * 1000) : (await stat(dir)).mtime;
    return {
      id,
      name: typeof project.name === "string" ? project.name : id,
      kind,
      date: date.toISOString(),
      splat,
      thumbnail: files.thumbnail && url(id, files.thumbnail),
      ready: !!(files.splat && files.scene),
      files,
      url: url(id, ""),
    };
  }

  // Every project in both roots, newest first.
  async list() {
    const out = [];
    const seen = new Set();
    for (const { dir } of this.roots) {
      const root = await real(dir);
      if (!root) continue;
      for (const e of await fs.readdir(root, { withFileTypes: true }).catch(() => [])) {
        if (seen.has(e.name) || !(e.isDirectory() || e.isSymbolicLink())) continue;
        const proj = await this.project(e.name);
        if (!proj || proj.dir === root) continue;
        seen.add(e.name);
        out.push(await this.describe(proj).catch(() => null));
      }
    }
    return out.filter(Boolean).sort((a, b) => b.date.localeCompare(a.date));
  }
}
