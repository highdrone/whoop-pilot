// Draws the FPV view with detections, a horizon line and status overlays.
const ACCENT = "#ffd84d";
const FONT = "600 13px ui-sans-serif, system-ui, -apple-system, sans-serif";
const C = { plain: "#1f2937", warn: "#b45309", bad: "#b91c1c", ok: "#15803d", ai: "#4c1d95" };

// The HUD's chips for where the drone is and why it flies as it does (pure, so tests read them). s: { pose (localizer's, or
// null outside the house), known, flown, flying, vision (vision localization drives the position), trust ("trusted" |
// "verified" | "advisory"), fix (localizer.fixQuality()), why (safety.status().reason), real, brake ({ measured }), video
// ({ fps, picture }), ai ({ spent, budget }) }. -> { right: [{ text, color }], notice: text (why it is slow or stopped,
// with what to do) | "" , banner: { text, color } | null }
export function statusChips(s) {
  const right = [], p = s.pose;
  if (s.real && s.video?.fps != null) right.push({ text: s.video.picture === "looking" ? "VIDEO: FINDING THE PICTURE" : `VIDEO ${s.video.fps} fps`, color: s.video.fps < 15 ? C.warn : C.plain });
  if (p && p.source !== "truth") {
    if (!s.known) right.push({ text: "NO POSITION · SET ON PAD", color: C.warn });
    else if (p.status === "lost") right.push({ text: s.flying ? "POSITION LOST · HOLDING" : "POSITION LOST · SET ON PAD", color: C.bad });
    else right.push({ text: `POS ±${(2 * p.sigma).toFixed(2)} m`, color: p.status === "degraded" ? C.warn : C.plain });
    if (s.vision) {
      const f = s.fix ?? {}, c = p.conflict;
      right.push(c ? { text: `VISION DISAGREES ${Number(c.apart ?? 0).toFixed(1)} m`, color: C.bad }
        : s.trust === "advisory" ? { text: "VISION: PAD CHECK NEEDED", color: C.warn }
        : f.age < 1500 ? { text: `VISION ${Number(f.rate ?? 0).toFixed(1)}/s`, color: C.plain }
        : { text: "NO VISION FIX", color: C.warn });
    }
  }
  if (s.ai?.spent > 0) right.push({ text: `AI $${s.ai.spent.toFixed(2)} of $${Number(s.ai.budget ?? 0).toFixed(2)}`, color: s.ai.spent >= s.ai.budget ? C.warn : C.ai });
  let notice = "";
  if (s.flying && s.why) notice = /^flying slowly|^at most/.test(s.why) ? s.why : `Holding back: ${s.why}`;
  else if (s.flying && s.real && s.brake && !s.brake.measured) notice = "at most 0.3 m/s until braking is measured";
  if (p && p.source !== "truth" && !s.known && !s.flying) notice ||= "Put the drone on its home pad and press “The drone is on its home pad”.";
  const banner = p && p.status === "lost" && s.flying && s.attached ? { text: "Position lost — holding. Take over with the sticks if it drifts.", color: "rgba(217,119,6,0.92)" } : null;
  return { right, notice, banner };
}

// Greedy word wrap to lines no wider than maxWidth in the current font.
function wrap(g, text, maxWidth) {
  const lines = [];
  let line = "";
  for (const word of text.split(/\s+/).filter(Boolean)) {
    const next = line ? `${line} ${word}` : word;
    if (line && g.measureText(next).width > maxWidth) {
      lines.push(line);
      line = word;
    } else line = next;
  }
  if (line) lines.push(line);
  return lines;
}

// No video: what to do, centred between the status chips (top: below their last row; phones wrap them) and the chips at
// the bottom (bottom); the lines that don't fit are left out (the title's last one ends in "…").
function noVideo(g, s, W, H, top, bottom) {
  g.font = "600 16px ui-sans-serif, system-ui, sans-serif";
  let title = wrap(g, s.noVideoText || "No video", W - 48);
  g.font = "13px ui-sans-serif, system-ui, sans-serif";
  let hint = wrap(g, s.noVideoHint || "", W - 48);
  const room = Math.max(21, bottom - top), need = () => title.length * 21 + hint.length * 18 + (hint.length ? 6 : 0);
  while (hint.length && need() > room) hint = hint.slice(0, -1);
  if (need() > room) title = [...title.slice(0, Math.max(1, Math.floor(room / 21)) - 1), `${title[Math.max(1, Math.floor(room / 21)) - 1]}…`];
  let y = Math.max(top, Math.min(H / 2 - need() / 2, bottom - need()));
  g.textAlign = "center";
  g.fillStyle = "#9aa4b2";
  g.font = "600 16px ui-sans-serif, system-ui, sans-serif";
  for (const line of title) g.fillText(line, W / 2, (y += 21) - 5);
  y += 6;
  g.fillStyle = "#6b7684";
  g.font = "13px ui-sans-serif, system-ui, sans-serif";
  for (const line of hint) g.fillText(line, W / 2, (y += 18) - 5);
  g.textAlign = "left";
}

export function drawFpv(canvas, source, s) {
  const dpr = window.devicePixelRatio || 1;
  const cw = Math.round(canvas.clientWidth * dpr);
  const ch = Math.round(canvas.clientHeight * dpr);
  if (canvas.width !== cw || canvas.height !== ch) {
    canvas.width = cw;
    canvas.height = ch;
  }
  const g = canvas.getContext("2d");
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.fillStyle = "#07090c";
  g.fillRect(0, 0, cw, ch);
  g.scale(dpr, dpr);
  const W = cw / dpr;
  const H = ch / dpr;

  // Fit the video frame (contain).
  let fx = 0;
  let fy = 0;
  let fw = W;
  let fh = H;
  const r = source && source.ready() ? source.region() : null;
  const ready = !!(r && r.sw && r.sh);
  if (ready) {
    const ar = r.sw / r.sh;
    if (W / H > ar) {
      fw = H * ar;
      fx = (W - fw) / 2;
    } else {
      fh = W / ar;
      fy = (H - fh) / 2;
    }
    g.imageSmoothingEnabled = true;
    g.imageSmoothingQuality = "high";
    g.drawImage(source.element(), r.sx, r.sy, r.sw, r.sh, fx, fy, fw, fh);
  }

  if (ready) {
    horizon(g, fx, fy, fw, fh, s);
    crosshair(g, fx + fw / 2, fy + fh / 2);
    for (const d of s.detections || []) {
      if (d.score < 0.35) continue;
      const target = s.targets?.includes(d.label);
      box(g, fx + d.box.x * fw, fy + d.box.y * fh, d.box.w * fw, d.box.h * fh, `${d.label} ${Math.round(d.score * 100)}%`, target);
    }
  }

  // Status chips.
  g.font = FONT;
  let x = 12;
  for (const chip of s.chips || []) x += drawChip(g, x, 12, chip.text, chip.color) + 8;
  let rx = W - 12, ry = 12, left = x;
  for (const chip of s.rightChips || []) {
    const w = Math.min(measureChip(g, chip.text), W - 24);
    if (rx - w < left && (rx < W - 12 || left > 12)) [rx, ry, left] = [W - 12, ry + 30, 12]; // no room beside the chips (phones): a row below
    drawChip(g, rx - w, ry, chip.text, chip.color, "#fff", w);
    rx -= w + 8;
  }
  if (!ready) (noVideo(g, s, W, H, (s.rightChips?.length ? ry : 12) + 30, H - 8 - (s.action ? 30 : 0) - (s.notice ? 30 : 0)), (g.font = FONT));

  // Current action, why the drone is slow or holding back, and the listening indicator.
  if (s.action) drawChip(g, 12, H - 36, s.action, "#1f2937", ACCENT);
  if (s.notice) {
    g.font = FONT;
    const text = wrap(g, s.notice, W - 60)[0] ?? "";
    drawChip(g, 12, H - (s.action ? 66 : 36), text + (text.length < s.notice.length ? "…" : ""), "rgba(120,53,15,0.92)", "#fde68a");
  }
  if (s.listening) {
    const t = performance.now() / 1000;
    g.fillStyle = `rgba(239,68,68,${0.6 + 0.4 * Math.sin(t * 6)})`;
    g.beginPath();
    g.arc(W - 22, H - 26, 7, 0, 7);
    g.fill();
  }

  // Subtitles: what the pilot said and what the drone says.
  const lines = (s.subtitles || []).slice(-2);
  let y = H - (s.action ? 58 : 20) - (s.notice ? 30 : 0);
  g.font = "600 15px ui-sans-serif, system-ui, sans-serif";
  for (let i = lines.length - 1; i >= 0; i--) {
    const l = lines[i];
    const text = l.text.length > 110 ? l.text.slice(0, 107) + "..." : l.text;
    const tw = g.measureText(text).width + 20;
    const bx = (W - tw) / 2;
    g.globalAlpha = l.alpha;
    g.fillStyle = "rgba(0,0,0,0.62)";
    roundRect(g, bx, y - 22, tw, 28, 8);
    g.fillStyle = l.who === "pilot" ? "#9ecbff" : "#ffffff";
    g.fillText(text, bx + 10, y - 3);
    g.globalAlpha = 1;
    y -= 34;
  }

  if (s.cropDrag) {
    const c = s.cropDrag;
    g.strokeStyle = ACCENT;
    g.setLineDash([6, 4]);
    g.lineWidth = 2;
    g.strokeRect(Math.min(c.x0, c.x1), Math.min(c.y0, c.y1), Math.abs(c.x1 - c.x0), Math.abs(c.y1 - c.y0));
    g.setLineDash([]);
  }

  if (s.banner) {
    g.font = "700 18px ui-sans-serif, system-ui, sans-serif";
    const tw = Math.min(W - 16, g.measureText(s.banner.text).width + 32), by = ry + 40; // below the rows of chips
    g.fillStyle = s.banner.color || "rgba(220,38,38,0.9)";
    roundRect(g, (W - tw) / 2, by, tw, 38, 10);
    g.fillStyle = "#fff";
    g.fillText(s.banner.text, (W - tw) / 2 + 16, by + 25, tw - 32);
  }
  return { fx, fy, fw, fh };
}

function horizon(g, fx, fy, fw, fh, s) {
  if (s.hfov === undefined) return;
  const vfov = (s.hfov * fh) / fw;
  const yFrac = 0.5 + (s.uptilt - s.pitchDeg) / vfov; // horizon sits below center when the camera is tilted up
  if (yFrac < 0 || yFrac > 1) return;
  const cx = fx + fw / 2;
  const cy = fy + yFrac * fh;
  const a = (-s.rollDeg * Math.PI) / 180;
  const L = fw * 0.18;
  g.strokeStyle = "rgba(255,255,255,0.55)";
  g.lineWidth = 1.5;
  g.beginPath();
  g.moveTo(cx - Math.cos(a) * L * 2, cy - Math.sin(a) * L * 2);
  g.lineTo(cx - Math.cos(a) * L, cy - Math.sin(a) * L);
  g.moveTo(cx + Math.cos(a) * L, cy + Math.sin(a) * L);
  g.lineTo(cx + Math.cos(a) * L * 2, cy + Math.sin(a) * L * 2);
  g.stroke();
}

function crosshair(g, x, y) {
  g.strokeStyle = "rgba(255,255,255,0.7)";
  g.lineWidth = 1.5;
  g.beginPath();
  g.moveTo(x - 10, y);
  g.lineTo(x - 4, y);
  g.moveTo(x + 4, y);
  g.lineTo(x + 10, y);
  g.moveTo(x, y - 10);
  g.lineTo(x, y - 4);
  g.moveTo(x, y + 4);
  g.lineTo(x, y + 10);
  g.stroke();
}

function box(g, x, y, w, h, label, target) {
  const c = target ? ACCENT : "rgba(125,211,252,0.9)";
  const k = Math.min(14, w / 3, h / 3);
  g.strokeStyle = c;
  g.lineWidth = target ? 3 : 2;
  g.beginPath();
  for (const [px, py, dx, dy] of [[x, y, 1, 1], [x + w, y, -1, 1], [x, y + h, 1, -1], [x + w, y + h, -1, -1]]) {
    g.moveTo(px + dx * k, py);
    g.lineTo(px, py);
    g.lineTo(px, py + dy * k);
  }
  g.stroke();
  if (target) {
    g.fillStyle = "rgba(255,216,77,0.08)";
    g.fillRect(x, y, w, h);
  }
  g.font = FONT;
  const tw = g.measureText(label).width + 10;
  g.fillStyle = target ? ACCENT : "rgba(12,74,110,0.85)";
  roundRect(g, x, Math.max(0, y - 20), tw, 18, 5);
  g.fillStyle = target ? "#111" : "#e0f2fe";
  g.fillText(label, x + 5, Math.max(13, y - 6));
}

function measureChip(g, text) {
  g.font = FONT;
  return g.measureText(text).width + 18;
}

function drawChip(g, x, y, text, bg, fg = "#fff", width = null) {
  const w = width ?? measureChip(g, text);
  g.fillStyle = bg;
  roundRect(g, x, y, w, 24, 12);
  g.fillStyle = fg;
  g.fillText(text, x + 9, y + 16.5, w - 18);
  return w;
}

function roundRect(g, x, y, w, h, r) {
  g.beginPath();
  g.roundRect(x, y, w, h, r);
  g.fill();
}

// Top-down map of the simulated apartment.
export function drawMinimap(canvas, sim) {
  const dpr = window.devicePixelRatio || 1;
  const cw = Math.round(canvas.clientWidth * dpr);
  const ch = Math.round(canvas.clientHeight * dpr);
  if (canvas.width !== cw || canvas.height !== ch) {
    canvas.width = cw;
    canvas.height = ch;
  }
  const g = canvas.getContext("2d");
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.clearRect(0, 0, cw, ch);
  const world = sim.world;
  const pad = 10 * dpr;
  const sc = Math.min((cw - 2 * pad) / 10, (ch - 2 * pad) / 8);
  if (!(sc > 0)) return; // not laid out yet, or hidden
  const ox = (cw - 10 * sc) / 2;
  const oy = (ch - 8 * sc) / 2;
  const X = (x) => ox + x * sc;
  const Y = (y) => oy + (8 - y) * sc;
  const floor = { living: "#3a3226", kitchen: "#2c3534", bedroom: "#2d2b3d", hall: "#3a3325" };
  for (const r of world.rooms) {
    g.fillStyle = floor[r.id] || "#333";
    g.fillRect(X(r.x0), Y(r.y1), (r.x1 - r.x0) * sc, (r.y1 - r.y0) * sc);
    g.fillStyle = "rgba(255,255,255,0.35)";
    g.font = `${10 * dpr}px ui-sans-serif, system-ui, sans-serif`;
    g.fillText(r.name, X(r.x0) + 4 * dpr, Y(r.y1) + 12 * dpr);
  }
  g.strokeStyle = "#cbd5e1";
  g.lineWidth = 2.5 * dpr;
  g.lineCap = "round";
  g.beginPath();
  for (const s of world.segments) {
    g.moveTo(X(s.ax), Y(s.ay));
    g.lineTo(X(s.bx), Y(s.by));
  }
  g.stroke();
  for (const f of world.furniture) {
    g.fillStyle = f.label ? "rgba(148,163,184,0.55)" : "rgba(100,116,139,0.4)";
    g.beginPath();
    g.arc(X(f.x), Y(f.y), Math.max(2 * dpr, f.r * sc), 0, 7);
    g.fill();
  }
  const tr = sim.trail;
  if (tr.length > 1) {
    g.strokeStyle = "rgba(255,216,77,0.35)";
    g.lineWidth = 1.5 * dpr;
    g.beginPath();
    g.moveTo(X(tr[0].x), Y(tr[0].y));
    for (const p of tr) g.lineTo(X(p.x), Y(p.y));
    g.stroke();
  }
  const cat = world.cat;
  g.fillStyle = "#f59e0b";
  g.beginPath();
  g.arc(X(cat.x), Y(cat.y), 4.5 * dpr, 0, 7);
  g.fill();
  const d = sim.drone;
  const hf = sim.renderer ? sim.renderer.hfov : 2.6;
  g.fillStyle = "rgba(255,216,77,0.16)";
  g.beginPath();
  g.moveTo(X(d.x), Y(d.y));
  g.arc(X(d.x), Y(d.y), 2.2 * sc, -d.yaw - hf / 2, -d.yaw + hf / 2);
  g.closePath();
  g.fill();
  g.save();
  g.translate(X(d.x), Y(d.y));
  g.rotate(-d.yaw);
  g.fillStyle = d.crashed ? "#ef4444" : d.airborne ? ACCENT : "#e5e7eb";
  g.beginPath();
  g.moveTo(9 * dpr, 0);
  g.lineTo(-6 * dpr, 5.5 * dpr);
  g.lineTo(-3 * dpr, 0);
  g.lineTo(-6 * dpr, -5.5 * dpr);
  g.closePath();
  g.fill();
  g.restore();
}
