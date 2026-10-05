// Billboard textures for the simulator, painted once with Canvas 2D and read back as RGBA arrays.

const PAINTERS = {
  couch: [128, 58, (g, w, h) => {
    g.fillStyle = "#3d5a80";
    rr(g, 6, 4, w - 12, h * 0.55, 8); // back
    g.fillStyle = "#4a6d99";
    rr(g, 10, h * 0.42, w - 20, h * 0.32, 5); // seat
    g.strokeStyle = "#35506f";
    g.lineWidth = 2;
    for (let i = 1; i < 3; i++) line(g, 10 + ((w - 20) * i) / 3, h * 0.44, 10 + ((w - 20) * i) / 3, h * 0.72);
    g.fillStyle = "#34506f";
    rr(g, 0, h * 0.25, 16, h * 0.55, 6); // armrests
    rr(g, w - 16, h * 0.25, 16, h * 0.55, 6);
    g.fillStyle = "#e0b04a";
    rr(g, w * 0.62, h * 0.24, 20, 16, 4); // cushion
    g.fillStyle = "#2b2b2b";
    g.fillRect(8, h * 0.8, 5, h * 0.2);
    g.fillRect(w - 13, h * 0.8, 5, h * 0.2);
  }],
  tv: [96, 66, (g, w, h) => {
    g.fillStyle = "#111";
    rr(g, 0, 0, w, h - 6, 3);
    const grad = g.createLinearGradient(0, 0, w, h);
    grad.addColorStop(0, "#1f3f8a");
    grad.addColorStop(0.5, "#7b3fa0");
    grad.addColorStop(1, "#e07a3a");
    g.fillStyle = grad;
    g.fillRect(4, 4, w - 8, h - 14);
    g.fillStyle = "rgba(255,255,255,0.8)";
    g.beginPath();
    g.arc(w * 0.3, h * 0.35, 8, 0, 7);
    g.fill();
    g.fillStyle = "#222";
    g.fillRect(w / 2 - 10, h - 6, 20, 6);
  }],
  tvstand: [96, 36, (g, w, h) => {
    g.fillStyle = "#6b4a2f";
    g.fillRect(0, 0, w, h);
    g.fillStyle = "#57391f";
    g.fillRect(4, 6, w / 2 - 6, h - 12);
    g.fillRect(w / 2 + 2, 6, w / 2 - 6, h - 12);
    g.fillStyle = "#c9a36b";
    g.fillRect(w / 2 - 10, h / 2 - 1, 6, 3);
    g.fillRect(w / 2 + 4, h / 2 - 1, 6, 3);
  }],
  plant: [40, 76, (g, w, h) => {
    g.fillStyle = "#b5602f";
    g.beginPath();
    g.moveTo(8, h * 0.66);
    g.lineTo(w - 8, h * 0.66);
    g.lineTo(w - 12, h);
    g.lineTo(12, h);
    g.fill();
    const leaves = [[20, 10, 9, 18, -0.3], [11, 22, 8, 16, -0.8], [29, 22, 8, 16, 0.8], [14, 38, 9, 14, -1.1], [27, 38, 9, 14, 1.1], [20, 30, 7, 16, 0]];
    for (const [x, y, rx, ry, a] of leaves) {
      g.fillStyle = a === 0 ? "#3f8f3a" : "#2f7a33";
      g.beginPath();
      g.ellipse(x, y, rx, ry, a, 0, 7);
      g.fill();
    }
  }],
  armchair: [64, 72, (g, w, h) => {
    g.fillStyle = "#a8432f";
    rr(g, 8, 2, w - 16, h * 0.6, 10);
    g.fillStyle = "#c0523b";
    rr(g, 10, h * 0.45, w - 20, h * 0.28, 6);
    g.fillStyle = "#8f3624";
    rr(g, 0, h * 0.3, 14, h * 0.5, 6);
    rr(g, w - 14, h * 0.3, 14, h * 0.5, 6);
    g.fillStyle = "#2b2b2b";
    g.fillRect(8, h * 0.8, 5, h * 0.2);
    g.fillRect(w - 13, h * 0.8, 5, h * 0.2);
  }],
  fridge: [50, 112, (g, w, h) => {
    const grad = g.createLinearGradient(0, 0, w, 0);
    grad.addColorStop(0, "#d9dde2");
    grad.addColorStop(0.5, "#f4f6f8");
    grad.addColorStop(1, "#c7ccd2");
    g.fillStyle = grad;
    rr(g, 0, 0, w, h, 5);
    g.strokeStyle = "#8c939b";
    g.lineWidth = 1.5;
    line(g, 2, h * 0.34, w - 2, h * 0.34);
    g.fillStyle = "#6f7780";
    g.fillRect(w - 9, h * 0.12, 3, h * 0.16);
    g.fillRect(w - 9, h * 0.4, 3, h * 0.24);
    g.fillStyle = "#e45b5b";
    g.fillRect(12, h * 0.46, 8, 6); // magnets
    g.fillStyle = "#4ba3e3";
    g.fillRect(22, h * 0.5, 7, 7);
  }],
  oven: [56, 72, (g, w, h) => {
    g.fillStyle = "#c9ccd0";
    g.fillRect(0, 4, w, h - 4);
    g.fillStyle = "#222";
    g.fillRect(0, 0, w, 5); // stovetop
    g.fillStyle = "#3a3d42";
    g.fillRect(0, 5, w, 10); // control panel
    g.fillStyle = "#ddd";
    for (let i = 0; i < 4; i++) {
      g.beginPath();
      g.arc(8 + i * 13, 10, 3, 0, 7);
      g.fill();
    }
    g.fillStyle = "#111";
    rr(g, 6, 22, w - 12, h - 32, 4); // door window
    g.fillStyle = "rgba(255,140,40,0.35)";
    g.fillRect(10, 30, w - 20, h - 46);
    g.fillStyle = "#777";
    g.fillRect(8, 18, w - 16, 3); // handle
  }],
  counter: [64, 72, (g, w, h) => {
    g.fillStyle = "#8f9aa3";
    g.fillRect(0, 0, w, 7);
    g.fillStyle = "#e9e1d3";
    g.fillRect(0, 7, w, h - 7);
    g.strokeStyle = "#b9ae9b";
    g.lineWidth = 2;
    g.strokeRect(4, 12, w / 2 - 6, h - 18);
    g.strokeRect(w / 2 + 2, 12, w / 2 - 6, h - 18);
    g.fillStyle = "#555";
    g.fillRect(w / 2 - 8, 16, 3, 10);
    g.fillRect(w / 2 + 5, 16, 3, 10);
  }],
  sink: [72, 88, (g, w, h) => {
    const top = h * 0.22;
    g.fillStyle = "#e9e1d3";
    g.fillRect(0, top, w, h - top);
    g.fillStyle = "#8f9aa3";
    g.fillRect(0, top, w, 7);
    g.fillStyle = "#b8c0c8";
    g.fillRect(12, top - 2, w - 24, 6); // basin rim
    g.strokeStyle = "#9aa3ab";
    g.lineWidth = 4;
    g.beginPath(); // faucet
    g.moveTo(w / 2, top);
    g.lineTo(w / 2, 4);
    g.lineTo(w / 2 + 14, 4);
    g.lineTo(w / 2 + 14, 12);
    g.stroke();
    g.strokeStyle = "#b9ae9b";
    g.lineWidth = 2;
    g.strokeRect(4, top + 12, w / 2 - 6, h - top - 18);
    g.strokeRect(w / 2 + 2, top + 12, w / 2 - 6, h - top - 18);
  }],
  microwave: [48, 28, (g, w, h) => {
    g.fillStyle = "#d4d7db";
    rr(g, 0, 0, w, h, 3);
    g.fillStyle = "#1b1d20";
    g.fillRect(4, 4, w * 0.62, h - 8);
    g.fillStyle = "#555";
    g.fillRect(w * 0.74, 5, w * 0.18, h - 10);
    g.fillStyle = "#6fe07a";
    g.fillRect(w * 0.76, 6, w * 0.14, 3);
  }],
  table: [96, 52, (g, w, h) => {
    g.fillStyle = "#8a5a33";
    g.fillRect(0, 0, w, 8);
    g.fillStyle = "#6e4526";
    g.fillRect(6, 8, 6, h - 8);
    g.fillRect(w - 12, 8, 6, h - 8);
    g.fillStyle = "#e8e2d8";
    g.beginPath();
    g.ellipse(w * 0.5, 2, 12, 3, 0, 0, 7); // plate
    g.fill();
    g.fillStyle = "#d94e3f";
    g.beginPath();
    g.arc(w * 0.72, -1, 5, 0, 7); // apple
    g.fill();
  }],
  chair: [32, 60, (g, w, h) => {
    g.fillStyle = "#7a4d2a";
    g.fillRect(4, 0, 5, h);
    g.fillRect(w - 9, 0, 5, h);
    g.fillRect(4, 6, w - 8, 5);
    g.fillRect(4, 16, w - 8, 5);
    g.fillStyle = "#9a6337";
    g.fillRect(0, h * 0.5, w, 6);
    g.fillStyle = "#7a4d2a";
    g.fillRect(1, h * 0.5, 4, h * 0.5);
    g.fillRect(w - 5, h * 0.5, 4, h * 0.5);
  }],
  bed: [128, 42, (g, w, h) => {
    g.fillStyle = "#5b3b25";
    g.fillRect(0, 0, 10, h); // headboard
    g.fillStyle = "#f2f2f2";
    rr(g, 8, h * 0.25, w - 10, h * 0.45, 6);
    g.fillStyle = "#3f6fb5";
    rr(g, w * 0.32, h * 0.22, w * 0.68, h * 0.5, 6); // blanket
    g.fillStyle = "#ffffff";
    rr(g, 14, h * 0.12, 30, h * 0.26, 6); // pillow
    g.fillStyle = "#4a3020";
    g.fillRect(8, h * 0.7, w - 8, h * 0.3);
  }],
  backpack: [32, 36, (g, w, h) => {
    g.fillStyle = "#c0392b";
    rr(g, 2, 3, w - 4, h - 3, 8);
    g.fillStyle = "#96281b";
    rr(g, 7, h * 0.5, w - 14, h * 0.4, 4);
    g.strokeStyle = "#333";
    g.lineWidth = 2;
    g.beginPath();
    g.arc(w / 2, 5, 6, Math.PI, 0);
    g.stroke();
  }],
  cat: [64, 40, (g, w, h) => cat(g, w, h, false)],
  catwalk: [64, 38, (g, w, h) => cat(g, w, h, true)],
  person: [40, 104, (g, w, h) => person(g, w, h, "stand")],
  personwalk: [44, 104, (g, w, h) => person(g, w, h, "walk")],
  personsit: [44, 76, (g, w, h) => person(g, w, h, "sit")],
  personlie: [112, 22, (g, w, h) => {
    g.save();
    g.translate(w, 0);
    g.rotate(Math.PI / 2);
    person(g, h, w, "stand");
    g.restore();
  }],
  dog: [72, 60, (g, w, h) => dog(g, w, h, "stand")],
  dogwalk: [72, 58, (g, w, h) => dog(g, w, h, "walk")],
  doglie: [76, 34, (g, w, h) => dog(g, w, h, "lie")],
};

// A person seen from the front (stand, sit) or the side (walk): skin, hair, blue shirt, dark trousers.
function person(g, w, h, pose) {
  const skin = "#d9a383", shirt = "#2f6db5", pants = "#34465f", s = h / 104;
  const cx = w / 2, head = 9 * s;
  const legTop = pose === "sit" ? h - 30 * s : 56 * s;
  g.fillStyle = "#1c1c1c"; // shoes
  if (pose === "walk") {
    g.fillStyle = pants;
    g.save();
    g.translate(cx, legTop);
    for (const a of [0.35, -0.35]) {
      g.save();
      g.rotate(a);
      g.fillRect(-4 * s, 0, 8 * s, 44 * s);
      g.fillStyle = "#1c1c1c";
      g.fillRect(-4 * s, 42 * s, 11 * s, 4 * s);
      g.restore();
      g.fillStyle = pants;
    }
    g.restore();
  } else if (pose === "sit") {
    g.fillStyle = pants;
    rr(g, cx - 12 * s, legTop - 8 * s, 24 * s, 14 * s, 4 * s); // thighs toward the camera
    g.fillRect(cx - 11 * s, legTop + 4 * s, 9 * s, 24 * s);
    g.fillRect(cx + 2 * s, legTop + 4 * s, 9 * s, 24 * s);
    g.fillStyle = "#1c1c1c";
    g.fillRect(cx - 12 * s, h - 4 * s, 11 * s, 4 * s);
    g.fillRect(cx + 1 * s, h - 4 * s, 11 * s, 4 * s);
  } else {
    g.fillStyle = pants;
    g.fillRect(cx - 10 * s, legTop, 9 * s, 44 * s);
    g.fillRect(cx + 1 * s, legTop, 9 * s, 44 * s);
    g.fillStyle = "#1c1c1c";
    g.fillRect(cx - 11 * s, h - 4 * s, 10 * s, 4 * s);
    g.fillRect(cx + 1 * s, h - 4 * s, 10 * s, 4 * s);
  }
  const top = legTop - 36 * s;
  g.fillStyle = shirt;
  rr(g, cx - (pose === "walk" ? 8 : 13) * s, top, (pose === "walk" ? 16 : 26) * s, 38 * s, 5 * s); // torso
  g.fillStyle = skin;
  if (pose === "walk") g.fillRect(cx - 3 * s, top + 4 * s, 6 * s, 30 * s);
  else {
    g.fillRect(cx - 18 * s, top + 4 * s, 5 * s, 32 * s); // arms
    g.fillRect(cx + 13 * s, top + 4 * s, 5 * s, 32 * s);
  }
  g.beginPath();
  g.arc(cx, top - head - 2 * s, head, 0, 7);
  g.fill();
  g.fillStyle = "#2e2018"; // hair
  g.beginPath();
  g.arc(cx, top - head - 4 * s, head, Math.PI, 0);
  g.fill();
}

// A black-and-tan shepherd seen from the side.
function dog(g, w, h, pose) {
  const tan = "#b07a3c", black = "#2a221c";
  const body = pose === "lie" ? h * 0.62 : h * 0.42;
  g.fillStyle = tan;
  if (pose === "lie") {
    g.fillRect(44, h - 7, 26, 6); // front legs forward on the floor
  } else {
    const k = pose === "walk" ? 5 : 0;
    for (const x of [12 - k, 20 + k, 46 - k, 54 + k]) g.fillRect(x, body, 6, h - body);
  }
  g.beginPath();
  g.ellipse(34, body, 24, pose === "lie" ? 9 : 11, 0, 0, 7);
  g.fill();
  g.fillStyle = black; // saddle
  g.beginPath();
  g.ellipse(30, body - 4, 17, 7, 0, 0, 7);
  g.fill();
  g.strokeStyle = tan; // tail
  g.lineWidth = 5;
  g.beginPath();
  g.moveTo(11, body - 2);
  g.quadraticCurveTo(0, body + 6, 3, body + 16);
  g.stroke();
  const hx = 60, hy = body - (pose === "lie" ? 8 : 16);
  g.fillStyle = tan;
  g.fillRect(52, hy, 8, body - hy); // neck
  g.beginPath();
  g.ellipse(hx, hy, 9, 8, 0, 0, 7);
  g.fill();
  g.beginPath();
  g.ellipse(hx + 8, hy + 3, 6, 4, 0, 0, 7); // muzzle
  g.fill();
  g.fillStyle = black;
  g.beginPath(); // ears
  g.moveTo(hx - 6, hy - 4);
  g.lineTo(hx - 3, hy - 16);
  g.lineTo(hx + 1, hy - 5);
  g.fill();
  g.fillRect(hx + 12, hy + 1, 3, 3); // nose
  g.fillRect(hx + 2, hy - 2, 2, 2); // eye
}

function cat(g, w, h, walking) {
  const fur = "#d98a3d";
  const dark = "#a8612a";
  g.fillStyle = fur;
  if (walking) {
    g.fillRect(14, h * 0.55, 5, h * 0.45); // legs
    g.fillRect(22, h * 0.55, 5, h * 0.45);
    g.fillRect(40, h * 0.55, 5, h * 0.45);
    g.fillRect(48, h * 0.55, 5, h * 0.45);
    g.beginPath();
    g.ellipse(32, h * 0.45, 22, 9, 0, 0, 7);
    g.fill();
    g.strokeStyle = fur;
    g.lineWidth = 4;
    g.beginPath();
    g.moveTo(11, h * 0.42);
    g.quadraticCurveTo(2, h * 0.2, 6, 2);
    g.stroke();
  } else {
    g.beginPath();
    g.ellipse(28, h * 0.62, 24, 14, 0, 0, 7);
    g.fill();
    g.strokeStyle = fur;
    g.lineWidth = 4;
    g.beginPath();
    g.moveTo(8, h * 0.8);
    g.quadraticCurveTo(22, h * 1.02, 42, h * 0.88);
    g.stroke();
  }
  const hx = walking ? 54 : 50;
  const hy = walking ? h * 0.3 : h * 0.45;
  g.fillStyle = fur;
  g.beginPath();
  g.arc(hx, hy, 10, 0, 7);
  g.fill();
  g.beginPath(); // ears
  g.moveTo(hx - 9, hy - 4);
  g.lineTo(hx - 6, hy - 15);
  g.lineTo(hx - 1, hy - 8);
  g.moveTo(hx + 1, hy - 8);
  g.lineTo(hx + 6, hy - 15);
  g.lineTo(hx + 9, hy - 4);
  g.fill();
  g.fillStyle = dark; // stripes
  for (let i = 0; i < 3; i++) g.fillRect(20 + i * 9, (walking ? h * 0.38 : h * 0.5) - 2, 3, 8);
  g.fillStyle = "#1a1a1a";
  g.fillRect(hx - 5, hy - 2, 3, 3);
  g.fillRect(hx + 3, hy - 2, 3, 3);
  g.fillStyle = "#f2a0a0";
  g.fillRect(hx - 1, hy + 3, 3, 2);
}

function rr(g, x, y, w, h, r) {
  g.beginPath();
  g.roundRect(x, y, w, h, r);
  g.fill();
}

function line(g, x0, y0, x1, y1) {
  g.beginPath();
  g.moveTo(x0, y0);
  g.lineTo(x1, y1);
  g.stroke();
}

// Returns { name: { w, h, data } } with RGBA pixel data.
export function paintSprites() {
  const out = {};
  for (const [name, [w, h, paint]] of Object.entries(PAINTERS)) {
    const c = document.createElement("canvas");
    c.width = w;
    c.height = h;
    const g = c.getContext("2d");
    paint(g, w, h);
    out[name] = { w, h, data: g.getImageData(0, 0, w, h).data };
  }
  return out;
}
