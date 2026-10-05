// Wire protocol between the app and radio/SCRIPTS/MIXES/aibrg.lua (keep both in sync).
//
// App -> radio, ~30 Hz:  $C,seq,mask,roll,pitch,thr,yaw,hover*CS\n
// Radio -> app, 10 Hz:   $T,ver,seq,flags,ail,ele,thr,rud,rxbt,rqly,rssi,ptch,roll,yaw,alt,rxok,rxbad,fm*CS\n
// Values are EdgeTX channel units (-1024..1024). CS = byte sum of the text between
// '$' and the last '*', mod 256, as two hex digits. fm is Betaflight's flight mode text, which
// itself ends in '*' while disarmed ("STAB*") and can be "!FS!".

export const MASK = { roll: 1, pitch: 2, thr: 4, yaw: 8 };

export function checksum(text) {
  let sum = 0;
  for (let i = 0; i < text.length; i++) sum = (sum + text.charCodeAt(i)) % 256;
  return sum;
}

const hex2 = (n) => n.toString(16).toUpperCase().padStart(2, "0");
const toRaw = (v) => Math.max(-1024, Math.min(1024, Math.round(v * 1024)));
const thrToRaw = (t) => toRaw(t * 2 - 1);

// Sticks are normalized: roll/pitch/yaw in [-1, 1] (+ = right / forward / clockwise),
// thr and hover in [0, 1] (0 = idle).
export function encodeCommand({ seq, mask, roll, pitch, thr, yaw, hover }) {
  const body = `C,${seq % 10000},${mask},${toRaw(roll)},${toRaw(pitch)},${thrToRaw(thr)},${toRaw(yaw)},${thrToRaw(hover)}`;
  return `$${body}*${hex2(checksum(body))}\n`;
}

const TELEMETRY = /^\$(T,.*)\*([0-9A-Fa-f]{2})\s*$/;

// Why a line from the radio isn't valid telemetry (for the diagnostics the app shows).
export function explainTelemetry(line) {
  const m = TELEMETRY.exec(line);
  if (!m) return "not a $T…*CS line";
  const want = hex2(checksum(m[1]));
  if (want !== m[2].toUpperCase()) return `checksum ${m[2]}, expected ${want}`;
  return `${m[1].split(",").length} fields, expected 18`;
}

export function decodeTelemetry(line) {
  const m = TELEMETRY.exec(line);
  if (!m || checksum(m[1]) !== parseInt(m[2], 16)) return null;
  const f = m[1].split(",");
  if (f.length < 18) return null;
  const n = (i) => Number(f[i]) || 0;
  const flags = n(3);
  return {
    version: n(1),
    seq: n(2),
    engaged: (flags & 1) !== 0,
    linkOk: (flags & 2) !== 0,
    failsafe: (flags & 4) !== 0,
    ptt: (flags & 8) !== 0,
    override: {
      roll: (flags & 16) !== 0,
      pitch: (flags & 32) !== 0,
      thr: (flags & 64) !== 0,
      yaw: (flags & 128) !== 0,
    },
    sticks: { roll: n(4) / 1024, pitch: n(5) / 1024, thr: (n(6) / 1024 + 1) / 2, yaw: n(7) / 1024 },
    vbat: n(8) / 100,
    lq: n(9),
    rssi: n(10),
    att: { pitch: n(11) / 1000, roll: n(12) / 1000, yaw: n(13) / 1000 },
    alt: n(14) / 100,
    rxOk: n(15),
    rxBad: n(16),
    fm: f[17] || "",
  };
}
