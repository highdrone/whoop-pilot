// Minimal H.264 Annex-B inspection: find NAL units, read a slice's type, spot SEI recovery points,
// and build the codec string WebCodecs needs.

export const NAL = { SLICE: 1, IDR: 5, SEI: 6, SPS: 7, PPS: 8, AUD: 9 };

// [{ type, start, end }] where start is the NAL header byte (after the start code).
export function nalUnits(b) {
  const out = [];
  let i = 0;
  let prev = -1;
  while (i + 3 <= b.length) {
    if (b[i] === 0 && b[i + 1] === 0 && b[i + 2] === 1) {
      const start = i + 3;
      if (prev >= 0) out[out.length - 1].end = i > 0 && b[i - 1] === 0 ? i - 1 : i;
      out.push({ type: b[start] & 0x1f, start, end: b.length });
      prev = start;
      i = start;
    } else i++;
  }
  return out;
}

// RBSP bytes of a NAL (emulation-prevention 0x03 removed), up to `max` bytes.
function rbsp(b, start, end, max = 64) {
  const out = [];
  for (let i = start + 1; i < end && out.length < max; i++) {
    if (i + 2 < end && b[i] === 0 && b[i + 1] === 0 && b[i + 2] === 3) {
      out.push(0, 0);
      i += 2;
    } else out.push(b[i]);
  }
  return out;
}

class Bits {
  constructor(bytes) {
    this.b = bytes;
    this.pos = 0;
  }
  bit() {
    const v = (this.b[this.pos >> 3] >> (7 - (this.pos & 7))) & 1;
    this.pos++;
    return v;
  }
  ue() {
    let zeros = 0;
    while (this.bit() === 0 && zeros < 32) zeros++;
    let v = 1;
    for (let i = 0; i < zeros; i++) v = (v << 1) | this.bit();
    return v - 1;
  }
}

// slice_type (0-9; % 5: 0 P, 1 B, 2 I, 3 SP, 4 SI) of a slice NAL, or -1.
export function sliceType(b, nal) {
  const r = rbsp(b, nal.start, nal.end, 12);
  if (r.length < 2) return -1;
  const bits = new Bits(r);
  bits.ue(); // first_mb_in_slice
  return bits.ue();
}

// Does an SEI NAL carry a recovery point (payload type 6)?
export function hasRecoveryPoint(b, nal) {
  const r = rbsp(b, nal.start, nal.end, 256);
  let i = 0;
  while (i < r.length && r[i] !== 0x80) {
    let type = 0;
    while (r[i] === 0xff) type += r[i++];
    type += r[i++];
    let size = 0;
    while (r[i] === 0xff) size += r[i++];
    size += r[i++];
    if (type === 6) return true;
    i += size;
  }
  return false;
}

const hex2 = (n) => n.toString(16).padStart(2, "0");
export const codecString = (b, sps) => `avc1.${hex2(b[sps.start + 1])}${hex2(b[sps.start + 2])}${hex2(b[sps.start + 3])}`;

// SEI with a recovery point (recovery_frame_cnt 0): Chrome's decoder accepts an access unit that
// carries one as a starting point, like an IDR.
export const RECOVERY_SEI = new Uint8Array([0, 0, 0, 1, 0x06, 0x06, 0x01, 0xc4, 0x80]);

export function describeUnit(b) {
  const nals = nalUnits(b);
  const info = { nals, types: nals.map((n) => n.type), idr: false, intra: false, recovery: false, sps: null, pps: null, slices: 0, firstSlice: null };
  let allIntra = true;
  for (const n of nals) {
    if (n.type === NAL.IDR) info.idr = true;
    else if (n.type === NAL.SPS) info.sps = n;
    else if (n.type === NAL.PPS) info.pps = n;
    else if (n.type === NAL.SEI && hasRecoveryPoint(b, n)) info.recovery = true;
    if (n.type === NAL.SLICE || n.type === NAL.IDR) {
      info.slices++;
      info.firstSlice ??= n;
      const t = sliceType(b, n) % 5;
      if (t !== 2 && t !== 4) allIntra = false;
    }
  }
  info.intra = info.slices > 0 && allIntra;
  return info;
}

export function concat(...parts) {
  let n = 0;
  for (const p of parts) n += p.length;
  const out = new Uint8Array(n);
  let o = 0;
  for (const p of parts) {
    out.set(p, o);
    o += p.length;
  }
  return out;
}

// A NAL with a 4-byte start code, copied out of a unit.
export const nalBytes = (b, n) => concat(new Uint8Array([0, 0, 0, 1]), b.subarray(n.start, n.end));
