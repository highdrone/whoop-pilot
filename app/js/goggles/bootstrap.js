// Synthetic bootstrap access units for a stream that never carries an IDR or an I frame (the
// goggles use intra refresh). A decoder needs one reference picture to start on; we hand it a
// flat mid-gray one coded ourselves, and the goggles' refresh wave then repaints the real picture
// within a second or two. This is what the SquirrelReceiver app does (its h264_bootstrap_image).
//
// Variants, best first (tools/test-bootstrap.mjs decodes each with ffmpeg):
//   iframe        recovery-point SEI + non-IDR I slice coded with the stream's own PPS (CABAC when
//                 the stream is CABAC), frame_num just before the first P frame: no PPS switch, no
//                 frame_num gap, and a decoder that keeps one PPS keeps the one the P frames use.
//   idr           the same picture as an IDR (frame_num 0); leaves a frame_num gap before the first
//                 P frame, but some decoders only start at an IDR.
//   iframe-cavlc  like iframe but coded with a CAVLC PPS of our own (only when the stream is CABAC).
//   idr-cavlc     like idr with our PPS.
// Every macroblock is I_16x16 with DC prediction and no residual, deblocking off, so each variant
// decodes to exactly Y=Cb=Cr=128 (ITU-T H.264 clauses 7.3, 7.4, 9.2 and 9.3).

// ---- bit reader / writer ------------------------------------------------------------------------

export const stripStartCode = (nal) => {
  let i = 0;
  while (i < nal.length - 1 && nal[i] === 0) i++;
  return nal[i] === 1 && i >= 2 ? nal.subarray(i + 1) : nal;
};

// RBSP of a NAL (header dropped, emulation-prevention bytes removed).
export function rbspOf(nal) {
  const b = stripStartCode(nal);
  const out = [];
  let zeros = 0;
  for (let i = 1; i < b.length; i++) {
    if (zeros >= 2 && b[i] === 3) {
      zeros = 0;
      continue;
    }
    out.push(b[i]);
    zeros = b[i] === 0 ? zeros + 1 : 0;
  }
  return Uint8Array.from(out);
}

export class BitReader {
  constructor(bytes) {
    this.b = bytes;
    this.pos = 0;
    let last = bytes.length * 8 - 1;
    while (last >= 0 && !((bytes[last >> 3] >> (7 - (last & 7))) & 1)) last--;
    this.stop = last; // position of the rbsp_stop_one_bit
  }
  bit() {
    const v = (this.b[this.pos >> 3] >> (7 - (this.pos & 7))) & 1;
    this.pos++;
    return v;
  }
  u(n) {
    let v = 0;
    for (let i = 0; i < n; i++) v = v * 2 + this.bit();
    return v;
  }
  ue() {
    let zeros = 0;
    while (this.bit() === 0) if (++zeros > 32) throw new Error("bad exp-golomb code");
    return (zeros ? 2 ** zeros - 1 + this.u(zeros) : 0);
  }
  se() {
    const k = this.ue();
    return k & 1 ? (k + 1) / 2 : 0 - k / 2; // 0 - k, not -k: no -0 for code 0
  }
  moreRbspData() {
    return this.pos < this.stop;
  }
}

export class BitWriter {
  constructor() {
    this.bytes = [];
    this.cur = 0;
    this.n = 0; // bits in cur
  }
  bit(v) {
    this.cur = (this.cur << 1) | (v & 1);
    if (++this.n === 8) {
      this.bytes.push(this.cur);
      this.cur = this.n = 0;
    }
  }
  u(v, n) {
    for (let i = n - 1; i >= 0; i--) this.bit(Math.floor(v / 2 ** i) & 1);
  }
  ue(v) {
    const len = Math.floor(Math.log2(v + 1));
    this.u(0, len);
    this.u(v + 1, len + 1);
  }
  se(v) {
    this.ue(v > 0 ? 2 * v - 1 : -2 * v);
  }
  get aligned() {
    return this.n === 0;
  }
  align(fill) {
    while (this.n) this.bit(fill);
  }
  trailing() {
    this.bit(1);
    this.align(0);
  }
  data() {
    if (this.n) throw new Error("bit writer not byte-aligned");
    return Uint8Array.from(this.bytes);
  }
}

// Annex-B NAL (4-byte start code) from a header and its RBSP, with emulation prevention.
export function makeNal(refIdc, type, rbsp) {
  const out = [0, 0, 0, 1, (refIdc << 5) | type];
  let zeros = 0;
  for (const x of rbsp) {
    if (zeros >= 2 && x <= 3) {
      out.push(3);
      zeros = 0;
    }
    out.push(x);
    zeros = x === 0 ? zeros + 1 : 0;
  }
  return Uint8Array.from(out);
}

// ---- parameter sets and slice headers -----------------------------------------------------------

const HIGH_PROFILES = new Set([100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135]);

function skipScalingList(r, size) {
  let last = 8;
  let next = 8;
  for (let j = 0; j < size && next !== 0; j++) {
    next = (last + r.se() + 256) % 256;
    last = next === 0 ? last : next;
  }
}

export function parseSps(nal) {
  const r = new BitReader(rbspOf(nal));
  const s = { profileIdc: r.u(8), constraints: r.u(8), levelIdc: r.u(8), spsId: r.ue() };
  s.chromaFormatIdc = 1;
  s.separateColourPlane = 0;
  s.bitDepthLuma = s.bitDepthChroma = 8;
  s.qpprimeYZeroTransformBypass = 0;
  s.scalingMatrixPresent = 0;
  if (HIGH_PROFILES.has(s.profileIdc)) {
    s.chromaFormatIdc = r.ue();
    if (s.chromaFormatIdc === 3) s.separateColourPlane = r.u(1);
    s.bitDepthLuma = 8 + r.ue();
    s.bitDepthChroma = 8 + r.ue();
    s.qpprimeYZeroTransformBypass = r.u(1);
    s.scalingMatrixPresent = r.u(1);
    if (s.scalingMatrixPresent) for (let i = 0; i < (s.chromaFormatIdc !== 3 ? 8 : 12); i++) if (r.u(1)) skipScalingList(r, i < 6 ? 16 : 64);
  }
  s.log2MaxFrameNum = 4 + r.ue();
  s.pocType = r.ue();
  s.log2MaxPocLsb = 0;
  s.deltaPicOrderAlwaysZero = 0;
  if (s.pocType === 0) s.log2MaxPocLsb = 4 + r.ue();
  else if (s.pocType === 1) {
    s.deltaPicOrderAlwaysZero = r.u(1);
    s.offsetForNonRefPic = r.se();
    s.offsetForTopToBottomField = r.se();
    const n = r.ue();
    s.offsetForRefFrame = [];
    for (let i = 0; i < n; i++) s.offsetForRefFrame.push(r.se());
  }
  s.maxNumRefFrames = r.ue();
  s.gapsAllowed = r.u(1);
  s.widthMbs = 1 + r.ue();
  s.heightMapUnits = 1 + r.ue();
  s.frameMbsOnly = r.u(1);
  s.mbaff = s.frameMbsOnly ? 0 : r.u(1);
  s.direct8x8Inference = r.u(1);
  s.crop = r.u(1) ? { left: r.ue(), right: r.ue(), top: r.ue(), bottom: r.ue() } : { left: 0, right: 0, top: 0, bottom: 0 };
  s.vuiPresent = r.u(1);
  s.chromaArrayType = s.separateColourPlane ? 0 : s.chromaFormatIdc;
  s.frameHeightMbs = (2 - s.frameMbsOnly) * s.heightMapUnits;
  const cropX = s.chromaArrayType === 0 || s.chromaArrayType === 3 ? 1 : 2;
  const cropY = (s.chromaArrayType === 1 ? 2 : 1) * (2 - s.frameMbsOnly);
  s.width = s.widthMbs * 16 - cropX * (s.crop.left + s.crop.right);
  s.height = s.frameHeightMbs * 16 - cropY * (s.crop.top + s.crop.bottom);
  return s;
}

export function parsePps(nal, sps = null) {
  const r = new BitReader(rbspOf(nal));
  const p = { ppsId: r.ue(), spsId: r.ue(), entropyCodingMode: r.u(1), bottomFieldPicOrderInFramePresent: r.u(1), numSliceGroups: 1 + r.ue() };
  if (p.numSliceGroups > 1) {
    p.sliceGroupMapType = r.ue();
    const t = p.sliceGroupMapType;
    if (t === 0) for (let i = 0; i < p.numSliceGroups; i++) r.ue();
    else if (t === 2) for (let i = 0; i < p.numSliceGroups - 1; i++) r.ue(), r.ue();
    else if (t >= 3 && t <= 5) r.u(1), r.ue();
    else if (t === 6) {
      const n = 1 + r.ue();
      const bits = Math.ceil(Math.log2(p.numSliceGroups));
      for (let i = 0; i < n; i++) r.u(bits);
    }
  }
  p.numRefIdxL0DefaultActive = 1 + r.ue();
  p.numRefIdxL1DefaultActive = 1 + r.ue();
  p.weightedPred = r.u(1);
  p.weightedBipredIdc = r.u(2);
  p.picInitQp = 26 + r.se();
  p.picInitQs = 26 + r.se();
  p.chromaQpIndexOffset = r.se();
  p.deblockingFilterControlPresent = r.u(1);
  p.constrainedIntraPred = r.u(1);
  p.redundantPicCntPresent = r.u(1);
  p.transform8x8Mode = 0;
  p.picScalingMatrixPresent = 0;
  p.secondChromaQpIndexOffset = p.chromaQpIndexOffset;
  if (r.moreRbspData()) {
    p.transform8x8Mode = r.u(1);
    p.picScalingMatrixPresent = r.u(1);
    if (p.picScalingMatrixPresent) {
      const n = 6 + ((sps?.chromaFormatIdc === 3 ? 6 : 2) * p.transform8x8Mode);
      for (let i = 0; i < n; i++) if (r.u(1)) skipScalingList(r, i < 6 ? 16 : 64);
    }
    p.secondChromaQpIndexOffset = r.se();
  }
  return p;
}

// The fields up to the picture order count; enough to place a bootstrap picture before this one.
export function parseSliceHeader(nal, sps, pps) {
  const b = stripStartCode(nal);
  const type = b[0] & 0x1f;
  const r = new BitReader(rbspOf(b));
  const h = { nalType: type, refIdc: (b[0] >> 5) & 3, idr: type === 5, firstMb: r.ue(), sliceType: r.ue(), ppsId: r.ue() };
  if (sps.separateColourPlane) h.colourPlaneId = r.u(2);
  h.frameNum = r.u(sps.log2MaxFrameNum);
  h.fieldPic = sps.frameMbsOnly ? 0 : r.u(1);
  if (h.fieldPic) h.bottomField = r.u(1);
  if (h.idr) h.idrPicId = r.ue();
  if (sps.pocType === 0) {
    h.pocLsb = r.u(sps.log2MaxPocLsb);
    if (pps.bottomFieldPicOrderInFramePresent && !h.fieldPic) h.deltaPocBottom = r.se();
  } else if (sps.pocType === 1 && !sps.deltaPicOrderAlwaysZero) {
    h.deltaPoc = [r.se()];
    if (pps.bottomFieldPicOrderInFramePresent && !h.fieldPic) h.deltaPoc.push(r.se());
  }
  if (pps.redundantPicCntPresent) h.redundantPicCnt = r.ue();
  return h;
}

// ---- CABAC encoder (the arithmetic coder of clause 9.3, run forwards) --------------------------

// rangeTabLPS[pStateIdx][qCodIRangeIdx]
const RANGE_LPS = [
  [128, 176, 208, 240], [128, 167, 197, 227], [128, 158, 187, 216], [123, 150, 178, 205], [116, 142, 169, 195], [111, 135, 160, 185], [105, 128, 152, 175], [100, 122, 144, 166],
  [95, 116, 137, 158], [90, 110, 130, 150], [85, 104, 123, 142], [81, 99, 117, 135], [77, 94, 111, 128], [73, 89, 105, 122], [69, 85, 100, 116], [66, 80, 95, 110],
  [62, 76, 90, 104], [59, 72, 86, 99], [56, 69, 81, 94], [53, 65, 77, 89], [51, 62, 73, 85], [48, 59, 69, 80], [46, 56, 66, 76], [43, 53, 63, 72],
  [41, 50, 59, 69], [39, 48, 56, 65], [37, 45, 54, 62], [35, 43, 51, 59], [33, 41, 48, 56], [32, 39, 46, 53], [30, 37, 43, 50], [29, 35, 41, 48],
  [27, 33, 39, 45], [26, 31, 37, 43], [24, 30, 35, 41], [23, 28, 33, 39], [22, 27, 32, 37], [21, 26, 30, 35], [20, 24, 29, 33], [19, 23, 27, 31],
  [18, 22, 26, 30], [17, 21, 25, 28], [16, 20, 23, 27], [15, 19, 22, 25], [14, 18, 21, 24], [14, 17, 20, 23], [13, 16, 19, 22], [12, 15, 18, 21],
  [12, 14, 17, 20], [11, 14, 16, 19], [11, 13, 15, 18], [10, 12, 15, 17], [10, 12, 14, 16], [9, 11, 13, 15], [9, 11, 12, 14], [8, 10, 12, 14],
  [8, 9, 11, 13], [7, 9, 11, 12], [7, 9, 10, 12], [7, 8, 10, 11], [6, 8, 9, 11], [6, 7, 9, 10], [6, 7, 8, 9], [2, 2, 2, 2],
];
const TRANS_LPS = [
  0, 0, 1, 2, 2, 4, 4, 5, 6, 7, 8, 9, 9, 11, 11, 12, 13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
  24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33, 33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63,
];

// Initialisation (m, n) pairs for I slices, only the contexts a gray I_16x16 picture touches.
const CTX_INIT_I = {
  3: [20, -15], 4: [2, 54], 5: [3, 74], 6: [-28, 127], 7: [-23, 104], 8: [-6, 53], 9: [-1, 54], 10: [7, 51], // mb_type
  60: [0, 41], // mb_qp_delta
  64: [-9, 83], // intra_chroma_pred_mode
  85: [-17, 123], 86: [-12, 115], 87: [-16, 122], 88: [-11, 115], // coded_block_flag, Intra16x16DCLevel
};

class CabacEncoder {
  constructor(writer, sliceQp) {
    this.w = writer;
    this.low = 0;
    this.range = 510;
    this.first = true;
    this.outstanding = 0;
    this.ctx = {};
    const qp = Math.min(51, Math.max(0, sliceQp));
    for (const [i, [m, n]] of Object.entries(CTX_INIT_I)) {
      const pre = Math.min(126, Math.max(1, ((m * qp) >> 4) + n));
      this.ctx[i] = pre <= 63 ? { state: 63 - pre, mps: 0 } : { state: pre - 64, mps: 1 };
    }
  }
  putBit(b) {
    if (this.first) this.first = false;
    else this.w.bit(b);
    while (this.outstanding > 0) {
      this.w.bit(1 - b);
      this.outstanding--;
    }
  }
  renorm() {
    while (this.range < 256) {
      if (this.low < 256) this.putBit(0);
      else if (this.low >= 512) {
        this.low -= 512;
        this.putBit(1);
      } else {
        this.low -= 256;
        this.outstanding++;
      }
      this.range <<= 1;
      this.low <<= 1;
    }
  }
  encode(ctxIdx, bin) {
    const c = this.ctx[ctxIdx];
    const lps = RANGE_LPS[c.state][(this.range >> 6) & 3];
    this.range -= lps;
    if (bin !== c.mps) {
      this.low += this.range;
      this.range = lps;
      if (c.state === 0) c.mps = 1 - c.mps;
      c.state = TRANS_LPS[c.state];
    } else c.state = Math.min(c.state + 1, 62);
    this.renorm();
  }
  terminate(bin) {
    this.range -= 2;
    if (bin) {
      this.low += this.range;
      this.range = 2;
      this.renorm();
      this.putBit((this.low >> 9) & 1);
      this.w.u(((this.low >> 7) & 3) | 1, 2); // ends with the rbsp_stop_one_bit
    } else this.renorm();
  }
}

// ---- the gray picture -------------------------------------------------------------------------

function writeSliceHeader(w, { idr, frameNum, pocLsb }, sps, pps) {
  w.ue(0); // first_mb_in_slice
  w.ue(7); // slice_type: I, and so are all slices of the picture
  w.ue(pps.ppsId);
  if (sps.separateColourPlane) throw new Error("separate colour planes are not supported");
  w.u(frameNum, sps.log2MaxFrameNum);
  if (!sps.frameMbsOnly) w.u(0, 1); // field_pic_flag
  if (idr) w.ue(0); // idr_pic_id
  if (sps.pocType === 0) {
    w.u(pocLsb, sps.log2MaxPocLsb);
    if (pps.bottomFieldPicOrderInFramePresent) w.se(0);
  } else if (sps.pocType === 1 && !sps.deltaPicOrderAlwaysZero) {
    w.se(0);
    if (pps.bottomFieldPicOrderInFramePresent) w.se(0);
  }
  if (pps.redundantPicCntPresent) w.ue(0);
  // dec_ref_pic_marking (nal_ref_idc is 3)
  if (idr) w.u(0, 2); // no_output_of_prior_pics_flag, long_term_reference_flag
  else w.u(0, 1); // adaptive_ref_pic_marking_mode_flag
  w.se(0); // slice_qp_delta
  if (pps.deblockingFilterControlPresent) w.ue(1); // disable_deblocking_filter_idc: off
  if (pps.numSliceGroups > 1) throw new Error("slice groups are not supported");
}

// One slice covering the picture: every macroblock I_16x16, DC prediction (mb_type 3), chroma DC,
// no residual, so the decoder paints 1 << (bitDepth - 1) everywhere.
export function graySlice({ idr = false, frameNum = 0, pocLsb = 0 }, sps, pps) {
  if (sps.mbaff) throw new Error("MBAFF streams are not supported");
  if (sps.chromaArrayType === 3) throw new Error("4:4:4 streams are not supported");
  const w = new BitWriter();
  writeSliceHeader(w, { idr, frameNum, pocLsb }, sps, pps);
  const W = sps.widthMbs;
  const count = W * sps.frameHeightMbs;
  const chroma = sps.chromaArrayType !== 0;
  if (pps.entropyCodingMode) {
    w.align(1); // cabac_alignment_one_bit
    const c = new CabacEncoder(w, pps.picInitQp);
    for (let mb = 0; mb < count; mb++) {
      const a = mb % W > 0 ? 1 : 0; // left neighbour available
      const b = mb >= W ? 1 : 0; // top neighbour available
      c.encode(3 + a + b, 1); // mb_type bins for I_16x16_2_0_0: 1 0 0 0 1 0
      c.terminate(0);
      c.encode(6, 0);
      c.encode(7, 0);
      c.encode(9, 1);
      c.encode(10, 0);
      if (chroma) c.encode(64, 0); // intra_chroma_pred_mode DC
      c.encode(60, 0); // mb_qp_delta 0
      c.encode(85 + (1 - a) + 2 * (1 - b), 0); // coded_block_flag of the DC block
      c.terminate(mb === count - 1 ? 1 : 0); // end_of_slice_flag
    }
    w.align(0);
  } else {
    for (let mb = 0; mb < count; mb++) {
      w.ue(3); // mb_type I_16x16_2_0_0
      if (chroma) w.ue(0); // intra_chroma_pred_mode DC
      w.se(0); // mb_qp_delta
      w.u(1, 1); // coeff_token: no DC coefficients (nC 0)
    }
    w.trailing();
  }
  return makeNal(3, idr ? 5 : 1, w.data());
}

// A CAVLC PPS of our own next to the stream's: same SPS, deblocking switchable, nothing else.
export function cavlcPps(sps, streamPps) {
  const w = new BitWriter();
  w.ue(streamPps.ppsId === 0 ? 1 : 0);
  w.ue(sps.spsId);
  w.u(0, 2); // entropy_coding_mode_flag, bottom_field_pic_order_in_frame_present_flag
  w.ue(0); // num_slice_groups_minus1
  w.ue(0);
  w.ue(0); // num_ref_idx_l0/l1_default_active_minus1
  w.u(0, 3); // weighted_pred_flag, weighted_bipred_idc
  w.se(0);
  w.se(0);
  w.se(0); // pic_init_qp_minus26, pic_init_qs_minus26, chroma_qp_index_offset
  w.u(1, 1); // deblocking_filter_control_present_flag
  w.u(0, 2); // constrained_intra_pred_flag, redundant_pic_cnt_present_flag
  w.trailing(); // no High-profile extension: transform_8x8_mode_flag etc. are inferred 0
  return makeNal(3, 8, w.data());
}

// Recovery point SEI: recovery_frame_cnt 0, exact_match 0, broken_link 0, changing_slice_group_idc 0.
export function recoverySei() {
  const w = new BitWriter();
  w.u(6, 8); // payloadType
  w.u(1, 8); // payloadSize
  w.ue(0);
  w.u(0, 4);
  w.trailing(); // payload bits end with their own stop bit
  w.trailing(); // and so does the SEI RBSP
  return makeNal(0, 6, w.data());
}

function concat(...parts) {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let o = 0;
  for (const p of parts) {
    out.set(p, o);
    o += p.length;
  }
  return out;
}

const withStartCode = (nal) => concat(new Uint8Array([0, 0, 0, 1]), stripStartCode(nal));

// Candidate access units to start a decoder just before the slice `firstSliceNal` (the first P
// frame we will feed after them), best first: [{ name, data }].
export function bootstrapUnits(spsNal, ppsNal, firstSliceNal) {
  const sps = parseSps(spsNal);
  const pps = parsePps(ppsNal, sps);
  const first = parseSliceHeader(firstSliceNal, sps, pps);
  const before = {
    frameNum: (first.frameNum - 1 + 2 ** sps.log2MaxFrameNum) % 2 ** sps.log2MaxFrameNum,
    pocLsb: sps.pocType === 0 ? (first.pocLsb - 2 + 2 ** sps.log2MaxPocLsb) % 2 ** sps.log2MaxPocLsb : 0,
  };
  const params = concat(withStartCode(spsNal), withStartCode(ppsNal));
  const out = [
    { name: "iframe", data: concat(params, recoverySei(), graySlice(before, sps, pps)) },
    { name: "idr", data: concat(params, graySlice({ idr: true }, sps, pps)) },
  ];
  if (pps.entropyCodingMode) {
    const ours = cavlcPps(sps, pps);
    const ownPps = parsePps(ours, sps);
    out.push({ name: "iframe-cavlc", data: concat(params, ours, recoverySei(), graySlice(before, sps, ownPps)) });
    out.push({ name: "idr-cavlc", data: concat(params, ours, graySlice({ idr: true }, sps, ownPps)) });
  }
  return out;
}
