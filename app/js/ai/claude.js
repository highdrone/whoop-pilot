// The one place the AI features (ai/survey.js, ai/inspect.js, ai/recall.js) call Claude: a single Messages request with a
// cached system prompt, pictures, and a JSON-schema answer through agent.js's client, model options (adaptive thinking,
// effort, server-side refusal fallbacks) and PRICES, its usage and cost reported through the agent's "usage" event and
// session total (with a purpose). The answer's shape: structured outputs (output_config.format, a json_schema), which
// work with adaptive thinking on Opus 5/5.5, Sonnet 5/5.5, Fable 5/5.1 and Haiku 4.5; a forced tool_choice is refused
// (400) by Opus 5.5, Sonnet 5.5 and Fable 5.1, so it is never used. A model without structured outputs (Opus 4.6/4.7,
// Sonnet 4.6) gets the schema in the prompt and its JSON is read from the text. Cost: priced by the model that answered
// (a refusal fallback answers as another model: usage.iterations, each attempt at its own model's price), and each
// purpose's real cost against its estimate keeps a ratio that later budget checks apply.
// Claude's picture checks (the survey, change and alert checks: setting aiVision) send pictures of the house only with
// consent ("on"; "ask" asks once through setConsent's prompt; "off" never; voice commands send the camera view either
// way, as they always have), and the calls of one flight stay within settings aiBudget (US$); a survey brings its own
// limit. The prompt is never raised by a call that must not wait for a person (ask: false: an alert, a background
// check) or while canAsk() says no (flying: a prompt would take the keyboard from push-to-talk, land and stop): such a
// call goes without, and "consent-needed" ({ purpose }) tells the UI once, for a banner whose answer goes to
// answerConsent(). With deadline (ms) a call gives up on a slow answer.
//   const claude = new Claude({ settings, agent });
//   claude.setConsent(async ({ purpose, text }) => true | "once" | false | "off", { canAsk: () => !ctl.isFlying() });
//   const r = await claude.call({ purpose: "change", system, content: [text("…"), image(b64)], schema });
//   r -> { ok: true, data, usage, cost } | { ok: false, reason, budget?, consent?, waiting?, late? }
// Events: "usage", "error", "consent-needed" ({ purpose }), "consent" (the answer: "on" | "once" | "not now" | "off").
import { PRICES, costOf, modelOptions, claudeClient, pricedUsage } from "../agent.js";
import { Emitter } from "../util.js";

export const AI = { maxTokens: 16000, effort: "low", expectOut: 800, fallbackPrice: "claude-fable-5", checkDeadline: 60000, learn: 0.3 };
// Models with structured outputs (output_config.format).
export const STRUCTURED = /^claude-(opus-5|opus-4-8|opus-4-5|opus-4-1|sonnet-5|fable|mythos|haiku-4-5)/;
export const OFF_TEXT = "Claude's picture checks are off in Settings";

// Models that take pictures up to 2576 px on the long edge (4784 tokens); older ones 1568 px (about 1600 tokens).
const HIGH_RES = /^claude-(opus-5|opus-4-[78]|sonnet-5|fable|mythos)/;
// Tokens a picture costs: about width × height / 750 after the API scales it to fit the model's limits.
export function imageTokens(width, height, model = "claude-opus-5") {
  const hi = HIGH_RES.test(model), k = Math.min(1, (hi ? 2576 : 1568) / Math.max(width, height));
  return Math.min(hi ? 4784 : 1600, Math.ceil((width * k * height * k) / 750));
}
// Text tokens, roughly (4 characters each).
export const textTokens = (s) => Math.ceil(String(s ?? "").length / 4);

export const text = (t) => ({ type: "text", text: String(t) });
export const image = (b64) => ({ type: "image", source: { type: "base64", media_type: "image/jpeg", data: b64 } });

export function bytesToBase64(bytes) {
  let s = "";
  for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(s);
}

// A picture as base64 JPEG: base64 already, a data URL, a Blob, or pixels ({ width, height, data } RGBA, ImageData,
// ImageBitmap, a canvas) through an OffscreenCanvas (browser). box { x, y, w, h } (normalized) crops it first, with
// half its size again around it. null if there's nothing to send.
export async function toJpegBase64(img, { box = null, quality = 0.85, minSide = 224 } = {}) {
  if (!img) return null;
  if (typeof img === "string" && !box) return img.startsWith("data:") ? img.slice(img.indexOf(",") + 1) : img;
  if (typeof OffscreenCanvas === "undefined") return typeof img === "string" ? img.replace(/^data:[^,]*,/, "") : img instanceof Blob ? bytesToBase64(new Uint8Array(await img.arrayBuffer())) : null;
  let src = img;
  if (typeof img === "string") src = new Blob([Uint8Array.from(atob(img.replace(/^data:[^,]*,/, "")), (c) => c.charCodeAt(0))], { type: "image/jpeg" });
  if (src instanceof Blob) {
    if (!box) return bytesToBase64(new Uint8Array(await src.arrayBuffer()));
    src = await createImageBitmap(src);
  }
  if (src.data && !(src instanceof ImageData)) src = new ImageData(new Uint8ClampedArray(src.data.buffer, src.data.byteOffset, src.data.length), src.width, src.height);
  const W = src.width, H = src.height;
  let [sx, sy, sw, sh] = [0, 0, W, H];
  if (box) {
    const cx = (box.x + box.w / 2) * W, cy = (box.y + box.h / 2) * H, side = Math.max(minSide, 1.5 * Math.max(box.w * W, box.h * H));
    [sw, sh] = [Math.min(W, side), Math.min(H, side)];
    [sx, sy] = [Math.max(0, Math.min(W - sw, cx - sw / 2)), Math.max(0, Math.min(H - sh, cy - sh / 2))];
  }
  const c = new OffscreenCanvas(Math.round(sw), Math.round(sh)), g = c.getContext("2d");
  if (src instanceof ImageData) {
    const full = new OffscreenCanvas(W, H);
    full.getContext("2d").putImageData(src, 0, 0);
    src = full;
  }
  g.drawImage(src, sx, sy, sw, sh, 0, 0, c.width, c.height);
  return bytesToBase64(new Uint8Array(await (await c.convertToBlob({ type: "image/jpeg", quality })).arrayBuffer()));
}

// Width and height of a JPEG or PNG (base64, a data URL, a Blob or bytes) or of pixels ({ width, height }); null if unknown.
export async function imageSize(img) {
  if (!img) return null;
  if (Number.isFinite(img.width) && Number.isFinite(img.height) && !(typeof Blob !== "undefined" && img instanceof Blob)) return { width: img.width, height: img.height };
  let b = img;
  if (typeof b === "string") b = Uint8Array.from(atob(b.replace(/^data:[^,]*,/, "").slice(0, 87384)), (c) => c.charCodeAt(0));
  else if (typeof Blob !== "undefined" && b instanceof Blob) b = new Uint8Array(await b.arrayBuffer());
  if (!(b instanceof Uint8Array)) return null;
  if (b[0] === 0x89 && b[1] === 0x50) return { width: (b[16] << 24) | (b[17] << 16) | (b[18] << 8) | b[19], height: (b[20] << 24) | (b[21] << 16) | (b[22] << 8) | b[23] };
  if (b[0] !== 0xff || b[1] !== 0xd8) return null;
  for (let i = 2; i + 9 < b.length; ) {
    if (b[i] !== 0xff) return null;
    const m = b[i + 1];
    if (m >= 0xc0 && m <= 0xcf && ![0xc4, 0xc8, 0xcc].includes(m)) return { width: (b[i + 7] << 8) | b[i + 8], height: (b[i + 5] << 8) | b[i + 6] };
    i += 2 + ((b[i + 2] << 8) | b[i + 3]);
  }
  return null;
}

function errorText(e) {
  if (e?.name === "AbortError" || /abort/i.test(e?.constructor?.name ?? "")) return "stopped";
  const s = e?.status;
  if (s === 401) return "the Claude API key was rejected";
  if (s === 403) return "this API key isn't allowed to use that model";
  if (s === 404) return "the model wasn't found";
  if (s === 429) return "Claude is rate-limiting us";
  if (s === 400) return `Claude rejected the request: ${e.message}`;
  if (s === 413) return "the request was too large for Claude (too many or too big pictures)";
  if (s === 529) return "Claude is overloaded right now: try again in a minute";
  if (s >= 500) return `Claude had a server error (${s})`;
  return /connect|fetch|network/i.test(e?.message ?? "") ? "can't reach Claude" : `Claude call failed: ${e?.message ?? e}`;
}

export class Claude extends Emitter {
  constructor({ settings, agent = null, fetch = null, canAsk = () => true }) {
    super();
    Object.assign(this, { settings, agent, fetch, canAsk, askConsent: null, declined: false, asking: null });
    this.needed = new Set(); // purposes "consent-needed" was emitted for, until an answer
    this.flight = { id: null, spent: 0 };
    this.session = { requests: 0, cost: 0, estimated: 0 };
    this.ratio = {}; // purpose -> real cost / estimate (moving average): later estimates for the budget are this much more
  }

  get model() {
    return this.settings.get("aiModel") || this.settings.get("model") || "claude-opus-5";
  }
  get configured() {
    return !!this.settings.get("apiKey");
  }
  get budget() {
    return Math.max(0, Number(this.settings.get("aiBudget") ?? 0.5));
  }

  // fn({ purpose, text }) -> true or "on" (allow, remembered: aiVision "on"), "once" (this time only), "off" (never:
  // aiVision "off"), false (not now: no more asking until the page reloads). The UI shows it as a banner that doesn't take
  // the keyboard. canAsk() false (flying): no prompt now.
  setConsent(fn, { canAsk = null } = {}) {
    this.askConsent = fn;
    if (canAsk) this.canAsk = canAsk;
    this.declined = false;
  }

  // The user's answer (the prompt's, or the "consent-needed" banner's): as setConsent's fn returns.
  answerConsent(a) {
    if (a === true || a === "on") this.settings.set("aiVision", "on");
    else if (a === "off") this.settings.set("aiVision", "off");
    else if (a !== "once") this.declined = true;
    this.needed.clear();
    this.emit("consent", a === true ? "on" : a === "on" || a === "once" || a === "off" ? a : "not now");
    return a;
  }

  // Each memory flight gets its own budget (memory.on("flight") -> newFlight(id) on start).
  newFlight(id = null) {
    this.flight = { id, spent: 0 };
  }

  left(scope = "flight") {
    return scope === "flight" ? Math.max(0, this.budget - this.flight.spent) : Math.max(0, scope.limit - scope.spent);
  }

  price(u, model = this.model) {
    return costOf(model, u) ?? costOf(AI.fallbackPrice, u);
  }

  // May pictures of the house go to Claude now? approved: the user just approved this exact job (the survey's estimate
  // dialog says pictures are sent; a "once" from the banner for the checks that waited), which counts as consent this
  // time. ask false, or canAsk() false: no prompt now ("consent-needed" instead; "once" then counts for no call).
  async permit(purpose = "vision", { approved = false, text: why = "", ask = true } = {}) {
    if (!this.configured) return { ok: false, reason: "no Claude API key (Settings → Brain)" };
    const v = this.settings.get("aiVision");
    if (v === "off") return { ok: false, consent: true, reason: OFF_TEXT };
    if (v === "on" || approved) return { ok: true };
    if (this.declined) return { ok: false, consent: true, reason: "you said not now to Claude's picture checks" };
    if (!this.asking && (!ask || !this.askConsent || !this.canAsk())) {
      if (!this.needed.has(purpose)) (this.needed.add(purpose), this.emit("consent-needed", { purpose }));
      return { ok: false, consent: true, waiting: true, reason: this.askConsent ? "waiting for your OK to Claude's picture checks" : "Claude's picture checks need your OK first" };
    }
    this.asking ??= Promise.resolve()
      .then(() => this.askConsent({ purpose, text: why }))
      .catch(() => false)
      .then((a) => this.answerConsent(a))
      .finally(() => (this.asking = null));
    if (!ask) return { ok: false, consent: true, waiting: true, reason: "waiting for your OK to Claude's picture checks" };
    const a = await this.asking;
    return a === true || a === "on" || a === "once" ? { ok: true } : { ok: false, consent: true, reason: a === "off" ? OFF_TEXT : "you said not now to Claude's picture checks" };
  }

  // Tokens and dollars a call should take (input from its pictures and text; output expectOut, thinking included).
  estimate({ system = "", content = [], expectOut = AI.expectOut, cached = false }, model = this.model) {
    let input = textTokens(system) * (cached ? 0 : 1);
    for (const b of content) input += b.type === "image" ? (b.tokens ?? imageTokens(b.width ?? 768, b.height ?? 576, model)) : textTokens(b.text);
    const u = { input, output: expectOut, cacheRead: cached ? textTokens(system) : 0, cacheWrite: 0 };
    return { ...u, cost: this.price(u, model) };
  }

  // purpose: "survey" | "change" | "detection" (usage events say it). scope: "flight" (aiBudget) or { limit, spent } (a
  // survey's own). approved, ask: see permit(). Pictures in content need permission; a call that would go over the budget
  // isn't made. deadline (ms): the answer is given up on after that (the request is aborted).
  async call({ purpose, system, content, schema, maxTokens = AI.maxTokens, expectOut = AI.expectOut, scope = "flight", approved = false, ask = true, signal = null, deadline = null, effort = AI.effort }) {
    if (!this.configured) return { ok: false, reason: "no Claude API key (Settings → Brain)" };
    if (content.some((b) => b.type === "image")) {
      const p = await this.permit(purpose, { approved, ask });
      if (!p.ok) return p;
    }
    const model = this.model, est = this.estimate({ system, content, expectOut }, model), left = this.left(scope), need = est.cost * Math.max(1, this.ratio[purpose] ?? 1);
    if (need > left) {
      const limit = scope === "flight" ? this.budget : scope.limit, spent = scope === "flight" ? this.flight.spent : scope.spent;
      return { ok: false, budget: true, reason: `the AI budget ${scope === "flight" ? "for this flight" : "for this job"} is used up ($${spent.toFixed(2)} of $${limit.toFixed(2)}; this would take about $${need.toFixed(3)})` };
    }
    const strip = (b) => (b.type === "image" ? { type: "image", source: b.source } : b), structured = STRUCTURED.test(model);
    const opts = modelOptions(model, effort), inPrompt = structured ? [] : [text(`Answer with only a JSON object (no other text) that matches this JSON schema: ${JSON.stringify(schema)}`)];
    const req = {
      model,
      max_tokens: maxTokens,
      system: [{ type: "text", text: system, cache_control: { type: "ephemeral" } }],
      messages: [{ role: "user", content: [...content.map(strip), ...inPrompt] }],
      ...opts,
      ...((structured || opts.output_config) && { output_config: { ...opts.output_config, ...(structured && { format: { type: "json_schema", schema } }) } }),
    };
    const late = deadline ? AbortSignal.timeout(deadline) : null, sig = late ? (signal ? AbortSignal.any([signal, late]) : late) : signal;
    let resp;
    try {
      resp = await claudeClient(this.settings, this.fetch).beta.messages.create(req, sig ? { signal: sig } : undefined);
    } catch (e) {
      if (late?.aborted && !signal?.aborted) return { ok: false, late: true, reason: `Claude didn't answer within ${Math.round(deadline / 1000)} s` };
      const reason = errorText(e);
      if (reason !== "stopped") this.emit("error", `Claude (${purpose}): ${reason}.`);
      return { ok: false, reason, stopped: reason === "stopped" };
    }
    const { usage, cost, served } = this.priced(resp, model);
    if (scope === "flight") this.flight.spent += cost;
    else scope.spent += cost;
    if (est.cost > 0) this.ratio[purpose] = (1 - AI.learn) * (this.ratio[purpose] ?? 1) + AI.learn * (cost / est.cost);
    this.report(served, usage, purpose, cost, est.cost);
    if (resp.stop_reason === "refusal") return { ok: false, refused: true, reason: `Claude declined to answer${resp.stop_details?.category ? ` (${resp.stop_details.category})` : ""}`, usage, cost };
    if (resp.stop_reason === "max_tokens") return { ok: false, reason: "Claude's answer was cut off", usage, cost };
    const out = (resp.content ?? []).filter((b) => b.type === "text").map((b) => b.text).join("");
    const json = structured ? out : out.slice(out.indexOf("{"), out.lastIndexOf("}") + 1);
    try {
      return { ok: true, data: JSON.parse(json), usage, cost, estimate: est.cost, model: served, ...(served !== model && { note: `answered by ${served} after ${model} declined` }) };
    } catch {
      return { ok: false, reason: "Claude's answer wasn't readable", usage, cost };
    }
  }

  // A response's tokens and dollars, priced by the model that ran each attempt (agent.js pricedUsage).
  priced(resp, model = this.model) {
    return pricedUsage(resp, model, (m, u) => this.price(u, m));
  }

  // How far the real costs ran from the estimates, per purpose (1: as estimated).
  accuracy() {
    return Object.fromEntries(Object.entries(this.ratio).map(([k, v]) => [k, +v.toFixed(2)]));
  }

  // Through the agent (its "usage" event and session total), else this object's own "usage" event.
  report(model, usage, purpose, cost, estimated = null) {
    this.session.requests += usage.requests;
    this.session.cost += cost ?? 0;
    this.session.estimated += estimated ?? 0;
    const ev = this.agent?.reportUsage?.(model, usage, purpose, cost) ?? { ...usage, model, cost, purpose, session: { ...this.session } };
    this.emit("usage", { ...ev, purpose, estimated, flight: { ...this.flight, budget: this.budget } });
  }
}

export { PRICES };
