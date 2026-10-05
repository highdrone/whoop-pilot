// Claude's second look (docs/HOME-DRONE.md, "Wave C contracts"):
//   confirmChange(change): the live picture against the 3D scan's expected one from the same pose -> is the difference
//     real and lasting, and what is it? -> a suggestion for the user (memory.resolveChange), and with apply only what is
//     safe to do on Claude's word: confirm (at >= 0.8, naming the same kind) a change that adds a blocker (obstacle,
//     door-closed); dismiss (no-change at >= 0.9) one that would free space (gone, door-open); dismiss (no-change at >=
//     INSPECT.weakAt) a weak suspicion (what nav/changes.js saw of it the drone's own height error or a misread could
//     make: dropping it is safer than a sortie blocked by it); keep one it takes for a person or pet
//     (>= 0.9) as an obstacle for a minute (memory.passing) rather than dismissing it. Never on pictures under
//     INSPECT.minEvidence px on their long side (or of unknown size). Freeing space (gone, moved, door-open) and waving
//     away a well-evidenced blocker always wait for the user. Claude's dismissals never stop a spot being raised again,
//     but by weak reports in the same flight.
//   confirmDetection(crop, label, score): is that really a person / cat / dog? -> "confirmed" (>= 0.8), "likely" (0.5 to
//     0.8), "rejected" (not there, >= 0.7) or "unconfirmed", for alerts.js before an uncertain alert goes out
//     (needsCheck: score under 0.6, or a label this house hadn't seen before this flight's latest sighting). alerts.js
//     never lets it wait for the consent prompt (ask: false) and gives up on a slow answer (deadline).
// Both respect consent and the flight budget (ai/claude.js) and answer "unconfirmed" (with the reason) when they can't ask.
// With watch: true, setMemory(memory) also has every new suspected change with pictures checked once (memory's "change"
// phase "suspected"; while aiVision isn't "off"; never prompting for consent then), the checks that waited for consent
// run again once it is given or after landing (the prompt may show then), and each memory flight gets its own budget.
import { text, image, toJpegBase64, imageSize, AI } from "./claude.js";

export const INSPECT = { uncertain: 0.6, confirmAt: 0.8, dismissAt: 0.9, weakAt: 0.6, suggestAt: 0.6, presentAt: 0.8, likelyAt: 0.5, rejectAt: 0.7, minEvidence: 200 };
const CHANGE_KINDS = ["obstacle", "door-closed", "door-open", "moved", "gone", "none"];
const ADDS = new Set(["obstacle", "door-closed"]), FREES = new Set(["gone", "door-open"]);

export const CHANGE_PROMPT = `You check changes in a home for a small indoor drone that navigates by comparing its camera with a 3D scan of the home (a Gaussian splat). You get two pictures from the same position and direction: first the drone's live camera, then what the 3D scan expected there (a render of the scan; it is softer, can have smears or holes where the scan was thin, and has no people or pets). A change detector flagged a difference; its spot is boxed in both pictures when the box is drawn. The pictures may be small crops, slightly misaligned with each other, differently exposed or coloured, blurred or compressed, and the live one may carry on-screen text or icons from the video link: none of that is a change.

Decide whether the home really changed in a way that lasts and matters for flying: a new obstacle (a box, a bag, furniture moved into the way), a door now closed or open, something moved, or something in the scan now gone. These are not changes: lighting, sunlight, screens, motion blur, video noise or compression, misalignment, the scan's own smears and holes, small things on surfaces that don't stick out into the air, and people or pets (they move; answer person-or-pet for those).

verdict: real-change, no-change, person-or-pet, or unclear. kind: what changed (none if nothing did). what: a few words naming it ("cardboard box", "bedroom door"). confidence 0..1 in your verdict. why: one short sentence. When you can't tell (too small, too blurry, too different to compare), say unclear with a low confidence rather than guess.`;

export const DETECTION_PROMPT = `You double-check a small indoor drone's object detector before it alerts the home owner. You get one picture from the drone's camera (a wide-angle view, possibly blurry, dim or with on-screen text from the video link) and the label the detector gave it. Say whether that label is really there: a real, live one, not a picture, a poster, a screen, a reflection, a toy, a statue, a coat or pile of clothes, a cushion or a shadow. present: true or false. actual: what it really is, in a few words. confidence 0..1. why: one short sentence.`;

const CHANGE_SCHEMA = {
  type: "object", additionalProperties: false, required: ["verdict", "kind", "what", "confidence", "why"],
  properties: {
    verdict: { type: "string", enum: ["real-change", "no-change", "person-or-pet", "unclear"] },
    kind: { type: "string", enum: CHANGE_KINDS },
    what: { type: "string" }, confidence: { type: "number" }, why: { type: "string" },
  },
};
const DETECTION_SCHEMA = {
  type: "object", additionalProperties: false, required: ["present", "actual", "confidence", "why"],
  properties: { present: { type: "boolean" }, actual: { type: "string" }, confidence: { type: "number" }, why: { type: "string" } },
};
const clamp01 = (v) => Math.max(0, Math.min(1, Number(v) || 0));
const theRoom = (name) => (/^(room|bedroom|bathroom)\s*\d/i.test(name) ? name : `the ${name}`);

export class Inspector {
  constructor({ claude, memory = null, watch = false }) {
    Object.assign(this, { claude, memory: null, watching: watch, offs: [] });
    this.checking = new Map(); // change id -> the check in progress
    this.setMemory(memory);
  }

  setMemory(memory) {
    this.offs.forEach((f) => f?.());
    this.offs = [];
    this.memory = memory;
    if (!memory || !this.watching) return;
    const vision = () => this.claude.settings.get("aiVision") !== "off";
    const waiting = () => memory.changes.filter((c) => c.status === "suspected" && c.evidence?.live && c.claude?.waiting);
    this.offs = [
      memory.on("change", (e) => e.phase === "suspected" && e.evidence?.live && vision() && this.confirmChange(e.change, { apply: true, ask: false }).catch(() => {})),
      memory.on("flight", (f) => (f.t1 == null ? this.claude.newFlight(f.id) : vision() && waiting().forEach((c) => this.confirmChange(c, { apply: true }).catch(() => {})))),
      this.claude.on("consent", (a) => (a === "on" || a === "once") && this.memory === memory
        && waiting().forEach((c) => this.confirmChange(c, { apply: true, approved: a === "once" }).catch(() => {}))),
    ];
  }

  // A detection worth Claude's look before an alert: a low score, or a label this house hasn't seen before.
  needsCheck({ label, score }) {
    const seen = this.memory?.seenBefore ? this.memory.seenBefore(label) : !!this.memory?.lastSeen(label);
    return !!label && ((score ?? 0) < INSPECT.uncertain || (!!this.memory && !seen));
  }

  // -> { status: "answered", verdict, kind, what, confidence, why, suggest: "confirmed" | "dismissed" | null, transient,
  //      small, resolved (the change once resolved, with apply), passing } | { status: "unconfirmed", reason, waiting? }.
  // One check per change at a time (a second call while one runs gets the same answer).
  confirmChange(change, o = {}) {
    const id = change?.id;
    if (id && this.checking.has(id)) return this.checking.get(id);
    const p = this.checkChange(change, o).finally(() => id && this.checking.delete(id));
    if (id) this.checking.set(id, p);
    return p;
  }

  async checkChange(change, { apply = false, signal = null, ask = true, approved = false, deadline = AI.checkDeadline } = {}) {
    const note = (fields) => change?.id && this.memory?.annotateChange(change.id, { claude: { ...fields, t: Date.now() } });
    const unconfirmed = (reason, waiting = false) => {
      const r = { status: "unconfirmed", reason, ...(waiting && { waiting }) };
      note(r);
      return r;
    };
    const ev = change?.evidence;
    const [live, expected] = await Promise.all([toJpegBase64(ev?.live), toJpegBase64(ev?.expected)]).catch(() => [null, null]);
    if (!live || !expected) return unconfirmed("no pictures to compare");
    const sizes = await Promise.all([imageSize(ev.live), imageSize(ev.expected)]).catch(() => [null, null]);
    const small = sizes.some((s) => !s || Math.max(s.width, s.height) < INSPECT.minEvidence);
    const room = change.room ? theRoom(this.memory?.roomName?.(change.room) ?? change.room) : "a place off the map";
    const floor = this.memory?.map?.floorAt?.(change.x, change.y);
    const r = await this.claude.call({
      purpose: "change", system: CHANGE_PROMPT, signal, ask, approved, deadline, expectOut: 500,
      content: [
        text("Live camera:"), image(live),
        text("What the 3D scan expected from the same spot:"), image(expected),
        text(`The change detector says: ${change.kind ?? "obstacle"}${change.what ? ` (${change.what})` : ""} in ${room}, about ${Math.round((change.size ?? 0.3) * 100)} cm across`
          + `${change.z != null && floor != null ? `, ${(change.z - floor).toFixed(1)} m above the floor` : ""}${change.n > 1 ? `, seen ${change.n} times` : ""}.`
          + `${ev.cut ? " It was at the edge of the camera's view, so only part of it is in the pictures (dark where the picture ends)." : ""} Is it real?`),
      ],
      schema: CHANGE_SCHEMA,
    });
    if (!r.ok) return unconfirmed(r.reason, !!r.waiting);
    const d = r.data, confidence = clamp01(d.confidence), transient = d.verdict === "person-or-pet", kind = d.kind === "none" ? null : d.kind;
    const suggest = confidence < INSPECT.suggestAt ? null : d.verdict === "real-change" ? "confirmed" : d.verdict === "no-change" || transient ? "dismissed" : null;
    const out = { status: "answered", verdict: d.verdict, kind, what: String(d.what ?? "").slice(0, 80), confidence, why: String(d.why ?? "").slice(0, 200), suggest, transient, small, cost: r.cost };
    if (change.id && this.memory) {
      this.memory.annotateChange(change.id, { claude: { ...out, t: Date.now() }, ...(out.what && !change.what && { what: out.what }) });
      const now = this.memory.changes?.find((c) => c.id === change.id), k = change.kind ?? "obstacle";
      if (apply && !small && now?.status === "suspected") {
        const why = `Claude: ${out.what || d.verdict} (${Math.round(confidence * 100)}% sure). ${out.why}`;
        if (suggest === "confirmed" && confidence >= INSPECT.confirmAt && kind === k && ADDS.has(k)) out.resolved = this.memory.resolveChange(change.id, "confirmed", why, { by: "claude" });
        else if (suggest === "dismissed" && !transient && ((confidence >= INSPECT.dismissAt && FREES.has(k)) || (now.weak && confidence >= INSPECT.weakAt)))
          out.resolved = this.memory.resolveChange(change.id, "dismissed", why, { by: "claude" });
        else if (transient && confidence >= INSPECT.dismissAt) out.passing = !!this.memory.passing?.(change.id, { note: why });
      }
    }
    return out;
  }

  // crop: the picture (base64 JPEG, Blob, pixels); box: the detection's normalized box, cropped around in the browser.
  // ask false: don't wait for the consent prompt; deadline: ms to wait for Claude's answer.
  // -> { status: "confirmed" | "likely" | "rejected" | "unconfirmed", label, actual?, confidence?, why?, reason? }
  async confirmDetection(crop, label, score = null, { box = null, signal = null, ask = true, deadline = null } = {}) {
    const cropped = !!box && typeof OffscreenCanvas !== "undefined", pic = await toJpegBase64(crop, { box: cropped ? box : null }).catch(() => null);
    if (!pic) return { status: "unconfirmed", label, reason: "no picture" };
    const where = box && !cropped ? ` Its box, as fractions of the picture from the top left: x ${box.x.toFixed(2)}, y ${box.y.toFixed(2)}, width ${box.w.toFixed(2)}, height ${box.h.toFixed(2)}.` : "";
    const r = await this.claude.call({
      purpose: "detection", system: DETECTION_PROMPT, signal, ask, deadline, expectOut: 300,
      content: [image(pic), text(`The detector says: ${label}${score != null ? ` (score ${Math.round(score * 100)}%)` : ""}.${where} Is there really a ${label} in this picture?`)],
      schema: DETECTION_SCHEMA,
    });
    if (!r.ok) return { status: "unconfirmed", label, reason: r.reason };
    const confidence = clamp01(r.data.confidence), present = !!r.data.present;
    const status = present ? (confidence >= INSPECT.presentAt ? "confirmed" : confidence >= INSPECT.likelyAt ? "likely" : "unconfirmed")
      : confidence >= INSPECT.rejectAt ? "rejected" : "unconfirmed";
    return { status, label, actual: String(r.data.actual ?? "").slice(0, 80), confidence, why: String(r.data.why ?? "").slice(0, 200), cost: r.cost, ...(status === "unconfirmed" && { reason: "Claude wasn't sure" }) };
  }
}
