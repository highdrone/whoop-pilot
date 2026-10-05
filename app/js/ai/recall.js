// Claude on the flight memory (memory/memory.js). recall() answers from the records by fixed rules; askMemory() lets
// Claude answer what those rules don't understand, from the same records as text (the latest flights, last seen per
// label, the timeline, and the rules' own answer), and summarizeFlight() writes a short note after landing from the
// deterministic flight report and that flight's timeline. Text only (no pictures), with the user's key, within the
// flight budget (ai/claude.js), never when Claude's checks are off in Settings, and only when the user asks (the History
// tab's "Ask Claude", the summary button): the caller keeps the deterministic answer when Claude can't (it says why).
//   const a = await askMemory({ claude, memory, question: "has the dog been upstairs today?" }); a.text
//   const s = await summarizeFlight({ claude, memory });  s.text
import { text, OFF_TEXT } from "./claude.js";
import { when } from "../memory/memory.js";

export const RECALL_PROMPT = `You answer questions about what a small indoor drone remembers from its flights in its owner's home. You get the memory as plain-text records: flights (when, how long, which rooms), sightings (people, pets and other things, where and when), differences from the home's 3D scan that flights noticed, and other events, plus the answer a simple keyword search gave. Answer only from the records. If they don't say, say that you don't know and what the records do show. found: whether the records answer the question. One or two short plain sentences; no lists, no coordinates; say times the way the records do.`;

export const SUMMARY_PROMPT = `You write the note a home owner reads after their small indoor drone landed. You get the flight's report (made by fixed rules from its records) and its timeline. Write two or three short plain sentences: what the flight did, anything that needs the owner (a person or pet seen, a possible change to check, rooms the camera didn't see well, position problems), and nothing else. Use only what the report and timeline say; no coordinates, no lists.`;

const RECALL_SCHEMA = { type: "object", additionalProperties: false, required: ["answer", "found"], properties: { answer: { type: "string" }, found: { type: "boolean" } } };
const SUMMARY_SCHEMA = { type: "object", additionalProperties: false, required: ["summary"], properties: { summary: { type: "string" } } };

// The memory as text for Claude: the digest, the totals and the newest `limit` timeline entries.
export function memoryText(memory, { limit = 120, since = 0, now = memory.now() } = {}) {
  const tl = memory.timeline({ since, limit }, now);
  return [memory.digest(now), memory.statsText(now), "Records, newest first:", ...tl.map((e) => `- ${when(e.t, now)}: ${e.text}`)].join("\n");
}

const off = (claude) => (claude.settings.get("aiVision") === "off" ? { ok: false, reason: OFF_TEXT } : null);

// -> { ok, text, found?, cost?, local (the rules' answer) } | { ok: false, text (the rules' answer), reason }
export async function askMemory({ claude, memory, question, signal = null }) {
  const local = memory.answer(question).text, no = off(claude);
  if (no) return { ...no, text: local, local };
  const r = await claude.call({ purpose: "recall", system: RECALL_PROMPT, signal, deadline: 30000, expectOut: 300, schema: RECALL_SCHEMA,
    content: [text(memoryText(memory)), text(`The keyword search answered: "${local}"`), text(`Question: ${String(question).slice(0, 300)}`)] });
  return r.ok ? { ok: true, text: String(r.data.answer).slice(0, 600), found: !!r.data.found, cost: r.cost, local } : { ok: false, text: local, local, reason: r.reason };
}

// flight: a memory flight record (the last one by default), ended. -> { ok, text, cost? } | { ok: false, text (the
// deterministic report), reason }
export async function summarizeFlight({ claude, memory, flight = memory.flights.at(-1), signal = null }) {
  const report = flight?.report ?? (flight ? memory.flightReport(flight) : ""), no = off(claude);
  if (!flight) return { ok: false, text: "", reason: "no flight to sum up" };
  if (no) return { ...no, text: report };
  const tl = memory.timeline({ since: flight.t0, limit: 60 }).filter((e) => e.t <= (flight.t1 ?? Infinity)).map((e) => `- ${when(e.t)}: ${e.text}`);
  const r = await claude.call({ purpose: "summary", system: SUMMARY_PROMPT, signal, deadline: 30000, expectOut: 300, schema: SUMMARY_SCHEMA,
    content: [text(`Flight report: ${report}`), text(`Timeline, newest first:\n${tl.join("\n")}`)] });
  return r.ok ? { ok: true, text: String(r.data.summary).slice(0, 600), cost: r.cost } : { ok: false, text: report, reason: r.reason };
}
