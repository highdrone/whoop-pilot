// Offline fallback when there's no Claude API key (or no internet): understands simple commands
// like "take off", "turn left 45", "find the cat", "follow the dog", "go to the kitchen". With a house map:
// "go to <room or landmark>", "look in <room>", "find people", "find the cat", "check on the pets", "patrol",
// "come home" become missions (tools.js mission tools), and "what changed", "where did you last see the cat", "survey the
// house" go to the memory tools; "go to" a place the map doesn't have is the mission's refusal, not the wander-and-look
// explore() (that one is for no map only).
// Emits the same events as the Claude agent so the UI treats them alike.
import { Emitter } from "./util.js";
import { Search, Turn, Move } from "./behaviors.js";
import { findRoom, findLandmark } from "./missions.js";
import { toDetectorLabel } from "./detector.js";

const MISSION_TOOLS = ["go_to", "look_in", "search_for", "patrol", "return_home", "where_am_i", "recall", "what_changed", "survey_house"];

// Objects that give a room away.
export const ROOM_LANDMARKS = {
  kitchen: ["refrigerator", "oven", "sink", "microwave"],
  "living room": ["couch", "tv"],
  lounge: ["couch", "tv"],
  bedroom: ["bed"],
  bathroom: ["toilet"],
  office: ["laptop", "keyboard"],
  "dining room": ["dining table"],
};

const WORD_NUMBERS = {
  a: 1, one: 1, two: 2, three: 3, four: 4, five: 5, ten: 10, fifteen: 15, twenty: 20, thirty: 30,
  "forty five": 45, "forty-five": 45, sixty: 60, ninety: 90, "hundred and eighty": 180, "one eighty": 180, "a hundred eighty": 180,
};

function num(text, fallback) {
  if (!text) return fallback;
  const n = parseFloat(text);
  if (Number.isFinite(n)) return n;
  return WORD_NUMBERS[text.trim()] ?? fallback;
}

const NUM = "(\\d+(?:\\.\\d+)?|a|one|two|three|four|five|ten|fifteen|twenty|thirty|forty[ -]five|sixty|ninety|one eighty)";

// Commands that mean a mission (or a question for the house memory) when there's a house map. text: as said.
function parseHouse(t, house, text = t) {
  let m;
  if (/\b(?:come|go|fly|head|get|return) (?:back )?home\b|\b(?:come|fly) back\b|\breturn to (?:base|the pad|home)\b|\bback to (?:base|the pad)\b/.test(t)) return { tool: "return_home", input: {} };
  if (/\bpatrol\b|\b(?:check|look around|go around) (?:the |every |all (?:the )?)?(?:house|rooms|home|place)\b/.test(t)) return { tool: "patrol", input: {} };
  if (/\bwhere am i\b|\bwhere are you\b/.test(t)) return { tool: "where_am_i", input: {} };
  if (/\bwhat(?:'?s| has| have)? changed\b|\bwhat(?:'?s| is) different\b|\bany(?:thing)? (?:new|different|changed)\b|\bany changes\b/.test(t))
    return { tool: "what_changed", input: (m = t.match(/\bsince (.+)$/)) ? { since: m[1] } : {} };
  if (/\b(?:where|when) (?:did|have) you (?:last )?(?:see|seen|spot|spotted|find|found)\b|\bhave you seen\b|\bwhere (?:have|did) you (?:been|go|fly|flown)\b|\bwhen (?:were|was) you (?:last )?in\b|\bwhat did you (?:see|find)\b|\bhow many flights\b/.test(t))
    return { tool: "recall", input: { question: text.trim() } };
  if (/\b(?:survey|label) (?:the |my |our )?(?:house|home|rooms)\b/.test(t)) return { tool: "survey_house", input: {} };
  if ((m = t.match(/\b(?:look|peek|check|see|what'?s) (?:in|into|inside) (?:the |my |our )?(.+)/))) {
    const r = findRoom(house, m[1]);
    if (r) return { tool: "look_in", input: { room: r.name } };
  }
  if ((m = t.match(/\b(?:find|search for|look for|where (?:is|are)|where's|locate|check on) (?:the |my |a |our |any )?(.+)/))) {
    const what = m[1].trim();
    if (/^(?:people|everyone|everybody)$/.test(what)) return { tool: "search_for", input: { target: "people" } };
    if (/^(?:a )?(?:person|someone|somebody|anyone|anybody|humans?)$/.test(what)) return { tool: "search_for", input: { target: "person" } };
    if (/^(?:pets?|animals?)$/.test(what)) return { tool: "search_for", input: { target: "pets" } };
    const label = toDetectorLabel(what);
    if (label === "cat" || label === "dog") return { tool: "search_for", input: { target: label } };
  }
  if ((m = t.match(/\b(?:go|fly|head|come|take me|move) (to|into|in|over to|toward|towards) (?:the |my |our )?(.+)/)) && !/^(?:left|right|front|back|forward|up|down)\b/.test(m[2])) {
    const place = m[2].trim();
    if (findRoom(house, place) || findLandmark(house, place) || /^(?:home|base|pad|home pad)$/.test(place)) return { tool: "go_to", input: { place } };
    // Not on the map: still the mission, which says so and lists the rooms (no wandering by hand through a mapped
    // house), unless it's something to fly at by sight.
    if (!/^toward/.test(m[1]) && !toDetectorLabel(place)) return { tool: "go_to", input: { place } };
  }
  return null;
}

// house: the active house (with a map), so places and missions are understood.
export function parseCommand(text, { house = null } = {}) {
  const t = text.toLowerCase().replace(/[.,!?]/g, "").replace(/\s+/g, " ").trim();
  let m;
  const mission = house && parseHouse(t, house, text);
  if (mission) return mission;
  if (/\b(hover|hold|wait|stay|freeze|stop)\b/.test(t) && t.split(" ").length <= 4) return { tool: "hover", input: { seconds: 5 } };
  if (/\b(land|touch down|come down)\b/.test(t)) return { tool: "land", input: {} };
  if (/\b(take ?off|lift ?off|launch|get up)\b/.test(t)) return { tool: "take_off", input: {} };
  if (/\bturn around\b|\bdo a 180\b/.test(t)) return { tool: "turn", input: { degrees: 180 } };
  if ((m = t.match(/\blook (left|right)\b/))) return { tool: "turn", input: { degrees: m[1] === "left" ? -60 : 60 } };
  if ((m = t.match(new RegExp(`\\b(?:turn|rotate|spin|yaw)(?: to the)? (left|right)(?: by)?(?: ${NUM})?`)))) {
    return { tool: "turn", input: { degrees: (m[1] === "left" ? -1 : 1) * Math.min(180, num(m[2], 90)) } };
  }
  if ((m = t.match(new RegExp(`\\b(?:go|move|fly|come|head|drift)? ?(forward|forwards|ahead|straight|back|backward|backwards|left|right|up|down|higher|lower)(?: by)?(?: ${NUM})? ?(m|meters?|metres?|feet|foot|ft)?\\b`)))) {
    const dir = { forwards: "forward", ahead: "forward", straight: "forward", backward: "back", backwards: "back", higher: "up", lower: "down" }[m[1]] || m[1];
    let meters = num(m[2], dir === "up" || dir === "down" ? 0.4 : 1);
    if (m[3] && /f/.test(m[3])) meters *= 0.3;
    if (/\b(a (little|bit)|slightly|a touch)\b/.test(t)) meters = 0.4;
    return { tool: "move", input: { direction: dir, meters } };
  }
  if (/\b(look around|scan|where are (we|you)|get your bearings)\b/.test(t)) return { tool: "look_around", input: {} };
  if ((m = t.match(/\bfollow (?:the |my |that |this )?(.+)/))) return { tool: "follow", input: { object: m[1], seconds: 60, distance: "far" } };
  if ((m = t.match(/\b(?:find|search for|look for|where is|where's|locate) (?:the |my |a |our )?(.+)/))) return { tool: "find", input: { object: m[1] } };
  if ((m = t.match(/\b(?:go|fly|head|come|take me) (?:to|into|in|over to|toward|towards) (?:the |my |our )?(.+)/)) || (m = t.match(/\bapproach (?:the |my )?(.+)/))) {
    const place = m[1].trim();
    const room = Object.keys(ROOM_LANDMARKS).find((r) => place.includes(r));
    if (room) return { explore: room };
    return { tool: "approach", input: { object: place, distance: "medium" } };
  }
  if (/\bwhat (do|can) you see\b|\bdescribe\b|\bwhat's (there|in front)\b/.test(t)) return { tool: "snapshot", input: {}, describe: true };
  return null;
}

export class LocalPlanner extends Emitter {
  constructor({ tools, ctl, settings }) {
    super();
    Object.assign(this, { tools, ctl, settings });
    this.ac = null;
    this.busy = false;
  }

  stop() {
    this.ac?.abort();
  }

  async run(command) {
    this.stop();
    const ac = new AbortController();
    this.ac = ac;
    this.busy = true;
    this.emit("status", "acting");
    try {
      const parts = command.split(/\b(?:and then|then|and|after that)\b/i).map((p) => p.trim()).filter(Boolean);
      const house = this.tools.missions ? this.tools.house : null;
      const plans = parts.map((p) => parseCommand(p, { house }));
      if (!plans.length || plans.some((p) => !p)) {
        this.emit("say", this.tools.missions
          ? "Offline I understand commands like go to the living room, look in a room, find people, find the cat, patrol, or come home. Add a Claude API key in Settings for anything else."
          : "Offline I only understand simple commands like take off, turn left, find the cat, follow the dog, or go to the kitchen. Add a Claude API key in Settings for anything else.");
        return;
      }
      for (const plan of plans) {
        if (ac.signal.aborted) return;
        const ok = plan.explore ? await this.explore(plan.explore, ac.signal) : await this.step(plan, ac.signal);
        if (!ok) return;
      }
    } finally {
      if (this.ac === ac) {
        this.ac = null;
        this.busy = false;
        this.emit("status", "idle");
      }
    }
  }

  async step(plan, signal) {
    const grounded = ["take_off", "snapshot", "land", ...MISSION_TOOLS].includes(plan.tool);
    if (!grounded && !this.ctl.isFlying() && !this.ctl.blocker()) {
      const r = await this.call("take_off", {}, signal);
      if (r.isError) return false;
    }
    const r = await this.call(plan.tool, plan.input, signal);
    if (plan.describe) this.emit("say", this.tools.describe().replace("Detector sees:", "I can see"));
    else this.emit("say", r.text.split("\n")[0]);
    return !r.isError;
  }

  async call(name, input, signal) {
    this.emit("tool", { name, input, phase: "start" });
    const r = await this.tools.call(name, input, signal);
    this.emit("tool", { name, input, phase: "done", result: r });
    return r;
  }

  // Wander room to room until a landmark of the room shows up, then face it.
  async explore(room, signal) {
    const why = this.tools.ready({ needFlying: false });
    if (why) {
      this.emit("say", why);
      return false;
    }
    if (!this.ctl.isFlying() && (await this.call("take_off", {}, signal)).isError) return false;
    const labels = ROOM_LANDMARKS[room];
    this.emit("say", `Heading for the ${room}. I'll look for a ${labels.slice(0, 2).join(" or ")}.`);
    const speed = this.settings.get("speed");
    for (let round = 0; round < 4 && !signal.aborted; round++) {
      this.emit("tool", { name: "find", input: { object: labels.join(" / ") }, phase: "start" });
      const s = await this.ctl.run(new Search(labels, 380), signal);
      this.emit("tool", { name: "find", input: { object: labels.join(" / ") }, phase: "done", result: { text: s.text, isError: !s.ok } });
      if (s.ok) {
        await this.call("approach", { object: s.data.label, distance: "close" }, signal);
        this.emit("say", `This looks like the ${room}: I found the ${s.data.label}.`);
        return true;
      }
      if (signal.aborted) return false;
      await this.ctl.run(new Turn(round % 2 ? -100 : 100), signal);
      const mv = await this.ctl.run(new Move("forward", 1.6, speed), signal);
      this.emit("tool", { name: "move", input: { direction: "forward", meters: 1.6 }, phase: "done", result: { text: mv.text, isError: !mv.ok } });
    }
    if (!signal.aborted) this.emit("say", `I couldn't find the ${room}. With a Claude API key I can navigate by what I see.`);
    return false;
  }
}

