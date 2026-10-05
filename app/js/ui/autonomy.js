// How the page hands the drone to the autonomy modules (main.js; docs/HOME-DRONE.md, "Wave B contracts"), here so the
// rules can be tested without a browser: when the safety layer may act, whether a command still gets to fly after
// waiting for the hand-over, and when "The drone is on its home pad" may move the position estimate.
import { sleep } from "../util.js";

// The safety layer hears every pose from its constructor on. It may only act (trip, alert, ask the pilot to land)
// while it is the controller's filter, not while the simulator flies the demo apartment or no house is in use.
export function onlyWhenAttached(safety) {
  const watch = safety.watch;
  safety.watch = (p) => (safety.ctl.safety === safety ? watch.call(safety, p) : undefined);
  return safety;
}

// Whether the safety layer flies with the drone: in the house, with a position to go on (the simulator's truth, or the
// real drone's once the home pad or a fix gave it one). Without one it could only hold and land, so until then flights
// work as they do without a house, and the HUD asks for the pad.
export const safetyWanted = ({ inHouse, sim, localizer }) => !!(inHouse && localizer && (sim || localizer.known));

// Commands wait for the hand-over; a stop or a newer command meanwhile cancels them. take() before waiting, stop() on
// every stop, live(n) after.
export class Turns {
  constructor() {
    this.n = 0;
  }
  take() {
    return ++this.n;
  }
  stop() {
    this.n++;
  }
  live(n) {
    return n === this.n;
  }
}

// Before the AI flies: the simulated pilot arms and flips the AI switch; a real one is asked to flip it (up to 10 s).
// Resolves whether this command still gets to fly.
export async function handOver({ turns, settings, sim, ctl, say, wait = sleep }) {
  const n = turns.take();
  if (settings.get("autonomy") !== "observer") {
    if (settings.get("mode") === "sim") {
      sim.prepareForMission({ copilot: settings.get("autonomy") === "copilot" });
      await wait(150);
    } else if (ctl.telemetryFresh && !ctl.tel.engaged) {
      say("Flip the AI switch on the radio to hand me control.");
      for (let i = 0; i < 40 && !ctl.tel?.engaged && turns.live(n); i++) await wait(250);
    }
  }
  return turns.live(n);
}

// "The drone is on its home pad", on the ground only: in the air it would move the estimate (and the geofence and the
// mission's path with it) by however far the drone is from the pad. Returns why not, or null once reset.
export function padReset({ ctl, missions, localizer, pad }) {
  if (!pad || !localizer) return "Set the home pad on the map first.";
  if (ctl.isFlying() || missions?.busy) return "Land on the home pad first, then press this.";
  ctl.snapHeading?.();
  localizer.reset({ x: pad.x, y: pad.y, yaw: pad.yaw ?? Math.PI / 2 });
  return null;
}
