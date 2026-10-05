export class Emitter {
  constructor() {
    this._handlers = new Map();
  }
  on(event, fn) {
    if (!this._handlers.has(event)) this._handlers.set(event, new Set());
    this._handlers.get(event).add(fn);
    return () => this._handlers.get(event)?.delete(fn);
  }
  emit(event, data) {
    for (const fn of this._handlers.get(event) || []) {
      try {
        fn(data);
      } catch (e) {
        console.error(`handler for "${event}" failed`, e);
      }
    }
  }
}

export const DEG = Math.PI / 180;
export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, k) => a + (b - a) * k;

export function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

export const sleep = (ms, signal) =>
  new Promise((resolve, reject) => {
    const t = setTimeout(resolve, ms);
    signal?.addEventListener("abort", () => {
      clearTimeout(t);
      reject(signal.reason ?? new DOMException("Aborted", "AbortError"));
    });
  });

// Betaflight "Actual" rates: stick [-1, 1] -> deg/s (clockwise +), and the inverse.
export function actualRate(stick, center, max) {
  return stick * center + Math.max(0, max - center) * stick * Math.abs(stick);
}
export function actualRateInverse(rate, center, max) {
  const k = Math.max(1e-6, max - center);
  const r = Math.abs(rate);
  const s = (-center + Math.sqrt(center * center + 4 * k * r)) / (2 * k);
  return Math.sign(rate) * Math.min(1, s);
}
