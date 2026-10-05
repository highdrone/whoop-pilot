// Small DOM and formatting helpers for the panels.

// h("button", { class: "btn", onclick }, "Text", child...): properties when the element has them, else attributes;
// on* are listeners; null, undefined and false are skipped (as props and as children).
export function h(tag, props = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(props ?? {})) {
    if (v == null || v === false) continue;
    if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (k === "class") e.className = v;
    else if (k in e) e[k] = v;
    else e.setAttribute(k, v === true ? "" : v);
  }
  e.append(...kids.flat(Infinity).filter((x) => x != null && x !== false));
  return e;
}

export const fmtBytes = (n) =>
  !Number.isFinite(n) ? "" : n >= 1e9 ? `${(n / 1e9).toFixed(1)} GB` : n >= 1e6 ? `${Math.round(n / 1e6)} MB` : `${Math.max(1, Math.round(n / 1e3))} KB`;

export const fmtDate = (d) => new Date(d).toLocaleString(undefined, { dateStyle: "medium", timeStyle: "short" });

export const fmtDuration = (ms) => {
  const s = Math.max(0, Math.round(ms / 1000));
  return s >= 60 ? `${Math.floor(s / 60)} min ${String(s % 60).padStart(2, "0")} s` : `${s} s`;
};

export const fmtClock = (ms) => {
  const s = Math.max(0, Math.round(ms / 1000));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
};

// A button that asks once more before doing something it can't undo: the first click arms it for a few seconds.
export function confirmButton(text, confirmText, action, props = {}) {
  let armed = 0;
  const b = h("button", { type: "button", class: "btn ghost danger-text", ...props, onclick: () => {
    if (Date.now() - armed < 4000) {
      armed = 0;
      b.textContent = text;
      return action();
    }
    armed = Date.now();
    b.textContent = confirmText;
    setTimeout(() => Date.now() - armed >= 4000 && (b.textContent = text), 4100);
  } }, text);
  return b;
}

export const nextPaint = () => new Promise((r) => requestAnimationFrame(() => setTimeout(r, 0)));
