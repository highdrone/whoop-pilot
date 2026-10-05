// Voice in (browser speech recognition, push-to-talk or wake word) and voice out (speech synthesis).
import { Emitter } from "./util.js";

const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;
const STOP_WORDS = /^(stop|stop stop|abort|hold|freeze|hover|land|land now|wait)$/;

export class Voice extends Emitter {
  constructor(settings) {
    super();
    this.settings = settings;
    this.supported = !!Recognition;
    this.rec = null;
    this.mode = null; // "ptt" | "handsfree"
    this.finalText = "";
    this.interim = "";
    this.voices = [];
    if ("speechSynthesis" in window) {
      const load = () => (this.voices = speechSynthesis.getVoices());
      load();
      speechSynthesis.addEventListener?.("voiceschanged", load);
    }
  }

  get listening() {
    return !!this.rec;
  }

  startPushToTalk() {
    if (!this.supported || this.mode === "ptt") return;
    this.stopRecognition();
    this.cancelSpeech();
    this.mode = "ptt";
    this.finalText = "";
    this.interim = "";
    this.startRecognition(true);
  }

  // Stop listening and hand over what was heard.
  endPushToTalk() {
    if (this.mode !== "ptt") return;
    this.pttEnding = true;
    this.rec?.stop();
  }

  setHandsFree(on) {
    if (on && this.mode !== "handsfree" && this.supported) {
      this.stopRecognition();
      this.mode = "handsfree";
      this.startRecognition(true);
    } else if (!on && this.mode === "handsfree") {
      this.stopRecognition();
    }
  }

  startRecognition(continuous) {
    const rec = new Recognition();
    rec.lang = navigator.language || "en-US";
    rec.continuous = continuous;
    rec.interimResults = true;
    rec.onresult = (ev) => {
      let interim = "";
      for (let i = ev.resultIndex; i < ev.results.length; i++) {
        const r = ev.results[i];
        if (r.isFinal) this.onFinal(r[0].transcript);
        else interim += r[0].transcript;
      }
      this.interim = interim;
      this.emit("interim", (this.finalText + " " + interim).trim());
    };
    rec.onerror = (ev) => {
      if (ev.error === "no-speech" || ev.error === "aborted") return;
      this.emit("error", ev.error === "not-allowed" ? "Microphone permission was denied." : `Speech recognition error: ${ev.error}`);
    };
    rec.onend = () => {
      if (this.rec !== rec) return;
      this.rec = null;
      if (this.mode === "ptt") {
        const text = (this.finalText + " " + (this.interim || "")).trim();
        this.mode = null;
        this.pttEnding = false;
        this.emit("state", false);
        if (text) this.emit("command", text);
      } else if (this.mode === "handsfree") {
        setTimeout(() => this.mode === "handsfree" && !this.rec && this.startRecognition(true), 250);
      }
    };
    this.rec = rec;
    try {
      rec.start();
      this.emit("state", true);
    } catch (e) {
      this.rec = null;
      this.emit("error", `Couldn't start the microphone: ${e.message}`);
    }
  }

  onFinal(text) {
    if (this.mode === "ptt") {
      this.finalText = (this.finalText + " " + text).trim();
      return;
    }
    // Hands-free: act on "<wake word> ..." and on a bare "stop".
    const t = text.toLowerCase().trim().replace(/[.,!?]/g, "");
    const wake = (this.settings.get("wakeWord") || "drone").toLowerCase();
    if (STOP_WORDS.test(t)) return this.emit("command", t);
    const i = t.indexOf(wake);
    if (i !== -1) {
      const cmd = text.trim().slice(i + wake.length).replace(/^[\s,.:!?]+/, "").trim();
      if (cmd) this.emit("command", cmd);
    }
  }

  stopRecognition() {
    const rec = this.rec;
    this.rec = null;
    this.mode = null;
    try {
      rec?.abort();
    } catch {}
    this.emit("state", false);
  }

  speak(text) {
    if (!this.settings.get("speak") || !("speechSynthesis" in window) || !text) return;
    const u = new SpeechSynthesisUtterance(text);
    const name = this.settings.get("voiceName");
    const v = this.voices.find((x) => x.name === name) || this.voices.find((x) => /Samantha|Google US English|Daniel/.test(x.name));
    if (v) u.voice = v;
    u.rate = 1.08;
    speechSynthesis.speak(u);
  }

  cancelSpeech() {
    if ("speechSynthesis" in window) speechSynthesis.cancel();
  }
}
