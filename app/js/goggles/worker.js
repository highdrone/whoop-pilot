// Runs the goggles video chain off the main thread, so the goggles always get their ACKs on time
// even while the page is busy (object detection, layout, loading models).
import { GogglesPipeline } from "./pipeline.js";

const pipeline = new GogglesPipeline({
  // at: when the decoder put it out, as an absolute time (the page's clock has another origin)
  onFrame: (frame) => self.postMessage({ type: "frame", frame, at: performance.timeOrigin + performance.now() }, [frame]),
  onEvent: (e) => self.postMessage(e),
});

self.onmessage = async ({ data }) => {
  try {
    if (data.cmd === "start-usb") {
      // The page already got the user's permission; find the same device here.
      const devices = await navigator.usb.getDevices();
      const f = data.filter;
      const dev = devices.find((d) => d.vendorId === f.vendorId && d.productId === f.productId && (!f.serialNumber || d.serialNumber === f.serialNumber));
      if (!dev) throw new Error("The goggles aren't available to the video worker.");
      await pipeline.startUsb(dev);
    } else if (data.cmd === "start-helper") {
      await pipeline.startHelper(data.url);
    } else if (data.cmd === "stop") {
      pipeline.stop();
      pipeline.report();
    }
  } catch (e) {
    pipeline.fail(e.name === "SecurityError" || /protected/i.test(e.message)
      ? "Chrome won't open this USB interface directly (protected class)."
      : `Couldn't open the goggles: ${e.message}`);
  }
};

setInterval(() => pipeline.state !== "off" && pipeline.report(), 500);
