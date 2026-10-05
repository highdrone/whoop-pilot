// The Augmenter (augment.js) off the page's thread: { init: { opts, seed } }, then { id, image (ImageBitmap), m } ->
// { id, bitmap } | { id, error }. The same pictures as an Augmenter with mulberry32(seed) in place.
import { Augmenter } from "./augment.js";
import { mulberry32 } from "./world.js";

let aug = null;
self.onmessage = async ({ data }) => {
  if (data.init) return void (aug = new Augmenter(data.init.opts, mulberry32(data.init.seed)));
  try {
    const bitmap = await aug.apply(data.image, data.m);
    data.image.close();
    self.postMessage({ id: data.id, bitmap }, [bitmap]);
  } catch (e) {
    self.postMessage({ id: data.id, error: String(e?.message || e) });
  }
};
