// Model files: fetched once from Hugging Face at a pinned revision, checked by size and SHA-256, and kept in Cache Storage
// so later starts (and flights without internet) reuse them. Works in pages and workers.

const HF = "https://huggingface.co/onnx-community";

// RF-DETR (Roboflow, Apache-2.0), COCO-trained, fp32. The fp16 exports return no detections on WebGPU (ORT-web 1.30.0).
export const MODELS = {
  "rfdetr-n": {
    name: "RF-DETR-N",
    res: 384, // nano only works at 384: mAP 48.2 there, 13.9 at 512
    url: `${HF}/rfdetr_nano-ONNX/resolve/eae21cee0687a91bcf9fa071605c48d7705d2d91/onnx/model.onnx`,
    size: 108074865,
    sha256: "9cbac6b11ce34a03034e4d5a24cfac5f18632fd6761d1311dd640232088d7fee",
  },
  "rfdetr-s": {
    name: "RF-DETR-S",
    res: 512,
    url: `${HF}/rfdetr_small-ONNX/resolve/63463b68b200177d1fea7015f11f3cebb0ba4eeb/onnx/model.onnx`,
    size: 114680416,
    sha256: "121cc1476a7b69d865ca4bdc2bca59a3239020d227e4c3f35a811bef81aeb7f1",
  },
  // DINOv2-small (Meta, Apache-2.0; onnx-community's export of facebook/dinov2-small): whole-image descriptors for
  // relocalization (nav/splatloc.js). fp16 on WebGPU, the 8-bit one on the WASM fallback. Input pixel_values float32.
  "dinov2-s": {
    name: "DINOv2-small",
    url: `${HF}/dinov2-small/resolve/8b1f705a3a7f6f062f6bdd21986c1583d3ef105d/onnx/model_fp16.onnx`,
    size: 44420939,
    sha256: "16845e153bbaf3fd1ef8a2154c454940f901f6fe8a80dd4c6c319eaecdb4d2ee",
  },
  "dinov2-s-q8": {
    name: "DINOv2-small (8-bit)",
    url: `${HF}/dinov2-small/resolve/8b1f705a3a7f6f062f6bdd21986c1583d3ef105d/onnx/model_quantized.onnx`,
    size: 24446700,
    sha256: "c179f8f7f592449c4c1bca4cd124a7538021428c5ffb89afde9503935b197efb",
  },
};

export const CACHE_NAME = "whoop-pilot-models-v1";

const cache = () => (globalThis.caches ? caches.open(CACHE_NAME).catch(() => null) : Promise.resolve(null));

export async function sha256(buf) {
  const d = new Uint8Array(await crypto.subtle.digest("SHA-256", buf));
  return Array.from(d, (b) => b.toString(16).padStart(2, "0")).join("");
}

async function verify(spec, buf) {
  if (buf.byteLength !== spec.size) throw new Error(`${spec.name}: got ${buf.byteLength} bytes, expected ${spec.size}`);
  if ((await sha256(buf)) !== spec.sha256) throw new Error(`${spec.name}: checksum mismatch`);
  return buf;
}

async function download(spec, onProgress) {
  const res = await fetch(spec.url);
  if (!res.ok) throw new Error(`${spec.name}: download failed (HTTP ${res.status})`);
  const out = new Uint8Array(spec.size);
  const reader = res.body.getReader();
  let loaded = 0;
  let shown = -1;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    if (loaded + value.length > spec.size) {
      reader.cancel();
      throw new Error(`${spec.name}: download is larger than expected`);
    }
    out.set(value, loaded);
    loaded += value.length;
    const pct = Math.floor((loaded / spec.size) * 100);
    if (pct !== shown) onProgress?.(loaded, spec.size, (shown = pct));
  }
  return out.buffer.byteLength === loaded ? out.buffer : out.slice(0, loaded).buffer;
}

// Whether a model is in Cache Storage already (by its size; fetchModel checks the checksum when it is used).
export async function isCached(spec) {
  const hit = await (await cache())?.match(spec.url).catch(() => null);
  return !!hit && +(hit.headers.get("content-length") ?? spec.size) === spec.size;
}

// -> { bytes: ArrayBuffer, cached }. Cached copies are re-verified too, so a damaged cache entry is replaced.
export async function fetchModel(spec, onProgress) {
  const c = await cache();
  const hit = await c?.match(spec.url);
  if (hit) {
    const buf = await hit.arrayBuffer();
    try {
      return { bytes: await verify(spec, buf), cached: true };
    } catch {
      await c.delete(spec.url);
    }
  }
  const buf = await verify(spec, await download(spec, onProgress));
  await c?.put(spec.url, new Response(buf, { headers: { "content-type": "application/octet-stream" } })).catch((e) => console.warn(`Couldn't cache ${spec.name}: ${e.message}`));
  return { bytes: buf, cached: false };
}
