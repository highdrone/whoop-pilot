# Spark 2.3.1 (@sparkjsdev/spark, MIT, see LICENSE)

`spark.module.js` from the npm tarball `sparkjsdev-spark-2.3.1.tgz`
(sha512-K+SyIbkO/dx1TLLudK8Jp3/noS13z/kdNPo6OK2waxQaGy81HBmKGnNBT/t/K9Ew3dsIXmWqsJXxGfmAu1T5Jg==, matches the registry).
Its sort worker and WASM are inlined. Only the three import lines are changed: `"three"` becomes
`"../three/build/three.module.js"` and `"three/addons/postprocessing/Pass.js"` becomes `"../three/addons/postprocessing/Pass.js"`,
so it loads in a module worker without an import map. Pinned: `app/js/twin/twin-worker.js` patches `onBeforeRender`
(sRGB blending, per-view render size), `readPause` and the renderer's async readback; re-run `tools/twin-check.html`
(PSNR, timing) before upgrading.

## Bundled third-party code

`spark.module.js` is Spark's own build and carries code from other projects:

- **fflate 0.8.x** (MIT), the region `//#region node_modules/fflate/esm/browser.js` near the top of the file. Its licence:

  > MIT License
  >
  > Copyright (c) 2023 Arjun Barrett
  >
  > Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
  > documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
  > rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
  > persons to whom the Software is furnished to do so, subject to the following conditions:
  >
  > The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
  > Software.
  >
  > THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  > WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  > COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
  > OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

- **The inlined WebAssembly** (`//#region rust/spark-rs/pkg/spark_rs_bg.wasm`) is Spark's own `rust/spark-rs` (MIT, as
  above), compiled with these Rust crates, read from the paths the binary embeds: the Rust standard library
  (MIT OR Apache-2.0); wasm-bindgen 0.2.117, js-sys 0.3.94, serde 1.0.228, serde_core 1.0.228, serde_json 1.0.145,
  anyhow 1.0.98, image 0.25.10, png 0.18.1, image-webp 0.2.4, fdeflate 0.3.7, indexmap 2.13.0, hashbrown 0.16.1,
  smallvec 1.15.1, console_error_panic_hook 0.1.7 (each MIT OR Apache-2.0); miniz_oxide 0.8.9 (MIT OR Zlib OR
  Apache-2.0); memchr 2.7.6 (MIT OR Unlicense); zip 7.2.0 and serde-wasm-bindgen 0.6.5 (MIT); zlib-rs 0.6.3 (Zlib, a
  permissive licence that asks for no notice in binaries). We take each dual-licensed crate under MIT.

## Public-export privacy adjustment

The inlined WASM contains upstream build-machine home directories. Only the `/Users/<build-user>/` substrings in WASM string data were replaced with equal-length neutral `/build/` paths; instructions, offsets and all byte lengths remain unchanged. This is in addition to the import-path rewrites above. The sanitized public `spark.module.js` SHA-256 is `ce6c34c33137fbf1b98753482326ab93ddeba6e945656b4cb15fd82f8d85211b`. The upstream hash is retained in private release evidence, not claimed to match the modified public file.
