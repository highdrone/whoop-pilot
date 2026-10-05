# OpenCV.js 4.12.0 (@techstark/opencv-js 4.12.0-release.1, Apache-2.0, see LICENSE)

`opencv.js.gz` is `gzip -9 -n` of `dist/opencv.js` from the npm tarball `techstark-opencv-js-4.12.0-release.1.tgz`
(sha512-LtTaph9v/HqLPXEg3m1xs2h7QJh10pUpuDT0nj8g77lelWnTwwQrehtd+fXElLOdrkqc4Fea6Z/sJBvEJLYPfw==, matches the registry),
otherwise unchanged: unpacked it is 10,872,779 bytes, sha256 bd0c3e6448043de04f6a64a12cb7b759f78c3ab8f7c35c9f2e0f71c88bb17103
(`OPENCV` in `app/js/nav/features.js`; the worker checks both after unpacking). Gzipped it is 3.5 MB, under the 10 MB
vendoring limit. OpenCV 4.x is Apache-2.0; the file is an Emscripten build (runtime MIT) with the WebAssembly inlined as
base64 and no threads, so it loads in a module worker without cross-origin isolation.

Used by `app/js/nav/features-worker.js` (and `tools/test-loc.mjs` under Node) for `solvePnPRansac` (EPnP) and
`solvePnPRefineLM`. The worker unpacks it with `DecompressionStream` (`openCvText`) while XFeat warms up, and evaluates
the text with a CommonJS-style `module` (a module worker cannot `importScripts`). The build has no fisheye or
undistortPoints functions: the fisheye model lives in `app/js/twin/lens.js` and `app/js/nav/lens.js`. To upgrade:
replace the file (`gzip -9 -n -c dist/opencv.js > opencv.js.gz`), update `OPENCV`'s size and sha256, and re-run
`node tools/test-loc.mjs` and `tools/loc-check.html`.
