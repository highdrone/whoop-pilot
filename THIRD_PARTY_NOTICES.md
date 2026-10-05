# Third-party licenses and provenance

The root MIT license applies to original project code, not a blanket relicensing of dependencies. Public dependency author/copyright contacts in license notices are intentionally retained.

| Component | Source/version | License / notices |
| --- | --- | --- |
| Anthropic TypeScript SDK | `@anthropic-ai/sdk` 0.128.0; bundled from the official npm package | MIT; `app/vendor/anthropic-licenses/`, including bundled dependency notices and `VENDOR.json`; standardwebhooks declares MIT in npm metadata but its upstream repository supplies Apache-2.0, also retained |
| Three.js | `three` 0.186.1 | MIT; `app/vendor/three/LICENSE`, `VENDOR.md` |
| Spark | `@sparkjsdev/spark` 2.3.1 | MIT; `app/vendor/spark/LICENSE`, `VENDOR.md`, bundled fflate/Rust attribution |
| ONNX Runtime Web | `onnxruntime-web` 1.30.0 | MIT and third-party notices; `app/vendor/ort/LICENSE`, `ThirdPartyNotices.txt` |
| OpenCV.js | `@techstark/opencv-js` 4.12.0-release.1 / OpenCV 4.12.0 | Apache-2.0; `app/vendor/opencv/LICENSE`, `VENDOR.md` |
| XFeat ONNX | Kazuhito00's public export, pinned commit and SHA-256 in `app/vendor/xfeat/VENDOR.md` | Apache-2.0; adjacent LICENSE |
| Node USB / Lua VM | `usb` 2.18.0, `fengari` 0.1.5 | Installed by npm; licenses and transitive versions recorded in `tools/package-lock.json` |

Larger RF-DETR, DINOv2 and depth models are fetched from their upstream sources by the app rather than redistributed here. Review their model cards/licenses before use. Procedural actor models are generated in project code; no downloaded people/pet assets are bundled.

Goggles protocol attribution is retained in the original implementation comments: public protocol research from `soldnenz/dji-goggles-lab` and `planktonwhc/orbit-rs` (MIT). No proprietary goggles firmware or SDK is included.
