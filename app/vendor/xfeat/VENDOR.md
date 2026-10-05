# XFeat backbone, 640x480 ONNX (Apache-2.0, see LICENSE)

XFeat: Accelerated Features for Lightweight Image Matching (Potje, Cadar, Araujo, Martins, Nascimento, CVPR 2024),
https://github.com/verlab/accelerated_features (code and weights Apache-2.0). `xfeat_640x480.onnx` is the fixed-shape
export of those weights by Kazuhito00, https://github.com/Kazuhito00/XFeat-Image-Matching-ONNX-Sample (Apache-2.0,
`LICENSE` here is that repository's), file `onnx_model/xfeat_640x480.onnx` at commit
6eb24aad8997243881f275a5963acc5d19f7c332 (git blob c8fd1a109a42e0364202ebca01f380ada2a4dedf), unchanged:
4,569,993 bytes, sha256 f99d7dd5fc454067fa59b53e7f184b2f6da3ddf636030ce7430e1276e7558b08.

Graph: input `input` float32 [1, 3, 480, 640] (RGB / 255); outputs at 1/8 resolution: descriptors [1, 64, 60, 80],
keypoint logits [1, 65, 60, 80] (64 sub-pixel cells + dustbin) and reliability [1, 1, 60, 80], told apart by their
channel count. Softmax, NMS, top-k and descriptor sampling run in `app/js/nav/features.js`; the mutual-nearest-neighbour
matcher is a graph `features.js` writes itself (MatMul + ArgMax + ReduceMax), no file.

Not used, and not to be added: the Derkai52 LighterGlue export (no licence), SuperPoint weights (non-commercial).
