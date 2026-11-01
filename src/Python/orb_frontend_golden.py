#!/usr/bin/env python3
"""
Golden model for the ORB front end that sits behind the image pyramid
(VSLAM 2022, Sec. 3.5 FAST, Sec. 4.3 FAST HW, Sec. 4.4 Gaussian, Fig. 8, Fig. 9).

It READS the pyramid hex files already produced by orb_pyramid_golden.py
(levelN_pixels.hex + manifest.json) and writes, per level N = 0..3:

  levelN_gauss.hex       7x7 binomial Gaussian of the interior pixels, 8-bit,
                         raster order, size (W-6) x (H-6)
  levelN_fastscore.hex   FAST score of the interior pixels, 3 hex digits,
                         raster order, size (W-6) x (H-6)
  levelN_kp.hex          keypoints after 3x3 NMS in raster order,
                         9 hex digits = {score[11:0], y[11:0], x[11:0]}
  levelN_kp_count.hex    number of keypoints (one line, 8 hex digits)
  frontend_manifest.json summary

Definitions (match the RTL bit-exactly)
---------------------------------------
FAST   : ring = 16 pixels of the radius-3 Bresenham circle, clockwise from top.
         brighter_i : ring_i  >  p + t        darker_i : p > ring_i + t
         corner     : 9 circularly contiguous brighter OR 9 contiguous darker
         score      : sum_i |ring_i - p| (all 16 pixels, paper Sec. 3.5),
                      forced to 0 when the pixel is not a corner
NMS    : 3x3 on the score map. Keypoint if score > 0 and
         score >  up-left, up, up-right, left        (earlier in raster order)
         score >= right, down-left, down, down-right (later in raster order)
Gauss  : kernel [1 6 15 20 15 6 1] (x) [1 6 15 20 15 6 1], sum 4096,
         out = floor(sum(k*p) / 4096) = >> 12   (Fig. 9, DSP-free division)

Coordinates are image coordinates of the level (origin top-left).
Interior FAST/Gauss centres: x in [3, W-4], y in [3, H-4].
NMS centres / keypoints:     x in [4, W-5], y in [4, H-5].

Usage
-----
    python3 orb_frontend_golden.py --dir imageTest --fast-thr 20
"""
import argparse
import json
import os

import numpy as np

from orb_pyramid_golden import write_hex

RING = [(0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3),
        (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3)]
KERNEL = np.array([1, 6, 15, 20, 15, 6, 1], dtype=np.int64)


def gaussian7(img):
    """Interior (H-6, W-6) floor(sum/4096); unrounded separable passes."""
    a = img.astype(np.int64)
    H, W = a.shape
    h = np.zeros((H, W - 6), dtype=np.int64)
    for c in range(7):
        h += KERNEL[c] * a[:, c:W - 6 + c]
    v = np.zeros((H - 6, W - 6), dtype=np.int64)
    for r in range(7):
        v += KERNEL[r] * h[r:H - 6 + r, :]
    return (v >> 12).astype(np.int64)


def fast_score(img, thr):
    a = img.astype(np.int64)
    H, W = a.shape
    p = a[3:H - 3, 3:W - 3]
    ring = [a[3 + dy:H - 3 + dy, 3 + dx:W - 3 + dx] for dx, dy in RING]
    bri = [r > p + thr for r in ring]
    dar = [p > r + thr for r in ring]
    corner = np.zeros(p.shape, dtype=bool)
    for j in range(16):
        b = np.ones(p.shape, dtype=bool)
        d = np.ones(p.shape, dtype=bool)
        for k in range(9):
            b &= bri[(j + k) % 16]
            d &= dar[(j + k) % 16]
        corner |= b | d
    total = np.zeros(p.shape, dtype=np.int64)
    for r in ring:
        total += np.abs(r - p)
    return np.where(corner, total, 0)


def nms3x3(s):
    """Return list of (x, y, score) in raster order; s is the (H-6, W-6) map."""
    c = s[1:-1, 1:-1]
    ul, u, ur = s[:-2, :-2], s[:-2, 1:-1], s[:-2, 2:]
    l, r = s[1:-1, :-2], s[1:-1, 2:]
    dl, d, dr = s[2:, :-2], s[2:, 1:-1], s[2:, 2:]
    kp = ((c > 0) & (c > ul) & (c > u) & (c > ur) & (c > l) &
          (c >= r) & (c >= dl) & (c >= d) & (c >= dr))
    ys, xs = np.nonzero(kp)
    return [(int(x) + 4, int(y) + 4, int(c[y, x])) for y, x in zip(ys, xs)]


def load_level(d, e):
    vals = [int(t, 16) for t in open(os.path.join(d, e["pixels_hex"])) if t.strip()]
    return np.array(vals, dtype=np.int64).reshape(e["height"], e["width"])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--dir", default="imageTest",
                    help="folder with levelN_pixels.hex and manifest.json")
    ap.add_argument("--fast-thr", type=int, default=20)
    args = ap.parse_args()

    man = json.load(open(os.path.join(args.dir, "manifest.json")))
    out = {"fast_thr": args.fast_thr, "levels": []}
    for e in man["levels"]:
        n = e["level"]
        img = load_level(args.dir, e)
        H, W = img.shape
        if min(H, W) < 9:
            raise SystemExit(f"level {n} is {W}x{H}; need at least 9x9")
        g = gaussian7(img)
        s = fast_score(img, args.fast_thr)
        kps = nms3x3(s)
        write_hex(f"{args.dir}/level{n}_gauss.hex", g, 2)
        write_hex(f"{args.dir}/level{n}_fastscore.hex", s, 3)
        with open(f"{args.dir}/level{n}_kp.hex", "w") as f:
            for x, y, sc in kps:
                f.write(f"{(sc << 24) | (y << 12) | x:09x}\n")
        with open(f"{args.dir}/level{n}_kp_count.hex", "w") as f:
            f.write(f"{len(kps):08x}\n")
        out["levels"].append({"level": n, "width": W, "height": H,
                              "map_w": W - 6, "map_h": H - 6,
                              "keypoints": len(kps)})
        print(f"level {n}: {W}x{H}  gauss/score maps {W-6}x{H-6}  keypoints {len(kps)}")
    json.dump(out, open(os.path.join(args.dir, "frontend_manifest.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
