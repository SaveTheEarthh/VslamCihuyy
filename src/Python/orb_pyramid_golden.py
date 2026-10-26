#!/usr/bin/env python3
"""
Golden model: ORB-SLAM FPGA image pyramid (VSLAM 2022, Sec. 3.4 and 4.2)
=======================================================================

Hardware behaviour modelled
---------------------------
* 4 pyramid levels: level 0 = original image, levels 1..3 = three cascaded
  image scalers (Fig. 6 shows three scalers in a chain).
* Scale factor 1.2 = 6/5  ->  output size is 5/6 of the input size.
* Bilinear interpolation using ONLY (x mod 6, y mod 6) of the input pixel.
* Weights for the 2x2 neighbourhood anchored at input pixel (x, y):

      w(x,y)     = (5 - x6) * (5 - y6)
      w(x+1,y)   =      x6  * (5 - y6)
      w(x,y+1)   = (5 - x6) *      y6
      w(x+1,y+1) =      x6  *      y6          (all four sum to 25)

  This is the matrix printed in Sec. 4.2 of the paper; the paper's matrix is
  written for a streaming 2x2 window whose top-left element is the NEWEST
  pixel, i.e. the (x+1, y+1) neighbour of the anchor.
* If x6 == 5 or y6 == 5 the resampled pixel is invalid (not emitted).
  -> 5 of every 6 columns and 5 of every 6 rows produce an output.
* The anchor needs a complete 2x2 window, so anchors with x == W-1 or
  y == H-1 are never emitted.

NOT specified by the paper (configurable here, please match your RTL):
* Normalisation of the 13-bit accumulator (0..6375) by 25:
    --rounding round  ->  (acc + 12) // 25     (default)
    --rounding floor  ->  acc // 25
  The raw accumulator is also dumped so you can test the MAC separately.

Usage
-----
    python3 orb_pyramid_golden.py --selftest
    python3 orb_pyramid_golden.py --synthetic 640x480 --outdir golden_out
    python3 orb_pyramid_golden.py --input frame.png   --outdir golden_out
"""
import argparse
import json
import os
import sys
from fractions import Fraction

import numpy as np

PERIOD = 6          # input pixels per modulo period
OUT_PER_PERIOD = 5  # valid outputs per period (x6 in 0..4)
W_SUM = 25          # sum of the four bilinear weights
ROUND_BIAS = 12     # floor(25/2) for round-half-up


# --------------------------------------------------------------------------
# Core hardware-faithful primitives
# --------------------------------------------------------------------------
def pixel_weights(x6, y6):
    """Return (w00, w01, w10, w11) for neighbours
    I(x,y), I(x+1,y), I(x,y+1), I(x+1,y+1).  Requires x6, y6 in 0..4."""
    assert 0 <= x6 <= 4 and 0 <= y6 <= 4
    return ((5 - x6) * (5 - y6),
            x6 * (5 - y6),
            (5 - x6) * y6,
            x6 * y6)


def normalize(acc, rounding):
    if rounding == "round":
        return (acc + ROUND_BIAS) // W_SUM
    if rounding == "floor":
        return acc // W_SUM
    raise ValueError("rounding must be 'round' or 'floor'")


def downscale_stream(img, rounding="round"):
    """Scalar, raster-order model (one input position per 'cycle').

    Returns (out_img, acc_img, valid_map)
      out_img   : uint8 [Ho, Wo]   normalised output pixels
      acc_img   : uint16[Ho, Wo]   raw weighted sums (0..6375)
      valid_map : uint8 [H,  W ]   1 at every ANCHOR (x,y) that yields an output
    """
    H, W = img.shape
    valid = np.zeros((H, W), dtype=np.uint8)
    acc_rows = []
    for y in range(H - 1):
        y6 = y % PERIOD
        if y6 == 5:                      # every 6th row: no output
            continue
        row = []
        for x in range(W - 1):
            x6 = x % PERIOD
            if x6 == 5:                  # every 6th column: no output
                continue
            w00, w01, w10, w11 = pixel_weights(x6, y6)
            acc = (w00 * int(img[y, x]) + w01 * int(img[y, x + 1]) +
                   w10 * int(img[y + 1, x]) + w11 * int(img[y + 1, x + 1]))
            valid[y, x] = 1
            row.append(acc)
        acc_rows.append(row)
    acc_img = np.array(acc_rows, dtype=np.uint16)
    out_img = np.vectorize(lambda a: normalize(int(a), rounding),
                           otypes=[np.uint8])(acc_img)
    return out_img, acc_img, valid


def downscale_vectorized(img, rounding="round"):
    """Same arithmetic with numpy (cross-check / speed)."""
    H, W = img.shape
    xs = np.array([x for x in range(W - 1) if x % PERIOD != 5])
    ys = np.array([y for y in range(H - 1) if y % PERIOD != 5])
    a = img.astype(np.int64)
    X6 = (xs % PERIOD)[None, :]
    Y6 = (ys % PERIOD)[:, None]
    acc = ((5 - X6) * (5 - Y6) * a[np.ix_(ys, xs)] +
           X6 * (5 - Y6) * a[np.ix_(ys, xs + 1)] +
           (5 - X6) * Y6 * a[np.ix_(ys + 1, xs)] +
           X6 * Y6 * a[np.ix_(ys + 1, xs + 1)])
    out = ((acc + ROUND_BIAS) // W_SUM if rounding == "round"
           else acc // W_SUM)
    return out.astype(np.uint8), acc.astype(np.uint16)


def build_pyramid(img, levels=4, rounding="round"):
    """Cascaded pyramid: level n is produced from level n-1."""
    pyr = [{"img": img, "acc": None, "valid": None}]
    for _ in range(1, levels):
        out, acc, valid = downscale_stream(pyr[-1]["img"], rounding)
        pyr.append({"img": out, "acc": acc, "valid": valid})
    return pyr


# --------------------------------------------------------------------------
# Self test
# --------------------------------------------------------------------------
def selftest():
    # 1. weight table: 25 combinations, each summing to 25
    combos = {pixel_weights(a, b) for a in range(5) for b in range(5)}
    assert len(combos) == 25
    assert all(sum(w) == W_SUM for w in combos)

    # 2. loop model == numpy model, both rounding modes
    rng = np.random.RandomState(1234)
    img = rng.randint(0, 256, size=(61, 83), dtype=np.uint8)
    for mode in ("round", "floor"):
        o1, a1, _ = downscale_stream(img, mode)
        o2, a2 = downscale_vectorized(img, mode)
        assert np.array_equal(o1, o2) and np.array_equal(a1, a2), mode

    # 3. mod-6 weights == exact bilinear sampling at source position 6j/5
    small = rng.randint(0, 256, size=(31, 37), dtype=np.uint8)
    _, acc, _ = downscale_stream(small, "floor")
    H, W = small.shape
    Ho, Wo = acc.shape
    for oy in range(Ho):
        for ox in range(Wo):
            py, px = Fraction(6 * oy, 5), Fraction(6 * ox, 5)
            y0, x0 = int(py), int(px)
            fy, fx = py - y0, px - x0
            exact = ((1 - fx) * (1 - fy) * int(small[y0, x0]) +
                     fx * (1 - fy) * int(small[y0, x0 + 1]) +
                     (1 - fx) * fy * int(small[y0 + 1, x0]) +
                     fx * fy * int(small[y0 + 1, x0 + 1]))
            assert exact == Fraction(int(acc[oy, ox]), W_SUM), (oy, ox)
    # no extra valid positions beyond the ones the loop produced
    assert (6 * Ho // 5) <= H - 1 and (6 * Wo // 5) <= W - 1

    # 4. dimensions for 640x480 (TUM) -> 533x400 -> 444x333 -> 370x277
    dims = [p["img"].shape[::-1]
            for p in build_pyramid(np.zeros((480, 640), np.uint8))]
    assert dims == [(640, 480), (533, 400), (444, 333), (370, 277)], dims

    # 5. value range: all-255 image stays 255, all-0 stays 0
    for v in (0, 255):
        o, _, _ = downscale_stream(np.full((20, 20), v, np.uint8), "round")
        assert (o == v).all()
    print("selftest PASSED (weights, loop==numpy, exact-bilinear, dims, range)")


# --------------------------------------------------------------------------
# I/O helpers
# --------------------------------------------------------------------------
def load_gray(path):
    from PIL import Image
    return np.asarray(Image.open(path).convert("L"), dtype=np.uint8)


def write_hex(path, arr, digits):
    """One value per line, raster order, $readmemh compatible."""
    with open(path, "w") as f:
        for v in arr.reshape(-1):
            f.write(f"{int(v):0{digits}x}\n")


def dump(pyr, outdir, rounding):
    os.makedirs(outdir, exist_ok=True)
    meta = {"rounding": rounding, "levels": []}
    try:
        from PIL import Image
    except ImportError:
        Image = None
    for n, lvl in enumerate(pyr):
        im = lvl["img"]
        H, W = im.shape
        write_hex(f"{outdir}/level{n}_pixels.hex", im, 2)
        np.save(f"{outdir}/level{n}_pixels.npy", im)
        if Image:
            Image.fromarray(im).save(f"{outdir}/level{n}.png")
        entry = {"level": n, "width": int(W), "height": int(H),
                 "pixels_hex": f"level{n}_pixels.hex"}
        if n > 0:
            write_hex(f"{outdir}/level{n}_acc.hex", lvl["acc"], 4)
            write_hex(f"{outdir}/level{n}_valid_by_anchor.hex",
                      lvl["valid"], 1)
            entry["acc_hex"] = f"level{n}_acc.hex"
            entry["valid_hex"] = f"level{n}_valid_by_anchor.hex"
            entry["input_width"] = int(pyr[n - 1]["img"].shape[1])
            entry["input_height"] = int(pyr[n - 1]["img"].shape[0])
        meta["levels"].append(entry)
    with open(f"{outdir}/manifest.json", "w") as f:
        json.dump(meta, f, indent=2)
    for e in meta["levels"]:
        print(f"level {e['level']}: {e['width']}x{e['height']}")
    print(f"files written to {outdir}/")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--input", help="image file (converted to 8-bit gray)")
    ap.add_argument("--synthetic", metavar="WxH",
                    help="deterministic pseudo-random test image, e.g. 640x480")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--outdir", default="golden_out")
    ap.add_argument("--levels", type=int, default=4)
    ap.add_argument("--rounding", choices=["round", "floor"], default="round")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return
    if args.input:
        img = load_gray(args.input)
    elif args.synthetic:
        w, h = (int(v) for v in args.synthetic.lower().split("x"))
        img = np.random.RandomState(args.seed).randint(
            0, 256, size=(h, w), dtype=np.uint8)
    else:
        ap.error("give --input, --synthetic or --selftest")
    if min(img.shape) < 2:
        sys.exit("image must be at least 2x2")

    pyr = build_pyramid(img, args.levels, args.rounding)
    # cross-check the scalar loop model against the numpy model
    for n in range(1, len(pyr)):
        o, a = downscale_vectorized(pyr[n - 1]["img"], args.rounding)
        assert np.array_equal(o, pyr[n]["img"])
        assert np.array_equal(a, pyr[n]["acc"])
    dump(pyr, args.outdir, args.rounding)


if __name__ == "__main__":
    main()
