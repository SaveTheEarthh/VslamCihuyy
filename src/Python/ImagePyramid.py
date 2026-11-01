#!/usr/bin/env python3
"""
Simulation of the FPGA image-scaler / pyramid hardware from the ORB-SLAM
FPGA accelerator paper (Section 4.2, "Image scalers").

Hardware behaviour being modelled
---------------------------------
* Scale factor 1.2 == downsample by 5/6 per pyramid level.
* Bilinear interpolation using a 2x2 window of original pixels.
* The 4 blending weights depend ONLY on (x mod 6, y mod 6):

      [ x6*y6,        (5-x6)*y6     ]
      [ x6*(5-y6),    (5-x6)*(5-y6) ]

  x6, y6 in 0..4 -> 25 possible weight sets, each summing to 25.
* If x6 == 5 or y6 == 5 the resampled pixel is invalid and is dropped
  (5 of every 6 pixels per row, 5 of every 6 rows).
* Normalisation: divide the weighted sum by 25 (with round-half-up) -> uint8.

Weight / window orientation
---------------------------
Geometrically, an output pixel with fractional offset k/5 (k = x6) sits
k/5 of the way from pixel x to pixel x+1, so pixel x+1 must get weight x6
and pixel x must get weight (5 - x6). The paper's matrix has x6 in its first
column, so it lines up with a window in which the NEWEST pixel comes first
(as in a shift register):

      window = [ p(x+1, y+1)   p(x, y+1) ]
               [ p(x+1, y  )   p(x, y  ) ]

This is the orientation used here; it reproduces exact 1.2x sampling
positions (1.2*u = 0, 1.2, 2.4, 3.6, 4.8, 6.0, ...).

No cv2.resize / cv2.pyrDown is used anywhere. OpenCV is only used for I/O,
colour conversion and drawing the synthetic test image.

Usage
-----
    python fpga_image_pyramid.py [path/to/image] [--levels 4] [--out pyramid.png]
"""

import argparse
import sys

import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

PERIOD = 6          # x and y are taken modulo 6
NORM = 25           # sum of the 4 weights for every valid (x6, y6)


# --------------------------------------------------------------------------- #
# Weight table (the "25 sets of weights" from the paper)
# --------------------------------------------------------------------------- #
def build_weight_lut() -> np.ndarray:
    """LUT[y6, x6] -> 2x2 integer weight matrix, for x6, y6 in 0..4."""
    lut = np.zeros((5, 5, 2, 2), dtype=np.int32)
    for y6 in range(5):
        for x6 in range(5):
            lut[y6, x6] = [
                [x6 * y6,       (5 - x6) * y6],
                [x6 * (5 - y6), (5 - x6) * (5 - y6)],
            ]
            assert lut[y6, x6].sum() == NORM
    return lut


WEIGHT_LUT = build_weight_lut()


def valid_coords(n: int) -> np.ndarray:
    """Input coordinates (along one axis) that produce an output pixel."""
    idx = np.arange(n)
    return idx[idx % PERIOD != 5]


def normalise(acc: np.ndarray) -> np.ndarray:
    """Divide by 25 with round-half-up and clip into uint8."""
    return np.clip((acc + NORM // 2) // NORM, 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------- #
# Vectorised model of one pyramid level (fast)
# --------------------------------------------------------------------------- #
def downscale_5_6(img: np.ndarray) -> np.ndarray:
    """Downscale a uint8 grayscale image by 5/6 using the paper's scaler."""
    if img.ndim != 2:
        raise ValueError("Expected a single-channel (grayscale) image")
    h, w = img.shape

    # Replicate the bottom row / right column so the 2x2 window never
    # reads outside the image (what a hardware border handler would do).
    p = np.pad(img, ((0, 1), (0, 1)), mode="edge").astype(np.int32)

    xs = valid_coords(w)                  # surviving columns
    ys = valid_coords(h)                  # surviving rows
    X, Y = xs[None, :], ys[:, None]

    # 2x2 window, newest pixel first, matching the weight matrix layout.
    win = np.empty((len(ys), len(xs), 2, 2), dtype=np.int32)
    win[..., 0, 0] = p[Y + 1, X + 1]      # p(x+1, y+1)
    win[..., 0, 1] = p[Y + 1, X]          # p(x,   y+1)
    win[..., 1, 0] = p[Y,     X + 1]      # p(x+1, y  )
    win[..., 1, 1] = p[Y,     X]          # p(x,   y  )

    # Per-pixel weights picked from the 25-entry table via (x mod 6, y mod 6).
    weights = WEIGHT_LUT[(ys % PERIOD)[:, None], (xs % PERIOD)[None, :]]

    acc = np.einsum("hwij,hwij->hw", weights, win)
    return normalise(acc)


# --------------------------------------------------------------------------- #
# Cycle-style streaming model (slow, used to cross-check the fast version)
# --------------------------------------------------------------------------- #
def downscale_5_6_streaming(img: np.ndarray) -> np.ndarray:
    """
    Raster-scan model: 1 pixel in per step, a 2x2 window formed from a line
    buffer + shift registers, and an output only when (x6, y6) are both != 5.
    """
    h, w = img.shape
    padded = np.pad(img, ((0, 1), (0, 1)), mode="edge").astype(np.int32)

    line_buffer = padded[0].copy()        # holds the previous row
    out_rows = []

    for r in range(1, h + 1):             # incoming row
        row_out = []
        win_prev = (line_buffer[0], padded[r, 0])   # column c-1 of window
        for c in range(1, w + 1):         # incoming column
            win_cur = (line_buffer[c], padded[r, c])
            # window = [[p(x,y), p(x+1,y)], [p(x,y+1), p(x+1,y+1)]] with
            # x = c-1, y = r-1; flipped to "newest first" order:
            window = np.array([[win_cur[1], win_prev[1]],
                               [win_cur[0], win_prev[0]]], dtype=np.int32)
            x, y = c - 1, r - 1
            x6, y6 = x % PERIOD, y % PERIOD
            if x6 != 5 and y6 != 5:       # otherwise: no output this cycle
                acc = int((WEIGHT_LUT[y6, x6] * window).sum())
                row_out.append(int(normalise(np.int32(acc))))
            win_prev = win_cur
        line_buffer = padded[r].copy()    # current row becomes the delayed row
        if row_out:                       # every 6th row emits nothing
            out_rows.append(row_out)

    return np.array(out_rows, dtype=np.uint8)


# --------------------------------------------------------------------------- #
# Pyramid
# --------------------------------------------------------------------------- #
def build_pyramid(img: np.ndarray, n_levels: int = 4) -> list:
    """Return [original, level1, ..., level_n]; each level built from the last."""
    pyramid = [img]
    for _ in range(n_levels):
        pyramid.append(downscale_5_6(pyramid[-1]))
    return pyramid


# --------------------------------------------------------------------------- #
# Test image helpers
# --------------------------------------------------------------------------- #
def make_synthetic_image(h: int = 480, w: int = 640) -> np.ndarray:
    """Detail-rich image so scaling artefacts are easy to see."""
    yy, xx = np.mgrid[0:h, 0:w]
    img = (0.35 * xx / w + 0.25 * yy / h) * 255
    img += 40 * np.sin(2 * np.pi * xx / 9) * (yy > h // 2)          # fine grating
    img += 70 * (((xx // 24) + (yy // 24)) % 2) * (xx < w // 3)     # checkerboard
    img = np.clip(img, 0, 255).astype(np.uint8)
    for r in (40, 80, 120):
        cv2.circle(img, (int(w * 0.65), int(h * 0.35)), r, 255, 2)
    cv2.rectangle(img, (int(w * 0.1), int(h * 0.1)), (int(w * 0.3), int(h * 0.3)), 0, 3)
    cv2.putText(img, "FPGA VSLAM", (int(w * 0.30), int(h * 0.88)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, 255, 3, cv2.LINE_AA)
    return img


def load_image(path):
    if path is None:
        print("No image given - using a synthetic test image.")
        return make_synthetic_image()
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"Could not read image: {path}")
    return img


# --------------------------------------------------------------------------- #
# Visualisation
# --------------------------------------------------------------------------- #
def plot_pyramid(pyramid: list, out_path=None) -> None:
    n = len(pyramid)
    widths = [im.shape[1] for im in pyramid]
    fig = plt.figure(figsize=(3.2 * n, 4.2))
    gs = GridSpec(1, n, width_ratios=widths, figure=fig)   # panel width ~ image width

    for i, im in enumerate(pyramid):
        ax = fig.add_subplot(gs[0, i])
        ax.imshow(im, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        name = "Original" if i == 0 else f"Level {i}"
        ax.set_title(f"{name}\n{im.shape[1]}x{im.shape[0]}  "
                     f"({(5 / 6) ** i:.3f}x)", fontsize=10)
        ax.set_xticks([])
        ax.set_yticks([])

    fig.suptitle("Image pyramid - 5/6 hardware bilinear scaler (x mod 6 / y mod 6 weights)",
                 fontsize=12)
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=150)
        print(f"Saved figure to {out_path}")
    plt.show()


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", nargs="?", help="input image (any format OpenCV reads)")
    ap.add_argument("--levels", type=int, default=4, help="number of downscaled levels")
    ap.add_argument("--out", default="pyramid_levels.png", help="where to save the figure")
    args = ap.parse_args()

    # Sanity check: vectorised model == streaming (line buffer + window) model.
    rng = np.random.default_rng(0)
    test = rng.integers(0, 256, size=(37, 41), dtype=np.uint8)
    assert np.array_equal(downscale_5_6(test), downscale_5_6_streaming(test)), \
        "Vectorised and streaming models disagree!"
    print("Self-check passed: vectorised model matches streaming model.")

    img = load_image(args.image)
    pyramid = build_pyramid(img, args.levels)

    for i, im in enumerate(pyramid):
        label = "Original" if i == 0 else f"Level {i}"
        print(f"{label:9s}: {im.shape[1]:4d} x {im.shape[0]:4d}   "
              f"(ideal {(5 / 6) ** i:.3f}x)")

    plot_pyramid(pyramid, args.out)


if __name__ == "__main__":
    main()