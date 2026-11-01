#!/usr/bin/env python3
"""
Orientation module ONLY (Section 4.5 of "ORB-based SLAM accelerator on SoC FPGA").
Pure Python, no third-party packages.

Per clock the module receives one 37x1 pixel column (from the 37-row line buffer):
    S(Cin)   = sum of the column
    m01(Cin) = sum of y * pixel          (y = -18 .. +18, top to bottom)
    Cout terms come from a 37-cycle delay line (Cout = Cin delayed by 37 clocks)

and updates the moment registers:
    m00' = m00 + S(Cin) - S(Cout)
    m01' = m01 + m01(Cin) - m01(Cout)
    m10' = m10 - 18*S(Cin) - 19*S(Cout) + m00

Angle: quadrant from the signs of (m10, m01); then N_D/4 parallel comparators
    |m10| * tan(theta_k) > |m01|
and a priority encoder pick the sector. Output = (quadrant, sector) + (x, y).

Convention: the newest column enters at window x = -18, so the hardware frame is
mirrored in x:  m10_hw = -m10_image  and  theta_img = 180 - theta_hw (mod 360).
"""
import math
import random
from collections import deque
from typing import NamedTuple

WINDOW = 37
HALF = WINDOW // 2          # 18
_YS = tuple(range(-HALF, HALF + 1))


class Result(NamedTuple):
    x: int            # window-centre column (keypoint x)
    y: int            # window-centre row    (keypoint y)
    m00: int
    m10: int          # hardware frame (x mirrored)
    m01: int
    quadrant: int     # 0..3
    sector: int       # 0..N_D/4-1 inside the first quadrant
    angle_deg: float  # quantised angle in the hardware frame


class OrientationModule:
    """
    n_sectors  N_D, number of sectors on the full circle (multiple of 4). Paper: 64.
    rounding   "paper"   comparators use tan(sector centre), as in Table 2
                         (angle is rounded up to the next sector centre).
               "nearest" comparators use tan(sector boundary) (true nearest sector).
    """

    def __init__(self, n_sectors=64, rounding="paper", tan_frac_bits=8):
        if n_sectors < 4 or n_sectors % 4:
            raise ValueError("n_sectors must be a positive multiple of 4")
        if rounding not in ("paper", "nearest"):
            raise ValueError("rounding must be 'paper' or 'nearest'")
        self.nq = n_sectors // 4
        width = 90.0 / self.nq
        self.centres = [(k + 0.5) * width for k in range(self.nq)]
        thresholds = self.centres if rounding == "paper" else \
            [k * width for k in range(1, self.nq)]
        self.frac_bits = tan_frac_bits
        # hard-coded fixed-point tan constants (shift-and-add multipliers in hardware)
        self.tan_consts = [round(math.tan(math.radians(a)) * (1 << tan_frac_bits))
                           for a in thresholds]
        self.reset()

    def reset(self):
        self.m00 = self.m10 = self.m01 = 0
        self._delay = deque([(0, 0)] * WINDOW, maxlen=WINDOW)   # 37-cycle delay line

    def step(self, col):
        """One clock. `col` = 37 pixels, top (y=-18) to bottom (y=+18). Returns (m00, m10, m01)."""
        s_in = sum(col)
        m01_in = sum(y * p for y, p in zip(_YS, col))
        s_out, m01_out = self._delay[0]          # terms of Cout (entered 37 clocks ago)
        self._delay.append((s_in, m01_in))
        m00, m10, m01 = self.m00, self.m10, self.m01      # values before the clock edge
        self.m10 = m10 - HALF * s_in - (HALF + 1) * s_out + m00
        self.m01 = m01 + m01_in - m01_out
        self.m00 = m00 + s_in - s_out
        return self.m00, self.m10, self.m01

    def quantize(self, m10, m01):
        """Moments -> (quadrant, sector, angle_deg)."""
        if m01 >= 0:
            quadrant = 0 if m10 >= 0 else 1
        else:
            quadrant = 3 if m10 >= 0 else 2
        a10, a01 = abs(m10), abs(m01)                    # fold into first quadrant
        rhs = a01 << self.frac_bits
        hits = [a10 * c > rhs for c in self.tan_consts]  # parallel comparators
        sector = hits.index(True) if True in hits else len(hits)   # priority encoder
        sector = min(sector, self.nq - 1)
        t = self.centres[sector]
        return quadrant, sector, (t, 180 - t, 180 + t, 360 - t)[quadrant]

    def process_image(self, img, keypoints=None):
        """
        Stream a 2-D grayscale image (list of rows) in raster order, 1 pixel per clock.
        Emits one Result per valid 37x37 window (centre >= 18 px from every border),
        or only at the given (x, y) keypoints.
        """
        rows = [[int(p) for p in r] for r in img]
        h, w = len(rows), len(rows[0])
        wanted = None if keypoints is None else {(int(x), int(y)) for x, y in keypoints}
        self.reset()
        linebuf = deque([[0] * w] * WINDOW, maxlen=WINDOW)    # rows r-36 .. r
        out = []
        for r in range(h):
            linebuf.append(rows[r])
            for c, col in enumerate(zip(*linebuf)):           # one column per clock
                self.step(col)
                if r >= WINDOW - 1 and c >= WINDOW - 1:       # window fully inside one row
                    x, y = c - HALF, r - HALF
                    if wanted is None or (x, y) in wanted:
                        q, s, a = self.quantize(self.m10, self.m01)
                        out.append(Result(x, y, self.m00, self.m10, self.m01, q, s, a))
        return out


def hw_to_image_angle(angle_hw_deg):
    """Hardware frame is mirrored in x -> convert to the usual image frame."""
    return (180.0 - angle_hw_deg) % 360.0


# ------------------------------------------------------------------------- #
# Demo / sanity checks
# ------------------------------------------------------------------------- #
def brute_force_moments(img, cx, cy):
    """Direct 37x37 sums in the module's (x-mirrored) frame."""
    m00 = m10 = m01 = 0
    for dy in range(-HALF, HALF + 1):
        for dx in range(-HALF, HALF + 1):
            p = img[cy + dy][cx + dx]
            m00 += p
            m10 += -dx * p
            m01 += dy * p
    return m00, m10, m01


def check_against_brute_force(h=60, w=70, seed=0):
    rng = random.Random(seed)
    img = [[rng.randrange(256) for _ in range(w)] for _ in range(h)]
    results = OrientationModule().process_image(img)
    assert len(results) == (h - WINDOW + 1) * (w - WINDOW + 1)
    for r in results:
        assert (r.m00, r.m10, r.m01) == brute_force_moments(img, r.x, r.y), (r.x, r.y)
    print(f"[ok] moments bit-exact vs brute force on all {len(results)} windows "
          f"({w}x{h} random image)\n")


def demo_blob(n_sectors=64, rounding="paper"):
    """Bright Gaussian blob 9 px away from the window centre in direction phi."""
    size, c0 = 80, 40
    mod = OrientationModule(n_sectors, rounding)
    print(f"Blob demo: N_D={n_sectors}, rounding='{rounding}'")
    print(f"{'phi':>5}{'m10_hw':>10}{'m01':>9}{'quad':>5}{'sec':>5}"
          f"{'angle_hw':>10}{'angle_img':>10}{'err':>8}")
    for phi in range(0, 360, 30):
        bx = c0 + 9 * math.cos(math.radians(phi))
        by = c0 + 9 * math.sin(math.radians(phi))
        img = [[int(255 * math.exp(-((x - bx) ** 2 + (y - by) ** 2) / 32.0))
                for x in range(size)] for y in range(size)]
        (r,) = mod.process_image(img, keypoints=[(c0, c0)])
        a_img = hw_to_image_angle(r.angle_deg)
        err = (a_img - phi + 180) % 360 - 180
        print(f"{phi:>5}{r.m10:>10}{r.m01:>9}{r.quadrant:>5}{r.sector:>5}"
              f"{r.angle_deg:>10.2f}{a_img:>10.2f}{err:>8.2f}")
    print()


if __name__ == "__main__":
    check_against_brute_force()
    demo_blob(64, "paper")
    demo_blob(64, "nearest")
