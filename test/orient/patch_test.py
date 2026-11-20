#!/usr/bin/env python3
"""
Visual test of the orientation module on single 37x37 patches.
Pure Python (standard library only).

For every patch it shows
    1. the 37x37 input,
    2. the expected result  (Python model, orientation_only.py),
    3. the actual result    (RTL simulation output, rtl_results.txt),
and flags MATCH / MISMATCH.

usage (run from test/orient):
    python patch_test.py          gen -> simulate with Icarus Verilog -> plot
    python patch_test.py gen      write patches.hex, patch_params.svh, patch_names.txt
    python patch_test.py sim      compile + run tb_patch.sv with Icarus Verilog (+gap=NN optional)
    python patch_test.py plot     draw patch_visual.html / .svg from rtl_results.txt
                                  (use this after running the ModelSim script)
"""
import math
import os
import random
import re
import shutil
import subprocess
import sys

from orientation_only import HALF, OrientationModule

HERE = os.path.dirname(os.path.abspath(__file__))
RTL_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "src", "verilog"))
N = 37


def P(name):
    return os.path.join(HERE, name)


# --------------------------------------------------------------------------- patches
def patch_blob(angle, dist=9.0, sigma=5.0, bg=20, peak=255):
    bx = HALF + dist * math.cos(math.radians(angle))
    by = HALF + dist * math.sin(math.radians(angle))
    return [[min(255, int(bg + (peak - bg) * math.exp(-((x - bx) ** 2 + (y - by) ** 2)
                                                     / (2 * sigma ** 2))))
             for x in range(N)] for y in range(N)]


def patch_wedge(direction, half=30, radius=17, lo=30, hi=230):
    img = [[lo] * N for _ in range(N)]
    for y in range(N):
        for x in range(N):
            dx, dy = x - HALF, y - HALF
            r = math.hypot(dx, dy)
            if r == 0:
                img[y][x] = hi
            elif r <= radius:
                ang = math.degrees(math.atan2(dy, dx))
                if abs((ang - direction + 180) % 360 - 180) <= half:
                    img[y][x] = hi
    return img


def make_patches():
    rng = random.Random(3)
    return [
        ("Bright blob toward 0 deg (right)", patch_blob(0)),
        ("Bright blob toward 45 deg (down-right)", patch_blob(45)),
        ("Bright blob toward 120 deg", patch_blob(120)),
        ("Bright blob toward 200 deg", patch_blob(200)),
        ("Bright blob toward 290 deg (up)", patch_blob(290)),
        ("Bright wedge (corner) toward 330 deg", patch_wedge(330)),
        ("Flat patch (m10 = m01 = 0)", [[128] * N for _ in range(N)]),
        ("Random noise", [[rng.randrange(256) for _ in range(N)] for _ in range(N)]),
    ]


def table_cfg():
    """N_SECT / rounding / frac_bits the RTL table was generated with."""
    try:
        txt = open(os.path.join(RTL_DIR, "orient_tan_table.svh")).read()
        m = re.search(r"N_SECT=(\d+), rounding=(\w+), frac_bits=(\d+)", txt)
        return int(m.group(1)), m.group(2), int(m.group(3))
    except (OSError, AttributeError):
        return 64, "paper", 8


# --------------------------------------------------------------------------- gen / sim
def cmd_gen():
    patches = make_patches()
    n_sect = table_cfg()[0]
    with open(P("patches.hex"), "w") as f:
        for _, img in patches:
            for row in img:
                for v in row:
                    f.write(f"{v:02x}\n")
    with open(P("patch_params.svh"), "w") as f:
        f.write(f"localparam int N_PATCH = {len(patches)};\nlocalparam int N_SECT = {n_sect};\n")
    with open(P("patch_names.txt"), "w") as f:
        f.write("\n".join(n for n, _ in patches) + "\n")
    print(f"gen: {len(patches)} patches, N_SECT={n_sect} -> patches.hex, patch_params.svh")


def find_tool(name):
    p = shutil.which(name)
    if p:
        return p
    exe = name + (".exe" if os.name == "nt" else "")
    for d in (r"C:\msys64\ucrt64\bin", r"C:\msys64\mingw64\bin", r"C:\iverilog\bin",
              r"C:\Program Files\iverilog\bin"):
        c = os.path.join(d, exe)
        if os.path.exists(c):
            return c
    return None


def cmd_sim(gap=0):
    iv, vvp = find_tool("iverilog"), find_tool("vvp")
    if not iv or not vvp:
        sys.exit("iverilog/vvp not found.  Install Icarus Verilog, or use ModelSim:\n"
                 "  python patch_test.py gen ; do run_modelsim_patch.do ; python patch_test.py plot")
    if os.path.exists(P("rtl_results.txt")):
        os.remove(P("rtl_results.txt"))
    rel = os.path.relpath(RTL_DIR, HERE).replace("\\", "/")      # relative: no spaces in paths
    rtl = sorted(f for f in os.listdir(RTL_DIR) if f.endswith(".sv"))
    subprocess.run([iv, "-g2012", "-o", "sim_patch", f"-I{rel}", "-I."]
                   + [f"{rel}/{f}" for f in rtl] + ["tb_patch.sv"], cwd=HERE, check=True)
    subprocess.run([vvp, "sim_patch"] + ([f"+gap={gap}"] if gap else []), cwd=HERE, check=True)


# --------------------------------------------------------------------------- loading
def load_patches():
    vals = [int(l, 16) for l in open(P("patches.hex")).read().split()]
    n = len(vals) // (N * N)
    return [[vals[p * N * N + y * N: p * N * N + (y + 1) * N] for y in range(N)] for p in range(n)]


def load_names(n):
    try:
        return [l.rstrip("\n") for l in open(P("patch_names.txt"))][:n]
    except OSError:
        return [f"patch {i}" for i in range(n)]


def load_rtl():
    out = []
    for l in open(P("rtl_results.txt")):
        t = l.split()
        if len(t) == 7:
            out.append(tuple(int(v) for v in t))                 # x y m00 m10 m01 q s
    return out


# --------------------------------------------------------------------------- drawing
def polar(cx, cy, r, deg):
    a = math.radians(deg)
    return cx + r * math.cos(a), cy + r * math.sin(a)


def img_angle(q, s, nq):
    """Quantised angle in the image frame (0 deg = right, 90 deg = down)."""
    t = (s + 0.5) * 90.0 / nq
    hw = (t, 180 - t, 180 + t, 360 - t)[q]            # hardware window frame (x mirrored)
    return (180.0 - hw) % 360.0


def exact_img_angle(m10, m01):
    if m10 == 0 and m01 == 0:
        return None
    return (180.0 - math.degrees(math.atan2(m01, m10))) % 360.0


def txt(x, y, s, size=11, anchor="middle", fill="#222", mono=True, weight="normal"):
    fam = "Consolas, 'DejaVu Sans Mono', Menlo, monospace" if mono else "'Segoe UI', Arial, sans-serif"
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" '
            f'fill="{fill}" font-family="{fam}" font-weight="{weight}">{s}</text>')


def draw_patch(out, img, x0, y0, cell=6):
    out.append('<g shape-rendering="crispEdges">')
    for y, row in enumerate(img):
        x = 0
        while x < N:                                    # merge runs of equal pixels
            e = x
            while e + 1 < N and row[e + 1] == row[x]:
                e += 1
            v = row[x]
            out.append(f'<rect x="{x0 + x * cell}" y="{y0 + y * cell}" width="{(e - x + 1) * cell}" '
                       f'height="{cell}" fill="rgb({v},{v},{v})"/>')
            x = e + 1
    out.append("</g>")
    cx, cy = x0 + HALF * cell, y0 + HALF * cell          # mark the window centre pixel
    out.append(f'<rect x="{cx}" y="{cy}" width="{cell}" height="{cell}" fill="none" '
               f'stroke="#e33" stroke-width="1.3"/>')
    out.append(f'<rect x="{x0}" y="{y0}" width="{N * cell}" height="{N * cell}" fill="none" stroke="#666"/>')


def draw_dial(out, cx, cy, R, nq, q, s, exact, colour):
    out.append(f'<circle cx="{cx}" cy="{cy}" r="{R}" fill="#fff" stroke="#888"/>')
    w = 90.0 / nq
    for j in range(4 * nq):                              # sector centres
        x1, y1 = polar(cx, cy, R, (j + 0.5) * w)
        x2, y2 = polar(cx, cy, R - 4, (j + 0.5) * w)
        out.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#bbb" stroke-width="0.8"/>')
    out.append(f'<line x1="{cx - R}" y1="{cy}" x2="{cx + R}" y2="{cy}" stroke="#e4e4e4"/>')
    out.append(f'<line x1="{cx}" y1="{cy - R}" x2="{cx}" y2="{cy + R}" stroke="#e4e4e4"/>')
    if exact is not None:                                # exact atan2 of the moments
        x2, y2 = polar(cx, cy, R - 2, exact)
        out.append(f'<line x1="{cx}" y1="{cy}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#555" '
                   f'stroke-width="1.2" stroke-dasharray="3 3"/>')
    a = img_angle(q, s, nq)                              # quantised sector -> arrow
    bx, by = polar(cx, cy, R - 14, a)
    tx, ty = polar(cx, cy, R - 4, a)
    p1 = polar(bx, by, 4.5, a + 90)
    p2 = polar(bx, by, 4.5, a - 90)
    out.append(f'<line x1="{cx}" y1="{cy}" x2="{bx:.1f}" y2="{by:.1f}" stroke="{colour}" stroke-width="3"/>')
    out.append(f'<polygon points="{tx:.1f},{ty:.1f} {p1[0]:.1f},{p1[1]:.1f} {p2[0]:.1f},{p2[1]:.1f}" fill="{colour}"/>')
    out.append(f'<circle cx="{cx}" cy="{cy}" r="2.5" fill="#222"/>')


def result_lines(out, cx, y, rec, nq):
    _, _, m00, m10, m01, q, s = rec
    ex = exact_img_angle(m10, m01)
    lines = [f"m00 {m00}", f"m10 {m10}", f"m01 {m01}", f"quadrant {q}  sector {s}",
             f"angle {img_angle(q, s, nq):.2f} deg"]
    for i, l in enumerate(lines):
        out.append(txt(cx, y + 15 * i, l))
    out.append(txt(cx, y + 15 * len(lines), "exact " + ("n/a" if ex is None else f"{ex:.2f} deg"),
                   fill="#777"))


def cmd_plot():
    patches = load_patches()
    names = load_names(len(patches))
    n_sect, rounding, frac = table_cfg()
    nq = n_sect // 4
    model = OrientationModule(n_sect, rounding, frac)
    rtl = load_rtl()

    CW, CH, GAP = 570, 292, 14
    COLS = 2
    rows = (len(patches) + COLS - 1) // COLS
    W = COLS * CW + (COLS + 1) * GAP
    TOP = 92
    H = TOP + rows * (CH + GAP) + GAP
    out = [f'<?xml version="1.0" encoding="UTF-8"?>',
           f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
           f'<rect width="{W}" height="{H}" fill="#fff"/>']

    ok_count = 0
    summary = []
    cards = []
    for i, (img, name) in enumerate(zip(patches, names)):
        e = model.process_image(img)[0]
        exp = (e.x, e.y, e.m00, e.m10, e.m01, e.quadrant, e.sector)
        act = rtl[i] if i < len(rtl) else None
        fields = ("x", "y", "m00", "m10", "m01", "quadrant", "sector")
        diff = [f for f, a, b in zip(fields, exp, act) if a != b] if act else ["no output"]
        ok = not diff
        ok_count += ok
        summary.append((name, exp, act, ok))

        col, row = i % COLS, i // COLS
        x0 = GAP + col * (CW + GAP)
        y0 = TOP + row * (CH + GAP)
        card = [f'<rect x="{x0}" y="{y0}" width="{CW}" height="{CH}" rx="8" fill="#f7f7f5" stroke="#ccc"/>',
                txt(x0 + 12, y0 + 22, f"{i + 1}. {name}", 14, "start", mono=False, weight="bold"),
                txt(x0 + CW - 12, y0 + 22, "MATCH" if ok else "MISMATCH: " + ", ".join(diff), 13, "end",
                    "#1a9a5b" if ok else "#d33", mono=False, weight="bold")]
        yt = y0 + 44
        card.append(txt(x0 + 12 + 111, yt - 6, "input 37x37", 11, mono=False, fill="#555"))
        draw_patch(card, img, x0 + 12, yt)
        d1x, d2x = x0 + 322, x0 + 482
        card.append(txt(d1x, yt - 6, "expected (Python)", 11, mono=False, fill="#555"))
        card.append(txt(d2x, yt - 6, "actual (Verilog)", 11, mono=False, fill="#555"))
        dcy = yt + 66
        draw_dial(card, d1x, dcy, 58, nq, exp[5], exp[6], exact_img_angle(exp[3], exp[4]), "#2a6fdb")
        result_lines(card, d1x, yt + 146, exp, nq)
        if act:
            draw_dial(card, d2x, dcy, 58, nq, act[5], act[6], exact_img_angle(act[3], act[4]),
                      "#1a9a5b" if ok else "#d33")
            result_lines(card, d2x, yt + 146, act, nq)
        else:
            card.append(txt(d2x, dcy, "no RTL output", 12, fill="#d33"))
        cards.append(card)

    out.append(txt(GAP, 30, "ORB orientation module: input patch vs expected vs Verilog", 20, "start",
                   mono=False, weight="bold"))
    out.append(txt(GAP, 52, f"37x37 window -> intensity-centroid angle, N_SECT={n_sect}, rounding={rounding}. "
                            f"Angles in the image frame: 0 deg = right, 90 deg = down.", 12, "start",
                   mono=False, fill="#444"))
    out.append(txt(GAP, 70, "Solid arrow = quantised sector, dashed line = exact atan2 of the moments, "
                            "grey ticks = sector centres, red box = window centre pixel.", 12, "start",
                   mono=False, fill="#444"))
    out.append(txt(GAP, 88, f"Result: {ok_count}/{len(patches)} patches identical "
                            f"(m00, m10, m01, quadrant, sector).", 13, "start", mono=False,
                   fill="#1a9a5b" if ok_count == len(patches) else "#d33", weight="bold"))
    for c in cards:
        out.extend(c)
    out.append("</svg>")

    svg = "\n".join(out)
    with open(P("patch_visual.svg"), "w", encoding="utf-8") as f:
        f.write(svg)
    with open(P("patch_visual.html"), "w", encoding="utf-8") as f:
        f.write('<!DOCTYPE html><html><head><meta charset="utf-8"><title>Orientation patch test</title>'
                '</head><body style="margin:0;background:#fff">' + svg.split("?>", 1)[1] + "</body></html>")

    print(f"{'patch':<42}{'expected (q,s | m10 m01)':<34}{'verilog':<34}")
    for name, exp, act, ok in summary:
        fmt = lambda r: "none" if r is None else f"q{r[5]} s{r[6]:<2} | {r[3]} {r[4]}"
        print(f"{name:<42}{fmt(exp):<34}{fmt(act):<34}{'MATCH' if ok else 'MISMATCH'}")
    print(f"\n{ok_count}/{len(patches)} patches identical -> patch_visual.html, patch_visual.svg")
    return ok_count == len(patches)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    gap = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 0
    if mode == "gen":
        cmd_gen()
    elif mode == "sim":
        cmd_sim(gap)
    elif mode == "plot":
        sys.exit(0 if cmd_plot() else 1)
    elif mode == "all":
        cmd_gen()
        cmd_sim(gap)
        sys.exit(0 if cmd_plot() else 1)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
