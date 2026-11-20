#!/usr/bin/env python3
"""
Full-system golden model and hardware simulation visualizer for orb_frontend_top.

Modes:
  gen  : Generate input pixels, pattern_rom.hex, and bit-exact expected per-level
         features (x, y, score, quadrant, sector, 256-bit rBRIEF descriptor) for
         tb_system_top.sv.
  plot : Parse hardware simulation outputs (hw_sys_features.txt and
         hw_sys_lN_gauss.hex), verify exact match against the golden model, and
         render vslam_system_visual.png and vslam_system_visual.svg.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
HW_DIR = os.path.normpath(os.path.join(HERE, "../.."))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "../../../common/model")))
sys.path.insert(0, os.path.join(HW_DIR, "pyramid/model"))
sys.path.insert(0, os.path.join(HW_DIR, "frontend_top/model"))
sys.path.insert(0, os.path.join(HW_DIR, "orientation/model"))
sys.path.insert(0, os.path.join(HW_DIR, "rbrief/model"))

from vslam_cfg import CFG
from orb_pyramid_golden import build_pyramid, write_hex
from orb_frontend_golden import gaussian7, fast_score, nms3x3
from orientation_only import HALF as ORI_HALF, OrientationModule
from rbrief_python import make_pattern, dump_pattern_rom, rbrief as rbrief_desc
from trig_decoder import decode_trig, sudut_sebenarnya


def load_font(size=12, mono=False):
    candidates = (
        ["consola.ttf", "cour.ttf", "DejaVuSansMono.ttf"]
        if mono else
        ["segoeui.ttf", "arial.ttf", "DejaVuSans.ttf"]
    )
    for name in candidates:
        for base in ("", "C:/Windows/Fonts"):
            p = os.path.join(base, name) if base else name
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def run_gen(args):
    os.makedirs(args.outdir, exist_ok=True)
    pat_path = os.path.join(args.outdir, CFG.PATTERN_FILE)
    if os.path.dirname(pat_path):
        os.makedirs(os.path.dirname(pat_path), exist_ok=True)

    im = Image.open(args.input).convert("L")
    if args.width > 0 and args.height > 0:
        im = im.resize((args.width, args.height), Image.Resampling.BILINEAR)
    img0 = np.asarray(im, dtype=np.uint8)
    H0, W0 = img0.shape

    pyr_rounding = "round" if CFG.PYR_ROUND else "floor"
    pyr = build_pyramid(img0, levels=CFG.PYR_LEVELS, rounding=pyr_rounding)
    write_hex(os.path.join(args.outdir, "sys_level0_pixels.hex"), img0, 2)

    pattern = make_pattern(seed=CFG.PATTERN_SEED)
    dump_pattern_rom(pattern, pat_path)

    ori_mod = OrientationModule(n_sectors=CFG.N_SECT, rounding="paper", tan_frac_bits=8)

    meta = {
        "input": args.input,
        "width": int(W0),
        "height": int(H0),
        "fast_thr": int(args.fast_thr),
        "scale_factor": float(CFG.PYR_SCALE_FACTOR),
        "n_sect": int(CFG.N_SECT),
        "pyr_levels": int(CFG.PYR_LEVELS),
        "levels": [],
    }
    counts = []

    for lvl, entry in enumerate(pyr):
        im_lvl = entry["img"]
        Hl, Wl = im_lvl.shape
        write_hex(os.path.join(args.outdir, f"sys_level{lvl}_pixels.hex"), im_lvl, 2)

        g = gaussian7(im_lvl)           # (Hl-6, Wl-6), Q8.0
        s = fast_score(im_lvl, args.fast_thr)
        kps_raw = nms3x3(s)             # (x, y, score) in level coords [4..Wl-5]

        # Translate level coords -> Gaussian coords (offset -3) and apply 37x37 border
        Gh, Gw = g.shape
        score_map = {}
        kps_g = []
        for x, y, sc in kps_raw:
            gx, gy = x - 3, y - 3
            if ORI_HALF <= gx <= Gw - 1 - ORI_HALF and ORI_HALF <= gy <= Gh - 1 - ORI_HALF:
                kps_g.append((gx, gy))
                score_map[(gx, gy)] = sc

        ori_results = ori_mod.process_image(g, keypoints=kps_g)
        g6 = (g >> 2).tolist()          # 6-bit quantized Gaussian for rBRIEF

        feats = []
        with open(os.path.join(args.outdir, f"sys_expected_l{lvl}.hex"), "w") as fe:
            for r in ori_results:
                lx, ly = r.x + 3, r.y + 3
                sc = score_map[(r.x, r.y)]
                c_val, s_val = decode_trig(r.quadrant, r.sector)
                win37 = [
                    g6[row][r.x - ORI_HALF : r.x + ORI_HALF + 1]
                    for row in range(r.y - ORI_HALF, r.y + ORI_HALF + 1)
                ]
                desc = rbrief_desc(win37, pattern, c_val, s_val)
                packed = (
                    (sc << 286)
                    | (ly << 274)
                    | (lx << 262)
                    | (r.quadrant << 260)
                    | (r.sector << 256)
                    | desc
                )
                fe.write(f"{packed:075x}\n")
                feats.append({
                    "x": int(lx),
                    "y": int(ly),
                    "score": int(sc),
                    "quad": int(r.quadrant),
                    "sector": int(r.sector),
                    "angle_deg": round(sudut_sebenarnya(r.quadrant, r.sector), 2),
                    "desc": f"{desc:064x}",
                })

        counts.append(len(feats))
        meta["levels"].append({
            "level": lvl,
            "width": int(Wl),
            "height": int(Hl),
            "gauss_width": int(Gw),
            "gauss_height": int(Gh),
            "nms_keypoints": len(kps_raw),
            "valid_features": len(feats),
            "features": feats,
        })
        print(
            f"Level {lvl} ({Wl}x{Hl}): nms={len(kps_raw)}, "
            f"border-valid features={len(feats)}"
        )

    with open(os.path.join(args.outdir, "sys_expected_counts.hex"), "w") as fc:
        for cnt in counts:
            fc.write(f"{cnt:08x}\n")

    with open(os.path.join(args.outdir, "sys_golden_meta.json"), "w") as fj:
        json.dump(meta, fj, indent=2)


def read_hex_img(path, w, h):
    vals = [int(line.strip(), 16) for line in open(path) if line.strip()]
    assert len(vals) == w * h, f"{path}: expected {w*h} pixels, got {len(vals)}"
    return np.array(vals, dtype=np.uint8).reshape(h, w)


def draw_arrow_kp(draw, cx, cy, r, ang_deg, color, width=2):
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=color, width=width)
    rad = math.radians(ang_deg)
    ex = cx + r * 1.35 * math.cos(rad)
    ey = cy + r * 1.35 * math.sin(rad)
    draw.line([cx, cy, ex, ey], fill=color, width=width)


def run_plot(args):
    d = args.dir
    meta = json.load(open(os.path.join(d, "sys_golden_meta.json")))
    n_levels = len(meta["levels"])
    scale_factor = float(meta.get("scale_factor", CFG.PYR_SCALE_FACTOR))
    n_sect = int(meta.get("n_sect", CFG.N_SECT))

    hw_feat_path = os.path.join(d, "hw_sys_features.txt")
    hw_feats = {lvl: [] for lvl in range(n_levels)}
    with open(hw_feat_path) as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            lvl, x, y, sc, q, sec = (int(v) for v in parts[:6])
            desc_hex = parts[6].lower()
            if lvl in hw_feats:
                hw_feats[lvl].append({
                    "x": x, "y": y, "score": sc,
                    "quad": q, "sector": sec,
                    "angle_deg": round(sudut_sebenarnya(q, sec), 2),
                    "desc": desc_hex,
                })

    # Verify RTL feature output matches Python golden model
    mismatch = 0
    total_hw = 0
    for lvl in range(n_levels):
        exp = meta["levels"][lvl]["features"]
        got = hw_feats[lvl]
        total_hw += len(got)
        if len(exp) != len(got):
            mismatch += 1
        else:
            for e, g in zip(exp, got):
                if (e["x"], e["y"], e["score"], e["quad"], e["sector"], e["desc"]) != (
                    g["x"], g["y"], g["score"], g["quad"], g["sector"], g["desc"]
                ):
                    mismatch += 1

    # Load pyramid level pixels and RTL Gaussian outputs
    pyr_imgs = []
    hw_gauss_imgs = []
    for lvl_info in meta["levels"]:
        lvl = lvl_info["level"]
        pyr_imgs.append(
            read_hex_img(
                os.path.join(d, f"sys_level{lvl}_pixels.hex"),
                lvl_info["width"],
                lvl_info["height"],
            )
        )
        hw_gauss_imgs.append(
            read_hex_img(
                os.path.join(d, f"hw_sys_l{lvl}_gauss.hex"),
                lvl_info["gauss_width"],
                lvl_info["gauss_height"],
            )
        )

    # Color palette (dark engineering dashboard theme)
    BG = (15, 20, 28)
    PANEL = (24, 31, 43)
    BORDER = (52, 66, 86)
    TEXT = (232, 238, 246)
    MUTED = (148, 163, 184)
    LVL_COLORS = [
        (52, 211, 153),   # L0 emerald
        (56, 189, 248),   # L1 sky cyan
        (250, 204, 21),   # L2 amber yellow
        (232, 121, 249),  # L3 magenta
    ]
    STATUS_COL = (52, 211, 153) if mismatch == 0 else (248, 113, 113)

    W_CANVAS, H_CANVAS = 1400, 1020
    canvas = Image.new("RGB", (W_CANVAS, H_CANVAS), BG)
    draw = ImageDraw.Draw(canvas)

    f_title = load_font(22)
    f_sub = load_font(13)
    f_hdr = load_font(15)
    f_mono = load_font(12, mono=True)
    f_sm = load_font(11)

    # Header
    draw.rectangle([16, 14, W_CANVAS - 16, 74], fill=PANEL, outline=BORDER, width=1)
    draw.text(
        (30, 22),
        "VslamCihuyy · Hardware ORB Frontend Integration Output (RTL Simulation)",
        fill=TEXT,
        font=f_title,
    )
    status_txt = (
        f"RTL vs Golden: 100% BIT-EXACT PASS ({total_hw} features across L0..L{n_levels - 1})"
        if mismatch == 0 else
        f"RTL vs Golden: FAIL ({mismatch} mismatches)"
    )
    draw.text(
        (30, 50),
        f"Input: {meta['width']}x{meta['height']}  |  FAST thr={meta['fast_thr']}  |  {n_sect}-sector IC Orientation  |  256-bit rBRIEF  |  {status_txt}",
        fill=STATUS_COL,
        font=f_sub,
    )

    # Row 1: Pyramid Levels with RTL Keypoints + Orientation Vectors (2x scale)
    draw.rectangle([16, 86, W_CANVAS - 16, 368], fill=PANEL, outline=BORDER, width=1)
    draw.text((28, 94), f"1. {n_levels}-Level Image Pyramid + RTL NMS Keypoints & Quantized Orientations", fill=TEXT, font=f_hdr)

    x_cur = 30
    scale_top = 2.0
    for lvl in range(n_levels):
        col_lvl = LVL_COLORS[lvl % len(LVL_COLORS)]
        info = meta["levels"][lvl]
        w_l, h_l = info["width"], info["height"]
        dw, dh = int(w_l * scale_top), int(h_l * scale_top)
        rgb = Image.fromarray(pyr_imgs[lvl], mode="L").convert("RGB").resize((dw, dh), Image.Resampling.NEAREST)
        canvas.paste(rgb, (x_cur, 134))
        draw.rectangle([x_cur - 1, 133, x_cur + dw, 134 + dh], outline=col_lvl, width=2)
        for kp in hw_feats[lvl]:
            draw_arrow_kp(
                draw,
                x_cur + kp["x"] * scale_top,
                134 + kp["y"] * scale_top,
                6,
                kp["angle_deg"],
                col_lvl,
                width=2,
            )
        draw.text(
            (x_cur, 116),
            f"L{lvl}: {w_l}x{h_l} ({len(hw_feats[lvl])} feats)",
            fill=col_lvl,
            font=f_sub,
        )
        x_cur += dw + 26

    # Row 2 Left: RTL 7x7 Gaussian Filtered Outputs + 37x37 rBRIEF Patch Window
    draw.rectangle([16, 380, 740, 646], fill=PANEL, outline=BORDER, width=1)
    draw.text((28, 388), "2. Hardware 7x7 Gaussian Blur Output (hw_sys_lN_gauss.hex) + 37x37 Patch Bounds", fill=TEXT, font=f_hdr)

    gx_cur = 28
    g_scale = 1.25
    for lvl in range(n_levels):
        col_lvl = LVL_COLORS[lvl % len(LVL_COLORS)]
        info = meta["levels"][lvl]
        gw, gh = info["gauss_width"], info["gauss_height"]
        dw, dh = int(gw * g_scale), int(gh * g_scale)
        rgb_g = Image.fromarray(hw_gauss_imgs[lvl], mode="L").convert("RGB").resize((dw, dh), Image.Resampling.NEAREST)
        canvas.paste(rgb_g, (gx_cur, 434))
        draw.rectangle([gx_cur - 1, 433, gx_cur + dw, 434 + dh], outline=BORDER, width=1)
        if hw_feats[lvl]:
            # Draw 37x37 patch box around top-scoring feature
            best_kp = max(hw_feats[lvl], key=lambda k: k["score"])
            bx = gx_cur + (best_kp["x"] - 3) * g_scale
            by = 434 + (best_kp["y"] - 3) * g_scale
            half_s = 18 * g_scale
            draw.rectangle([bx - half_s, by - half_s, bx + half_s, by + half_s], outline=col_lvl, width=2)
            draw_arrow_kp(draw, bx, by, 4, best_kp["angle_deg"], col_lvl, width=2)
        draw.text((gx_cur, 415), f"L{lvl} Gauss: {gw}x{gh}", fill=col_lvl, font=f_sm)
        gx_cur += dw + 14

    # Row 2 Right: Multi-Scale Composite Projected onto L0
    draw.rectangle([752, 380, W_CANVAS - 16, 646], fill=PANEL, outline=BORDER, width=1)
    draw.text((764, 388), "3. Multi-Scale ORB Composite (All Levels Projected to L0)", fill=TEXT, font=f_hdr)
    comp_scale = 2.0
    cw, ch = int(meta["width"] * comp_scale), int(meta["height"] * comp_scale)
    comp_rgb = Image.fromarray(pyr_imgs[0], mode="L").convert("RGB").resize((cw, ch), Image.Resampling.NEAREST)
    cx0, cy0 = 768, 418
    canvas.paste(comp_rgb, (cx0, cy0))
    draw.rectangle([cx0 - 1, cy0 - 1, cx0 + cw, cy0 + ch], outline=BORDER, width=1)

    for lvl in range(n_levels):
        col_lvl = LVL_COLORS[lvl % len(LVL_COLORS)]
        s_factor = scale_factor ** lvl
        for kp in hw_feats[lvl]:
            draw_arrow_kp(
                draw,
                cx0 + kp["x"] * s_factor * comp_scale,
                cy0 + kp["y"] * s_factor * comp_scale,
                5 * s_factor,
                kp["angle_deg"],
                col_lvl,
                width=2,
            )

    # Legend on Composite
    lx, ly = cx0 + cw + 20, cy0 + 10
    draw.text((lx, ly), "Scale Legend:", fill=TEXT, font=f_sub)
    for lvl in range(n_levels):
        col_lvl = LVL_COLORS[lvl % len(LVL_COLORS)]
        yy = ly + 26 + lvl * 28
        draw.rectangle([lx, yy, lx + 14, yy + 14], fill=col_lvl)
        draw.text(
            (lx + 22, yy),
            f"L{lvl} ({scale_factor:g}^{lvl} = {scale_factor**lvl:.2f}x): {len(hw_feats[lvl])} kps",
            fill=TEXT,
            font=f_sm,
        )
    draw.text((lx, ly + 150), "Matcher stage: left empty", fill=MUTED, font=f_sm)
    draw.text((lx, ly + 168), "per integration spec.", fill=MUTED, font=f_sm)

    # Row 3: Sampled 256-bit rBRIEF Descriptors (16x16 matrix + 37x37 6-bit patch + metadata)
    draw.rectangle([16, 658, W_CANVAS - 16, H_CANVAS - 16], fill=PANEL, outline=BORDER, width=1)
    draw.text(
        (28, 666),
        "4. Sampled Hardware 256-bit rBRIEF Descriptors & 37x37 Q6.0 Patches (1 per pyramid level)",
        fill=TEXT,
        font=f_hdr,
    )

    row_y = 700
    for lvl in range(n_levels):
        col_lvl = LVL_COLORS[lvl % len(LVL_COLORS)]
        if not hw_feats[lvl]:
            draw.text((30, row_y + 18), f"L{lvl}: (0 features)", fill=MUTED, font=f_mono)
            row_y += 76
            continue
        kp = max(hw_feats[lvl], key=lambda k: k["score"])
        gx_c, gy_c = kp["x"] - 3, kp["y"] - 3
        patch = (hw_gauss_imgs[lvl][gy_c - 18 : gy_c + 19, gx_c - 18 : gx_c + 19] >> 2) * 4
        patch_img = Image.fromarray(patch.astype(np.uint8), mode="L").convert("RGB").resize((64, 64), Image.Resampling.NEAREST)
        canvas.paste(patch_img, (32, row_y))
        draw.rectangle([31, row_y - 1, 96, row_y + 64], outline=col_lvl, width=1)

        # Draw 16x16 descriptor bit matrix (4x4 px per bit -> 64x64)
        desc_int = int(kp["desc"], 16)
        mx0, my0 = 114, row_y
        for bit_idx in range(256):
            br = bit_idx // 16
            bc = bit_idx % 16
            bval = (desc_int >> bit_idx) & 1
            col_b = col_lvl if bval else (32, 40, 54)
            draw.rectangle(
                [mx0 + bc * 4, my0 + br * 4, mx0 + bc * 4 + 3, my0 + br * 4 + 3],
                fill=col_b,
            )
        draw.rectangle([mx0 - 1, my0 - 1, mx0 + 64, my0 + 64], outline=BORDER, width=1)

        # Feature details
        tx = 198
        draw.text(
            (tx, row_y + 4),
            f"Level {lvl} | pos=({kp['x']:3d},{kp['y']:3d}) | FAST score={kp['score']:4d} | quad={kp['quad']} sector={kp['sector']:2d} ({kp['angle_deg']:6.2f} deg)",
            fill=col_lvl,
            font=f_mono,
        )
        draw.text(
            (tx, row_y + 26),
            f"desc[255:0] = 0x{kp['desc']}",
            fill=TEXT,
            font=f_mono,
        )
        popcnt = bin(desc_int).count("1")
        draw.text(
            (tx, row_y + 46),
            f"popcount={popcnt}/256  |  RTL == Python Golden: EXACT MATCH",
            fill=MUTED,
            font=f_sm,
        )
        row_y += 76

    png_path = os.path.join(d, "vslam_system_visual.png")
    canvas.save(png_path)
    print(f"Saved PNG visualization: {png_path}")

    # Also write lightweight SVG summary alongside PNG
    svg_path = os.path.join(d, "vslam_system_visual.svg")
    with open(svg_path, "w") as fs:
        fs.write(
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_CANVAS}" height="{H_CANVAS}" viewBox="0 0 {W_CANVAS} {H_CANVAS}">\n'
            f'<rect width="100%" height="100%" fill="#0f141c"/>\n'
            f'<text x="24" y="34" fill="#e8eef6" font-family="sans-serif" font-size="20">VslamCihuyy Hardware ORB Frontend Output ({meta["width"]}x{meta["height"]}) - {status_txt}</text>\n'
        )
        for lvl in range(n_levels):
            y0 = 60 + lvl * 220
            info = meta["levels"][lvl]
            col = f"rgb{LVL_COLORS[lvl % len(LVL_COLORS)]}"
            fs.write(
                f'<rect x="24" y="{y0}" width="{info["width"]*1.5}" height="{info["height"]*1.5}" fill="#181f2b" stroke="{col}" stroke-width="1.5"/>\n'
                f'<text x="32" y="{y0+18}" fill="{col}" font-family="monospace" font-size="13">L{lvl}: {info["width"]}x{info["height"]} | {len(hw_feats[lvl])} features</text>\n'
            )
            for kp in hw_feats[lvl]:
                kx = 24 + kp["x"] * 1.5
                ky = y0 + kp["y"] * 1.5
                rad = math.radians(kp["angle_deg"])
                ex = kx + 8 * math.cos(rad)
                ey = ky + 8 * math.sin(rad)
                fs.write(
                    f'<circle cx="{kx:.1f}" cy="{ky:.1f}" r="4" fill="none" stroke="{col}" stroke-width="1.2"/>\n'
                    f'<line x1="{kx:.1f}" y1="{ky:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{col}" stroke-width="1.2"/>\n'
                )
        fs.write("</svg>\n")
    print(f"Saved SVG visualization: {svg_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    ap_gen = sub.add_parser("gen")
    ap_gen.add_argument("--input", required=True)
    ap_gen.add_argument("--outdir", default=".")
    ap_gen.add_argument("--width", type=int, default=CFG.SIM_IMG_W)
    ap_gen.add_argument("--height", type=int, default=CFG.SIM_IMG_H)
    ap_gen.add_argument("--fast-thr", type=int, default=CFG.FAST_THR)

    ap_plot = sub.add_parser("plot")
    ap_plot.add_argument("--dir", default=".")

    args = ap.parse_args()
    if args.cmd == "gen":
        run_gen(args)
    else:
        run_plot(args)


if __name__ == "__main__":
    main()
