#!/usr/bin/env python3
"""End-to-end verification and visual plot for the PS backend (sw/host/)."""

import inspect
import math
import os
import re
import sys
import tempfile
from typing import Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "../.."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "common/model"))
sys.path.insert(0, os.path.join(ROOT, "hw/pyramid/model"))
sys.path.insert(0, os.path.join(ROOT, "hw/frontend_top/model"))
sys.path.insert(0, os.path.join(ROOT, "hw/orientation/model"))
sys.path.insert(0, os.path.join(ROOT, "hw/rbrief/model"))

from vslam_cfg import CFG, DEFAULT_CFG_PATH, load_cfg
import orb_records
import orb_backend
from orb_records import parse_frame_bin, write_frame_bin, pack_matcher_buffer_b, angle_u16_from_quad_sec
from orb_backend import (
    distribute_oct_tree,
    scale_to_level0,
    undistort_points,
    match_descriptors,
    run_backend_pair,
)
from orb_pyramid_golden import build_pyramid
from orb_frontend_golden import gaussian7, fast_score, nms3x3
from orientation_only import HALF as ORI_HALF, OrientationModule
from rbrief_python import make_pattern, rbrief as rbrief_desc
from trig_decoder import decode_trig


def extract_golden_to_bin(img_u8: np.ndarray, out_bin: str, fast_thr: int = CFG.FAST_THR, frame_id: int = 1) -> None:
    """Run the hardware-faithful golden frontend on an image array and emit a Buffer-A .bin."""
    H0, W0 = img_u8.shape
    pyr_rounding = "round" if CFG.PYR_ROUND else "floor"
    pyr = build_pyramid(img_u8, levels=CFG.PYR_LEVELS, rounding=pyr_rounding)
    ori_mod = OrientationModule(n_sectors=CFG.N_SECT, rounding="paper", tan_frac_bits=8)
    pattern = make_pattern(seed=CFG.PATTERN_SEED)

    kps = []
    descs = []
    for lvl, entry in enumerate(pyr):
        im_lvl = entry["img"]
        g = gaussian7(im_lvl)
        s = fast_score(im_lvl, fast_thr)
        kps_raw = nms3x3(s)
        Gh, Gw = g.shape
        score_map = {}
        kps_g = []
        for x, y, sc in kps_raw:
            gx, gy = x - 3, y - 3
            if ORI_HALF <= gx <= Gw - 1 - ORI_HALF and ORI_HALF <= gy <= Gh - 1 - ORI_HALF:
                kps_g.append((gx, gy))
                score_map[(gx, gy)] = sc
        ori_results = ori_mod.process_image(g, keypoints=kps_g)
        g6 = (g >> 2).tolist()
        for r in ori_results:
            c_val, s_val = decode_trig(r.quadrant, r.sector)
            win37 = [g6[row][r.x - ORI_HALF : r.x + ORI_HALF + 1] for row in range(r.y - ORI_HALF, r.y + ORI_HALF + 1)]
            d_int = rbrief_desc(win37, pattern, c_val, s_val)
            kps.append({
                "x": r.x + 3,
                "y": r.y + 3,
                "level": lvl,
                "score": score_map[(r.x, r.y)],
                "quad": r.quadrant,
                "sector": r.sector,
                "angle_u16": angle_u16_from_quad_sec(r.quadrant, r.sector),
            })
            descs.append(np.frombuffer(int(d_int).to_bytes(32, "little"), dtype=np.uint8))

    desc_arr = np.vstack(descs) if descs else np.zeros((0, 32), dtype=np.uint8)
    write_frame_bin(out_bin, W0, H0, kps, desc_arr, frame_id=frame_id, timestamp_ns=frame_id * 33_333_333)


def test_a_rtl_roundtrip(rtl_bin_path: str, rtl_txt_path: str) -> None:
    pkt = parse_frame_bin(rtl_bin_path)
    assert pkt.header.width == CFG.SIM_IMG_W and pkt.header.height == CFG.SIM_IMG_H
    assert pkt.header.n_dropped == 0

    txt_rows = []
    with open(rtl_txt_path) as f:
        for line in f:
            p = line.strip().split()
            if p:
                txt_rows.append((int(p[0]), int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5]), p[6].lower()))

    assert len(txt_rows) > 0 and pkt.header.n_records == len(txt_rows)
    exp_lvl_cnt = [0] * CFG.PYR_LEVELS
    for row in txt_rows:
        exp_lvl_cnt[row[0]] += 1
    assert pkt.header.level_counts[:CFG.PYR_LEVELS] == tuple(exp_lvl_cnt)

    for i, ((lvl, x, y, sc, q, sec, dhex), kp) in enumerate(zip(txt_rows, pkt.keypoints)):
        assert kp["valid"] is True
        assert kp["level"] == lvl
        assert int(kp["x"]) == x and int(kp["y"]) == y
        assert kp["score"] == sc
        assert kp["angle_u16"] == angle_u16_from_quad_sec(q, sec, n_sect=CFG.N_SECT)
        assert kp["desc_hex"] == dhex, f"Record {i} desc mismatch"
    print(f"[PASS] Test A: RTL frame0_rtl.bin ({pkt.header.n_records} records) bit-exact vs hw_sys_features.txt")


def test_b_prep_and_buffer_b(rtl_bin_path: str) -> None:
    pkt = parse_frame_bin(rtl_bin_path)
    kps, descs = distribute_oct_tree(pkt.keypoints, pkt.descriptors, pkt.header.width, pkt.header.height, 180)
    assert len(kps) <= 180 and len(kps) >= 120
    kps = scale_to_level0(kps, CFG.PYR_SCALE_FACTOR)
    assert all("x_l0" in kp and "y_l0" in kp for kp in kps)

    K = np.array([[200.0, 0.0, 72.0], [0.0, 200.0, 54.0], [0.0, 0.0, 1.0]])
    dc = (-0.18, 0.05, 0.002, -0.001, 0.0)
    rng = np.random.default_rng(7)
    pts_clean = np.column_stack([rng.uniform(10, 134, 40), rng.uniform(10, 98, 40)])
    x_n = (pts_clean[:, 0] - K[0, 2]) / K[0, 0]
    y_n = (pts_clean[:, 1] - K[1, 2]) / K[1, 1]
    r2 = x_n ** 2 + y_n ** 2
    rad = 1.0 + r2 * (dc[0] + r2 * dc[1])
    xd = x_n * rad + 2 * dc[2] * x_n * y_n + dc[3] * (r2 + 2 * x_n ** 2)
    yd = y_n * rad + dc[2] * (r2 + 2 * y_n ** 2) + 2 * dc[3] * x_n * y_n
    pts_dist = np.column_stack([xd * K[0, 0] + K[0, 2], yd * K[1, 1] + K[1, 2]])
    pts_rec = undistort_points(pts_dist, K, dc, iters=12)
    max_err = float(np.max(np.linalg.norm(pts_rec - pts_clean, axis=1)))
    assert max_err < 1e-4, f"Undistortion error {max_err}"

    buf_b = pack_matcher_buffer_b(kps, descs, pkt.header.width, pkt.header.height)
    assert len(buf_b["train_bytes"]) == len(kps) * CFG.RECORD_BYTES
    assert len(buf_b["ratio_lim"]) == 257 and buf_b["ratio_lim"][100] == int(math.floor(100 * CFG.MATCH_RATIO))
    assert buf_b["grid_dims"] == (CFG.BUF_B_GRID_W, CFG.BUF_B_GRID_H)
    print(f"[PASS] Test B: OctTree ({len(kps)} kps), undistort max err={max_err:.2e} px, Buffer B ({len(buf_b['train_bytes'])} B)")


def _sample_texture(u: np.ndarray, v: np.ndarray, seed_off: float = 0.0) -> np.ndarray:
    """Non-repeating multi-scale analytic texture so every 37x37 patch has a unique descriptor."""
    h, w = u.shape
    val = np.full((h, w), 120.0, dtype=np.float64)
    val += 28.0 * np.sin(u * 3.7 + seed_off) * np.cos(v * 2.9 - seed_off)
    val += 22.0 * np.sin((u * 1.3 + v * 2.1) * 5.3 + seed_off * 2.0)
    rng = np.random.default_rng(int(round(seed_off * 100)) + 17)
    for _ in range(95):
        cu = rng.uniform(-1.8, 1.8)
        cv = rng.uniform(-1.5, 1.5)
        ru = rng.uniform(0.03, 0.14)
        rv = rng.uniform(0.03, 0.14)
        amp = rng.choice([-90.0, -65.0, 65.0, 90.0])
        mask = (np.abs(u - cu) < ru) & (np.abs(v - cv) < rv)
        val = np.where(mask, val + amp, val)
    return np.clip(val, 8, 248).astype(np.uint8)


def _render_synthetic_3d_view(
    W: int, H: int, K: np.ndarray, R_cw: np.ndarray, t_cw: np.ndarray
) -> np.ndarray:
    """Analytic raycast of multi-depth 3D planes (back wall + two foreground panels) from camera (R_cw, t_cw)."""
    u_grid, v_grid = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    K_inv = np.linalg.inv(K)
    rays_c = np.stack([
        (u_grid - K[0, 2]) / K[0, 0],
        (v_grid - K[1, 2]) / K[1, 1],
        np.ones_like(u_grid),
    ], axis=-1)
    R_wc = R_cw.T
    C_w = -R_wc @ t_cw
    d_w = rays_c @ R_wc.T  # shape (H, W, 3)

    best_t = np.full((H, W), 1e9, dtype=np.float64)
    img = np.zeros((H, W), dtype=np.uint8)

    # Planes: (point_on_plane, normal_w, u_axis, v_axis, xlim, ylim, seed_off)
    planes = [
        # Back wall at Z = 5.5
        (np.array([0.0, 0.0, 5.5]), np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), (-10.0, 10.0), (-10.0, 10.0), 0.3),
        # Left tilted panel Z ~ 2.9
        (np.array([-0.65, -0.05, 2.9]), np.array([0.15, -0.05, 0.987]), np.array([0.988, 0.0, -0.15]), np.array([0.0, 1.0, 0.05]), (-0.75, 0.65), (-0.7, 0.7), 1.7),
        # Right tilted panel Z ~ 3.5
        (np.array([0.65, 0.1, 3.5]), np.array([-0.18, 0.08, 0.98]), np.array([0.983, 0.0, 0.18]), np.array([0.0, 1.0, -0.08]), (-0.65, 0.75), (-0.7, 0.7), 3.1),
    ]

    for p0, n, ax_u, ax_v, xlim, ylim, soff in planes:
        denom = np.einsum("ijk,k->ij", d_w, n)
        valid = np.abs(denom) > 1e-6
        t_hit = np.where(valid, np.dot(p0 - C_w, n) / np.where(valid, denom, 1.0), 1e9)
        hit_w = C_w + t_hit[..., None] * d_w
        rel = hit_w - p0
        pu = np.einsum("ijk,k->ij", rel, ax_u)
        pv = np.einsum("ijk,k->ij", rel, ax_v)
        inside = (
            valid
            & (t_hit > 0.2)
            & (t_hit < best_t)
            & (pu >= xlim[0]) & (pu <= xlim[1])
            & (pv >= ylim[0]) & (pv <= ylim[1])
        )
        tex = _sample_texture(pu, pv, soff)
        best_t = np.where(inside, t_hit, best_t)
        img = np.where(inside, tex, img)

    return img


def test_c_real_image_matching(rtl_bin_path: str) -> Tuple[np.ndarray, np.ndarray, dict]:
    custom_img = Image.open(os.path.join(ROOT, "hw/frontend_top/sim/input_custom.jpg")).convert("L")
    img0 = np.asarray(custom_img.resize((CFG.SIM_IMG_W, CFG.SIM_IMG_H), Image.Resampling.BILINEAR), dtype=np.uint8)
    # Apply small rigid shift + rotation to create realistic frame 1
    img1_pil = Image.fromarray(img0, mode="L").rotate(-6.0, resample=Image.Resampling.BILINEAR, translate=(8, -3))
    img1 = np.asarray(img1_pil, dtype=np.uint8)

    bin1_path = os.path.join(HERE, "frame1_real_shift.bin")
    extract_golden_to_bin(img1, bin1_path, fast_thr=CFG.FAST_THR, frame_id=1)

    pkt0 = parse_frame_bin(rtl_bin_path)
    pkt1 = parse_frame_bin(bin1_path)
    kps0, descs0 = distribute_oct_tree(pkt0.keypoints, pkt0.descriptors, CFG.SIM_IMG_W, CFG.SIM_IMG_H, CFG.BACKEND_MAX_FEATURES)
    kps1, descs1 = distribute_oct_tree(pkt1.keypoints, pkt1.descriptors, CFG.SIM_IMG_W, CFG.SIM_IMG_H, CFG.BACKEND_MAX_FEATURES)
    kps0 = scale_to_level0(kps0, CFG.PYR_SCALE_FACTOR)
    kps1 = scale_to_level0(kps1, CFG.PYR_SCALE_FACTOR)

    mres = match_descriptors(
        kps0, descs0, kps1, descs1,
        max_dist=CFG.MATCH_MAX_DIST,
        ratio=CFG.MATCH_RATIO,
        check_orientation=True,
        hist_bins=CFG.ORIENT_HIST_BINS,
    )
    matches = mres["matches"]
    assert len(matches) >= 25, f"Expected >= 25 real matches, got {len(matches)}"
    ang_deltas = [((kps0[i]["angle_deg"] - kps1[j]["angle_deg"] + 180.0) % 360.0) - 180.0 for i, j, _ in matches]
    med_rot = float(np.median(ang_deltas))
    assert abs(med_rot - (-6.0)) < 8.0, f"Median angle delta {med_rot:.2f} deg far from -6 deg"
    print(f"[PASS] Test C: Real RTL frame0 vs rotated frame1 -> {len(matches)} matches, median angle delta={med_rot:.2f} deg")
    return img0, img1, {
        "kps0": kps0,
        "kps1": kps1,
        "matches": matches,
        "hist": mres["hist"],
        "pkt0_n": pkt0.header.n_records,
    }


def test_d_synthetic_3d_pose() -> dict:
    W, H = 240, 180
    K = np.array([[220.0, 0.0, W * 0.5], [0.0, 220.0, H * 0.5], [0.0, 0.0, 1.0]])

    yaw = math.radians(7.0)
    pitch = math.radians(1.5)
    Ry = np.array([[math.cos(yaw), 0.0, math.sin(yaw)], [0.0, 1.0, 0.0], [-math.sin(yaw), 0.0, math.cos(yaw)]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(pitch), -math.sin(pitch)], [0.0, math.sin(pitch), math.cos(pitch)]])
    R_gt = Rx @ Ry
    t_gt = np.array([-0.42, 0.04, 0.10])
    t_gt_unit = t_gt / np.linalg.norm(t_gt)

    im0 = _render_synthetic_3d_view(W, H, K, np.eye(3), np.zeros(3))
    im1 = _render_synthetic_3d_view(W, H, K, R_gt, t_gt)

    b0 = os.path.join(HERE, "syn3d_frame0.bin")
    b1 = os.path.join(HERE, "syn3d_frame1.bin")
    extract_golden_to_bin(im0, b0, fast_thr=CFG.FAST_THR, frame_id=10)
    extract_golden_to_bin(im1, b1, fast_thr=CFG.FAST_THR, frame_id=11)

    res = run_backend_pair(b0, b1, K, dist_coeffs=None)
    pose = res["pose"]
    R_est = pose["R"]
    t_est = pose["t"]

    tr = np.clip((np.trace(R_est @ R_gt.T) - 1.0) * 0.5, -1.0, 1.0)
    r_err_deg = math.degrees(math.acos(float(tr)))
    t_cos = np.clip(abs(float(np.dot(t_est, t_gt_unit))), -1.0, 1.0)
    # Also verify exact sign alignment (positive dot product)
    t_dot = float(np.dot(t_est, t_gt_unit))
    t_err_deg = math.degrees(math.acos(float(np.clip(t_dot, -1.0, 1.0))))
    inlier_cnt = int(pose["inliers"].sum())

    assert r_err_deg < 2.0, f"Rotation error {r_err_deg:.2f} deg >= 2.0 deg"
    assert t_err_deg < 3.0, f"Translation direction error {t_err_deg:.2f} deg >= 3.0 deg (dot={t_dot:.3f})"
    assert pose["median_reproj_px"] < 1.0, f"Median reproj error {pose['median_reproj_px']:.2f} px >= 1.0"
    assert np.all(pose["pts3d"][:, 2] > 0), "Negative Z depth found in triangulated points"

    print(
        f"[PASS] Test D: 3D Pose Recovery ({len(res['matches'])} matches, {inlier_cnt} RANSAC inliers) "
        f"R_err={r_err_deg:.2f} deg, t_err={t_err_deg:.2f} deg, med_reproj={pose['median_reproj_px']:.2f} px"
    )
    return {
        "res": res,
        "R_gt": R_gt,
        "t_gt_unit": t_gt_unit,
        "r_err_deg": r_err_deg,
        "t_err_deg": t_err_deg,
    }


def test_e_single_config_propagation(real_res: dict) -> None:
    """Verify vslam_cfg_pkg.sv drives backend defaults directly and edits propagate cleanly."""
    assert orb_records.HEADER_MAGIC == CFG.MAGIC_ORBK
    assert orb_records.FMT_VERSION == CFG.FMT_VER
    assert orb_records.MAX_RECORDS == (1 << CFG.REC_AW)

    s_params = inspect.signature(orb_backend.scale_to_level0).parameters
    assert s_params["scale_factor"].default == CFG.PYR_SCALE_FACTOR

    m_params = inspect.signature(orb_backend.match_descriptors).parameters
    assert m_params["max_dist"].default == CFG.MATCH_MAX_DIST
    assert m_params["ratio"].default == CFG.MATCH_RATIO
    assert m_params["hist_bins"].default == CFG.ORIENT_HIST_BINS

    r_params = inspect.signature(orb_backend.run_backend_pair).parameters
    assert r_params["max_features"].default == CFG.BACKEND_MAX_FEATURES
    assert r_params["max_dist"].default == CFG.MATCH_MAX_DIST
    assert r_params["ratio"].default == CFG.MATCH_RATIO
    assert r_params["scale_factor"].default == CFG.PYR_SCALE_FACTOR
    assert r_params["ransac_iters"].default == CFG.RANSAC_ITERS
    assert r_params["ransac_thresh_px"].default == CFG.RANSAC_THRESH_PX

    e_params = inspect.signature(orb_backend.estimate_pose_and_triangulate).parameters
    assert e_params["ransac_iters"].default == CFG.RANSAC_ITERS
    assert e_params["thresh_px"].default == CFG.RANSAC_THRESH_PX
    assert e_params["seed"].default == CFG.PATTERN_SEED

    with open(DEFAULT_CFG_PATH, "r", encoding="utf-8") as f:
        text = f.read()

    overrides = {
        "MAGIC_ORBK": "32'hCAFE_1234",
        "REC_AW": "11",
        "PYR_SCALE_FACTOR": "1.25",
        "MATCH_RATIO": "0.75",
        "BUF_B_GRID_W": "32",
        "BUF_B_GRID_H": "32",
        "BUF_B_COORD_INT_BITS": "11",
        "BUF_B_COORD_FRAC_BITS": "8",
        "ORIENT_HIST_BINS": "36",
        "FAST_THR": "25",
    }
    for name, val in overrides.items():
        text, n_sub = re.subn(
            rf"(localparam\s+(?:(?:int|real|string|logic\s*\[[^\]]+\])\s+)?{name}\s*=\s*)([^;]+)(;)",
            rf"\g<1>{val}\g<3>",
            text,
        )
        assert n_sub == 1, f"Expected 1 replacement for {name}, got {n_sub}"

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_sv = os.path.join(tmpdir, "vslam_cfg_pkg.sv")
        with open(tmp_sv, "w", encoding="utf-8") as f:
            f.write(text)
        cfg2 = load_cfg(tmp_sv)

    assert cfg2.MAGIC_ORBK == 0xCAFE1234 and isinstance(cfg2.MAGIC_ORBK, int)
    assert cfg2.REC_AW == 11 and isinstance(cfg2.REC_AW, int)
    assert cfg2.PYR_SCALE_FACTOR == 1.25 and isinstance(cfg2.PYR_SCALE_FACTOR, float)
    assert cfg2.MATCH_RATIO == 0.75 and isinstance(cfg2.MATCH_RATIO, float)
    assert cfg2.BUF_B_GRID_W == 32 and cfg2.BUF_B_GRID_H == 32
    assert cfg2.BUF_B_COORD_INT_BITS == 11 and cfg2.BUF_B_COORD_FRAC_BITS == 8
    assert cfg2.ORIENT_HIST_BINS == 36 and isinstance(cfg2.ORIENT_HIST_BINS, int)
    assert cfg2.FAST_THR == 25 and isinstance(cfg2.FAST_THR, int)

    # Q11.8 (38b) + angle16 (16b) + 32x32 grid (5b + 5b) = 64 bits exact
    dummy_descs = np.zeros((len(real_res["kps0"]), 32), dtype=np.uint8)
    buf_b2 = pack_matcher_buffer_b(
        real_res["kps0"],
        dummy_descs,
        CFG.SIM_IMG_W,
        CFG.SIM_IMG_H,
        ratio=cfg2.MATCH_RATIO,
        grid_w=cfg2.BUF_B_GRID_W,
        grid_h=cfg2.BUF_B_GRID_H,
        coord_int_bits=cfg2.BUF_B_COORD_INT_BITS,
        coord_frac_bits=cfg2.BUF_B_COORD_FRAC_BITS,
    )
    assert buf_b2["grid_dims"] == (32, 32)
    assert buf_b2["ratio_lim"][100] == 75

    m_custom = match_descriptors(
        real_res["kps0"], dummy_descs, real_res["kps1"], np.zeros((len(real_res["kps1"]), 32), dtype=np.uint8),
        hist_bins=cfg2.ORIENT_HIST_BINS,
    )
    assert len(m_custom["hist"]) == 36

    # Exceeding 64-bit Buffer-B metadata budget (Q11.8 + 64x64 cell = 66 bits) must raise ValueError
    raised = False
    try:
        pack_matcher_buffer_b(
            real_res["kps0"][:1], dummy_descs[:1], CFG.SIM_IMG_W, CFG.SIM_IMG_H,
            grid_w=64, grid_h=64, coord_int_bits=11, coord_frac_bits=8,
        )
    except ValueError:
        raised = True
    assert raised, "Expected ValueError when Buffer-B metadata exceeds 64 bits"
    print("[PASS] Test E: Single-file vslam_cfg_pkg.sv defaults, override round-trip, and Buffer-B bit-budget check")


def _load_font(size: int = 12, mono: bool = False):
    names = ["consola.ttf", "cour.ttf"] if mono else ["segoeui.ttf", "arial.ttf"]
    for n in names:
        for base in ("", "C:/Windows/Fonts"):
            p = os.path.join(base, n) if base else n
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def render_backend_visual(img0: np.ndarray, img1: np.ndarray, real_res: dict, syn_res: dict, out_path: str) -> None:
    BG = (15, 20, 28)
    PANEL = (24, 31, 43)
    BORDER = (52, 66, 86)
    TEXT = (232, 238, 246)
    MUTED = (148, 163, 184)
    GREEN = (52, 211, 153)
    CYAN = (56, 189, 248)
    AMBER = (250, 204, 21)
    RED = (248, 113, 113)

    W, H = 1280, 860
    canvas = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(canvas)
    f_title = _load_font(20)
    f_hdr = _load_font(15)
    f_sub = _load_font(13)
    f_mono = _load_font(12, mono=True)

    # Header
    draw.rectangle([16, 14, W - 16, 70], fill=PANEL, outline=BORDER)
    draw.text((28, 20), "VslamCihuyy · PS Backend Verification (Record Parser + Matcher + Pose & Triangulation)", fill=TEXT, font=f_title)
    n_bins = len(real_res["hist"])
    draw.text(
        (28, 46),
        f"RTL frame0_rtl.bin ({real_res['pkt0_n']} feats) -> {len(real_res['matches'])} Real Matches  |  "
        f"3D Pose: R_err={syn_res['r_err_deg']:.2f} deg, t_err={syn_res['t_err_deg']:.2f} deg, "
        f"reproj={syn_res['res']['pose']['median_reproj_px']:.2f} px  |  BACKEND_TEST: PASS",
        fill=GREEN,
        font=f_sub,
    )

    # Top panel: Real RTL Frame 0 vs Frame 1 matches (3x upscale)
    draw.rectangle([16, 84, W - 16, 462], fill=PANEL, outline=BORDER)
    draw.text((28, 92), f"1. RTL Buffer-A (frame0_rtl.bin) vs Rotated Frame 1: Quadtree + Hamming + {n_bins}-Bin Orientation Matches", fill=TEXT, font=f_hdr)

    sc = 3.0
    h0, w0 = img0.shape
    h1, w1 = img1.shape
    im0_rgb = Image.fromarray(img0, mode="L").convert("RGB").resize((int(w0 * sc), int(h0 * sc)), Image.Resampling.NEAREST)
    im1_rgb = Image.fromarray(img1, mode="L").convert("RGB").resize((int(w1 * sc), int(h1 * sc)), Image.Resampling.NEAREST)
    x0, y0 = 36, 122
    x1 = x0 + im0_rgb.width + 80
    canvas.paste(im0_rgb, (x0, y0))
    canvas.paste(im1_rgb, (x1, y0))
    draw.rectangle([x0 - 1, y0 - 1, x0 + im0_rgb.width, y0 + im0_rgb.height], outline=GREEN, width=2)
    draw.rectangle([x1 - 1, y0 - 1, x1 + im1_rgb.width, y0 + im1_rgb.height], outline=CYAN, width=2)

    for kp in real_res["kps0"]:
        px, py = x0 + kp["x_l0"] * sc, y0 + kp["y_l0"] * sc
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], outline=MUTED)
    for kp in real_res["kps1"]:
        px, py = x1 + kp["x_l0"] * sc, y0 + kp["y_l0"] * sc
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], outline=MUTED)

    for idx_m, (i, j, dist) in enumerate(real_res["matches"][:60]):
        p0x, p0y = x0 + real_res["kps0"][i]["x_l0"] * sc, y0 + real_res["kps0"][i]["y_l0"] * sc
        p1x, p1y = x1 + real_res["kps1"][j]["x_l0"] * sc, y0 + real_res["kps1"][j]["y_l0"] * sc
        col = GREEN if dist <= 25 else AMBER
        draw.line([p0x, p0y, p1x, p1y], fill=col, width=1)
        draw.ellipse([p0x - 3, p0y - 3, p0x + 3, p0y + 3], fill=col)
        draw.ellipse([p1x - 3, p1y - 3, p1x + 3, p1y + 3], fill=col)

    sx_txt = x1 + im1_rgb.width + 24
    draw.text((sx_txt, y0 + 10), "Real Image Pair Stats:", fill=TEXT, font=f_sub)
    draw.text((sx_txt, y0 + 36), f"Frame 0 (RTL): {real_res['pkt0_n']} -> {len(real_res['kps0'])} kps", fill=GREEN, font=f_mono)
    draw.text((sx_txt, y0 + 58), f"Frame 1: {len(real_res['kps1'])} kps", fill=CYAN, font=f_mono)
    draw.text((sx_txt, y0 + 80), f"Matched: {len(real_res['matches'])} pairs", fill=AMBER, font=f_mono)
    draw.text((sx_txt, y0 + 102), "Green line: dist <= 25", fill=GREEN, font=f_mono)
    draw.text((sx_txt, y0 + 122), f"Amber line: 25 < dist <= {CFG.MATCH_MAX_DIST}", fill=AMBER, font=f_mono)

    # Bottom-Left: orientation histogram
    deg_per_bin = 360.0 / max(1, n_bins)
    draw.rectangle([16, 474, 620, H - 16], fill=PANEL, outline=BORDER)
    draw.text((28, 484), f"2. Orientation Consistency Histogram ({n_bins} bins, {deg_per_bin:g} deg/bin)", fill=TEXT, font=f_hdr)
    hist = real_res["hist"]
    max_h = max(1, int(hist.max()))
    bx0, by0, bw, bh = 44, 790, 540, 240
    draw.line([bx0, by0, bx0 + bw, by0], fill=MUTED, width=1)
    draw.line([bx0, by0 - bh, bx0, by0], fill=MUTED, width=1)
    bar_w = max(2, bw // max(1, n_bins) - 3)
    top_bin = int(np.argmax(hist))
    for b in range(n_bins):
        h_px = int((hist[b] / max_h) * (bh - 16))
        rx = bx0 + 4 + b * (bw // max(1, n_bins))
        col = GREEN if b == top_bin else CYAN
        if h_px > 0:
            draw.rectangle([rx, by0 - h_px, rx + bar_w, by0], fill=col)
    draw.text((bx0, by0 + 8), f"0 deg          180 deg         360 deg | peak bin={top_bin} ({top_bin*deg_per_bin:g}-{(top_bin+1)*deg_per_bin:g} deg, n={hist[top_bin]})", fill=MUTED, font=f_mono)

    # Bottom-Right: Top-down 3D XZ map & camera poses
    draw.rectangle([634, 474, W - 16, H - 16], fill=PANEL, outline=BORDER)
    draw.text((646, 484), "3. Recovered 3D Triangulated Points & Camera Poses (Top-Down X-Z)", fill=TEXT, font=f_hdr)

    pose = syn_res["res"]["pose"]
    # Scale unit-baseline reconstruction to ground-truth baseline (0.433 m) for metric overlay
    baseline_gt = 0.43359
    pts3d = pose["pts3d"] * baseline_gt
    t_est_dir = pose["t"]
    R_est = pose["R"]
    C1_est = -R_est.T @ t_est_dir * baseline_gt
    C1_gt = -syn_res["R_gt"].T @ (syn_res["t_gt_unit"] * baseline_gt)

    mx0, my0, mw, mh = 666, 526, 560, 265
    draw.rectangle([mx0, my0, mx0 + mw, my0 + mh], fill=(16, 22, 32), outline=BORDER)

    # Map X in [-3.2, 3.2], Z in [-0.4, 6.4]
    def to_px(xw, zw):
        px = mx0 + int((xw + 3.2) / 6.4 * mw)
        py = my0 + mh - int((zw + 0.4) / 6.8 * mh)
        return px, py

    for p in pts3d:
        if -3.1 <= p[0] <= 3.1 and 0.2 <= p[2] <= 6.2:
            px, py = to_px(p[0], p[2])
            draw.ellipse([px - 3, py - 3, px + 3, py + 3], fill=AMBER)

    c0p = to_px(0.0, 0.0)
    c1ep = to_px(C1_est[0], C1_est[2])
    c1gp = to_px(C1_gt[0], C1_gt[2])
    draw.ellipse([c0p[0] - 5, c0p[1] - 5, c0p[0] + 5, c0p[1] + 5], fill=GREEN)
    draw.ellipse([c1gp[0] - 5, c1gp[1] - 5, c1gp[0] + 5, c1gp[1] + 5], outline=RED, width=2)
    draw.ellipse([c1ep[0] - 4, c1ep[1] - 4, c1ep[0] + 4, c1ep[1] + 4], fill=CYAN)

    draw.text(
        (mx0 + 6, my0 + mh + 10),
        f"Green=Cam0 | Red=Cam1 GT | Cyan=Cam1 Est (R_err={syn_res['r_err_deg']:.2f} deg, t_err={syn_res['t_err_deg']:.2f} deg) | Pts={len(pts3d)}",
        fill=TEXT,
        font=f_mono,
    )

    canvas.save(out_path)
    print(f"Saved backend visual: {out_path}")


def main() -> None:
    rtl_bin = os.path.join(HERE, "frame0_rtl.bin")
    rtl_txt = os.path.join(ROOT, "hw/frontend_top/sim/hw_sys_features.txt")

    test_a_rtl_roundtrip(rtl_bin, rtl_txt)
    test_b_prep_and_buffer_b(rtl_bin)
    img0, img1, real_res = test_c_real_image_matching(rtl_bin)
    syn_res = test_d_synthetic_3d_pose()
    test_e_single_config_propagation(real_res)

    vis_path = os.path.join(HERE, "backend_visual.png")
    render_backend_visual(img0, img1, real_res, syn_res, vis_path)
    print("BACKEND_TEST: PASS")


if __name__ == "__main__":
    main()
