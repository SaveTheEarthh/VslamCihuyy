#!/usr/bin/env python3
"""PS-side ORB backend: quadtree culling, L0 scaling, undistortion, matching, pose + triangulation."""

import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import least_squares

_COMMON_MODEL = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../common/model"))
if _COMMON_MODEL not in sys.path:
    sys.path.insert(0, _COMMON_MODEL)

from vslam_cfg import CFG
from orb_records import FramePacket, pack_matcher_buffer_b, parse_frame_bin

_POPCOUNT_LUT = np.array([bin(i).count("1") for i in range(256)], dtype=np.uint8)


def distribute_oct_tree(
    keypoints: List[Dict],
    descriptors: np.ndarray,
    width: int,
    height: int,
    max_features: int,
) -> Tuple[List[Dict], np.ndarray]:
    """ORB-SLAM3 quadtree spatial distribution keeping highest-scoring corner per leaf."""
    if max_features <= 0 or len(keypoints) <= max_features:
        return list(keypoints), np.array(descriptors, copy=True)

    n_ini = max(1, round(width / max(1, height)))
    h_step = width / n_ini
    nodes = []
    for i in range(n_ini):
        x0, x1 = i * h_step, (i + 1) * h_step
        idxs = [k for k, kp in enumerate(keypoints) if x0 <= kp["x"] < x1 or (i == n_ini - 1 and kp["x"] <= x1)]
        if idxs:
            nodes.append({"x0": x0, "x1": x1, "y0": 0.0, "y1": float(height), "idxs": idxs})

    while len(nodes) < max_features:
        splittable = [n for n in nodes if len(n["idxs"]) > 1]
        if not splittable:
            break
        # Split nodes with most keypoints first (matches ORB-SLAM3 priority expansion)
        splittable.sort(key=lambda n: len(n["idxs"]), reverse=True)
        new_nodes = []
        split_set = set()
        budget = max_features - len(nodes)
        for n in splittable:
            if budget <= 0:
                break
            xm = 0.5 * (n["x0"] + n["x1"])
            ym = 0.5 * (n["y0"] + n["y1"])
            quads = [[], [], [], []]
            for idx in n["idxs"]:
                kp = keypoints[idx]
                q = (0 if kp["x"] < xm else 1) + (0 if kp["y"] < ym else 2)
                quads[q].append(idx)
            bounds = [
                (n["x0"], xm, n["y0"], ym),
                (xm, n["x1"], n["y0"], ym),
                (n["x0"], xm, ym, n["y1"]),
                (xm, n["x1"], ym, n["y1"]),
            ]
            children = [
                {"x0": bx0, "x1": bx1, "y0": by0, "y1": by1, "idxs": q_idxs}
                for (bx0, bx1, by0, by1), q_idxs in zip(bounds, quads)
                if q_idxs
            ]
            if len(children) > 1:
                split_set.add(id(n))
                new_nodes.extend(children)
                budget -= (len(children) - 1)
        if not split_set:
            break
        nodes = [n for n in nodes if id(n) not in split_set] + new_nodes

    kept_idx = []
    for n in nodes:
        best = max(n["idxs"], key=lambda idx: keypoints[idx]["score"])
        kept_idx.append(best)
    kept_idx.sort()
    if len(kept_idx) > max_features:
        kept_idx.sort(key=lambda idx: keypoints[idx]["score"], reverse=True)
        kept_idx = sorted(kept_idx[:max_features])

    return [dict(keypoints[i]) for i in kept_idx], descriptors[kept_idx]


def scale_to_level0(keypoints: List[Dict], scale_factor: float = CFG.PYR_SCALE_FACTOR) -> List[Dict]:
    """Scale each keypoint coordinate from its pyramid level back to Level 0."""
    out = []
    for kp in keypoints:
        s = scale_factor ** int(kp["level"])
        item = dict(kp)
        item["scale"] = s
        item["x_l0"] = float(kp["x"]) * s
        item["y_l0"] = float(kp["y"]) * s
        out.append(item)
    return out


def undistort_points(
    pts: np.ndarray,
    K: np.ndarray,
    dist_coeffs: Optional[Tuple[float, ...]] = None,
    iters: int = CFG.UNDISTORT_ITERS,
) -> np.ndarray:
    """Iterative Brown-Conrady undistortion matching OpenCV/ORB-SLAM3."""
    if len(pts) == 0 or dist_coeffs is None:
        return np.asarray(pts, dtype=np.float64).copy()

    dc = list(dist_coeffs) + [0.0] * max(0, 5 - len(dist_coeffs))
    k1, k2, p1, p2, k3 = dc[:5]
    if k1 == 0.0 and k2 == 0.0 and p1 == 0.0 and p2 == 0.0 and k3 == 0.0:
        return np.asarray(pts, dtype=np.float64).copy()

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    xd = (pts[:, 0] - cx) / fx
    yd = (pts[:, 1] - cy) / fy
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / rad
        y = (yd - dy) / rad
    return np.column_stack([x * fx + cx, y * fy + cy])


def match_descriptors(
    kps0: List[Dict],
    descs0: np.ndarray,
    kps1: List[Dict],
    descs1: np.ndarray,
    max_dist: int = CFG.MATCH_MAX_DIST,
    ratio: float = CFG.MATCH_RATIO,
    check_orientation: bool = True,
    hist_bins: int = CFG.ORIENT_HIST_BINS,
) -> Dict:
    """Hamming descriptor matcher with Lowe ratio test, 1-to-1 check, and orientation histogram filter."""
    n0, n1 = len(kps0), len(kps1)
    if n0 == 0 or n1 < 2:
        return {"matches": [], "hist": np.zeros(hist_bins, dtype=np.int32)}

    dists = _POPCOUNT_LUT[descs0[:, None, :] ^ descs1[None, :, :]].sum(axis=2, dtype=np.int32)
    order = np.argpartition(dists, 1, axis=1)[:, :2]
    best_j = order[:, 0]
    sec_j = order[:, 1]
    best_d = dists[np.arange(n0), best_j]
    sec_d = dists[np.arange(n0), sec_j]
    swap = sec_d < best_d
    best_j = np.where(swap, sec_j, best_j)
    d1 = np.where(swap, sec_d, best_d)
    d2 = np.where(swap, best_d, sec_d)

    # 1-to-1 uniqueness on train keypoint j
    best_for_j: Dict[int, Tuple[int, int]] = {}
    for i in range(n0):
        if d1[i] <= max_dist and d1[i] <= ratio * d2[i]:
            j = int(best_j[i])
            if j not in best_for_j or d1[i] < best_for_j[j][1]:
                best_for_j[j] = (i, int(d1[i]))

    candidates = [(i, j, dist) for j, (i, dist) in best_for_j.items()]
    candidates.sort(key=lambda m: m[0])

    hist = np.zeros(hist_bins, dtype=np.int32)
    if check_orientation and candidates:
        bins = [[] for _ in range(hist_bins)]
        factor = float(hist_bins) / 360.0
        for idx_m, (i, j, _) in enumerate(candidates):
            rot = (kps0[i]["angle_deg"] - kps1[j]["angle_deg"]) % 360.0
            b = min(hist_bins - 1, int(rot * factor))
            bins[b].append(idx_m)
            hist[b] += 1
        top3 = np.argsort(hist)[-min(3, hist_bins):]
        m_top = hist[top3[-1]]
        kept = []
        for b in top3:
            if b == top3[-1] or (m_top > 0 and hist[b] >= 0.1 * m_top):
                kept.extend(bins[b])
        candidates = [candidates[idx_m] for idx_m in sorted(kept)]

    return {"matches": candidates, "hist": hist}


def _normalize_2d(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    mean = pts.mean(axis=0)
    shifted = pts - mean
    rms = math.sqrt(float((shifted ** 2).sum(axis=1).mean()) + 1e-12)
    scale = math.sqrt(2.0) / rms
    T = np.array([[scale, 0.0, -scale * mean[0]], [0.0, scale, -scale * mean[1]], [0.0, 0.0, 1.0]])
    pts_h = np.column_stack([pts, np.ones(len(pts))])
    return (T @ pts_h.T).T, T


def _eight_point_essential(x0_norm: np.ndarray, x1_norm: np.ndarray) -> np.ndarray:
    p0, T0 = _normalize_2d(x0_norm)
    p1, T1 = _normalize_2d(x1_norm)
    A = np.column_stack([
        p1[:, 0] * p0[:, 0], p1[:, 0] * p0[:, 1], p1[:, 0],
        p1[:, 1] * p0[:, 0], p1[:, 1] * p0[:, 1], p1[:, 1],
        p0[:, 0], p0[:, 1], np.ones(len(p0)),
    ])
    _, _, vt = np.linalg.svd(A)
    E = vt[-1].reshape(3, 3)
    U, S, Vt = np.linalg.svd(E)
    if np.linalg.det(U) < 0:
        U[:, -1] *= -1
    if np.linalg.det(Vt) < 0:
        Vt[-1, :] *= -1
    s = 0.5 * (S[0] + S[1])
    return T1.T @ (U @ np.diag([s, s, 0.0]) @ Vt) @ T0


def _enforce_essential(E_raw: np.ndarray) -> np.ndarray:
    U, S, Vt = np.linalg.svd(E_raw)
    if np.linalg.det(U) < 0:
        U[:, -1] *= -1
    if np.linalg.det(Vt) < 0:
        Vt[-1, :] *= -1
    s = 0.5 * (S[0] + S[1])
    return U @ np.diag([s, s, 0.0]) @ Vt


def _triangulate_dlt(P0: np.ndarray, P1: np.ndarray, x0: np.ndarray, x1: np.ndarray) -> np.ndarray:
    pts3d = np.zeros((len(x0), 3), dtype=np.float64)
    for i in range(len(x0)):
        A = np.vstack([
            x0[i, 0] * P0[2] - P0[0],
            x0[i, 1] * P0[2] - P0[1],
            x1[i, 0] * P1[2] - P1[0],
            x1[i, 1] * P1[2] - P1[1],
        ])
        _, _, vt = np.linalg.svd(A)
        X = vt[-1]
        pts3d[i] = X[:3] / (X[3] if abs(X[3]) > 1e-12 else 1e-12)
    return pts3d


def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def _rot_to_rvec(R: np.ndarray) -> np.ndarray:
    tr = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    theta = math.acos(float(tr))
    if theta < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (theta / (2.0 * math.sin(theta)))


def estimate_pose_and_triangulate(
    pts0: np.ndarray,
    pts1: np.ndarray,
    K: np.ndarray,
    ransac_iters: int = CFG.RANSAC_ITERS,
    thresh_px: float = CFG.RANSAC_THRESH_PX,
    refine: bool = True,
    seed: int = CFG.PATTERN_SEED,
) -> Dict:
    """Recover relative camera pose (R, t) and triangulate 3D map points from pixel matches."""
    if len(pts0) < 8:
        raise ValueError(f"Need >= 8 matches for 8-point pose estimation, got {len(pts0)}")

    K_inv = np.linalg.inv(K)
    pts0_h = np.column_stack([pts0, np.ones(len(pts0))])
    pts1_h = np.column_stack([pts1, np.ones(len(pts1))])
    x0 = (K_inv @ pts0_h.T).T[:, :2]
    x1 = (K_inv @ pts1_h.T).T[:, :2]

    rng = np.random.default_rng(seed)
    best_inliers = None
    best_score = (0, -1e18)
    best_E = None
    thresh_sq = thresh_px * thresh_px

    for _ in range(ransac_iters):
        idx = rng.choice(len(pts0), size=8, replace=False)
        E_cand = _enforce_essential(_eight_point_essential(x0[idx], x1[idx]))
        F_cand = K_inv.T @ E_cand @ K_inv

        Fp0 = (F_cand @ pts0_h.T)
        Ftp1 = (F_cand.T @ pts1_h.T)
        num = np.sum(pts1_h.T * Fp0, axis=0)
        denom = Fp0[0] ** 2 + Fp0[1] ** 2 + Ftp1[0] ** 2 + Ftp1[1] ** 2 + 1e-12
        err_sq = (num * num) / denom

        inliers = err_sq < thresh_sq
        cnt = int(inliers.sum())
        score = (cnt, -float(err_sq[inliers].sum()) if cnt > 0 else -1e18)
        if score > best_score:
            best_score = score
            best_inliers = inliers
            best_E = E_cand
            if cnt > 0.95 * len(pts0):
                break

    if best_inliers is None or int(best_inliers.sum()) < 8:
        raise RuntimeError("RANSAC failed to find >= 8 inliers")

    # Refit E on all inliers
    best_E = _enforce_essential(_eight_point_essential(x0[best_inliers], x1[best_inliers]))
    U, _, Vt = np.linalg.svd(best_E)
    if np.linalg.det(U) < 0:
        U[:, -1] *= -1
    if np.linalg.det(Vt) < 0:
        Vt[-1, :] *= -1
    W = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    t_vec = U[:, 2]
    candidates = [
        (U @ W @ Vt, t_vec),
        (U @ W @ Vt, -t_vec),
        (U @ W.T @ Vt, t_vec),
        (U @ W.T @ Vt, -t_vec),
    ]

    P0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    x0_in = x0[best_inliers]
    x1_in = x1[best_inliers]

    R_best, t_best, X_best = None, None, None
    best_cheirality = -1
    for R_c, t_c in candidates:
        if np.linalg.det(R_c) < 0:
            R_c = -R_c
        P1 = np.hstack([R_c, t_c.reshape(3, 1)])
        X_c = _triangulate_dlt(P0, P1, x0_in, x1_in)
        z0 = X_c[:, 2]
        z1 = (R_c @ X_c.T + t_c.reshape(3, 1))[2]
        good = int(((z0 > 0) & (z1 > 0)).sum())
        if good > best_cheirality:
            best_cheirality = good
            R_best, t_best, X_best = R_c, t_c / np.linalg.norm(t_c), X_c

    if refine and len(X_best) >= 6:
        rvec0 = _rot_to_rvec(R_best)
        az0 = math.atan2(t_best[1], t_best[0])
        el0 = math.asin(float(np.clip(t_best[2], -1.0, 1.0)))
        params0 = np.concatenate([rvec0, [az0, el0], X_best.ravel()])
        p0_in = pts0[best_inliers]
        p1_in = pts1[best_inliers]

        def residuals(p):
            R_opt = _rodrigues(p[:3])
            az, el = p[3], p[4]
            t_opt = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
            X_opt = p[5:].reshape(-1, 3)

            proj0 = (K @ X_opt.T).T
            uv0 = proj0[:, :2] / np.where(np.abs(proj0[:, 2:3]) > 1e-6, proj0[:, 2:3], 1e-6)

            X1 = (R_opt @ X_opt.T + t_opt.reshape(3, 1)).T
            proj1 = (K @ X1.T).T
            uv1 = proj1[:, :2] / np.where(np.abs(proj1[:, 2:3]) > 1e-6, proj1[:, 2:3], 1e-6)
            return np.concatenate([(uv0 - p0_in).ravel(), (uv1 - p1_in).ravel()])

        sol = least_squares(residuals, params0, loss="soft_l1", f_scale=1.0, max_nfev=80)
        R_best = _rodrigues(sol.x[:3])
        az, el = sol.x[3], sol.x[4]
        t_best = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        X_best = sol.x[5:].reshape(-1, 3)

    proj0 = (K @ X_best.T).T
    uv0 = proj0[:, :2] / proj0[:, 2:3]
    X1 = (R_best @ X_best.T + t_best.reshape(3, 1)).T
    proj1 = (K @ X1.T).T
    uv1 = proj1[:, :2] / proj1[:, 2:3]
    err0 = np.linalg.norm(uv0 - pts0[best_inliers], axis=1)
    err1 = np.linalg.norm(uv1 - pts1[best_inliers], axis=1)
    reproj_err = 0.5 * (err0 + err1)

    return {
        "R": R_best,
        "t": t_best,
        "pts3d": X_best,
        "inliers": best_inliers,
        "reproj_err": reproj_err,
        "median_reproj_px": float(np.median(reproj_err)),
        "mean_reproj_px": float(np.mean(reproj_err)),
    }


def _prep_frame(
    pkt: FramePacket,
    K: np.ndarray,
    dist_coeffs: Optional[Tuple[float, ...]],
    max_features: int,
    scale_factor: float = CFG.PYR_SCALE_FACTOR,
) -> Tuple[List[Dict], np.ndarray]:
    kps, descs = distribute_oct_tree(
        pkt.keypoints, pkt.descriptors, pkt.header.width, pkt.header.height, max_features
    )
    kps = scale_to_level0(kps, scale_factor=scale_factor)
    pts = np.array([[kp["x_l0"], kp["y_l0"]] for kp in kps], dtype=np.float64)
    undist = undistort_points(pts, K, dist_coeffs)
    for kp, uv in zip(kps, undist):
        kp["x_u"] = float(uv[0])
        kp["y_u"] = float(uv[1])
    return kps, descs


def run_backend_pair(
    bin0_path: str,
    bin1_path: str,
    K: np.ndarray,
    dist_coeffs: Optional[Tuple[float, ...]] = None,
    max_features: int = CFG.BACKEND_MAX_FEATURES,
    max_dist: int = CFG.MATCH_MAX_DIST,
    ratio: float = CFG.MATCH_RATIO,
    scale_factor: float = CFG.PYR_SCALE_FACTOR,
    ransac_iters: int = CFG.RANSAC_ITERS,
    ransac_thresh_px: float = CFG.RANSAC_THRESH_PX,
) -> Dict:
    """High-level PS backend entry point consuming two PL Buffer-A .bin files."""
    pkt0 = parse_frame_bin(bin0_path)
    pkt1 = parse_frame_bin(bin1_path)

    kps0, descs0 = _prep_frame(pkt0, K, dist_coeffs, max_features, scale_factor=scale_factor)
    kps1, descs1 = _prep_frame(pkt1, K, dist_coeffs, max_features, scale_factor=scale_factor)

    buf_b = pack_matcher_buffer_b(kps1, descs1, pkt1.header.width, pkt1.header.height, ratio=ratio)
    match_res = match_descriptors(
        kps0,
        descs0,
        kps1,
        descs1,
        max_dist=max_dist,
        ratio=ratio,
        check_orientation=True,
        hist_bins=CFG.ORIENT_HIST_BINS,
    )
    matches = match_res["matches"]

    pts0 = np.array([[kps0[i]["x_u"], kps0[i]["y_u"]] for i, _, _ in matches], dtype=np.float64)
    pts1 = np.array([[kps1[j]["x_u"], kps1[j]["y_u"]] for _, j, _ in matches], dtype=np.float64)

    pose = estimate_pose_and_triangulate(
        pts0,
        pts1,
        K,
        ransac_iters=ransac_iters,
        thresh_px=ransac_thresh_px,
    )
    return {
        "pkt0": pkt0,
        "pkt1": pkt1,
        "kps0": kps0,
        "kps1": kps1,
        "descs0": descs0,
        "descs1": descs1,
        "buffer_b": buf_b,
        "matches": matches,
        "hist": match_res["hist"],
        "pose": pose,
    }
