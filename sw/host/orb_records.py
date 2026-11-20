#!/usr/bin/env python3
"""Binary codec for PL->PS Buffer-A records (fig2) and PS->PL Buffer-B matcher input (fig1)."""

from dataclasses import dataclass
import math
import os
import struct
import sys
from typing import Dict, List, Tuple

import numpy as np

_COMMON_MODEL = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../common/model"))
if _COMMON_MODEL not in sys.path:
    sys.path.insert(0, _COMMON_MODEL)

from vslam_cfg import CFG

HEADER_MAGIC = CFG.MAGIC_ORBK
FMT_VERSION = CFG.FMT_VER
HEADER_SIZE = CFG.HEADER_BYTES
RECORD_SIZE = CFG.RECORD_BYTES
MAX_RECORDS = 1 << CFG.REC_AW
HEADER_STRUCT = struct.Struct("<8Q")
RECORD_META_STRUCT = struct.Struct("<Q")


def angle_u16_from_quad_sec(quad: int, sector: int, n_sect: int = CFG.N_SECT) -> int:
    """Convert (quadrant, sector) from orient_top to 16-bit binary angle (65536 = 360 deg)."""
    sec_bits = int(round(math.log2(max(4, n_sect) // 4)))
    sec_mask = (1 << sec_bits) - 1
    shift = 14 - sec_bits
    half = (1 << (shift - 1)) if shift > 0 else 0
    q = quad & 0x3
    s = sector & sec_mask
    if q == 1:
        bin_idx = (0 << sec_bits) | s
    elif q == 0:
        bin_idx = (1 << sec_bits) | ((~s) & sec_mask)
    elif q == 3:
        bin_idx = (2 << sec_bits) | s
    else:
        bin_idx = (3 << sec_bits) | ((~s) & sec_mask)
    return (bin_idx << shift) | half


def angle_deg_from_u16(angle_u16: int) -> float:
    return (float(angle_u16) * 360.0) / 65536.0


@dataclass
class FrameHeader:
    version: int
    frame_id: int
    n_records: int
    timestamp_ns: int
    width: int
    height: int
    n_dropped: int
    status: int
    level_counts: Tuple[int, ...]


@dataclass
class FramePacket:
    header: FrameHeader
    keypoints: List[Dict]
    descriptors: np.ndarray  # shape (N, 32), dtype uint8


def parse_frame_bin(path: str) -> FramePacket:
    """Parse a Buffer-A binary file written by orb_record_writer.sv."""
    with open(path, "rb") as f:
        data = f.read()

    if len(data) < HEADER_SIZE:
        raise ValueError(f"{path}: file smaller than {HEADER_SIZE}-byte header")

    h = HEADER_STRUCT.unpack_from(data, 0)
    magic = h[0] & 0xFFFFFFFF
    version = (h[0] >> 32) & 0xFFFF
    if magic != HEADER_MAGIC:
        raise ValueError(f"{path}: bad magic 0x{magic:08x}, expected 0x{HEADER_MAGIC:08x}")

    frame_id = h[1] & 0xFFFFFFFF
    n_records = (h[1] >> 32) & 0xFFFFFFFF
    timestamp_ns = h[2]
    width = h[3] & 0xFFFF
    height = (h[3] >> 16) & 0xFFFF
    n_dropped = (h[3] >> 32) & 0xFFFF
    status = (h[3] >> 48) & 0xFFFF
    level_counts = (
        h[4] & 0xFFFF,
        (h[4] >> 16) & 0xFFFF,
        (h[4] >> 32) & 0xFFFF,
        (h[4] >> 48) & 0xFFFF,
        h[5] & 0xFFFF,
        (h[5] >> 16) & 0xFFFF,
        (h[5] >> 32) & 0xFFFF,
        (h[5] >> 48) & 0xFFFF,
    )

    expected_bytes = HEADER_SIZE + n_records * RECORD_SIZE
    if len(data) != expected_bytes:
        raise ValueError(f"{path}: size {len(data)} != expected {expected_bytes}")

    header = FrameHeader(
        version=version,
        frame_id=frame_id,
        n_records=n_records,
        timestamp_ns=timestamp_ns,
        width=width,
        height=height,
        n_dropped=n_dropped,
        status=status,
        level_counts=level_counts,
    )

    keypoints: List[Dict] = []
    descs = np.zeros((n_records, 32), dtype=np.uint8)
    for idx in range(n_records):
        off = HEADER_SIZE + idx * RECORD_SIZE
        descs[idx] = np.frombuffer(data, dtype=np.uint8, count=32, offset=off)
        (w4,) = RECORD_META_STRUCT.unpack_from(data, off + 32)
        x = w4 & 0xFFF
        y = (w4 >> 12) & 0xFFF
        lvl = (w4 >> 24) & 0x7
        valid = bool((w4 >> 27) & 0x1)
        score = (w4 >> 28) & 0xFFFF
        angle_u16 = (w4 >> 44) & 0xFFFF
        keypoints.append({
            "x": float(x),
            "y": float(y),
            "level": int(lvl),
            "valid": valid,
            "score": int(score),
            "angle_u16": int(angle_u16),
            "angle_deg": angle_deg_from_u16(angle_u16),
            "desc_hex": int.from_bytes(descs[idx].tobytes(), "little").to_bytes(32, "big").hex(),
        })

    return FramePacket(header=header, keypoints=keypoints, descriptors=descs)


def write_frame_bin(
    path: str,
    width: int,
    height: int,
    keypoints: List[Dict],
    descriptors: np.ndarray,
    frame_id: int = 0,
    timestamp_ns: int = 0,
) -> None:
    """Serialize keypoints and 256-bit descriptors into the PL Buffer-A binary format."""
    n_records = len(keypoints)
    lvl_counts = [0] * 8
    for kp in keypoints:
        lvl_counts[int(kp["level"]) & 0x7] += 1

    h0 = HEADER_MAGIC | (FMT_VERSION << 32)
    h1 = (frame_id & 0xFFFFFFFF) | ((n_records & 0xFFFFFFFF) << 32)
    h2 = timestamp_ns & 0xFFFFFFFFFFFFFFFF
    h3 = (width & 0xFFFF) | ((height & 0xFFFF) << 16)
    h4 = (
        (lvl_counts[0] & 0xFFFF)
        | ((lvl_counts[1] & 0xFFFF) << 16)
        | ((lvl_counts[2] & 0xFFFF) << 32)
        | ((lvl_counts[3] & 0xFFFF) << 48)
    )
    h5 = (
        (lvl_counts[4] & 0xFFFF)
        | ((lvl_counts[5] & 0xFFFF) << 16)
        | ((lvl_counts[6] & 0xFFFF) << 32)
        | ((lvl_counts[7] & 0xFFFF) << 48)
    )

    chunks = [HEADER_STRUCT.pack(h0, h1, h2, h3, h4, h5, 0, 0)]
    for i, kp in enumerate(keypoints):
        if "angle_u16" in kp:
            ang_u16 = int(kp["angle_u16"]) & 0xFFFF
        else:
            ang_u16 = angle_u16_from_quad_sec(int(kp["quad"]), int(kp["sector"]))
        w4 = (
            (int(round(kp["x"])) & 0xFFF)
            | ((int(round(kp["y"])) & 0xFFF) << 12)
            | ((int(kp["level"]) & 0x3) << 24)
            | (1 << 27)
            | ((int(kp["score"]) & 0xFFF) << 28)
            | (ang_u16 << 44)
        )
        chunks.append(np.ascontiguousarray(descriptors[i], dtype=np.uint8).tobytes())
        chunks.append(RECORD_META_STRUCT.pack(w4))

    with open(path, "wb") as f:
        f.write(b"".join(chunks))


def pack_matcher_buffer_b(
    keypoints_l0: List[Dict],
    descriptors: np.ndarray,
    width: int,
    height: int,
    ratio: float = CFG.MATCH_RATIO,
    grid_w: int = CFG.BUF_B_GRID_W,
    grid_h: int = CFG.BUF_B_GRID_H,
    coord_int_bits: int = CFG.BUF_B_COORD_INT_BITS,
    coord_frac_bits: int = CFG.BUF_B_COORD_FRAC_BITS,
) -> Dict:
    """Pack train features sorted by grid_w x grid_h cell with Q(m.n) coords and ratio_lim[257]."""
    coord_bits = coord_int_bits + coord_frac_bits
    coord_scale = float(1 << coord_frac_bits)
    coord_mask = (1 << coord_bits) - 1
    cx_bits = max(1, (grid_w - 1).bit_length())
    cy_bits = max(1, (grid_h - 1).bit_length())
    total_bits = 2 * coord_bits + 16 + cx_bits + cy_bits
    if total_bits > 64:
        raise ValueError(
            f"Buffer-B metadata requires {total_bits} bits (> 64): "
            f"2*Q{coord_int_bits}.{coord_frac_bits} + angle16 + {grid_w}x{grid_h} cell"
        )
    cx_mask = (1 << cx_bits) - 1
    cy_mask = (1 << cy_bits) - 1

    indexed = []
    for idx, kp in enumerate(keypoints_l0):
        cx = min(grid_w - 1, max(0, int(kp["x_l0"] * grid_w / max(1, width))))
        cy = min(grid_h - 1, max(0, int(kp["y_l0"] * grid_h / max(1, height))))
        indexed.append((cx, cy, idx, kp))
    indexed.sort(key=lambda item: (item[0], item[1], item[2]))

    buf = bytearray()
    sorted_indices = []
    for cx, cy, orig_idx, kp in indexed:
        xq = int(round(kp["x_l0"] * coord_scale)) & coord_mask
        yq = int(round(kp["y_l0"] * coord_scale)) & coord_mask
        ang = int(kp["angle_u16"]) & 0xFFFF
        meta = (
            xq
            | (yq << coord_bits)
            | (ang << (2 * coord_bits))
            | ((cx & cx_mask) << (2 * coord_bits + 16))
            | ((cy & cy_mask) << (2 * coord_bits + 16 + cx_bits))
        )
        buf.extend(np.ascontiguousarray(descriptors[orig_idx], dtype=np.uint8).tobytes())
        buf.extend(RECORD_META_STRUCT.pack(meta))
        sorted_indices.append(orig_idx)

    ratio_lim = np.array([min(256, int(math.floor(d * ratio))) for d in range(257)], dtype=np.uint16)
    return {
        "train_bytes": bytes(buf),
        "sorted_indices": sorted_indices,
        "ratio_lim": ratio_lim,
        "grid_dims": (grid_w, grid_h),
    }
