# Integration Map

> **Last updated:** 2026-11-20 — Centralized all tunable frontend, record-writer, golden-model, and PS-backend parameters into single source `common/rtl/vslam_cfg_pkg.sv` + runtime Python parser `common/model/vslam_cfg.py`.

## 1. Repository layout

| Location | Status | Contents |
|---|---|---|
| `common/` | **Implemented** | `rtl/`: `vslam_cfg_pkg.sv` (single parameter source, compiled first); `model/`: `vslam_cfg.py` (runtime SV package parser) |
| `hw/pyramid/` | **Implemented** | `rtl/`: `orb_input_fifo.sv`, `orb_pyr_scaler.sv`, `orb_image_pyramid.sv`, `orb_pyramid_top.sv`; `tb/`: `tb_image_pyramid.sv`, `tb_orb_pyramid_top.sv`; `model/`: `orb_pyramid_golden.py`, `image_pyramid.py` |
| `hw/fast/` | **Implemented** | `rtl/`: `orb_window7.sv`, `orb_fast_core.sv`, `orb_nms3x3.sv` |
| `hw/gaussian/` | **Implemented** | `rtl/`: `orb_gauss7.sv` |
| `hw/orientation/` | **Implemented** | `rtl/`: 9 modules + `orient_tan_table.svh`; `tb/`: `tb_orient.sv`, `tb_patch.sv`; `model/`: `orientation_only.py` |
| `hw/rbrief/` | **Implemented** | `rtl/`: `trig_decoder.sv`, `rbrief.sv`, `rbrief_top.sv`; `tb/`: `tb_trig_decoder.sv`, `tb_rbrief.sv`, `tb_rbrief_top.sv`; `model/`: `rbrief_python.py`, `trig_decoder.py` |
| `hw/frontend_top/` | **Implemented** | `rtl/`: `orb_frontend_level.sv`, `orb_extractor_level.sv`, `orb_frontend_top.sv`, `orb_record_writer.sv`; `tb/`: `tb_frontend.sv`, `tb_system_top.sv`, `tb_record_writer.sv`; `model/`: `orb_frontend_golden.py`, `orb_system_golden.py` |
| `sw/host/` | **Implemented** | `orb_records.py` (Buffer-A/B binary codec), `orb_backend.py` (OctTree, scaling, undistort, matcher, 8-pt RANSAC pose + BA/triangulation), `test_backend.py` |
| `hw/matcher/` | **Stub** | `.gitkeep` only — hardware descriptor matcher left empty for later stage |
| `scripts/` | **Implemented** | `rtl.f` (compile order for `vslam_cfg_pkg` + 24 RTL `.sv` modules), `run_all.ps1` (vector gen + Icarus + PS backend regression) |
| `docs/notes/` | Docs | `pyramid_verilog_explanation.md`, `frontend_verilog_explanation.md`, `pyramid_python_explanation.md`, `image_pyramid_workflow_guide.md` |

---

## 2. Module catalog

### 2.0 `vslam_cfg_pkg` & `vslam_cfg.py` *(Single Parameter Source)*
- **Paths:** `common/rtl/vslam_cfg_pkg.sv` (SystemVerilog package, compiled first in `scripts/rtl.f` & `run_modelsim.do`), `common/model/vslam_cfg.py` (stdlib Python parser exporting `CFG` & `load_cfg()`).
- **Purpose:** Single editable file holding production (`IMG_W`, `IMG_H`, `PYR_*`, `FAST_THR`, `ENABLE_EXTRACTOR`, `N_SECT`, `*_FIFO_AW`, `PATTERN_*`), Buffer-A wire (`REC_AW`, `MAGIC_ORBK`, `FMT_VER`, `HEADER_BYTES`, `RECORD_BYTES`), Buffer-B/backend (`BUF_B_*`, `BACKEND_MAX_FEATURES`, `MATCH_*`, `ORIENT_HIST_BINS`, `RANSAC_*`, `UNDISTORT_ITERS`), and fast-sim (`SIM_*`) parameters. Consumed directly by `orb_extractor_level`, `orb_frontend_top`, `orb_record_writer`, `tb_system_top`, `tb_record_writer`, `orb_system_golden.py`, and `sw/host/` (`orb_records.py`, `orb_backend.py`, `test_backend.py`).

### 2.1 `orb_input_fifo`
- **Path:** `hw/pyramid/rtl/orb_input_fifo.sv`
- **Purpose:** AXI-Stream slave FIFO + little-endian byte serializer converting `S_DATA_W`-bit DMA beats into an 8-bit pixel stream.
- **Parameters:** `S_DATA_W = 32`, `DEPTH = 1024` (power of two, 4..32768).
- **Ports:**
  - `clk` (in, 1), `rst` (in, 1, sync active-high)
  - `s_tdata` (in, `S_DATA_W`), `s_tvalid` (in, 1), `s_tready` (out, 1)
  - `pix_valid` (out, 1), `pix_data` (out, 8, unsigned Q8.0)
  - `level` (out, 16, unsigned occupancy in beats)
- **Handshake:** AXI-Stream (`s_tvalid`/`s_tready`) in; `pix_valid` (no back-pressure) out.
- **Latency / Throughput:** FWFT RAM read latency handled internally; 1 pixel/clock out when non-empty.

### 2.2 `orb_pyr_scaler`
- **Path:** `hw/pyramid/rtl/orb_pyr_scaler.sv`
- **Purpose:** Streaming 1.2× (5/6) bilinear downscaler using mod-6 coordinates and a 1-row BRAM line buffer. Divides accumulator by 25 via `(nrm * 5243) >> 17`.
- **Parameters:** `IMG_W = 640`, `IMG_H = 480`, `ROUND = 1`.
- **Ports:** `clk` (in, 1), `rst` (in, 1, sync active-high), `in_valid` (in, 1), `in_pixel` (in, 8, Q8.0 unsigned), `out_valid` (out, 1), `out_pixel` (out, 8, Q8.0 unsigned), `out_acc` (out, 13, Q13.0 unsigned weighted sum 0..6375), `out_eol` (out, 1), `out_eof` (out, 1).
- **Handshake:** `in_valid` → `out_valid` (no back-pressure).
- **Latency / Throughput:** 4 clocks from completing `(x+1, y+1)` pixel; up to 1 pixel/clock (drops every 6th column and row).

### 2.3 `orb_image_pyramid`
- **Path:** `hw/pyramid/rtl/orb_image_pyramid.sv`
- **Purpose:** Cascaded 4-level image pyramid (L0 = input passthrough, L1..L3 = three chained `orb_pyr_scaler` stages).
- **Parameters:** `IMG_W = 640`, `IMG_H = 480`, `ROUND = 1`.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `in_pixel[7:0]`, and per-level `l{1,2,3}_{valid, pixel[7:0], acc[12:0], eol, eof}`.
- **Latency / Throughput:** L1 = 4 clk, L2 = 8 clk, L3 = 12 clk; concurrent streaming across all levels.

### 2.4 `orb_pyramid_top`
- **Path:** `hw/pyramid/rtl/orb_pyramid_top.sv`
- **Purpose:** Wrapper connecting `orb_input_fifo` to `orb_image_pyramid` with AXI-Stream slave input.
- **Parameters:** `S_DATA_W = 32`, `FIFO_DEPTH = 1024`, `IMG_W = 640`, `IMG_H = 480`, `ROUND = 1`.
- **Ports:** `clk`, `rst` (sync active-high), `s_axis_{tdata[S_DATA_W-1:0], tvalid, tready}`, `fifo_level[15:0]`, `l0_{valid, pixel[7:0]}`, `l{1,2,3}_{valid, pixel[7:0], acc[12:0], eol, eof}`.

### 2.5 `orb_window7`
- **Path:** `hw/fast/rtl/orb_window7.sv`
- **Purpose:** Streaming 7×7 pixel window generator (`IMG_W` × 48-bit line buffer) shared by FAST and Gaussian filter.
- **Parameters:** `IMG_W = 640`, `IMG_H = 480`.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `in_pixel[7:0]` (Q8.0), `win_valid`, `win_x[11:0]`, `win_y[11:0]`, `win_data[391:0]` (49×8-bit row-major, center at `(3,3)`).
- **Latency / Throughput:** 2 clocks from completing pixel `(cx, cy)` (`cx >= 6 && cy >= 6`); 1 window/clock.

### 2.6 `orb_fast_core`
- **Path:** `hw/fast/rtl/orb_fast_core.sv`
- **Purpose:** FAST-16 corner detector (9 contiguous brighter/darker pixels on radius-3 Bresenham circle) + 16-pixel SAD score.
- **Parameters:** None.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `win[391:0]`, `thr[7:0]` (Q8.0 unsigned), `out_valid`, `out_score[11:0]` (Q12.0 unsigned, 0..4080; 0 if not a corner).
- **Latency / Throughput:** 3 clocks; 1 window/clock.

### 2.7 `orb_nms3x3`
- **Path:** `hw/fast/rtl/orb_nms3x3.sv`
- **Purpose:** 3×3 non-maximum suppression on the FAST score map (`MAP_W × 24`-bit line buffer), emitting raster-ordered keypoints in level coordinates.
- **Parameters:** `MAP_W = 474`, `MAP_H = 634`, `X_OFF = 3`, `Y_OFF = 3`.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `in_score[11:0]` (Q12.0), `out_valid`, `out_kp[35:0]` (`{score[11:0], y[11:0], x[11:0]}`).
- **Latency / Throughput:** 3 clocks from bottom-right score neighbor; 1 keypoint/clock max.

### 2.8 `orb_gauss7`
- **Path:** `hw/gaussian/rtl/orb_gauss7.sv`
- **Purpose:** Multiplier-free separable 7×7 binomial Gaussian filter (`[1 6 15 20 15 6 1]`, floor `>> 12`).
- **Parameters:** None.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `win[391:0]`, `out_valid`, `out_pixel[7:0]` (Q8.0 unsigned).
- **Latency / Throughput:** 2 clocks; 1 pixel/clock.

### 2.9 `orb_frontend_level`
- **Path:** `hw/frontend_top/rtl/orb_frontend_level.sv`
- **Purpose:** Single-level wrapper combining `orb_window7`, `orb_fast_core`, `orb_nms3x3`, and `orb_gauss7`.
- **Parameters:** `IMG_W = 640`, `IMG_H = 480`.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `in_pixel[7:0]`, `thr[7:0]`, `g_{valid, pixel[7:0]}`, `fs_{valid, score[11:0]}`, `kp_{valid, data[35:0]}`.

### 2.10 `pipelined_adder_tree`
- **Path:** `hw/orientation/rtl/pipelined_adder_tree.sv`
- **Purpose:** Generic pipelined adder tree (`L = $clog2(N)`, `OW = W + L`).
- **Parameters:** `N = 37`, `W = 8`, `SIGNED = 1'b0`, `L = $clog2(N)`, `OW = W + L`.
- **Ports:** `clk` (no reset), `din[N*W-1:0]`, `dout[OW-1:0]` (signed).
- **Latency / Throughput:** `L` clocks; 1 result/clock.

### 2.11 `orient_linebuf`
- **Path:** `hw/orientation/rtl/orient_linebuf.sv`
- **Purpose:** 37-row BRAM line buffer outputting one 37×1 column per valid pixel with sideband tag. Used for both 8-bit orientation and 6-bit rBRIEF window buffering.
- **Parameters:** `PIX_W = 8`, `IMG_W = 640`, `CW = $clog2(IMG_W)`, `TAG_W = 1`.
- **Ports:** `clk`, `rst_n` (sync active-low), `pix_valid`, `pix_data[PIX_W-1:0]`, `pix_col[CW-1:0]`, `tag_in[TAG_W-1:0]`, `col_valid`, `col_data[37*PIX_W-1:0]`, `tag_out[TAG_W-1:0]`.
- **Latency / Throughput:** 2 clocks; 1 column/clock.

### 2.12 `orient_colsum`
- **Path:** `hw/orientation/rtl/orient_colsum.sv`
- **Purpose:** Computes column sum `S(C)` (`SW = PIX_W+6` unsigned) and `m01(C)` (`MCW = PIX_W+11` signed) using `pipelined_adder_tree`.
- **Parameters:** `PIX_W = 8`, `TAG_W = 1`, `SW = PIX_W + 6`, `MCW = PIX_W + 11`.
- **Ports:** `clk`, `rst_n`, `valid_in`, `col_data[37*PIX_W-1:0]`, `tag_in`, `valid_out`, `s_out[SW-1:0]`, `m01c_out[MCW-1:0]` (signed), `tag_out`.
- **Latency / Throughput:** 7 clocks; 1 result/clock.

### 2.13 `orient_moments`
- **Path:** `hw/orientation/rtl/orient_moments.sv`
- **Purpose:** Sliding 37-column delay line + recursive accumulators for `m00` (`M0W = PIX_W+11` unsigned), `m10` and `m01` (`MW = PIX_W+14` signed, `m10` mirrored).
- **Parameters:** `PIX_W = 8`, `TAG_W = 1`, `SW`, `MCW`, `M0W`, `MW`.
- **Ports:** `clk`, `rst_n`, `valid_in`, `s_in[SW-1:0]`, `m01c_in[MCW-1:0]`, `tag_in`, `valid_out`, `m00_out[M0W-1:0]`, `m10_out[MW-1:0]`, `m01_out[MW-1:0]`, `tag_out`.
- **Latency / Throughput:** 2 clocks; 1 result/clock.

### 2.14 `orient_quantizer`
- **Path:** `hw/orientation/rtl/orient_quantizer.sv`
- **Purpose:** Angle quantization `(m10, m01)` → `(quadrant[1:0], sector[SEC_W-1:0])` using Q5.8 `TAN_TABLE` (`orient_tan_table.svh`).
- **Parameters:** `MW = 22`, `N_SECT = 64`, `NQ = N_SECT/4`, `SEC_W = $clog2(N_SECT/4)`, `TAG_W = 1`.
- **Ports:** `clk`, `rst_n`, `valid_in`, `m10[MW-1:0]`, `m01[MW-1:0]`, `tag_in`, `valid_out`, `quadrant[1:0]`, `sector[SEC_W-1:0]`, `tag_out`.
- **Latency / Throughput:** 4 clocks; 1 result/clock.

### 2.15 `orient_core`
- **Path:** `hw/orientation/rtl/orient_core.sv`
- **Purpose:** Full streaming 37×37 orientation pipeline (`orient_linebuf` → `orient_colsum` → `orient_moments` → `orient_quantizer`).
- **Parameters:** `PIX_W = 8`, `IMG_W = 640`, `IMG_H = 480`, `N_SECT = 64`, `CW`, `RW`, `SEC_W`, `SW`, `MCW`, `M0W`, `MW`.
- **Ports:** `clk`, `rst_n`, `pix_valid`, `pix_sof`, `pix_data[PIX_W-1:0]`, `res_valid`, `res_x[CW-1:0]`, `res_y[RW-1:0]`, `res_quad[1:0]`, `res_sector[SEC_W-1:0]`, `res_m00[M0W-1:0]`, `res_m10[MW-1:0]`, `res_m01[MW-1:0]`.
- **Latency / Throughput:** 15 clocks from completing window pixel; 1 result/clock.

### 2.16 `kp_fifo`
- **Path:** `hw/orientation/rtl/kp_fifo.sv`
- **Purpose:** Synchronous FWFT FIFO (`depth = 2**AW`) with synchronous `flush` and `rst_n`.
- **Parameters:** `DW = 16`, `AW = 6`.
- **Ports:** `clk`, `rst_n`, `flush`, `wr_en`, `wr_data[DW-1:0]`, `full`, `rd_en`, `rd_data[DW-1:0]`, `empty`.

### 2.17 `kp_match`
- **Path:** `hw/orientation/rtl/kp_match.sv`
- **Purpose:** Matches streaming `orient_core` window coordinates against `kp_fifo` head (popping matches and stale entries).
- **Parameters:** `CW = 7`, `RW = 6`, `SEC_W = 4`.
- **Ports:** `clk`, `rst_n`, `res_{valid, x, y, quad, sector}`, `kp_{valid, x, y, pop}`, `out_{valid, x, y, quad, sector}`.
- **Latency:** 1 clock output register.

### 2.18 `orient_top`
- **Path:** `hw/orientation/rtl/orient_top.sv`
- **Purpose:** Keypoint-gated orientation wrapper (`orient_core` + border filter + `kp_fifo` + `kp_match`).
- **Parameters:** `PIX_W = 8`, `IMG_W = 640`, `IMG_H = 480`, `N_SECT = 64`, `KP_AW = 6`, `CW`, `RW`, `SEC_W`.
- **Ports:** `clk`, `rst_n` (sync active-low), `pix_valid`, `pix_sof`, `pix_data`, `kp_in_{valid, x, y, ready}`, `kp_flush`, `out_{valid, x, y, quad, sector}`.
- **Latency / Throughput:** 16 clocks from completing pixel; 1 matched keypoint/clock.

### 2.19 `trig_decoder`
- **Path:** `hw/rbrief/rtl/trig_decoder.sv`
- **Purpose:** Combinational LUT converting `(quadrant[1:0], sector[3:0])` into signed Q1.7 `(cos_val[8:0], sin_val[8:0])` (`-128..+128`).
- **Ports:** `quadrant[1:0]` (in), `sector[3:0]` (in), `cos_val[8:0]` (out, signed Q1.7), `sin_val[8:0]` (out, signed Q1.7).

### 2.20 `rbrief`
- **Path:** `hw/rbrief/rtl/rbrief.sv`
- **Purpose:** Sequential 256-bit rBRIEF descriptor engine processing 1 learned pair `(A, B)` per clock from `PATTERN_FILE` over a frozen 37×37×6-bit window.
- **Parameters:** `PATTERN_FILE = "vectors/pattern_rom.hex"`.
- **Ports:** `clk`, `rst_n` (async active-low), `start`, `window_flat[8213:0]` (1369×6-bit row-major Q6.0), `cos_val[8:0]` (signed Q1.7), `sin_val[8:0]` (signed Q1.7), `busy`, `done`, `descriptor[255:0]`.
- **Latency / Throughput:** 257 clocks from `start` assertion to `done` pulse.

### 2.21 `rbrief_top`
- **Path:** `hw/rbrief/rtl/rbrief_top.sv`
- **Purpose:** Connects `trig_decoder` to `rbrief` so angle input is `(quadrant, sector)`.
- **Parameters:** `PATTERN_FILE = "vectors/pattern_rom.hex"`.
- **Ports:** `clk`, `rst_n` (async active-low), `start`, `window_flat[8213:0]`, `quadrant[1:0]`, `sector[3:0]`, `busy`, `done`, `descriptor[255:0]`.

### 2.22 `orb_extractor_level` *(Integration Wrapper)*
- **Path:** `hw/frontend_top/rtl/orb_extractor_level.sv`
- **Purpose:** Complete single-level ORB feature extractor integrating `orb_frontend_level`, `orient_top`, parallel `u_score_fifo`, cycle-aligned 6-bit 37×37 window buffer (`orient_linebuf` + 13-clock delay + 37-column shift register), `u_win_fifo`, and `rbrief_top`.
- **Parameters:** `IMG_W = vslam_cfg_pkg::IMG_W`, `IMG_H = vslam_cfg_pkg::IMG_H`, `N_SECT = vslam_cfg_pkg::N_SECT`, `KP_FIFO_AW = vslam_cfg_pkg::KP_FIFO_AW`, `WIN_FIFO_AW = vslam_cfg_pkg::WIN_FIFO_AW`, `PATTERN_FILE = vslam_cfg_pkg::PATTERN_FILE`.
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `in_pixel[7:0]`, `thr[7:0]`, intermediate `g_*`, `fs_*`, `kp_*`, `ori_*`, and final `feat_{valid, x[11:0], y[11:0], score[11:0], quad[1:0], sector[3:0], desc[255:0]}`, `busy`.

### 2.23 `orb_frontend_top` *(System Top Level)*
- **Path:** `hw/frontend_top/rtl/orb_frontend_top.sv`
- **Purpose:** Full 4-level ORB front-end top level: `orb_image_pyramid` + 4× `orb_extractor_level` (when `ENABLE_EXTRACTOR=1`, else `orb_frontend_level`) + 4× `(1 << OUT_FIFO_AW)`-entry output FIFOs + `L0 -> L1 -> L2 -> L3` priority arbiter into a unified feature stream.
- **Parameters:** `IMG_W`, `IMG_H`, `ROUND` (`PYR_ROUND`), `ENABLE_EXTRACTOR`, `N_SECT`, `KP_FIFO_AW`, `WIN_FIFO_AW`, `OUT_FIFO_AW`, `PATTERN_FILE` (all defaulted from `vslam_cfg_pkg::*`).
- **Ports:** `clk`, `rst` (sync active-high), `in_valid`, `in_pixel[7:0]`, `thr[7:0]`, per-level `l{0..3}_{g_*, fs_*, kp_*, feat_*}`, and unified arbitrated `feat_{valid, level[1:0], x[11:0], y[11:0], score[11:0], quad[1:0], sector[3:0], desc[255:0]}`, `busy`.

### 2.24 `orb_record_writer` *(PL-to-DDR4 Buffer-A DMA Packer)*
- **Path:** `hw/frontend_top/rtl/orb_record_writer.sv`
- **Purpose:** Buffers unified `orb_frontend_top` features in BRAM (`MAX_RECORDS = 1 << REC_AW`) during frame capture and emits a 64-byte frame header (`MAGIC_ORBK`, `FMT_VER`, `frame_id`, `n_records`, `timestamp_ns`, `width`, `height`, `n_dropped`, `status`, `L0..L7` counts) followed by `n_records × 40`-byte records over 64-bit little-endian AXI4-Stream (`m_axis_tvalid`, `m_axis_tready`, `m_axis_tdata`, `m_axis_tlast`).
- **Parameters:** `IMG_W`, `IMG_H`, `REC_AW`, `MAGIC_ORBK`, `FMT_VER` (all defaulted from `vslam_cfg_pkg::*`).
- **Ports:** `clk`, `rst` (sync active-high), `frame_start`, `frame_done`, `frame_id[31:0]`, `frame_timestamp_ns[63:0]`, `feat_{valid, level[1:0], x[11:0], y[11:0], score[11:0], quad[1:0], sector[3:0], desc[255:0]}`, `m_axis_{tvalid, tready, tdata[63:0], tlast}`, `busy`, `n_dropped[15:0]`.
- **Handshake / Format:** AXI4-Stream master (`tvalid` independent of `tready`, `tdata`/`tlast` stable under stall). Angle packed as 16-bit binary angle (`65536 = 360°`, matching `trig_decoder` quadrants).

---

## 3. Include files

| File | Location | Contents |
|---|---|---|
| `orient_tan_table.svh` | `hw/orientation/rtl/` | `TBL_NSECT=64`, `FRAC_BITS=8`, `N_CMP=16`, `TC_W=13`, packed `TAN_TABLE` |
| `sim_params.svh` | `hw/orientation/sim/` | `IMG_W=96`, `IMG_H=64`, `N_SECT=64`, `N_WIN=1680`, `N_KP=70`, `N_TOP=60` |
| `patch_params.svh` | `hw/orientation/sim/` | `N_PATCH=8`, `N_SECT=64` |

---

## 4. Integrated data path and compatibility resolutions

```mermaid
flowchart LR
    IN["8-bit Pixel Stream\n(orb_pyramid_top optional)"] --> PYR["orb_image_pyramid\n(L0..L3, ÷1.2)"]
    PYR -->|"L0..L3 pixels"| EXT["4× orb_extractor_level:\norb_frontend_level -> orient_top\n+ 37x37x6b buf -> rbrief_top"]
    EXT -->|"per-level features"| ARB["16-deep FIFOs +\nL0..L3 Priority Arbiter"]
    ARB -->|"feat_valid, lvl, x, y,\nscore, quad, sec, desc[255:0]"| RW["orb_record_writer\n(64B hdr + 40B rec, AXIS64)"]
    RW -->|"DDR4 Buffer A (.bin)"| PS["PS Backend (sw/host/):\nOctTree, Scale, Undistort,\nMatcher, 8-pt Pose + BA"]
    PS -->|"Buffer B (Q10.8 + ratio_lim)"| MATCH["hw/matcher\n(STUB)"]
```

**Integration bridges implemented in `orb_extractor_level` and `orb_frontend_top`:**
1. **Reset polarity:** `orb_*` modules use synchronous active-high `rst`; `orient_*` uses synchronous active-low `rst_n`; `rbrief*` uses asynchronous active-low `rst_n`. `orb_extractor_level` drives `rst_n = ~rst` from a synchronized reset source.
2. **Coordinate & SOF alignment:** `orb_gauss7` emits `(IMG_W-6)×(IMG_H-6)` interior pixels without `pix_sof`, whereas `orb_nms3x3` outputs keypoints in `[4..IMG_W-5]` level coordinates (`+3` offset). `orb_extractor_level` generates `g_sof` at `(0,0)` of the Gaussian raster, translates NMS keypoints by `(-3,-3)` for `orient_top`, and restores `(+3,+3)` at output.
3. **Score retention & FIFO lockstep:** `u_score_fifo` mirrors `orient_top`'s internal border filter (`kp_ok && kp_in_ready`) and pops on `ori_out_valid` so FAST scores stay paired 1-to-1 with oriented keypoints.
4. **37×37×6-bit window buffering:** `u_win37_lb` (`orient_linebuf` with `PIX_W=6` on `g_pixel[7:2]`) + 13-stage delay (`7+2+4`) + 37-column shift register aligns the 37×37 quantized patch to `ori_out_valid` (cycle 16), queues matched patches in `u_win_fifo`, and transposes column-major to row-major for `rbrief_top` (257 cycles/keypoint).
5. **Multi-level arbitration:** `orb_frontend_top` drains per-level 16-entry `kp_fifo`s with `L0 → L1 → L2 → L3` priority so simultaneous level completions never drop.

---

## 5. Testbenches and results (Icarus Verilog 12, 2026-11-20)

All 9 RTL testbenches (`TBImagePyramid`, `tb_orb_pyramid_top`, `TB_Frontend`, `tb_system_top`, `tb_record_writer`, `tb_orient`, `tb_trig_decoder`, `tb_rbrief`, `tb_rbrief_top`) compiled cleanly with `iverilog -g2012 -Wall -c scripts\rtl.f` (`0` new warnings) after adding `common/rtl/vslam_cfg_pkg.sv`. Integrated TBs + PS backend re-ran after the final `BACKEND_MAX_FEATURES = 450` edit; leaf unit TBs were compile-checked only in this pass (last simulated prior session).

| Testbench | Location | DUT | Result | Notes |
|---|---|---|---|---|
| `tb_image_pyramid` | `hw/pyramid/tb/tb_image_pyramid.sv` | `orb_image_pyramid` | **PASS** *(prior sim; compiled 2026-11-20)* | 480×640, 1 frame, L1=213200 L2=147852 L3=102490 px, 0 err |
| `tb_orb_pyramid_top` | `hw/pyramid/tb/tb_orb_pyramid_top.sv` | `orb_pyramid_top` | **PASS** *(prior sim; compiled 2026-11-20)* | 480×640, S_DATA_W=32, FIFO_DEPTH=16, 2 frames, 460739 back-pressure cycles, 0 err |
| `tb_frontend` | `hw/frontend_top/tb/tb_frontend.sv` | `orb_frontend_top` (`ENABLE_EXTRACTOR=0`) | **PASS** *(prior sim; compiled 2026-11-20)* | 480×640, L0..L3 Gauss/FAST/NMS exact match (`4510, 2849, 1890, 1329` kps) |
| `tb_system_top` | `hw/frontend_top/tb/tb_system_top.sv` | `orb_frontend_top` (`ENABLE_EXTRACTOR=1`) | **PASS** *(2026-11-20)* | 144×108 (`SIM_IMG_W×SIM_IMG_H`, `FAST_THR=20`) + mid-frame reset + 10% input bubbles; L0=217, L1=139, L2=68, L3=35 (`459/459` features bit-exact, `arb_err=0`, `rst_leak=0`); `orb_system_golden.py gen/plot` PASS (`vslam_system_visual.png/.svg`) |
| `tb_record_writer` | `hw/frontend_top/tb/tb_record_writer.sv` | `orb_record_writer` + `orb_frontend_top` | **PASS** *(2026-11-20)* | Empty frame (`8` words), overflow+backpressure (`8/11`, `n_dropped=3`), mid-frame reset, real RTL `459` feats -> `sw/host/frame0_rtl.bin` (`18424` B), live `96x84` `orb_frontend_top -> orb_record_writer` (`21` feats, `113` words, `0` err) |
| `test_backend.py` | `sw/host/test_backend.py` | `vslam_cfg` + `orb_records` + `orb_backend` | **PASS** *(2026-11-20)* | Tests A–E PASS: RTL `frame0_rtl.bin` bit-exact (`459` records), OctTree (`180` kps), Brown-Conrady undistort (`1.59e-14` px), Buffer B (`7200` B), real pair `256` matches (`-5.62°` median rot), synthetic 3D pose (`102` matches, `95` inliers, `R_err=0.51°`, `t_err=1.85°`, `med_reproj=0.16` px), Test E single-file override + Buffer-B bit-budget check |
| `tb_orient` (gap=0) | `hw/orientation/tb/tb_orient.sv` | `orient_core` + `orient_top` | **PASS** *(prior sim; compiled 2026-11-20)* | core: 1680/1680, top: 60/60, 0 err |
| `tb_orient` (gap=40) | `hw/orientation/tb/tb_orient.sv` | `orient_core` + `orient_top` | **PASS** *(prior sim; compiled 2026-11-20)* | 40% bubbles; core: 1680/1680, top: 60/60, 0 err |
| `tb_trig_decoder` | `hw/rbrief/tb/tb_trig_decoder.sv` | `trig_decoder` | **PASS** *(prior sim; compiled 2026-11-20)* | 64/64 quadrant×sector combinations exact |
| `tb_rbrief` | `hw/rbrief/tb/tb_rbrief.sv` | `rbrief` | **PASS** *(prior sim; compiled 2026-11-20)* | 100/100 descriptors exact |
| `tb_rbrief_top` | `hw/rbrief/tb/tb_rbrief_top.sv` | `rbrief_top` | **PASS** *(prior sim; compiled 2026-11-20)* | 100/100 descriptors exact |

---

## 6. Known / Open architectural items

1. **4 vs 8 pyramid levels:** `orb_image_pyramid` and `orb_frontend_top` currently implement 4 levels (`L0..L3`); `hw/pyramid/README.md` specifies 8 levels.
2. **AXI-Stream top wrapper:** `orb_frontend_top` accepts `in_valid`/`in_pixel` directly; `orb_pyramid_top` (`orb_input_fifo`) is not yet wired in front of `orb_frontend_top` or connected to an AXI DMA S2MM record writer.
3. **rBRIEF pattern table:** `make_pattern(seed=42)` in `rbrief_python.py` uses a synthetic verification pattern; for real ORB-SLAM3 accuracy, replace `pattern_rom.hex` with OpenCV `bit_pattern_31_`.
4. **Legacy duplicate folders:** `test/orient/` and `src/python/vslam.py` remain untouched from earlier sessions.

---

## 7. Open parameters

All tunable system parameters live in one editable file: `common/rtl/vslam_cfg_pkg.sv` (parsed automatically at runtime by `common/model/vslam_cfg.py` for Python models/backend). Note: editing `SIM_IMG_W`, `SIM_IMG_H`, or `FAST_THR` requires regenerating golden vectors before running `tb_system_top`.

| Parameter | Current value | Source | Notes |
|---|---|---|---|
| Camera/sensor interface | UNKNOWN | — | `orb_pyramid_top` assumes AXI-Stream MM2S DMA |
| Production resolution (`IMG_W`, `IMG_H`) / fast-sim (`SIM_IMG_W`, `SIM_IMG_H`) | `640×480` (prod) / `144×108` (sim) | `vslam_cfg_pkg.sv` | Verified at 480×640 (`tb_frontend`) & 144×108 (`tb_system_top`) |
| Frame rate / clock frequency | UNKNOWN | — | No synthesis/timing closure run yet |
| Pyramid levels & scale (`PYR_LEVELS`, `PYR_ROUND`, `PYR_SCALE_FACTOR`) | 4 levels (`L0..L3`), `ROUND=1`, `1.2×` (5/6) | `vslam_cfg_pkg.sv`, `orb_image_pyramid.sv` | RTL pyramid fixed at 4 levels (`README.md` targets 8) |
| Pyramid accumulator / norm | Q13.0 unsigned (`0..6375`), `(acc+12)*5243 >> 17` | `orb_pyr_scaler.sv` | Round-half-up when `PYR_ROUND=1` |
| Gaussian kernel & format | 7×7 binomial `[1 6 15 20 15 6 1]`, Q20.0 `>> 12` → Q8.0 | `orb_gauss7.sv` | Floor division |
| FAST threshold & score (`FAST_THR`) | `20`, Q12.0 SAD score (`0..4080`) | `vslam_cfg_pkg.sv`, `orb_fast_core.sv` | Retry threshold 7 not implemented |
| Orientation angle format (`N_SECT`) | 64 sectors (`quadrant[1:0]`, `sector[3:0]`), Q5.8 tan LUT | `vslam_cfg_pkg.sv`, `orient_quantizer.sv` | Fixed to 64 in `trig_decoder.sv` & `orient_tan_table.svh` |
| Trig decoder & rBRIEF patch (`PATTERN_FILE`, `PATTERN_SEED`) | Signed Q1.7 `cos/sin`; Q6.0 (`g_pixel[7:2]`) 37×37; `seed=42` | `vslam_cfg_pkg.sv`, `trig_decoder.sv`, `rbrief.sv` | 257 clk/kp sequential rBRIEF |
| FIFO / Record caps (`KP_FIFO_AW`, `WIN_FIFO_AW`, `OUT_FIFO_AW`, `REC_AW`) | `9` (512), `7` (`SIM=8`), `4` (16), `10` (1024 records) | `vslam_cfg_pkg.sv`, `orb_record_writer.sv` | Provisional; `n_dropped` counts record overflow |
| Buffer-A record format (`MAGIC_ORBK`, `FMT_VER`, `HEADER_BYTES`, `RECORD_BYTES`) | `"ORBK"` (`0x4B42524F`), `v1`, `64 B` hdr, `40 B` rec | `vslam_cfg_pkg.sv`, `orb_record_writer.sv`, `orb_records.py` | `w0..w3` = `desc[255:0]` LE; `w4` = `{res[3:0], angle16, score[15:0], V, lvl[2:0], y[11:0], x[11:0]}` |
| Buffer-B & PS backend (`BUF_B_*`, `BACKEND_MAX_FEATURES`, `MATCH_*`, `RANSAC_*`) | `64×48` grid, `Q10.8`, `450` kps, `dist<=64`, `ratio=0.8`, `30` bins, `600` iters, `1.5` px | `vslam_cfg_pkg.sv`, `orb_records.py`, `orb_backend.py` | 64-bit metadata bit budget checked in `pack_matcher_buffer_b` |
| Matcher interface | `hw/matcher` empty; PS software Hamming+ratio+30-bin matcher active in `sw/host/` | `hw/matcher/README.md`, `orb_backend.py` | Hardware matcher left empty per spec |
