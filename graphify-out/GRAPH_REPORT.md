# Graph Report - VslamCihuyy  (2026-11-20)

## Corpus Check
- 91 files · ~181,368 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 131 file(s) not represented in the graph (top: .hex 74, (none) 36, .npy 8)
- WARNING: 131 of 253 files seen (52%) have no supported extractor and are missing from this graph entirely. It may look complete while representing only part of the codebase.

## Summary
- 511 nodes · 860 edges · 54 communities (20 shown, 34 thin omitted)
- Extraction: 98% EXTRACTED · 2% INFERRED · 0% AMBIGUOUS · INFERRED: 20 edges (avg confidence: 0.84)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `8967c9fa`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- orb_system_golden.py
- orient/patch_test.py
- 2. Module catalog
- numpy
- orb_image_pyramid
- orb_extractor_level
- image_pyramid.py
- sim/patch_test.py
- orient/orientation_only.py
- model/orientation_only.py
- random
- FPGA Development
- numpy
- VslamCihuyy: FPGA-accelerated VSLAM (SystemVerilog), integration stage
- 1. `orb_pyr_scaler.v` (The Core Engine)
- Python Golden Model (`orb_pyramid_golden.py`) - Detailed Breakdown
- Penjelasan Modul Verilog FPGA VSLAM
- Panduan Alur Kerja Sistem Frontend VSLAM (Python & Verilog)
- VslamCihuyy: FPGA-accelerated ORB front-end for VSLAM
- frontend_top/sim/manifest.json
- orient_colsum
- pyramid/sim/manifest.json
- data/README.md
- interfaces.md
- fast/README.md
- frontend_top/README.md
- gaussian/README.md
- matcher/README.md
- orientation/README.md
- kp_match.sv
- orient_moments.sv
- pyramid/README.md
- rbrief/README.md
- vslam_cfg.py

## God Nodes (most connected - your core abstractions)
1. `2. Module catalog` - 26 edges
2. `run_gen()` - 13 edges
3. `extract_golden_to_bin()` - 13 edges
4. `estimate_pose_and_triangulate()` - 12 edges
5. `parse_frame_bin()` - 12 edges
6. `cmd_plot()` - 12 edges
7. `cmd_plot()` - 12 edges
8. `run_backend_pair()` - 11 edges
9. `pack_matcher_buffer_b()` - 11 edges
10. `VslamCihuyy: FPGA-accelerated VSLAM (SystemVerilog), integration stage` - 11 edges

## Surprising Connections (you probably didn't know these)
- `7. Open parameters` --references--> `pack_matcher_buffer_b()`  [INFERRED]
  docs/integration.md → sw/host/orb_records.py
- `Top-Level & Testbench` --references--> `fast_score()`  [INFERRED]
  docs/notes/frontend_verilog_explanation.md → hw/frontend_top/model/orb_frontend_golden.py
- `2.0 `vslam_cfg_pkg` & `vslam_cfg.py` *(Single Parameter Source)*` --references--> `load_cfg()`  [INFERRED]
  docs/integration.md → common/model/vslam_cfg.py
- `test_e_single_config_propagation()` --calls--> `load_cfg()`  [INFERRED]
  sw/host/test_backend.py → common/model/vslam_cfg.py
- `extract_golden_to_bin()` --calls--> `fast_score()`  [INFERRED]
  sw/host/test_backend.py → hw/frontend_top/model/orb_frontend_golden.py

## Import Cycles
- None detected.

## Communities (54 total, 34 thin omitted)

### Community 0 - "orb_system_golden.py"
Cohesion: 0.06
Nodes (27): fast_score(), gaussian7(), load_level(), main(), nms3x3(), draw_arrow_kp(), load_font(), main() (+19 more)

### Community 1 - "orient/patch_test.py"
Cohesion: 0.17
Nodes (20): cmd_gen(), cmd_plot(), cmd_sim(), draw_dial(), draw_patch(), exact_img_angle(), find_tool(), img_angle() (+12 more)

### Community 2 - "2. Module catalog"
Cohesion: 0.06
Nodes (32): 1. Repository layout, 2.10 `pipelined_adder_tree`, 2.11 `orient_linebuf`, 2.12 `orient_colsum`, 2.13 `orient_moments`, 2.14 `orient_quantizer`, 2.15 `orient_core`, 2.16 `kp_fifo` (+24 more)

### Community 3 - "numpy"
Cohesion: 0.24
Nodes (8): build_pyramid(), downscale_stream(), downscale_vectorized(), load_gray(), main(), normalize(), pixel_weights(), selftest()

### Community 4 - "orb_image_pyramid"
Cohesion: 0.06
Nodes (16): orb_frontend_top, orb_record_writer, orb_stream_chk, TB_Frontend, report_level, tb_record_writer, collect_a, write_u64_le (+8 more)

### Community 5 - "orb_extractor_level"
Cohesion: 0.05
Nodes (17): orb_fast_core, orb_nms3x3, orb_window7, orb_extractor_level, orb_frontend_level, orb_gauss7, kp_fifo, tb_orient (+9 more)

### Community 6 - "image_pyramid.py"
Cohesion: 0.16
Nodes (10): build_pyramid(), build_weight_lut(), downscale_5_6(), downscale_5_6_streaming(), load_image(), main(), make_synthetic_image(), normalise() (+2 more)

### Community 7 - "sim/patch_test.py"
Cohesion: 0.15
Nodes (20): cmd_gen(), cmd_plot(), cmd_sim(), draw_dial(), draw_patch(), exact_img_angle(), find_tool(), img_angle() (+12 more)

### Community 8 - "orient/orientation_only.py"
Cohesion: 0.13
Nodes (6): brute_force_moments(), check_against_brute_force(), demo_blob(), hw_to_image_angle(), OrientationModule, Result

### Community 9 - "model/orientation_only.py"
Cohesion: 0.13
Nodes (6): brute_force_moments(), check_against_brute_force(), demo_blob(), hw_to_image_angle(), OrientationModule, Result

### Community 10 - "random"
Cohesion: 0.17
Nodes (8): main(), make_image(), write_tan_table(), run_sim.sh script, main(), make_image(), write_tan_table(), run_sim.sh script

### Community 11 - "FPGA Development"
Cohesion: 0.15
Nodes (12): Advanced Techniques, AXI Protocol Compliance, Clock Domain Crossing, Debugging & Simulation, DMA Integration, FPGA Development, Latency Reduction, Modular Design & Code Organization (+4 more)

### Community 12 - "numpy"
Cohesion: 0.09
Nodes (31): distribute_oct_tree(), _eight_point_essential(), _enforce_essential(), estimate_pose_and_triangulate(), residuals(), match_descriptors(), _normalize_2d(), _prep_frame() (+23 more)

### Community 13 - "VslamCihuyy: FPGA-accelerated VSLAM (SystemVerilog), integration stage"
Cohesion: 0.17
Nodes (11): Commands (PowerShell), Compact instructions, Current stage, Error checklist (apply on every integration and review), First session: build the integration map, Honesty and boundaries, Integration workflow (in order), SystemVerilog rules (+3 more)

### Community 14 - "1. `orb_pyr_scaler.v` (The Core Engine)"
Cohesion: 0.17
Nodes (11): 1. `orb_pyr_scaler.v` (The Core Engine), 2. `orb_image_pyramid.v` (The Wrapper), 3. `TBImagePyramid.v` (The Testbench), Memory Initialization, Stage 0: Coordinates, Stage 1: The Line Buffer and 2x2 Window, Stage 2 & 3: Weights and Multiply-Accumulate (MAC), Stage 4: Normalization (+3 more)

### Community 15 - "Python Golden Model (`orb_pyramid_golden.py`) - Detailed Breakdown"
Cohesion: 0.25
Nodes (7): 1. Setup and Argument Parsing, 2. Image Loading and Preprocessing, 3. The Core Scaling Function (The Modulo-6 Math), 4. Bilinear Weight Calculation and Accumulation, 5. Normalization, 6. File Export (Hex Generation), Python Golden Model (`orb_pyramid_golden.py`) - Detailed Breakdown

### Community 16 - "Penjelasan Modul Verilog FPGA VSLAM"
Cohesion: 0.40
Nodes (4): Modul Ekstraksi Fitur (Frontend), Modul Image Pyramid, Penjelasan Modul Verilog FPGA VSLAM, Top-Level & Testbench

### Community 17 - "Panduan Alur Kerja Sistem Frontend VSLAM (Python & Verilog)"
Cohesion: 0.40
Nodes (4): Langkah 1: Menghasilkan Data Referensi (Golden Data) dengan Python, Langkah 2: Kompilasi Sistem Hardware di Verilog, Langkah 3: Menjalankan Simulasi, Panduan Alur Kerja Sistem Frontend VSLAM (Python & Verilog)

### Community 18 - "VslamCihuyy: FPGA-accelerated ORB front-end for VSLAM"
Cohesion: 0.50
Nodes (3): Layout, Rules, VslamCihuyy: FPGA-accelerated ORB front-end for VSLAM

### Community 52 - "vslam_cfg.py"
Cohesion: 0.15
Nodes (5): load_cfg(), _parse_sv_literal(), _strip_line_comment(), VslamConfig, 2.0 `vslam_cfg_pkg` & `vslam_cfg.py` *(Single Parameter Source)*

## Knowledge Gaps
- **6 isolated node(s):** `collect_a`, `write_u64_le`, `check_feat`, `kp_match`, `orient_moments` (+1 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 272 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **34 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `2. Module catalog` connect `2. Module catalog` to `vslam_cfg.py`?**
  _High betweenness centrality (0.059) - this node is a cross-community bridge._
- **What connects `collect_a`, `write_u64_le`, `check_feat` to the rest of the system?**
  _6 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `orb_system_golden.py` be split into smaller, more focused modules?**
  _Cohesion score 0.06328320802005012 - nodes in this community are weakly interconnected._
- **Why does `pack_matcher_buffer_b()` connect `numpy` to `2. Module catalog`?**
  _High betweenness centrality (0.048) - this node is a cross-community bridge._
- **Should `2. Module catalog` be split into smaller, more focused modules?**
  _Cohesion score 0.06060606060606061 - nodes in this community are weakly interconnected._
- **Why does `7. Open parameters` connect `2. Module catalog` to `numpy`?**
  _High betweenness centrality (0.040) - this node is a cross-community bridge._
- **Should `orb_image_pyramid` be split into smaller, more focused modules?**
  _Cohesion score 0.05555555555555555 - nodes in this community are weakly interconnected._