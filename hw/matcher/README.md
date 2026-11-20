# matcher (owner: Zaki)

ORB-SLAM3's frame-to-frame feature matching (`ORBmatcher::SearchForInitialization`) for the
KV260, in two versions that both give **the same matches as ORB-SLAM3**, bit for bit:
a Vitis HLS kernel in plain C (`hls/`) and a hand-written SystemVerilog version (`rtl/`).
Design notes and the theory behind it: `docs/notes/W8_Feature_Matching_FPGA.md`.

| File | What it is |
|---|---|
| `hls/orb_match.h` | Data formats (fixed-point, bit layouts), constants, kernel API |
| `hls/orb_match_hls.c` | **The HLS kernel** (top: `orb_match_hls`), synthesisable |
| `hls/run_hls.sh`, `hls/run_hls.tcl` | Vitis HLS flow: csim → csynth → cosim → IP export, then Vivado P&R |
| `model/orb_match_ref.c` | Golden model: same API, written like `ORBmatcher.cc`, no HW tricks |
| `model/orb_host.c/.h` | ARM-side code: load test pairs, grid sort, scan ranges, packing |
| `tb/tb_orb_match.c` | C testbench (gcc, HLS C-sim and C/RTL co-sim) |
| `tb/tb_top.sv`, `tb/tb_dpi.c` | SV testbench for `rtl/` (DPI-C, runs the golden model) |
| `rtl/` | **The same matcher in SystemVerilog** (AXI-Lite + AXI-Stream). See `rtl/README.md` |
| `sim/Makefile`, `sim/run_rtl.sh` | gcc test run; xsim run of `rtl/` (+ P&R) |
| `sim/data/mh01_<frame>_<offset>.bin` | 5 golden EuRoC MH_01 frame pairs from real ORB-SLAM3 |

Shared with other stages: `impl/scripts/vivado_ooc.tcl` (out-of-context synth + P&R on the K26)
and `sw/orbslam3/` (`dump_orbslam3.cc` + `build_dump.sh`, which run ORB-SLAM3's extractor and
matcher on EuRoC and write `sim/data/*.bin`).

The golden pairs are committed (3 MB) because regenerating them needs a built ORB-SLAM3 and
the EuRoC dataset.

## Run

```bash
cd hw/matcher/sim
make test                       # gcc: all tests, ~10 s
make hls                        # Vitis 2025.2 in xilinx-box: csim csynth cosim export impl
HLS_STEPS="csim csynth" ../hls/run_hls.sh   # just some steps
./run_rtl.sh                    # SV version in xsim; ./run_rtl.sh impl adds P&R
../../../sw/orbslam3/build_dump.sh          # regenerate golden pairs (ros2-box, needs ~/vslam_ws/ORB_SLAM3)
```

`run_hls.sh` and `run_rtl.sh` copy the sources to `~/vslam_ws/hls/orb_matcher` and
`~/vslam_ws/hls/orb_matcher_rtl` because Vitis and Vivado refuse paths with spaces (override
with `BUILD=...`). HLS reports end up in `.../orb_match_prj/sol_kv260/syn/report` and
`.../ooc/`. The IP for Vivado's catalog is in `.../sol_kv260/impl/ip/*.zip`.

## Using the kernel from the ARM

1. Train = frame 2 keypoints, **sorted by ORB-SLAM3 grid cell** `(cell_x, cell_y, index)`.
   Drop keypoints that are not in the grid. Keep the permutation.
2. Query = frame 1 keypoints. For each, compute the grid-column range of its search window,
   which is one contiguous index range `[lo, hi)` of the sorted train list.
3. Pack everything with `orb_pack_train / orb_pack_query0 / orb_pack_query1`, and build the
   ratio table with `orb_ratio_table(0.9f, lim)`.
4. Start the kernel with `th_dist = 50`, `unique = 1`, `check_orient = 1`. Map `match_out[i]`
   back through the permutation.

`orb_prepare_init()` in `model/orb_host.c` does steps 1–3. Brute-force matching (eSLAM-style)
is the same kernel with `lo = 0, hi = n_train`, no window, all levels.

AXI-Lite registers: see `orb_match_prj/sol_kv260/impl/ip/drivers/*/src/xorb_match_hls_hw.h`.
All six buffers (plus the 257-entry ratio table) go through one AXI master (`m_axi_gmem`),
so connect it to one HP port and pass physical addresses of contiguous buffers.
