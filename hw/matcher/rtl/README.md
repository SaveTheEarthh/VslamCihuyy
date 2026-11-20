# ORB matcher in SystemVerilog

Hand-written RTL version of the same matcher as `../hls/orb_match_hls.c`. It produces
the same results bit for bit as the golden model `../model/orb_match_ref.c`, and therefore as
ORB-SLAM3's `SearchForInitialization`.

| File | What it is |
|---|---|
| `orb_match_pkg.sv` | Constants, popcount adder tree, rotation-bin function |
| `orb_lane.sv` | One of 8 parallel compare units: its memory bank, candidate checks, Hamming distance, own best/second |
| `orb_match_core.sv` | Control FSM: load, per-query scan, merge, accept/steal, histogram, write-back |
| `orb_match_top.sv` | AXI-Lite registers + AXI-Stream ports (the IP you put in a block design) |
| `../tb/tb_top.sv` | SV testbench: AXI-Lite master, stream driver/collector with random stalls |
| `../tb/tb_dpi.c` | C side of the testbench (DPI-C): builds tests, runs the golden model, compares |
| `../sim/run_rtl.sh` | xsim run (+ `impl` for Vivado place & route), inside `xilinx-box` |

```bash
../sim/run_rtl.sh                  # 60 random tests + 5 EuRoC pairs in xsim
../sim/run_rtl.sh impl             # ... then out-of-context synthesis + P&R on the K26
ORB_QUICK=1 ORB_RANDOM=10 ../sim/run_rtl.sh   # fast smoke test
```

## Where each algorithm step lives

The steps are the ones labelled `STEP 1..6` in `../model/orb_match_ref.c`.

| Step (C golden model) | RTL |
|---|---|
| Loop over queries | `orb_match_core`: `S_FETCH` → `S_SETUP` → `S_SCAN` → … → `S_NEXT` |
| STEP 1 candidate checks (range, valid, level, window) | `orb_lane`, `cand_ok2` (stage s3) |
| STEP 2 Hamming distance | `orb_lane`: XOR (s3), `popcount_part` (s4) + `popcount_sum` (s5), functions in `orb_match_pkg` |
| STEP 3 best / second best | `orb_lane` stage s6 (per lane), then `S_MERGE` in the core (8 lanes → 1) |
| STEP 4 threshold + ratio test | core `S_DEC0`, `S_DEC0B` (read `lim[best2]`, 2 clocks) and `S_DEC1` (decide) |
| STEP 5 one-to-one / steal | lane: `take` uses `matched_dist`; core `S_DEC1` (steal) and `S_DEC2` (own) |
| STEP 6 rotation histogram | core `S_DEC2` (`histo[new_bin]++`), `S_MAX3` + `S_MAX3_FIN` (three maxima, 10% rule), `S_WB` (`wb_reject`) |

## Interfaces

The HLS kernel fetches its own data with an AXI master. The RTL instead takes **streams**, so
in the block design it sits behind an **AXI DMA** (MM2S → `s_axis`, `m_axis` → S2MM). That
needs much less RTL, and the DMA is a standard Xilinx IP.

Input stream, 64-bit words, in this order:
1. 257 words: ratio table (`orb_ratio_table`), value in bits [8:0]
2. `n_train` × 5 words: descriptor words 0..3, then the train meta word
3. `n_query` × 6 words: descriptor words 0..3, query meta w0, w1

The meta word layouts are the same as in `../hls/orb_match.h`.

Output stream, 32-bit, one word per query (in query order, `tlast` on the last word):
`[15:0]` = train index or `0xFFFF` (no match), `[31:16]` = best distance (511 = none).

Registers: `0x00` CTRL (W bit0 start; R bit0 busy, bit1 done), `0x04` N_TRAIN, `0x08` N_QUERY,
`0x0C` TH_DIST, `0x10` FLAGS (bit0 one-to-one, bit1 orientation check), `0x14` N_MATCHES.

A run, as the ARM does it: write the 4 config registers, write CTRL = 1, start the S2MM DMA
(n_query × 4 bytes) and the MM2S DMA (the input buffer), wait for CTRL.done, read N_MATCHES.

## Results (K26, out-of-context place & route, 300 MHz) vs the HLS kernel

| | HLS (`../hls/orb_match_hls.c`) | SystemVerilog (this folder) |
|---|---|---|
| Correctness | bit-exact, 1,883/1,883 = ORB-SLAM3 | bit-exact, 1,883/1,883 = ORB-SLAM3 (65 tests, random stream stalls) |
| Timing @ 300 MHz | met, WNS +0.122 ns | met, WNS +0.047 ns |
| LUT | 11,103 (9.5%) | 11,508 (9.8%) |
| FF | 11,022 (4.7%) | 4,759 (2.0%) |
| BRAM36 | 30 (21%) | 10.5 (7.3%) |
| URAM | 40 (62.5%) | 40 (62.5%) |
| DSP | 0 | 0 |
| Cycles, pair `mh01_1500_10` | 300,781 (1.00 ms) | 268,157 (0.89 ms) |
| Data movement | own AXI master (reads DDR itself) | AXI-Stream: needs an AXI DMA next to it (extra LUT/BRAM not counted here) |
| Lines of design code (with comments) | ~290 C | ~900 SV |
| Getting to 300 MHz | first try | 3 rounds of timing fixes (see below) |

Timing fixes needed in the RTL (each found in `ooc/top_paths.rpt`):
1. Ratio-table RAM output → compare → `m12` RAM write enable: added a RAM output-register stage (`S_DEC0B`).
2. Angle subtract → ×12 → round → histogram increment in one clock: register the bin early (`new_bin`).
3. `n_train` → skip test → RAM write enable; and URAM → XOR → popcount: moved the skip writes to
   `S_NEXT` from a register, gave the XOR its own stage, rebalanced the adder tree to 5 + 3 levels.
