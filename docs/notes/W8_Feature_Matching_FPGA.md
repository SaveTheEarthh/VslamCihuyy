# W8: ORB Feature Matching on the FPGA

*Companion to `VSLAM_Master_Notes.md` §6.3 and §11. Explains what ORB-SLAM3's matcher does, how the HLS kernel in `hw/matcher/` reproduces it, and the measured results. Written 2026-10-26.*

**Summary.** The kernel runs ORB-SLAM3's monocular-initialisation matcher (`SearchForInitialization`) in programmable logic. On 5 real EuRoC MH_01 frame pairs it returns **exactly ORB-SLAM3's matches (1,883 of 1,883, same indices)**. It meets **300 MHz** on the KV260 using **≈10% of the LUTs and no DSPs**, and it finishes one pair in **≈1.0 ms** (ORB-SLAM3 needs 5.0–6.5 ms on a 5 GHz Zen 4 laptop core; a Cortex-A53 will be slower still).

---

## 1. What "feature matching" means in ORB-SLAM3

After extraction, every frame has ~1000 keypoints (5000 during mono initialisation). Each keypoint has a position, a pyramid level (octave), an angle and a **256-bit rBRIEF descriptor**. Matching answers one question: *which keypoint in frame B is the same 3D point as keypoint i in frame A?*

ORB-SLAM3 never compares every pair. All its matchers are **guided**, with a different rule for picking candidates in each:

| Function (`src/ORBmatcher.cc`) | Used for | Candidates | Rules |
|---|---|---|---|
| **`SearchForInitialization`** | Mono init, frame 1 ↔ frame 2 | Window ±100 px around the previous match, level 0 only | best ≤ TH_LOW (50), ratio 0.9, one-to-one, orientation check |
| `SearchByProjection` (last frame) | Tracking every frame | Window around the projected map point, ±1 level | best ≤ TH_HIGH (100), best only, orientation check |
| `SearchByProjection` (local map) | Tracking every frame | Window sized by viewing angle | TH_HIGH, ratio 0.8 unless best and second are on different levels |
| `SearchByBoW` | Relocalisation, reference KF, loop closing | Same vocabulary node | TH_LOW, ratio 0.7/0.75, orientation check |
| `SearchForTriangulation` | Local mapping (new map points) | Same vocabulary node + epipolar check | TH_LOW, ratio 0.6 |

The core is the same everywhere: **gate candidates → Hamming distance → best and second best → threshold and ratio tests → (one-to-one) → rotation-consistency filter.** The kernel implements that core. `SearchForInitialization` is the first target because it is the cleanest and uses every part of the core.

---

## 2. The building blocks

### 2.1 Hamming distance

Distance = number of differing bits = `popcount(a XOR b)`, from 0 to 256. Two views of the same corner typically differ by 10–50 bits. Two random 256-bit strings differ by 128 ± 8 bits; real unrelated ORB descriptors are in that region too.

ORB-SLAM3 (`DescriptorDistance`) does eight 32-bit XORs plus the SWAR popcount trick:

```
v = v - ((v >> 1) & 0x55555555);                 // 16 x 2-bit counts
v = (v & 0x33333333) + ((v >> 2) & 0x33333333);  //  8 x 4-bit counts
v = (v + (v >> 4)) & 0x0F0F0F0F;                 //  4 x 8-bit counts
dist += (v * 0x01010101) >> 24;                  // sum of the 4 bytes
```

Each line is one level of an **adder tree**. The kernel uses the same three lines (on 64-bit words), but replaces the multiply with explicit additions: a multiplier would use DSPs, and an adder tree uses a few LUTs. One distance costs 256 XORs plus a 256-input popcount, all combinational, so it pipelines at one per clock.

### 2.2 Best, second best and the ratio test

For query *i*, scan the candidates and keep the smallest distance (`best`, with its index) and the second smallest (`best2`). Accept only if

- `best ≤ TH` (absolute quality), and
- `best < ratio × best2` (Lowe's test). If the runner-up is almost as good, the match is ambiguous: think of repeated textures like the ladders and pipes in the Machine Hall.

### 2.3 One-to-one rule (`vMatchedDistance`, `vnMatches21`)

A train keypoint can belong to only one query:

- A candidate *j* is skipped if it is already matched with a distance ≤ the current one.
- If query *i* wins *j* with a smaller distance, it **steals** *j*: the old query's match is cleared.

This makes the result depend on query order, which is why the kernel processes queries **one at a time, in order**. The parallelism lives *inside* a query.

### 2.4 Rotation-consistency histogram

Between two nearby frames, all true matches rotate by roughly the same in-plane angle. ORB-SLAM3 histograms `rot = angle1 − angle2`, keeps the **three tallest bins** (dropping the 2nd and 3rd if they are under 10% of the tallest), and throws away all other matches.

⚠ **Quirk you must reproduce.** `HISTO_LENGTH = 30` but `factor = 1/30`, so `bin = round(rot/30)`. Only **bins 0–12 are used, each 30° wide**, and 345–360° lands in bin 12 instead of wrapping to bin 0. It looks like a bug, but it is what ORB-SLAM3 does, and the kernel copies it. Matching the code rather than the intent is the rule for a golden model (Master Notes §6.2).

---

## 3. Hardware/software split

```
            ARM (Cortex-A53)                                  PL (orb_match_hls)
 ┌──────────────────────────────────────┐        ┌──────────────────────────────────────────┐
 │ keypoints + descriptors (2 frames)   │        │ 1 LOAD   train desc+meta -> 32 URAM banks │
 │ sort frame-2 by grid cell (ix,iy,idx)│  AXI   │ 2 MATCH  per query: scan [lo,hi),        │
 │ per query: grid columns -> [lo, hi)  │ ─────► │          8 lanes/clk: XOR+popcount+gate, │
 │ fixed-point pack, ratio table        │  DDR   │          best/2nd per lane -> merge ->   │
 │                                      │ ◄───── │          accept / steal                  │
 │ map indices back, use matches        │        │ 3 ORIENT 30-bin hist -> 3 maxima ->      │
 └──────────────────────────────────────┘        │          filter -> write match/dist      │
                                                 └──────────────────────────────────────────┘
```

**Why the ARM does the grid.** ORB-SLAM3 puts keypoints into a 64×48 grid, and `GetFeaturesInArea` visits only the cells under the window. Building variable-length cell lists in hardware is awkward. The trick used here: **sort the train keypoints by (cell_x, cell_y, index)**. Then

1. all cells of grid columns `min_cx … max_cx` form **one contiguous index range** `[lo, hi)`, so the kernel scans a simple range and its exact window test removes the extra rows;
2. the scan order equals ORB-SLAM3's visit order (ix outer, iy inner, index inside a cell), so **even ties resolve to the same index**.

The sort is O(N) (counting sort) and the range costs two table lookups per query, which is cheap on the ARM.

The same kernel also does **brute force** (eSLAM style): `lo = 0, hi = n_train`, no window, all levels.

---

## 4. Kernel micro-architecture (`orb_match_hls.c`)

### 4.1 Lanes and banking

- `ORB_LANES = 8`: eight train descriptors are compared per clock.
- The train arrays are **cyclically partitioned by 8**, so lane *l* owns indices *l, l+8, l+16, …*. Each clock reads row *k* from every bank (indices 8k…8k+7) with no port conflicts.
- Each lane keeps **its own** best, second and index. The loop-carried path is one compare plus a mux, which is easy at II=1 and 300 MHz. After the scan, the 8 lane results are merged: lowest distance wins, ties go to the lower index, and second = the second smallest overall. Because each lane sees its indices in increasing order with a strict `<`, this gives the same result as a sequential scan.
- Partial first and last rows (when `lo` and `hi` are not multiples of 8) are masked by the gate.

### 4.2 Gate (all per lane, combinational)

```
ok = (lo <= j < hi) && valid[j] && min_lvl <= lvl[j] <= max_lvl
     && |x[j]-qx| < r && |y[j]-qy| < r            // exact GetFeaturesInArea test
     && (!unique || d < matched_dist[j])          // one-to-one rule
```

### 4.3 Memory: why URAM

8 lanes × 256 bits = **2048 bits per clock** must come out of the descriptor RAM, so it needs 32 banks of 64 bits **whatever the depth**:

| Storage | Cost of the 32 descriptor banks | Note |
|---|---|---|
| BRAM18 | 128 (44% of the K26's BRAM) | First synthesis: 227 BRAM18 in total = 78%, leaving no room for the extractor |
| **URAM** | **32 (+8 for metadata)** | Each URAM is 4096 deep, so the same 32 URAMs would hold up to 32k descriptors |

The ORB extractor's line buffers need BRAM (Master Notes §6.4), so the matcher uses UltraRAM, which nothing else in the design wants yet. Set `-DORB_TRAIN_URAM=0` to go back to BRAM.

### 4.4 Fixed point (everything integer, so HW = golden model bit for bit)

| Quantity | Format | Why it is safe |
|---|---|---|
| x, y | signed Q11.8 (1/256 px), 20 bits | Window test differs from float only within 1/512 px of the edge |
| Angle | 16-bit binary angle (65536 = 360°) | Subtraction wraps for free; bin = `(rot16·12 + 32768) >> 16`, checked against the float formula for all 65,536 values |
| Ratio test | **257-entry limit table** `lim[best2]` | See the trap below |
| Three maxima | `10·max2 < max1` | Checked against `max2 < 0.1f·max1` for every count up to 8192 |

⚠ **The 0.6f trap.** The obvious integer ratio test `best·10 < 6·best2` is **wrong** for ORB-SLAM3's default ratio 0.6. `0.6f` is really 0.60000002, and after float rounding `15 < 25·0.6f` is *true* while some other exact-0.6 pairs come out false: 18 disagreements in total. 0.7, 0.75, 0.8 and 0.9 happen to be fine. The robust fix is to let the ARM evaluate the real float expression once per possible `best2` (0…256) and pass the table. That costs one tiny RAM and works for any ratio. Lesson: **test every float-to-integer rewrite exhaustively**. The domain here is small enough to do it in milliseconds.

### 4.5 Cycle model

```
cycles ≈ 5·n_train            (load train)
       + Σ_valid (⌈range/8⌉ + ~20)   (scan + per-query overhead)
       + ~4 per invalid query + ~6 per query (fetch) + 2 per query (write back)
```

For mono init on EuRoC: ~4800 train, ~4800 queries of which ~1087 are level 0, average range ≈1330 candidates (≈166 rows).

---

## 5. Verification

| Level | What | Result |
|---|---|---|
| Arithmetic | Every float→int rewrite vs ORB-SLAM3's float code, exhaustive | Identical (ratio via table; integer 6/10 would have failed 18 cases) |
| Unit/random | 310 random cases: edge sizes (0, 1, 7, 8, 9, 256, 257 …), random ranges unaligned to lanes, window on/off, unique on/off, orientation on/off, all ratios, max sizes 8192 | Kernel = golden model, bit-exact |
| Real data | 5 EuRoC MH_01 pairs (offsets 1–10 frames), keypoints from **ORB-SLAM3's own extractor**, answer from **ORB-SLAM3's own `SearchForInitialization`** | **1,883 / 1,883 identical** (310, 40, 696, 314, 523), 0 extra, 0 missing |
| Sanitizers | ASan + UBSan on the whole testbench | Clean |
| Vitis C-sim | Same testbench inside Vitis HLS 2025.2 | Pass |
| C/RTL co-sim | Generated Verilog in xsim: quick suite + one real pair | **Pass**: RTL gives the same 314 matches on pair `mh01_1500_10` |

The golden-data tool (`sw/orbslam3/dump_orbslam3.cc`) links `libORB_SLAM3.so` and builds real `Frame` objects with the EuRoC calibration. The reference is therefore ORB-SLAM3 itself, not a re-implementation of it. (It must be compiled with `-march=native`, like the library; otherwise Eigen members inside `Frame` get a different alignment and the constructor segfaults.)

---

## 6. Results (KV260 / K26, `xck26-sfvc784-2LV-c`, 300 MHz target)

| | Value |
|---|---|
| Timing, post-route OOC | Met: WNS +0.122 ns (fmax ≈ 311 MHz) |
| LUT | 11,103 (9.5%) |
| FF | 11,022 (4.7%) |
| BRAM36 tiles | 30 of 144 (21%) |
| URAM | 40 of 64 (62.5%) |
| DSP | 0 |
| Latency, one init pair (co-sim) | 300,781 cycles = **1.00 ms** (pair `mh01_1500_10`: 5003 queries × 5002 train). Co-sim memory has ideal latency, so expect a little more on the board |
| ORB-SLAM3 `SearchForInitialization`, Ryzen 7 8840HS | 5.0–6.5 ms (median of 50 runs) |

HLS's own estimate said 32k LUTs. Vivado's real figure is about a third of that, because HLS costs every 64-bit SWAR add as a full carry chain. **Always quote post-synthesis or post-route numbers in the report**, not `csynth`.

**Putting the speed-up in context (Master Notes §11, "compare like with like").** The fair baseline is the KV260's own A53 running ORB-SLAM3, which we have not measured yet. Also, init matching runs only until the map is initialised. The per-frame cost that matters for FPS is `SearchByProjection`, where windows are ~15 px instead of 100 px and frames have 1000 keypoints: about 3–4 grid columns ≈ 60 candidates ≈ 8 rows + ~20 cycles overhead ≈ 0.1 µs per query. That is fast, but so is the CPU there. The bigger structural win is that **descriptors never have to leave the PL** if the extractor writes them into the matcher's URAM directly.

---

## 7. What is not done yet (next steps, in priority order)

1. **On-board test**: block design with Zynq PS + this IP on one HP port, driver from the generated `xorb_match_hls_hw.h` (or PYNQ `MMIO` + `allocate`), and timing of A53-only vs PL on the same pairs. That gives the CPU-vs-FPGA row for the report.
2. **`SearchByProjection` mode** (the per-frame matcher): add a "best only" flag, the level-aware ratio rule (`bestLevel == bestLevel2`), and per-candidate "already has a map point" masking. The stereo `ur` check can stay on the ARM or become one more gate.
3. **BoW matching** maps onto the same kernel: sort train by vocabulary node, and then each query's candidates are one contiguous range.
4. **Throughput**: double-buffer the query fetch (needs C++ `hls::stream`/dataflow), widen the AXI port to 128/256 bits (the load is 5 cycles per train keypoint now), and 16 lanes if needed (LUTs scale linearly; URAM stays at 32 because of bandwidth).
5. **Integration with the extractor**: stream descriptors from the rBRIEF unit straight into the train banks.

---

## Check yourself

1. Why are queries processed sequentially while the train scan is parallel? Which ORB-SLAM3 rule forces this?
2. How does sorting train keypoints by grid cell let a single `[lo, hi)` range replace `GetFeaturesInArea`, and why does it also reproduce the tie-breaking?
3. Why does each lane keep its own best/second instead of updating one global pair every clock?
4. Why does 8 lanes × 256 bits force 32 RAM banks, and why is URAM the better home here?
5. Why can't `best < 0.6f * best2` be replaced by `10*best < 6*best2`? How would you discover that kind of bug?
6. Which bins does ORB-SLAM3's rotation histogram actually use, and why did we copy that behaviour?
7. What would you change to support `SearchByProjection`?
