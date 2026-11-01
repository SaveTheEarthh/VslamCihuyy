# VslamCihuyy: FPGA-accelerated ORB front-end for VSLAM

ORB feature extraction and matching in the PL of a **Kria KV260** (`xck26-sfvc784-2LV-c`);
tracking, local BA and loop closing stay on the ARM in ORB-SLAM3.
ORB-SLAM3's code is the golden model: every hardware stage must be bit-exact against it.

## Layout

| Path | What |
|---|---|
| `hw/<stage>/` | One folder per pipeline stage, all with the same layout (below) |
| `hw/frontend_top/` | Integration: stages wired together, end-to-end testbench |
| `common/rtl/`, `common/model/` | Types, parameters and helpers shared by every stage |
| `docs/interfaces.md` | **Integration contract**: extractor-output record and frame header |
| `docs/figures/`, `docs/notes/`, `docs/results/` | Diagrams, theory notes, post-route and accuracy results |
| `sw/orbslam3/` | ORB-SLAM3 patches, golden-data dump tools, build/run scripts |
| `sw/host/` | ARM-side driver for the KV260 |
| `impl/` | Vivado constraints (`.xdc`) and implementation scripts |
| `data/frames/` | Small shared test frames (see `data/README.md`) |
| `tools/` | Dataset fetch, evaluation and other helper scripts |

Stages and owners:

| Stage | Owner |
|---|---|
| `hw/pyramid/` | Timothy |
| `hw/fast/` | Fadhly |
| `hw/gaussian/` | Fadhly |
| `hw/orientation/` | Raegrand |
| `hw/rbrief/` | Kenny |
| `hw/matcher/` | Zaki |
| `hw/frontend_top/` | TBD |

Inside every `hw/<stage>/`:

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model (Python or C) |
| `sim/` | `run.sh` (plus `.do`/`.ps1` if needed): generates vectors into `build/`, simulates, exits 0 only if the RTL is bit-exact with the model |

## Rules

1. **Never commit generated files.** Simulator libraries, waveforms, vector dumps and build
   output go in `build/` (ignored). Commit the script that makes them.
2. **Lowercase file and folder names.** `Verilog/` and `verilog/` are different folders on Linux
   and the same folder on Windows.
3. **Test on the shared frames in `data/frames/`** (EuRoC, 600×350, the monocular input size) so
   one stage's output can feed the next.
4. **Stay inside your own `hw/<stage>/`.** Changes to `common/` or `docs/interfaces.md` affect
   everyone, so agree on them first.
5. **One branch per feature, merged into `main` by pull request.**
6. **No datasets or PDFs in the repo.** Report post-route numbers, not `csynth` estimates.
