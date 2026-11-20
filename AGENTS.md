# VslamCihuyy: FPGA-accelerated VSLAM (SystemVerilog), integration stage

## Current stage
- The individual .sv modules already exist. The job now: integrate them, build testbenches, verify, and find errors.
- Board: Kria [EDIT: KV260 / KR260], K26 SOM. PL = my SystemVerilog, PS = ARM.
- OS: Windows, PowerShell only (no bash, no make).
- Do not redo earlier work (research, algorithm design, module specs). Do not rewrite or refactor working modules.

## First session: build the integration map
Before connecting anything, read the .sv sources (only rtl/ and tb/, not the whole repo) and write docs/integration.md:
- For every module: path, parameters, ports (direction, width, signed/unsigned, fixed-point format), clock/reset,
  handshake protocol, latency, throughput. Take everything from the source. Mark unknowns as "UNKNOWN", never guess.
- The intended data path from top input to top output, and which module feeds which.
- Existing testbenches and whether they pass today (run them, show the tail of the output).
Then show me a 10-line summary and the list of UNKNOWNs. Keep docs/integration.md updated as integration proceeds.

## Unspecified specs (decided later)
Camera/sensor, resolution, frame rate, interface, IMU, dataset, clock frequency, and some fixed-point formats may still
be undecided. Never assume them. Keep them as parameters. If a task needs one, use a labeled provisional default,
record it in the "Open parameters" table in docs/integration.md, tell me in one line, and continue.
Ask only if the work would be wasted without my decision, one question at a time.

## SystemVerilog rules
- Match the existing code style and conventions. Do not restyle files you were not asked to change.
- New code: `default_nettype none` at the top of each file (and `default_nettype wire` at the end),
  `logic` instead of reg/wire, `always_ff` / `always_comb`, `typedef enum` for FSM states.
- Non-blocking (<=) in always_ff, blocking (=) in always_comb. Synchronous active-high reset unless the repo uses otherwise.
- No latches: assign every signal in every branch of always_comb.
- Shared types, widths and constants go in a package (rtl/pkg/), compiled first. No magic numbers.
- Connect ports by name (.port(sig)), never by position. No implicit nets. Check width and signedness of every connection.
- Interfaces, structs, and SVA only if the existing code already uses them and the simulator supports them.
  Otherwise use plain ports and `if (...) $error` checks in testbenches.
- No #delays or initial blocks in synthesizable code. Comment the fixed-point format (Qm.n) on datapath signals.
- Choose saturate or wrap explicitly wherever a result can overflow.
- One clock domain per module. Clock crossings only through a documented synchronizer or async FIFO.
- Streaming handshake (AXI4-Stream or the repo's own): valid never depends on ready; data stable while valid && !ready.

## Integration workflow (in order)
1. Pick the next two modules (or subsystem) to connect, according to the data path in docs/integration.md.
2. Check compatibility on paper first: widths, signedness, fixed-point format, bit/pixel/coordinate order, handshake,
   latency, frame/line boundaries (sof/eol/tlast), reset. Report any mismatch before writing the glue.
3. Write the integration wrapper in rtl/. Add delay registers where parallel paths have different latency.
4. Write a testbench tb/<name>_tb.sv, self-checking: directed cases, random data, back-pressure, reset mid-frame,
   boundary values, image edges, empty and full buffers. Use a golden model or reference vectors from sw/ when they
   exist, otherwise a simple reference model inside the testbench. State the tolerance.
5. Simulate and fix in a loop. After 3 failed attempts on the same error, stop and report hypotheses.
6. Grow in stages: pairwise, then subsystem, then top. Run the full regression after every stage.
7. Review with the error checklist below, report findings, fix them, update docs/integration.md.

## Commands (PowerShell)
Keep compile order in scripts/rtl.f (packages first, then modules in dependency order). Never rely on wildcards for order.
Fast: iverilog -g2012 -Wall -s <tb_top> -o sim\<tb>.out -c scripts\rtl.f tb\<tb>.sv ; vvp sim\<tb>.out
If Icarus rejects valid SystemVerilog, do not rewrite the design to suit it. Switch to Vivado xsim (from sim\ so outputs stay there):
  xvlog -sv -f ..\scripts\rtl.f ..\tb\<tb>.sv ; xelab <tb_top> -s snap -debug typical ; xsim snap -runall
  (Vivado bin path: [EDIT]. If xsim is license-limited for big designs, say so and fall back to Icarus or Verilator.)
- Testbenches print `TB_RESULT: PASS` or `TB_RESULT: FAIL` as the last line, then $finish.
- scripts/run_all.ps1 builds and runs every testbench, prints one PASS/FAIL line each and a total. Create it if missing.
- Fix all -Wall / lint warnings in code you touch. Run Vivado synthesis or implementation only when I ask.

## Error checklist (apply on every integration and review)
- Port connections: width mismatch, signed/unsigned mix, swapped ports, unconnected ports, wrong bit/pixel order.
- Fixed-point: different Q formats joined without alignment, truncation vs rounding, overflow, saturate vs wrap.
- Timing of data: latency mismatch between parallel paths, off-by-one frames/lines/pixels, first and last pixel, window borders.
- Control: reset values and mid-frame reset, valid/ready deadlock, dropped or duplicated data under back-pressure,
  tlast/sof/eol misplaced, FIFO/buffer overflow or underflow, depth too small for the worst case.
- Clock domains: crossings without synchronizers, multi-bit crossings without handshake or gray code.
- RTL hygiene: latches, missing defaults, blocking/non-blocking misuse, incomplete case statements, X propagation at startup.
- Memory: line buffers and BRAM inferable, read latency accounted for.
- Timing risk: long combinational paths between modules (suggest pipeline registers). DDR bandwidth sanity check once camera spec is known.
- VSLAM-specific: feature ordering differs from software, descriptor bit order, coordinate conventions and units,
  timestamp or frame sync, precision loss that changes the estimated pose.
- Testbench quality: only the happy path, no check on outputs, passing because the check never runs, expected values
  computed by the same logic as the design.

## Honesty and boundaries
- Never say something works without simulation output from this session.
- Do not edit a testbench to make it pass unless the testbench is wrong; say so when you do.
- Do not invent pin names, register addresses, IP parameters, or device resources. Read them from constraints/ or ask.
- Do not delete files, change constraints, or rewrite working modules without asking.
- Small diffs, one integration step per task.
- Finish every task with: files changed, commands run, pass/fail counts, what was NOT verified
  (synthesis, timing, hardware, end-to-end metrics), and open risks.
- Report resource and timing numbers only from Vivado reports. Never invent measurements.

## Token efficiency
- Read only the files needed. Name files explicitly; do not scan the repo.
- Never print large logs or waveforms. Use `| Select-Object -Last 40` on simulator output.
- Never open VCD, bitstream, or Vivado project files, or large datasets.
- Keep replies short unless I ask for detail.

## Compact instructions
When compacting, keep: module list and interfaces (docs/integration.md), open parameters, the current integration step,
failing test output, open risks.
