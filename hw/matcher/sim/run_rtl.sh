#!/usr/bin/env bash
# Simulate (and optionally implement) the SystemVerilog matcher with Vivado 2025.2.
#   ./run_rtl.sh            xsim: random tests + all golden EuRoC pairs
#   ./run_rtl.sh impl       ... then Vivado OOC synthesis + place & route
#   ORB_RANDOM=200 ./run_rtl.sh   more random tests;  ORB_QUICK=1: skip the EuRoC pairs
# Runs inside the xilinx-box distrobox; sources are copied to $BUILD (no spaces).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
XILINX=${XILINX:-$HOME/ContainerApps/xilinx/2025.2}
BUILD=${BUILD:-$HOME/vslam_ws/hls/orb_matcher_rtl}
if [ -z "${CONTAINER_ID:-}" ]; then
    exec distrobox enter xilinx-box -- env ORB_RANDOM="${ORB_RANDOM:-60}" ORB_QUICK="${ORB_QUICK:-}" BUILD="$BUILD" bash "$0" "$@"
fi
rm -rf "$BUILD"
mkdir -p "$BUILD/rtl" "$BUILD/data"
cp "$HERE"/../rtl/*.sv "$BUILD/rtl/"
cp "$HERE"/../tb/tb_top.sv "$HERE"/../tb/tb_dpi.c "$HERE"/../hls/orb_match.h "$HERE"/../model/orb_host.[ch] \
   "$HERE"/../model/orb_match_ref.c "$HERE"/../../../impl/scripts/vivado_ooc.tcl "$BUILD/"
cp "$HERE"/data/*.bin "$BUILD/data/"
source "$XILINX/Vivado/settings64.sh"
cd "$BUILD"

RTL="rtl/orb_match_pkg.sv rtl/orb_lane.sv rtl/orb_match_core.sv rtl/orb_match_top.sv"
xvlog -sv $RTL tb_top.sv > xvlog.log || { grep -E "ERROR|WARNING" xvlog.log; exit 1; }
xsc tb_dpi.c orb_host.c orb_match_ref.c --gcc_compile_options "-I$BUILD -O2" > xsc.log 2>&1 || { cat xsc.log; exit 1; }
xelab tb_top -sv_lib dpi --timescale 1ns/1ps -s tb_sim > xelab.log || { grep -E "ERROR|WARNING" xelab.log; exit 1; }
if [ -z "${ORB_QUICK:-}" ]; then export ORB_PAIRS="$(ls "$BUILD"/data/*.bin | tr "\n" " ")"; fi
xsim tb_sim -R > xsim.log
grep -E "^\[rtl\]|TESTS|FAILED|Error|ERROR" xsim.log

if [ "${1:-}" = "impl" ]; then
    vivado -mode batch -nojournal -log vivado_ooc.log -source vivado_ooc.tcl \
        -tclargs "$BUILD/rtl" "$BUILD/ooc" orb_match_top aclk > /dev/null
    cat "$BUILD/ooc/summary.txt"
fi
