#!/usr/bin/env bash
# Run run_hls.tcl with Vitis 2025.2 inside the xilinx-box distrobox.
# Vitis refuses paths with spaces, so sources are copied to $BUILD first.
#   HLS_STEPS="csim csynth" ./run_hls.sh   to run only some steps.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
XILINX=${XILINX:-$HOME/ContainerApps/xilinx/2025.2}
BUILD=${BUILD:-$HOME/vslam_ws/hls/orb_matcher}
if [ -z "${CONTAINER_ID:-}" ]; then
    exec distrobox enter xilinx-box -- env HLS_STEPS="${HLS_STEPS:-}" BUILD="$BUILD" bash "$0" "$@"
fi
[ -z "${HLS_STEPS:-}" ] && unset HLS_STEPS
mkdir -p "$BUILD/data"
# flatten hls/ model/ tb/ into $BUILD: run_hls.tcl expects everything next to it
cp "$HERE"/*.c "$HERE"/*.h "$HERE/run_hls.tcl" "$BUILD/"
cp "$HERE"/../model/*.c "$HERE"/../model/*.h "$HERE"/../tb/tb_orb_match.c "$BUILD/"
cp "$HERE"/../sim/data/*.bin "$BUILD/data/" 2>/dev/null || true
cp "$HERE"/../../../impl/scripts/vivado_ooc.tcl "$BUILD/"
source "$XILINX/Vitis/settings64.sh"
cd "$BUILD"
vitis-run --mode hls --tcl "$BUILD/run_hls.tcl" 2>&1 | tee hls.log
if [[ " ${HLS_STEPS:-impl} " == *" impl "* ]]; then
    vivado -mode batch -nojournal -log vivado_ooc.log -source "$BUILD/vivado_ooc.tcl" \
        -tclargs "$BUILD/orb_match_prj/sol_kv260/syn/verilog" "$BUILD/ooc" > /dev/null
    cat "$BUILD/ooc/summary.txt"
fi
echo "reports: $BUILD/orb_match_prj/sol_kv260/syn/report  $BUILD/ooc"
