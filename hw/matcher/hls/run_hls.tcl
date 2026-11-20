# Vitis HLS flow for the ORB matcher, target KV260 (K26 SOM).
#   ./run_hls.sh   copies this script, ../model, ../tb and ../sim/data into one flat
#                  build folder and runs it there (it expects all sources next to it)
# Steps are selected with env HLS_STEPS (default "csim csynth cosim export").
set here   [file dirname [file normalize [info script]]]
set steps  [expr {[info exists ::env(HLS_STEPS)] ? $::env(HLS_STEPS) : "csim csynth cosim export"}]
set pairs  [lsort [glob -nocomplain $here/data/*.bin]]
set cflags "-std=c99 -I$here"

open_project -reset orb_match_prj
set_top orb_match_hls
add_files     $here/orb_match_hls.c -cflags $cflags
add_files -tb $here/orb_match_ref.c -cflags $cflags
add_files -tb $here/orb_host.c      -cflags $cflags
add_files -tb $here/tb_orb_match.c  -cflags $cflags

open_solution -reset sol_kv260 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 3.333 -name default   ;# 300 MHz
config_export -format ip_catalog -vendor itb.ac.id -library vslam -version 1.0 -display_name "ORB matcher"

if {"csim" in $steps}   { csim_design -argv [join $pairs " "] }
if {"csynth" in $steps} { csynth_design }
# co-sim: RTL simulation of the short test + one real EuRoC pair
if {"cosim" in $steps}  { cosim_design -argv "--quick [lindex $pairs 0]" -trace_level none }
# IP for Vivado's IP catalog. Real resource/timing numbers come from
# vivado_ooc.tcl (run by run_hls.sh): export_design -flow builds a block-design
# wrapper whose header Vivado 2025.2 corrupts inside the distrobox.
if {"export" in $steps} { export_design -format ip_catalog }
exit
