# Out-of-context synthesis + place & route on the K26 at 300 MHz.
#   vivado -mode batch -source vivado_ooc.tcl -tclargs <rtl dir> <out dir> [top] [clock port]
# Used for both the HLS output (default top orb_match_hls) and rtl/ (orb_match_top aclk).
lassign $argv rtl out top clk
if {$top eq ""} { set top orb_match_hls }
if {$clk eq ""} { set clk ap_clk }
file mkdir $out
create_project -in_memory -part xck26-sfvc784-2LV-c
set v [glob -nocomplain $rtl/*.v]
if {[llength $v]} { read_verilog $v }
# SystemVerilog: packages first
set sv [lsort [glob -nocomplain $rtl/*.sv]]
set pkgs [lsearch -all -inline $sv *_pkg.sv]
set rest [lsearch -all -inline -not $sv *_pkg.sv]
if {[llength $sv]} { read_verilog -sv [concat $pkgs $rest] }
foreach f [glob -nocomplain $rtl/*.dat] { add_files $f }
synth_design -top $top -mode out_of_context -flatten_hierarchy rebuilt
create_clock -name clk -period 3.333 [get_ports $clk]
report_utilization -file $out/util_synth.rpt
opt_design
place_design
phys_opt_design
route_design
report_utilization -file $out/util_impl.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file $out/util_hier.rpt
report_timing_summary -file $out/timing.rpt
report_timing -max_paths 20 -nworst 1 -file $out/top_paths.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set fh [open $out/summary.txt w]
puts $fh "post-route @ 3.333 ns (300 MHz): WNS = $wns ns  -> fmax ~ [format %.0f [expr {1000.0 / (3.333 - $wns)}]] MHz"
foreach {name pat} {LUT "CLB LUTs" FF "CLB Registers" BRAM "Block RAM Tile" URAM "URAM" DSP "DSPs"} {
    set line [lsearch -inline -regexp [split [report_utilization -return_string] "\n"] "^\\| $pat"]
    puts $fh "$name: [string trim $line]"
}
close $fh
