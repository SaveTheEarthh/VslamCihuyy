# ModelSim script for the 37x37 patch test.
#   1) python patch_test.py gen        (writes patches.hex, patch_params.svh)
#   2) do run_modelsim_patch.do        (this script: writes rtl_results.txt)
#   3) python patch_test.py plot       (draws patch_visual.html / .svg)
# Optional:  set GAP 40   -> 40 % random bubbles on pix_valid
set here [file dirname [file normalize [info script]]]
cd $here
if {![info exists GAP]} { set GAP 0 }

catch {quit -sim}
if {![file exists work]} { vlib work }
vmap work work

set rtl ../../src/verilog
vlog -sv -work work +incdir+$rtl +incdir+. \
    $rtl/pipelined_adder_tree.sv $rtl/orient_linebuf.sv $rtl/orient_colsum.sv \
    $rtl/orient_moments.sv $rtl/orient_quantizer.sv $rtl/orient_core.sv \
    $rtl/kp_fifo.sv $rtl/kp_match.sv $rtl/orient_top.sv tb_patch.sv

vsim -voptargs=+acc -onfinish stop work.tb_patch +gap=$GAP
if {![batch_mode]} {
    add wave -radix unsigned /tb_patch/clk /tb_patch/pix_valid /tb_patch/pix_sof /tb_patch/pix_data
    add wave -radix decimal /tb_patch/res_valid /tb_patch/res_m00 /tb_patch/res_m10 /tb_patch/res_m01 \
        /tb_patch/res_quad /tb_patch/res_sector
}
run -all
if {[batch_mode]} { quit -f }
