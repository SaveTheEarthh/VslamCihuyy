# ModelSim / Questa script for the full ORB frontend system testbench (tb_system_top).
#   GUI console :  do D:/ACADS/Semester5/FPGA Drone VSLAM/VslamCihuyy/hw/frontend_top/sim/run_modelsim.do
#   Batch       :  vsim -c -do "do run_modelsim.do"
# Optional (set before running):
#   set REGEN 1                      ;# force regenerate golden vectors
#   set IMG   input_custom.jpg       ;# image used when generating vectors

set here [file dirname [file normalize [info script]]]
cd $here

if {![info exists IMG]}   { set IMG "input_custom.jpg" }
if {![info exists REGEN]} { set REGEN 0 }

if {$REGEN || ![file exists "sys_expected_counts.hex"]} {
    if {![file exists $IMG]} { set IMG "../../rbrief/sim/tes_gambar.jpg" }
    exec python ../model/orb_system_golden.py gen --input $IMG --outdir .
}

catch {quit -sim}
if {![file exists work]} { vlib work }
vmap work work

set cfg ../../../common/rtl
set pyr ../../pyramid/rtl
set fst ../../fast/rtl
set gau ../../gaussian/rtl
set ori ../../orientation/rtl
set rbr ../../rbrief/rtl
set top ../rtl

vlog -sv -work work +incdir+$ori \
    $cfg/vslam_cfg_pkg.sv \
    $pyr/orb_pyr_scaler.sv \
    $pyr/orb_image_pyramid.sv \
    $fst/orb_window7.sv \
    $fst/orb_fast_core.sv \
    $fst/orb_nms3x3.sv \
    $gau/orb_gauss7.sv \
    $ori/pipelined_adder_tree.sv \
    $ori/orient_linebuf.sv \
    $ori/orient_colsum.sv \
    $ori/orient_moments.sv \
    $ori/orient_quantizer.sv \
    $ori/orient_core.sv \
    $ori/kp_fifo.sv \
    $ori/kp_match.sv \
    $ori/orient_top.sv \
    $rbr/trig_decoder.sv \
    $rbr/rbrief.sv \
    $rbr/rbrief_top.sv \
    $top/orb_frontend_level.sv \
    $top/orb_extractor_level.sv \
    $top/orb_frontend_top.sv \
    ../tb/tb_system_top.sv

vsim -voptargs=+acc -onfinish stop work.tb_system_top

if {![batch_mode]} {
    add wave -divider {input & control}
    add wave -radix unsigned /tb_system_top/clk /tb_system_top/rst /tb_system_top/in_valid /tb_system_top/in_pixel /tb_system_top/busy
    add wave -divider {pyramid streams}
    add wave -radix unsigned /tb_system_top/dut/p1_valid /tb_system_top/dut/p1_pixel \
        /tb_system_top/dut/p2_valid /tb_system_top/dut/p2_pixel \
        /tb_system_top/dut/p3_valid /tb_system_top/dut/p3_pixel
    add wave -divider {L0 intermediate & features}
    add wave -radix unsigned /tb_system_top/l0_g_valid /tb_system_top/l0_g_pixel \
        /tb_system_top/l0_kp_valid /tb_system_top/l0_kp_data \
        /tb_system_top/l0_feat_valid /tb_system_top/l0_feat_x /tb_system_top/l0_feat_y \
        /tb_system_top/l0_feat_score /tb_system_top/l0_feat_quad /tb_system_top/l0_feat_sector
    add wave -divider {L1..L3 feature valids}
    add wave -radix unsigned /tb_system_top/l1_feat_valid /tb_system_top/l2_feat_valid /tb_system_top/l3_feat_valid
    add wave -divider {unified arbitrated output}
    add wave -radix unsigned /tb_system_top/feat_valid /tb_system_top/feat_level \
        /tb_system_top/feat_x /tb_system_top/feat_y /tb_system_top/feat_score \
        /tb_system_top/feat_quad /tb_system_top/feat_sector
    add wave -radix hexadecimal /tb_system_top/feat_desc
    add wave -divider {checker}
    add wave -radix decimal /tb_system_top/lvl_cnt /tb_system_top/lvl_err \
        /tb_system_top/arb_total /tb_system_top/arb_err /tb_system_top/reset_leak_err
}

run -all

catch {exec python ../model/orb_system_golden.py plot --dir .}

if {[batch_mode]} { quit -f } else { wave zoom full }
