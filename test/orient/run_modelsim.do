# ModelSim / Questa script for the orientation-module testbench.
#   GUI console :  do C:/path/to/project/test/orient/run_modelsim.do
#   Batch       :  vsim -c -do "do run_modelsim.do"
# Optional (set before running):  set GAP 40    -> 40 % random bubbles on pix_valid

set here [file dirname [file normalize [info script]]]
cd $here                                   ;# testbench reads img.hex etc. from this folder
if {![info exists GAP]} { set GAP 0 }

# Vectors are already here.  To regenerate from the Python model (python must be on PATH):
# exec python gen_vectors.py

catch {quit -sim}                          ;# close a previous simulation, if any
if {![file exists work]} { vlib work }
vmap work work

set rtl ../../src/verilog
vlog -sv -work work +incdir+$rtl +incdir+. \
    $rtl/pipelined_adder_tree.sv $rtl/orient_linebuf.sv $rtl/orient_colsum.sv \
    $rtl/orient_moments.sv $rtl/orient_quantizer.sv $rtl/orient_core.sv \
    $rtl/kp_fifo.sv $rtl/kp_match.sv $rtl/orient_top.sv tb_orient.sv

vsim -voptargs=+acc -onfinish stop work.tb_orient +gap=$GAP

if {![batch_mode]} {
    add wave -divider {pixel input}
    add wave -radix unsigned /tb_orient/clk /tb_orient/pix_valid /tb_orient/pix_sof /tb_orient/pix_data
    add wave -divider {moment registers (inside orient_moments)}
    add wave -radix decimal /tb_orient/dut_core/u_mo/valid_out /tb_orient/dut_core/u_mo/r00 \
        /tb_orient/dut_core/u_mo/r10 /tb_orient/dut_core/u_mo/r01
    add wave -divider {core result: every 37x37 window}
    add wave -radix decimal /tb_orient/c_valid /tb_orient/c_x /tb_orient/c_y /tb_orient/c_m00 \
        /tb_orient/c_m10 /tb_orient/c_m01 /tb_orient/c_quad /tb_orient/c_sec
    add wave -divider {top result: keypoints only}
    add wave -radix decimal /tb_orient/t_valid /tb_orient/t_x /tb_orient/t_y /tb_orient/t_quad /tb_orient/t_sec
    add wave -divider {checker}
    add wave -radix decimal /tb_orient/n_core /tb_orient/err_core /tb_orient/n_top /tb_orient/err_top
}

run -all

if {[batch_mode]} { quit -f } else { wave zoom full }
