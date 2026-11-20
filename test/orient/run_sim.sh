#!/bin/sh
# Works from any directory:  sh test/orient/run_sim.sh   (extra args go to gen_vectors.py)
set -e
cd "$(dirname "$0")"                      # the testbench reads img.hex etc. from here
python3 gen_vectors.py "$@"               # writes vectors here, orient_tan_table.svh to src/verilog
iverilog -g2012 -Wall -o sim_orient -c files.f
vvp sim_orient                            # back-to-back pixels
vvp sim_orient +gap=40                    # 40 % random bubbles
