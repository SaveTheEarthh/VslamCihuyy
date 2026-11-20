# Software build of the testbench (same sources Vitis HLS uses for C-sim).
#   make test      -> run all tests on the golden EuRoC pairs in data/
#   make hls       -> Vitis HLS csim + csynth + cosim (see ../hls/run_hls.tcl)
#   make rtl       -> SystemVerilog version in xsim (see run_rtl.sh)
CC     ?= gcc
CFLAGS ?= -O2 -std=c99 -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-label
CFLAGS += -I../hls -I../model
SRC     = ../hls/orb_match_hls.c ../model/orb_match_ref.c ../model/orb_host.c ../tb/tb_orb_match.c
PAIRS   = $(wildcard data/*.bin)

tb_orb_match: $(SRC) ../hls/orb_match.h ../model/orb_host.h
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

test: tb_orb_match
	./tb_orb_match $(PAIRS)

hls:
	../hls/run_hls.sh

rtl:
	./run_rtl.sh

clean:
	rm -f tb_orb_match

.PHONY: test hls rtl clean
