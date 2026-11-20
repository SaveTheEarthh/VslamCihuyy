// ============================================================================
// vslam_cfg_pkg.sv
// Single source of truth for tunable VslamCihuyy system parameters.
// Compiled first in scripts/rtl.f and run_modelsim.do, and parsed directly at
// runtime by common/model/vslam_cfg.py for golden models and sw/host backend.
// Keep each parameter as a single-line localparam declaration.
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

package vslam_cfg_pkg;

    // ---- Production Image & Pyramid Frontend -------------------------------
    localparam int  IMG_W            = 640;
    localparam int  IMG_H            = 480;
    localparam int  PYR_LEVELS       = 4;     // RTL pyramid currently fixed to 4 levels L0..L3
    localparam int  PYR_ROUND        = 1;     // 1 = round-half-up, 0 = floor
    localparam real PYR_SCALE_FACTOR = 1.2;
    localparam int  FAST_THR         = 20;    // Q8.0 default FAST-16 corner threshold
    localparam int  ENABLE_EXTRACTOR = 1;

    // ---- Orientation, rBRIEF & Internal FIFOs ------------------------------
    localparam int  N_SECT           = 64;    // Must match trig_decoder.sv & orient_tan_table.svh
    localparam int  KP_FIFO_AW       = 9;
    localparam int  WIN_FIFO_AW      = 7;     // Production 128-deep 37x37x6b window FIFO
    localparam int  OUT_FIFO_AW      = 4;     // 16-deep per-level arbiter output FIFO
    localparam      PATTERN_FILE     = "vectors/pattern_rom.hex"; // Untyped string localparam for $readmemh
    localparam int  PATTERN_SEED     = 42;

    // ---- PL-to-PS Record Writer (Buffer-A Wire Protocol) -------------------
    localparam int          REC_AW       = 10;
    localparam logic [31:0] MAGIC_ORBK   = 32'h4B42_524F;
    localparam logic [15:0] FMT_VER      = 16'h0001;
    localparam int          HEADER_BYTES = 64;
    localparam int          RECORD_BYTES = 40;

    // ---- PS-to-PL Matcher (Buffer-B) & PS Backend --------------------------
    localparam int  BUF_B_GRID_W          = 64;
    localparam int  BUF_B_GRID_H          = 48;
    localparam int  BUF_B_COORD_INT_BITS  = 10;
    localparam int  BUF_B_COORD_FRAC_BITS = 8;
    localparam int  BACKEND_MAX_FEATURES  = 450;
    localparam int  MATCH_MAX_DIST        = 64;
    localparam real MATCH_RATIO           = 0.8;
    localparam int  ORIENT_HIST_BINS      = 30;
    localparam int  RANSAC_ITERS          = 600;
    localparam real RANSAC_THRESH_PX      = 1.5;
    localparam int  UNDISTORT_ITERS       = 8;

    // ---- Fast Simulation Defaults (tb_system_top / tb_record_writer) -------
    localparam int  SIM_IMG_W        = 144;
    localparam int  SIM_IMG_H        = 108;
    localparam int  SIM_WIN_FIFO_AW  = 8;
    localparam int  SIM_MAX_FEAT     = 1024;
    localparam int  SIM_TIMEOUT_CYC  = 200000;

endpackage

`default_nettype wire
