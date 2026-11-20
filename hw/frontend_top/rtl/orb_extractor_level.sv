// ============================================================================
// orb_extractor_level.sv
// Full single-level ORB feature extractor integrating:
//   1. orb_frontend_level : 7x7 window + FAST-16 + 3x3 NMS + 7x7 Gaussian
//   2. orient_top         : 37x37 intensity-centroid orientation (64 sectors)
//   3. orient_linebuf     : 37x37x6-bit quantized Gaussian window delay-aligned
//                           with orient_top output
//   4. kp_fifo + rbrief_top : buffered 37x37x6-bit window + trig_decoder + rBRIEF
//
// Latency / coordination:
//   * orb_frontend_level produces Gaussian pixels (G_W = IMG_W-6, G_H = IMG_H-6)
//     and NMS keypoints in full-level coords (offset by +3 from Gaussian coords).
//   * Coordinates are translated by (-3,-3) into Gaussian coords for orient_top,
//     then translated back (+3,+3) at feature output.
//   * rbrief_top processes 1 keypoint in 257 clocks; u_win_fifo absorbs burst
//     keypoints arriving from orient_top during raster scan.
//
// Synchronous active-high reset (`rst`), inverted internally for rst_n modules.
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module orb_extractor_level #(
    parameter int IMG_W        = vslam_cfg_pkg::IMG_W,
    parameter int IMG_H        = vslam_cfg_pkg::IMG_H,
    parameter int N_SECT       = vslam_cfg_pkg::N_SECT,
    parameter int KP_FIFO_AW   = vslam_cfg_pkg::KP_FIFO_AW,
    parameter int WIN_FIFO_AW  = vslam_cfg_pkg::WIN_FIFO_AW,
    parameter     PATTERN_FILE = vslam_cfg_pkg::PATTERN_FILE
)(
    input  wire         clk,
    input  wire         rst,

    // Level pixel stream (Q8.0 unsigned) + FAST threshold (Q8.0 unsigned)
    input  wire         in_valid,
    input  wire [7:0]   in_pixel,
    input  wire [7:0]   thr,

    // Intermediate frontend streams (Q8.0 Gauss, Q12.0 FAST score, 36-bit NMS kp)
    output wire         g_valid,
    output wire [7:0]   g_pixel,
    output wire         fs_valid,
    output wire [11:0]  fs_score,
    output wire         kp_valid,
    output wire [35:0]  kp_data,

    // Intermediate orientation outputs (level coordinates, Q12.0 unsigned)
    output wire         ori_valid,
    output wire [11:0]  ori_x,
    output wire [11:0]  ori_y,
    output wire [11:0]  ori_score,
    output wire [1:0]   ori_quad,
    output wire [3:0]   ori_sector,

    // Final ORB feature output (level coordinates Q12.0, 256-bit descriptor)
    output logic        feat_valid,
    output logic [11:0] feat_x,
    output logic [11:0] feat_y,
    output logic [11:0] feat_score,
    output logic [1:0]  feat_quad,
    output logic [3:0]  feat_sector,
    output logic [255:0] feat_desc,
    output wire         busy
);

    localparam int G_W      = IMG_W - 6;
    localparam int G_H      = IMG_H - 6;
    localparam int GCW      = $clog2(G_W);
    localparam int GRW      = $clog2(G_H);
    localparam int SEC_W    = $clog2(N_SECT / 4);
    localparam int COL_BITS = 37 * 6;              // 222 bits per 37-px column
    localparam int WIN_BITS = 37 * COL_BITS;       // 8214 bits per 37x37x6-bit window
    localparam int META_BITS = 12 + 12 + 12 + 2 + 4; // score(12), y(12), x(12), quad(2), sector(4) = 42

    logic rst_n;
    assign rst_n = ~rst;

    // ------------------------------------------------------------------------
    // 1. Frontend stage: 7x7 window, FAST-16, 3x3 NMS, 7x7 Gaussian
    // ------------------------------------------------------------------------
    orb_frontend_level #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H)
    ) u_fe (
        .clk      (clk),
        .rst      (rst),
        .in_valid (in_valid),
        .in_pixel (in_pixel),
        .thr      (thr),
        .g_valid  (g_valid),
        .g_pixel  (g_pixel),
        .fs_valid (fs_valid),
        .fs_score (fs_score),
        .kp_valid (kp_valid),
        .kp_data  (kp_data)
    );

    // ------------------------------------------------------------------------
    // 2. Gaussian raster coordinate & SOF generator (G_W x G_H)
    // ------------------------------------------------------------------------
    logic [GCW-1:0] g_col;
    logic [GRW-1:0] g_row;
    logic           g_sof;

    always_ff @(posedge clk) begin
        if (rst) begin
            g_col <= '0;
            g_row <= '0;
        end else if (g_valid) begin
            if (g_col == GCW'(G_W - 1)) begin
                g_col <= '0;
                g_row <= (g_row == GRW'(G_H - 1)) ? '0 : (g_row + 1'b1);
            end else begin
                g_col <= g_col + 1'b1;
            end
        end
    end

    assign g_sof = g_valid && (g_col == '0) && (g_row == '0);

    // ------------------------------------------------------------------------
    // 3. Keypoint coordinate translation (level -> Gaussian) & orient_top
    // ------------------------------------------------------------------------
    logic [11:0]    kp_raw_x, kp_raw_y, kp_raw_score;
    logic [11:0]    kp_gx_full, kp_gy_full;
    logic [GCW-1:0] kp_gx;
    logic [GRW-1:0] kp_gy;
    logic           kp_ok;
    logic           kp_in_ready;

    assign kp_raw_x     = kp_data[11:0];
    assign kp_raw_y     = kp_data[23:12];
    assign kp_raw_score = kp_data[35:24];
    assign kp_gx_full   = kp_raw_x - 12'd3;
    assign kp_gy_full   = kp_raw_y - 12'd3;
    assign kp_gx        = kp_gx_full[GCW-1:0];
    assign kp_gy        = kp_gy_full[GRW-1:0];

    // Match orient_top's internal 37x37 border filter so u_score_fifo stays in lockstep
    assign kp_ok = (kp_raw_x >= 12'd3) && (kp_raw_y >= 12'd3) &&
                   (kp_gx_full >= 12'd18) && (kp_gx_full <= (12'(G_W) - 12'd19)) &&
                   (kp_gy_full >= 12'd18) && (kp_gy_full <= (12'(G_H) - 12'd19));

    logic             ori_out_valid;
    logic [GCW-1:0]   ori_gx;
    logic [GRW-1:0]   ori_gy;
    logic [1:0]       ori_out_quad;
    logic [SEC_W-1:0] ori_out_sector;
    logic [11:0]      ori_out_score;
    logic             score_fifo_full_unused, score_fifo_empty_unused;

    orient_top #(
        .PIX_W  (8),
        .IMG_W  (G_W),
        .IMG_H  (G_H),
        .N_SECT (N_SECT),
        .KP_AW  (KP_FIFO_AW)
    ) u_orient (
        .clk         (clk),
        .rst_n       (rst_n),
        .pix_valid   (g_valid),
        .pix_sof     (g_sof),
        .pix_data    (g_pixel),
        .kp_in_valid (kp_valid && (kp_raw_x >= 12'd3) && (kp_raw_y >= 12'd3)),
        .kp_in_x     (kp_gx),
        .kp_in_y     (kp_gy),
        .kp_in_ready (kp_in_ready),
        .kp_flush    (1'b0),
        .out_valid   (ori_out_valid),
        .out_x       (ori_gx),
        .out_y       (ori_gy),
        .out_quad    (ori_out_quad),
        .out_sector  (ori_out_sector)
    );

    // Parallel score FIFO kept 1-to-1 with orient_top's internal kp_fifo
    kp_fifo #(
        .DW (12),
        .AW (KP_FIFO_AW)
    ) u_score_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .flush   (1'b0),
        .wr_en   (kp_valid && kp_ok && kp_in_ready),
        .wr_data (kp_raw_score),
        .full    (score_fifo_full_unused),
        .rd_en   (ori_out_valid),
        .rd_data (ori_out_score),
        .empty   (score_fifo_empty_unused)
    );

    assign ori_valid  = ori_out_valid;
    assign ori_x      = {{(12-GCW){1'b0}}, ori_gx} + 12'd3;
    assign ori_y      = {{(12-GRW){1'b0}}, ori_gy} + 12'd3;
    assign ori_score  = ori_out_score;
    assign ori_quad   = ori_out_quad;
    assign ori_sector = 4'(ori_out_sector);

    // ------------------------------------------------------------------------
    // 4. 37x37x6-bit Gaussian window buffer aligned with orient_top output
    //    g_pixel[7:2] (Q6.0) -> orient_linebuf (2 clk) -> 13-clk delay -> 37-col SR
    //    Total to win37_cols valid = 2 + 13 + 1 = 16 clocks (matches orient_top)
    // ------------------------------------------------------------------------
    logic                lb6_valid;
    logic [COL_BITS-1:0] lb6_col;
    logic                lb6_tag_unused;

    orient_linebuf #(
        .PIX_W (6),
        .IMG_W (G_W),
        .CW    (GCW),
        .TAG_W (1)
    ) u_win37_lb (
        .clk       (clk),
        .rst_n     (rst_n),
        .pix_valid (g_valid),
        .pix_data  (g_pixel[7:2]),         // Quantize Q8.0 -> Q6.0 for rBRIEF
        .pix_col   (g_col),
        .tag_in    (1'b0),
        .col_valid (lb6_valid),
        .col_data  (lb6_col),
        .tag_out   (lb6_tag_unused)
    );

    logic [12:0]         dly_v;
    logic [COL_BITS-1:0] dly_col [0:12];

    always_ff @(posedge clk) begin
        if (rst) begin
            dly_v <= '0;
            for (int i = 0; i < 13; i++) dly_col[i] <= '0;
        end else begin
            dly_v[0]   <= lb6_valid;
            dly_col[0] <= lb6_col;
            for (int i = 1; i < 13; i++) begin
                dly_v[i]   <= dly_v[i-1];
                dly_col[i] <= dly_col[i-1];
            end
        end
    end

    logic [COL_BITS-1:0] win37_cols [0:36];
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int c = 0; c < 37; c++) win37_cols[c] <= '0;
        end else if (dly_v[12]) begin
            for (int c = 0; c < 36; c++) win37_cols[c] <= win37_cols[c+1];
            win37_cols[36] <= dly_col[12];
        end
    end

    logic [WIN_BITS-1:0] win_cols_bus;
    genvar gc;
    generate
        for (gc = 0; gc < 37; gc++) begin : g_pack_cols
            assign win_cols_bus[gc*COL_BITS +: COL_BITS] = win37_cols[gc];
        end
    endgenerate

    // ------------------------------------------------------------------------
    // 5. Keypoint + 37x37 window FIFO -> rbrief_top controller
    // ------------------------------------------------------------------------
    localparam int WFIFO_W = META_BITS + WIN_BITS;

    logic [WFIFO_W-1:0] wfifo_wr_data, wfifo_rd_data;
    logic               wfifo_full, wfifo_empty, wfifo_pop;

    assign wfifo_wr_data = {ori_score, ori_y, ori_x, ori_quad, ori_sector, win_cols_bus};

    kp_fifo #(
        .DW (WFIFO_W),
        .AW (WIN_FIFO_AW)
    ) u_win_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .flush   (1'b0),
        .wr_en   (ori_out_valid),
        .wr_data (wfifo_wr_data),
        .full    (wfifo_full),
        .rd_en   (wfifo_pop),
        .rd_data (wfifo_rd_data),
        .empty   (wfifo_empty)
    );

    logic                rbrief_start;
    logic                rbrief_busy;
    logic                rbrief_done;
    logic [255:0]        rbrief_desc;
    logic [WIN_BITS-1:0] rbrief_window_flat;
    logic [11:0]         latched_x, latched_y, latched_score;
    logic [1:0]          latched_quad;
    logic [3:0]          latched_sector;

    logic [WIN_BITS-1:0] head_cols_bus;
    logic [META_BITS-1:0] head_meta;
    assign head_cols_bus = wfifo_rd_data[WIN_BITS-1:0];
    assign head_meta     = wfifo_rd_data[WFIFO_W-1:WIN_BITS];

    assign wfifo_pop = !wfifo_empty && !rbrief_busy && !rbrief_start && !rbrief_done;

    always_ff @(posedge clk) begin
        if (rst) begin
            rbrief_start       <= 1'b0;
            latched_x          <= '0;
            latched_y          <= '0;
            latched_score      <= '0;
            latched_quad       <= '0;
            latched_sector     <= '0;
            rbrief_window_flat <= '0;
        end else begin
            rbrief_start <= 1'b0;
            if (wfifo_pop) begin
                rbrief_start   <= 1'b1;
                latched_sector <= head_meta[3:0];
                latched_quad   <= head_meta[5:4];
                latched_x      <= head_meta[17:6];
                latched_y      <= head_meta[29:18];
                latched_score  <= head_meta[41:30];
                // Transpose from column-major (col c, row r) to row-major (r*37 + c)
                for (int r = 0; r < 37; r++) begin
                    for (int c = 0; c < 37; c++) begin
                        rbrief_window_flat[(r*37 + c)*6 +: 6] <= head_cols_bus[(c*37 + r)*6 +: 6];
                    end
                end
            end
        end
    end

    rbrief_top #(
        .PATTERN_FILE (PATTERN_FILE)
    ) u_rbrief (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (rbrief_start),
        .window_flat (rbrief_window_flat),
        .quadrant    (latched_quad),
        .sector      (latched_sector),
        .busy        (rbrief_busy),
        .done        (rbrief_done),
        .descriptor  (rbrief_desc)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            feat_valid  <= 1'b0;
            feat_x      <= '0;
            feat_y      <= '0;
            feat_score  <= '0;
            feat_quad   <= '0;
            feat_sector <= '0;
            feat_desc   <= '0;
        end else begin
            feat_valid <= rbrief_done;
            if (rbrief_done) begin
                feat_x      <= latched_x;
                feat_y      <= latched_y;
                feat_score  <= latched_score;
                feat_quad   <= latched_quad;
                feat_sector <= latched_sector;
                feat_desc   <= rbrief_desc;
            end
        end
    end

    // Track in-flight pixel pipeline stages (4 frontend + 16 orient + 2 margin = 22 clocks)
    logic [21:0] in_flight_sr;
    always_ff @(posedge clk) begin
        if (rst) in_flight_sr <= '0;
        else     in_flight_sr <= {in_flight_sr[20:0], in_valid};
    end

    assign busy = in_valid || (|in_flight_sr) || !wfifo_empty ||
                  rbrief_start || rbrief_busy || rbrief_done || feat_valid;

endmodule

`default_nettype wire
