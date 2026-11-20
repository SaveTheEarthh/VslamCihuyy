// ============================================================================
// orb_frontend_top.sv
// Integrated ORB front-end top level:
//   * 4-level image pyramid (L0 = input stream, L1..L3 = cascaded 1.2x scalers)
//   * Per-level ORB extractor (ENABLE_EXTRACTOR=1):
//       orb_window7 + orb_fast_core + orb_nms3x3 + orb_gauss7
//       -> orient_top (64-sector intensity centroid) + 37x37x6-bit window buffer
//       -> rbrief_top (trig_decoder + 256-bit descriptor)
//   * Multi-level priority arbiter (L0 -> L1 -> L2 -> L3) producing a unified
//     feature stream (`feat_*`) for downstream Matcher / Record Writer.
//
// Setting ENABLE_EXTRACTOR=0 builds only the pyramid + FAST + NMS + Gaussian
// datapath (for fast standalone frontend verification).
//
// Synchronous active-high reset (`rst`).
// ============================================================================
`timescale 1ns/1ps
`default_nettype none

module orb_frontend_top #(
    parameter int IMG_W            = vslam_cfg_pkg::IMG_W,
    parameter int IMG_H            = vslam_cfg_pkg::IMG_H,
    parameter int ROUND            = vslam_cfg_pkg::PYR_ROUND,
    parameter int ENABLE_EXTRACTOR = vslam_cfg_pkg::ENABLE_EXTRACTOR,
    parameter int N_SECT           = vslam_cfg_pkg::N_SECT,
    parameter int KP_FIFO_AW       = vslam_cfg_pkg::KP_FIFO_AW,
    parameter int WIN_FIFO_AW      = vslam_cfg_pkg::WIN_FIFO_AW,
    parameter int OUT_FIFO_AW      = vslam_cfg_pkg::OUT_FIFO_AW,
    parameter     PATTERN_FILE     = vslam_cfg_pkg::PATTERN_FILE
)(
    input  wire         clk,
    input  wire         rst,

    input  wire         in_valid,
    input  wire [7:0]   in_pixel,      // Q8.0 unsigned grayscale input
    input  wire [7:0]   thr,           // Q8.0 unsigned FAST threshold

    // Level 0 intermediate streams
    output wire         l0_g_valid,   output wire [7:0]   l0_g_pixel,
    output wire         l0_fs_valid,  output wire [11:0]  l0_fs_score,
    output wire         l0_kp_valid,  output wire [35:0]  l0_kp_data,
    // Level 1 intermediate streams
    output wire         l1_g_valid,   output wire [7:0]   l1_g_pixel,
    output wire         l1_fs_valid,  output wire [11:0]  l1_fs_score,
    output wire         l1_kp_valid,  output wire [35:0]  l1_kp_data,
    // Level 2 intermediate streams
    output wire         l2_g_valid,   output wire [7:0]   l2_g_pixel,
    output wire         l2_fs_valid,  output wire [11:0]  l2_fs_score,
    output wire         l2_kp_valid,  output wire [35:0]  l2_kp_data,
    // Level 3 intermediate streams
    output wire         l3_g_valid,   output wire [7:0]   l3_g_pixel,
    output wire         l3_fs_valid,  output wire [11:0]  l3_fs_score,
    output wire         l3_kp_valid,  output wire [35:0]  l3_kp_data,

    // Per-level extracted features (level coordinates, angle bin, 256-bit descriptor)
    output wire         l0_feat_valid,
    output wire [11:0]  l0_feat_x,
    output wire [11:0]  l0_feat_y,
    output wire [11:0]  l0_feat_score,
    output wire [1:0]   l0_feat_quad,
    output wire [3:0]   l0_feat_sector,
    output wire [255:0] l0_feat_desc,

    output wire         l1_feat_valid,
    output wire [11:0]  l1_feat_x,
    output wire [11:0]  l1_feat_y,
    output wire [11:0]  l1_feat_score,
    output wire [1:0]   l1_feat_quad,
    output wire [3:0]   l1_feat_sector,
    output wire [255:0] l1_feat_desc,

    output wire         l2_feat_valid,
    output wire [11:0]  l2_feat_x,
    output wire [11:0]  l2_feat_y,
    output wire [11:0]  l2_feat_score,
    output wire [1:0]   l2_feat_quad,
    output wire [3:0]   l2_feat_sector,
    output wire [255:0] l2_feat_desc,

    output wire         l3_feat_valid,
    output wire [11:0]  l3_feat_x,
    output wire [11:0]  l3_feat_y,
    output wire [11:0]  l3_feat_score,
    output wire [1:0]   l3_feat_quad,
    output wire [3:0]   l3_feat_sector,
    output wire [255:0] l3_feat_desc,

    // Unified arbitrated feature stream (to Matcher / Record Writer)
    output logic         feat_valid,
    output logic [1:0]   feat_level,
    output logic [11:0]  feat_x,
    output logic [11:0]  feat_y,
    output logic [11:0]  feat_score,
    output logic [1:0]   feat_quad,
    output logic [3:0]   feat_sector,
    output logic [255:0] feat_desc,
    output logic         busy
);

    // Same size rule as orb_image_pyramid
    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam int W1 = osz(IMG_W);
    localparam int H1 = osz(IMG_H);
    localparam int W2 = osz(W1);
    localparam int H2 = osz(H1);
    localparam int W3 = osz(W2);
    localparam int H3 = osz(H2);
    localparam int FEAT_W = 12 + 12 + 12 + 2 + 4 + 256; // 298 bits

    // ---- Image pyramid -----------------------------------------------------
    wire       p1_valid, p2_valid, p3_valid;
    wire [7:0] p1_pixel, p2_pixel, p3_pixel;

    orb_image_pyramid #(
        .IMG_W (IMG_W),
        .IMG_H (IMG_H),
        .ROUND (ROUND)
    ) u_pyr (
        .clk       (clk),
        .rst       (rst),
        .in_valid  (in_valid),
        .in_pixel  (in_pixel),
        .l1_valid  (p1_valid), .l1_pixel (p1_pixel), .l1_acc (), .l1_eol (), .l1_eof (),
        .l2_valid  (p2_valid), .l2_pixel (p2_pixel), .l2_acc (), .l2_eol (), .l2_eof (),
        .l3_valid  (p3_valid), .l3_pixel (p3_pixel), .l3_acc (), .l3_eol (), .l3_eof ()
    );

    // ---- Per-level processing ----------------------------------------------
    generate
        if (ENABLE_EXTRACTOR != 0) begin : g_full_extractor
            logic [3:0]   lvl_fvalid;
            logic [3:0]   lvl_busy;
            logic [11:0]  lvl_fx      [0:3];
            logic [11:0]  lvl_fy      [0:3];
            logic [11:0]  lvl_fscore  [0:3];
            logic [1:0]   lvl_fquad   [0:3];
            logic [3:0]   lvl_fsector [0:3];
            logic [255:0] lvl_fdesc   [0:3];

            orb_extractor_level #(
                .IMG_W       (IMG_W),
                .IMG_H       (IMG_H),
                .N_SECT      (N_SECT),
                .KP_FIFO_AW  (KP_FIFO_AW),
                .WIN_FIFO_AW (WIN_FIFO_AW),
                .PATTERN_FILE(PATTERN_FILE)
            ) u_ext0 (
                .clk         (clk),
                .rst         (rst),
                .in_valid    (in_valid),
                .in_pixel    (in_pixel),
                .thr         (thr),
                .g_valid     (l0_g_valid),   .g_pixel    (l0_g_pixel),
                .fs_valid    (l0_fs_valid),  .fs_score   (l0_fs_score),
                .kp_valid    (l0_kp_valid),  .kp_data    (l0_kp_data),
                .ori_valid   (), .ori_x (), .ori_y (), .ori_score (), .ori_quad (), .ori_sector (),
                .feat_valid  (lvl_fvalid[0]),
                .feat_x      (lvl_fx[0]),
                .feat_y      (lvl_fy[0]),
                .feat_score  (lvl_fscore[0]),
                .feat_quad   (lvl_fquad[0]),
                .feat_sector (lvl_fsector[0]),
                .feat_desc   (lvl_fdesc[0]),
                .busy        (lvl_busy[0])
            );

            orb_extractor_level #(
                .IMG_W       (W1),
                .IMG_H       (H1),
                .N_SECT      (N_SECT),
                .KP_FIFO_AW  (KP_FIFO_AW),
                .WIN_FIFO_AW (WIN_FIFO_AW),
                .PATTERN_FILE(PATTERN_FILE)
            ) u_ext1 (
                .clk         (clk),
                .rst         (rst),
                .in_valid    (p1_valid),
                .in_pixel    (p1_pixel),
                .thr         (thr),
                .g_valid     (l1_g_valid),   .g_pixel    (l1_g_pixel),
                .fs_valid    (l1_fs_valid),  .fs_score   (l1_fs_score),
                .kp_valid    (l1_kp_valid),  .kp_data    (l1_kp_data),
                .ori_valid   (), .ori_x (), .ori_y (), .ori_score (), .ori_quad (), .ori_sector (),
                .feat_valid  (lvl_fvalid[1]),
                .feat_x      (lvl_fx[1]),
                .feat_y      (lvl_fy[1]),
                .feat_score  (lvl_fscore[1]),
                .feat_quad   (lvl_fquad[1]),
                .feat_sector (lvl_fsector[1]),
                .feat_desc   (lvl_fdesc[1]),
                .busy        (lvl_busy[1])
            );

            orb_extractor_level #(
                .IMG_W       (W2),
                .IMG_H       (H2),
                .N_SECT      (N_SECT),
                .KP_FIFO_AW  (KP_FIFO_AW),
                .WIN_FIFO_AW (WIN_FIFO_AW),
                .PATTERN_FILE(PATTERN_FILE)
            ) u_ext2 (
                .clk         (clk),
                .rst         (rst),
                .in_valid    (p2_valid),
                .in_pixel    (p2_pixel),
                .thr         (thr),
                .g_valid     (l2_g_valid),   .g_pixel    (l2_g_pixel),
                .fs_valid    (l2_fs_valid),  .fs_score   (l2_fs_score),
                .kp_valid    (l2_kp_valid),  .kp_data    (l2_kp_data),
                .ori_valid   (), .ori_x (), .ori_y (), .ori_score (), .ori_quad (), .ori_sector (),
                .feat_valid  (lvl_fvalid[2]),
                .feat_x      (lvl_fx[2]),
                .feat_y      (lvl_fy[2]),
                .feat_score  (lvl_fscore[2]),
                .feat_quad   (lvl_fquad[2]),
                .feat_sector (lvl_fsector[2]),
                .feat_desc   (lvl_fdesc[2]),
                .busy        (lvl_busy[2])
            );

            orb_extractor_level #(
                .IMG_W       (W3),
                .IMG_H       (H3),
                .N_SECT      (N_SECT),
                .KP_FIFO_AW  (KP_FIFO_AW),
                .WIN_FIFO_AW (WIN_FIFO_AW),
                .PATTERN_FILE(PATTERN_FILE)
            ) u_ext3 (
                .clk         (clk),
                .rst         (rst),
                .in_valid    (p3_valid),
                .in_pixel    (p3_pixel),
                .thr         (thr),
                .g_valid     (l3_g_valid),   .g_pixel    (l3_g_pixel),
                .fs_valid    (l3_fs_valid),  .fs_score   (l3_fs_score),
                .kp_valid    (l3_kp_valid),  .kp_data    (l3_kp_data),
                .ori_valid   (), .ori_x (), .ori_y (), .ori_score (), .ori_quad (), .ori_sector (),
                .feat_valid  (lvl_fvalid[3]),
                .feat_x      (lvl_fx[3]),
                .feat_y      (lvl_fy[3]),
                .feat_score  (lvl_fscore[3]),
                .feat_quad   (lvl_fquad[3]),
                .feat_sector (lvl_fsector[3]),
                .feat_desc   (lvl_fdesc[3]),
                .busy        (lvl_busy[3])
            );

            assign l0_feat_valid  = lvl_fvalid[0];
            assign l0_feat_x      = lvl_fx[0];
            assign l0_feat_y      = lvl_fy[0];
            assign l0_feat_score  = lvl_fscore[0];
            assign l0_feat_quad   = lvl_fquad[0];
            assign l0_feat_sector = lvl_fsector[0];
            assign l0_feat_desc   = lvl_fdesc[0];

            assign l1_feat_valid  = lvl_fvalid[1];
            assign l1_feat_x      = lvl_fx[1];
            assign l1_feat_y      = lvl_fy[1];
            assign l1_feat_score  = lvl_fscore[1];
            assign l1_feat_quad   = lvl_fquad[1];
            assign l1_feat_sector = lvl_fsector[1];
            assign l1_feat_desc   = lvl_fdesc[1];

            assign l2_feat_valid  = lvl_fvalid[2];
            assign l2_feat_x      = lvl_fx[2];
            assign l2_feat_y      = lvl_fy[2];
            assign l2_feat_score  = lvl_fscore[2];
            assign l2_feat_quad   = lvl_fquad[2];
            assign l2_feat_sector = lvl_fsector[2];
            assign l2_feat_desc   = lvl_fdesc[2];

            assign l3_feat_valid  = lvl_fvalid[3];
            assign l3_feat_x      = lvl_fx[3];
            assign l3_feat_y      = lvl_fy[3];
            assign l3_feat_score  = lvl_fscore[3];
            assign l3_feat_quad   = lvl_fquad[3];
            assign l3_feat_sector = lvl_fsector[3];
            assign l3_feat_desc   = lvl_fdesc[3];

            // ---- Output buffering & priority arbitration (L0 -> L1 -> L2 -> L3) ----
            logic              rst_n;
            logic [3:0]        arb_pop, arb_empty, arb_full_unused;
            logic [FEAT_W-1:0] arb_rd_data [0:3];
            logic [FEAT_W-1:0] arb_wr_data [0:3];

            assign rst_n = ~rst;

            genvar k;
            for (k = 0; k < 4; k++) begin : g_out_fifo
                assign arb_wr_data[k] = {
                    lvl_fscore[k], lvl_fy[k], lvl_fx[k],
                    lvl_fquad[k], lvl_fsector[k], lvl_fdesc[k]
                };
                kp_fifo #(
                    .DW (FEAT_W),
                    .AW (OUT_FIFO_AW)
                ) u_out_fifo (
                    .clk     (clk),
                    .rst_n   (rst_n),
                    .flush   (1'b0),
                    .wr_en   (lvl_fvalid[k]),
                    .wr_data (arb_wr_data[k]),
                    .full    (arb_full_unused[k]),
                    .rd_en   (arb_pop[k]),
                    .rd_data (arb_rd_data[k]),
                    .empty   (arb_empty[k])
                );
            end

            logic              arb_any;
            logic [1:0]        arb_sel;
            logic [FEAT_W-1:0] arb_pdata;

            assign arb_any = ~(&arb_empty);
            assign arb_sel = !arb_empty[0] ? 2'd0 :
                             !arb_empty[1] ? 2'd1 :
                             !arb_empty[2] ? 2'd2 : 2'd3;
            assign arb_pop[0] = !arb_empty[0];
            assign arb_pop[1] =  arb_empty[0] && !arb_empty[1];
            assign arb_pop[2] =  arb_empty[0] &&  arb_empty[1] && !arb_empty[2];
            assign arb_pop[3] =  arb_empty[0] &&  arb_empty[1] &&  arb_empty[2] && !arb_empty[3];
            assign arb_pdata = !arb_empty[0] ? arb_rd_data[0] :
                               !arb_empty[1] ? arb_rd_data[1] :
                               !arb_empty[2] ? arb_rd_data[2] :
                               !arb_empty[3] ? arb_rd_data[3] : '0;

            always_ff @(posedge clk) begin
                if (rst) begin
                    feat_valid  <= 1'b0;
                    feat_level  <= 2'd0;
                    feat_x      <= '0;
                    feat_y      <= '0;
                    feat_score  <= '0;
                    feat_quad   <= '0;
                    feat_sector <= '0;
                    feat_desc   <= '0;
                end else begin
                    feat_valid <= arb_any;
                    if (arb_any) begin
                        feat_level  <= arb_sel;
                        feat_desc   <= arb_pdata[255:0];
                        feat_sector <= arb_pdata[259:256];
                        feat_quad   <= arb_pdata[261:260];
                        feat_x      <= arb_pdata[273:262];
                        feat_y      <= arb_pdata[285:274];
                        feat_score  <= arb_pdata[297:286];
                    end
                end
            end

            assign busy = (|lvl_busy) || (!(&arb_empty)) || feat_valid;

        end else begin : g_fe_only
            orb_frontend_level #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_fe0 (
                .clk(clk), .rst(rst), .in_valid(in_valid), .in_pixel(in_pixel), .thr(thr),
                .g_valid(l0_g_valid), .g_pixel(l0_g_pixel),
                .fs_valid(l0_fs_valid), .fs_score(l0_fs_score),
                .kp_valid(l0_kp_valid), .kp_data(l0_kp_data)
            );
            orb_frontend_level #(.IMG_W(W1), .IMG_H(H1)) u_fe1 (
                .clk(clk), .rst(rst), .in_valid(p1_valid), .in_pixel(p1_pixel), .thr(thr),
                .g_valid(l1_g_valid), .g_pixel(l1_g_pixel),
                .fs_valid(l1_fs_valid), .fs_score(l1_fs_score),
                .kp_valid(l1_kp_valid), .kp_data(l1_kp_data)
            );
            orb_frontend_level #(.IMG_W(W2), .IMG_H(H2)) u_fe2 (
                .clk(clk), .rst(rst), .in_valid(p2_valid), .in_pixel(p2_pixel), .thr(thr),
                .g_valid(l2_g_valid), .g_pixel(l2_g_pixel),
                .fs_valid(l2_fs_valid), .fs_score(l2_fs_score),
                .kp_valid(l2_kp_valid), .kp_data(l2_kp_data)
            );
            orb_frontend_level #(.IMG_W(W3), .IMG_H(H3)) u_fe3 (
                .clk(clk), .rst(rst), .in_valid(p3_valid), .in_pixel(p3_pixel), .thr(thr),
                .g_valid(l3_g_valid), .g_pixel(l3_g_pixel),
                .fs_valid(l3_fs_valid), .fs_score(l3_fs_score),
                .kp_valid(l3_kp_valid), .kp_data(l3_kp_data)
            );

            assign l0_feat_valid = 1'b0; assign l0_feat_x = '0; assign l0_feat_y = '0;
            assign l0_feat_score = '0;   assign l0_feat_quad = '0; assign l0_feat_sector = '0; assign l0_feat_desc = '0;
            assign l1_feat_valid = 1'b0; assign l1_feat_x = '0; assign l1_feat_y = '0;
            assign l1_feat_score = '0;   assign l1_feat_quad = '0; assign l1_feat_sector = '0; assign l1_feat_desc = '0;
            assign l2_feat_valid = 1'b0; assign l2_feat_x = '0; assign l2_feat_y = '0;
            assign l2_feat_score = '0;   assign l2_feat_quad = '0; assign l2_feat_sector = '0; assign l2_feat_desc = '0;
            assign l3_feat_valid = 1'b0; assign l3_feat_x = '0; assign l3_feat_y = '0;
            assign l3_feat_score = '0;   assign l3_feat_quad = '0; assign l3_feat_sector = '0; assign l3_feat_desc = '0;

            always_comb begin
                feat_valid  = 1'b0;
                feat_level  = 2'd0;
                feat_x      = '0;
                feat_y      = '0;
                feat_score  = '0;
                feat_quad   = '0;
                feat_sector = '0;
                feat_desc   = '0;
                busy        = 1'b0;
            end
        end
    endgenerate

endmodule

`default_nettype wire
