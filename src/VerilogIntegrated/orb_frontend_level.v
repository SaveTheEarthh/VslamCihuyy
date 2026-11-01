// ============================================================================
// orb_frontend_level.v
// ORB front end for ONE pyramid level (VSLAM2022 Fig. 8):
//
//   in_pixel/in_valid ─► orb_window7 ─┬─► orb_fast_core ─┬─► fs_valid/fs_score
//   (IMG_W x IMG_H)                   │                  └─► orb_nms3x3 ─► kp_valid/kp_data
//                                     └─► orb_gauss7 ─────► g_valid/g_pixel
//
//  * IMG_W / IMG_H are the size of THIS level's valid-pixel raster (the number
//    of valid beats per row / rows per frame), not of the original image.
//  * g_*  : 7x7 Gaussian of the interior pixels, (W-6)x(H-6), raster order
//           (= levelN_gauss.hex)
//  * fs_* : FAST score of the interior pixels, (W-6)x(H-6), raster order
//           (= levelN_fastscore.hex)
//  * kp_* : keypoints after 3x3 NMS, {score[11:0], y[11:0], x[11:0]}, raster
//           order, image coordinates of the level (= levelN_kp.hex)
//  The three streams are independent (different latencies, no alignment).
// ============================================================================
`timescale 1ns/1ps

module orb_frontend_level #(
    parameter IMG_W = 640,
    parameter IMG_H = 480
)(
    input  wire         clk,
    input  wire         rst,

    input  wire         in_valid,
    input  wire [7:0]   in_pixel,
    input  wire [7:0]   thr,            // FAST threshold

    output wire         g_valid,
    output wire [7:0]   g_pixel,

    output wire         fs_valid,
    output wire [11:0]  fs_score,

    output wire         kp_valid,
    output wire [35:0]  kp_data
);

    wire         win_valid;
    wire [391:0] win_data;
    wire [11:0]  win_x_unused, win_y_unused;

    orb_window7 #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_win (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_pixel(in_pixel),
        .win_valid(win_valid), .win_x(win_x_unused), .win_y(win_y_unused),
        .win_data(win_data)
    );

    orb_fast_core u_fast (
        .clk(clk), .rst(rst),
        .in_valid(win_valid), .win(win_data), .thr(thr),
        .out_valid(fs_valid), .out_score(fs_score)
    );

    orb_nms3x3 #(.MAP_W(IMG_W - 6), .MAP_H(IMG_H - 6), .X_OFF(3), .Y_OFF(3)) u_nms (
        .clk(clk), .rst(rst),
        .in_valid(fs_valid), .in_score(fs_score),
        .out_valid(kp_valid), .out_kp(kp_data)
    );

    orb_gauss7 u_gauss (
        .clk(clk), .rst(rst),
        .in_valid(win_valid), .win(win_data),
        .out_valid(g_valid), .out_pixel(g_pixel)
    );

endmodule
