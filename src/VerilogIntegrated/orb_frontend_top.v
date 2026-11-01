// ============================================================================
// orb_frontend_top.v
// Image pyramid (levels 0..3) + one ORB front end (window7 / FAST / NMS /
// Gaussian) per level, all running concurrently on the same pixel clock.
//
//  * Level 0 taps the input stream directly; levels 1..3 tap the pyramid.
//  * IMG_W / IMG_H are the size of the INPUT image (level 0). Level sizes are
//    derived with the same osz() as orb_image_pyramid, so they track the
//    manifest.json (e.g. 480x640 -> 400x533 -> 333x444 -> 277x370).
//  * Per level n (n = 0..3) the outputs are:
//        ln_g_valid  / ln_g_pixel     blurred pixels  (levelN_gauss.hex)
//        ln_fs_valid / ln_fs_score    FAST score map  (levelN_fastscore.hex)
//        ln_kp_valid / ln_kp_data     keypoints {score,y,x} (levelN_kp.hex)
// ============================================================================
`timescale 1ns/1ps

module orb_frontend_top #(
    parameter IMG_W = 640,
    parameter IMG_H = 480,
    parameter ROUND = 1
)(
    input  wire         clk,
    input  wire         rst,

    input  wire         in_valid,
    input  wire [7:0]   in_pixel,
    input  wire [7:0]   thr,

    // level 0
    output wire         l0_g_valid,   output wire [7:0]  l0_g_pixel,
    output wire         l0_fs_valid,  output wire [11:0] l0_fs_score,
    output wire         l0_kp_valid,  output wire [35:0] l0_kp_data,
    // level 1
    output wire         l1_g_valid,   output wire [7:0]  l1_g_pixel,
    output wire         l1_fs_valid,  output wire [11:0] l1_fs_score,
    output wire         l1_kp_valid,  output wire [35:0] l1_kp_data,
    // level 2
    output wire         l2_g_valid,   output wire [7:0]  l2_g_pixel,
    output wire         l2_fs_valid,  output wire [11:0] l2_fs_score,
    output wire         l2_kp_valid,  output wire [35:0] l2_kp_data,
    // level 3
    output wire         l3_g_valid,   output wire [7:0]  l3_g_pixel,
    output wire         l3_fs_valid,  output wire [11:0] l3_fs_score,
    output wire         l3_kp_valid,  output wire [35:0] l3_kp_data
);

    // same size rule as orb_image_pyramid
    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam W1 = osz(IMG_W);
    localparam H1 = osz(IMG_H);
    localparam W2 = osz(W1);
    localparam H2 = osz(H1);
    localparam W3 = osz(W2);
    localparam H3 = osz(H2);

    // ---- pyramid ------------------------------------------------------------
    wire        p1_valid, p2_valid, p3_valid;
    wire [7:0]  p1_pixel, p2_pixel, p3_pixel;

    orb_image_pyramid #(.IMG_W(IMG_W), .IMG_H(IMG_H), .ROUND(ROUND)) u_pyr (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_pixel(in_pixel),
        .l1_valid(p1_valid), .l1_pixel(p1_pixel), .l1_acc(), .l1_eol(), .l1_eof(),
        .l2_valid(p2_valid), .l2_pixel(p2_pixel), .l2_acc(), .l2_eol(), .l2_eof(),
        .l3_valid(p3_valid), .l3_pixel(p3_pixel), .l3_acc(), .l3_eol(), .l3_eof()
    );

    // ---- one front end per level -------------------------------------------
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

endmodule
