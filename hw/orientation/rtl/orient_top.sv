// -----------------------------------------------------------------------------
// orient_top.sv  --  orientation module + keypoint FIFO + coordinate match
//
//   FAST/NMS --(x,y)--> [range filter] -> kp_fifo --\
//                                                    >-- kp_match --> BRIEF
//   pixels --> orient_core ---(x,y,quadrant,sector)-/
//
// Keypoints closer than 18 px to a border have no complete 37x37 window and are
// dropped on the way in.  kp_in_ready = FIFO not full (hold the keypoint if low).
// FAST must deliver a keypoint before its window result appears (the window result
// lags the corner by ~18 rows + 18 columns, which FAST latency easily satisfies).
// -----------------------------------------------------------------------------
module orient_top #(
    parameter int PIX_W  = 8,
    parameter int IMG_W  = 640,
    parameter int IMG_H  = 480,
    parameter int N_SECT = 64,
    parameter int KP_AW  = 6,                      // keypoint FIFO depth = 2**KP_AW
    parameter int CW     = $clog2(IMG_W),
    parameter int RW     = $clog2(IMG_H),
    parameter int SEC_W  = $clog2(N_SECT / 4)
)(
    input  logic              clk,
    input  logic              rst_n,
    // pixel stream (smoothed image)
    input  logic              pix_valid,
    input  logic              pix_sof,
    input  logic [PIX_W-1:0]  pix_data,
    // keypoints from FAST
    input  logic              kp_in_valid,
    input  logic [CW-1:0]     kp_in_x,
    input  logic [RW-1:0]     kp_in_y,
    output logic              kp_in_ready,
    input  logic              kp_flush,
    // result for BRIEF
    output logic              out_valid,
    output logic [CW-1:0]     out_x,
    output logic [RW-1:0]     out_y,
    output logic [1:0]        out_quad,
    output logic [SEC_W-1:0]  out_sector
);
    logic              res_valid;
    logic [CW-1:0]     res_x;
    logic [RW-1:0]     res_y;
    logic [1:0]        res_quad;
    logic [SEC_W-1:0]  res_sector;

    orient_core #(.PIX_W(PIX_W), .IMG_W(IMG_W), .IMG_H(IMG_H), .N_SECT(N_SECT)) u_core (
        .clk(clk), .rst_n(rst_n),
        .pix_valid(pix_valid), .pix_sof(pix_sof), .pix_data(pix_data),
        .res_valid(res_valid), .res_x(res_x), .res_y(res_y),
        .res_quad(res_quad), .res_sector(res_sector),
        .res_m00(), .res_m10(), .res_m01());

    // keypoint input filter: only points with a complete 37x37 window
    logic kp_ok;
    assign kp_ok = (kp_in_x >= CW'(18)) && (kp_in_x <= CW'(IMG_W - 19)) &&
                   (kp_in_y >= RW'(18)) && (kp_in_y <= RW'(IMG_H - 19));

    logic              fifo_full, fifo_empty, kp_pop;
    logic [CW-1:0]     head_x;
    logic [RW-1:0]     head_y;
    assign kp_in_ready = ~fifo_full;

    kp_fifo #(.DW(CW + RW), .AW(KP_AW)) u_fifo (
        .clk(clk), .rst_n(rst_n), .flush(kp_flush),
        .wr_en(kp_in_valid & kp_ok), .wr_data({kp_in_y, kp_in_x}), .full(fifo_full),
        .rd_en(kp_pop), .rd_data({head_y, head_x}), .empty(fifo_empty));

    kp_match #(.CW(CW), .RW(RW), .SEC_W(SEC_W)) u_match (
        .clk(clk), .rst_n(rst_n),
        .res_valid(res_valid), .res_x(res_x), .res_y(res_y),
        .res_quad(res_quad), .res_sector(res_sector),
        .kp_valid(~fifo_empty), .kp_x(head_x), .kp_y(head_y), .kp_pop(kp_pop),
        .out_valid(out_valid), .out_x(out_x), .out_y(out_y),
        .out_quad(out_quad), .out_sector(out_sector));
endmodule
