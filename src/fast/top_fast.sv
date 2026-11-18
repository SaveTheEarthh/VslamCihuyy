// top_fast.sv - FAST subsystem for ONE pyramid level:
//   pixels -> fast_linebuf (7 rows, 7x7) -> fast_core (corner + SAD score) -> fast_nms (3x3) -> fast_kp_fifo
// Keypoints leave through the FIFO as {score[11:0], y[11:0], x[11:0]} (image coords of this level).
// fs_* is the raw score map (golden check / debug); leave unconnected if unused.
// shortcut: owns its own line buffer; when the Gaussian filter is added, share one fast_linebuf per level (~2 BRAM saved).
module top_fast #(
    parameter int IMG_W         = 640,    // size of THIS level's pixel raster
    parameter int IMG_H         = 480,
    parameter int KP_FIFO_DEPTH = 256
)(
    input  logic        clk,
    input  logic        rst,              // synchronous, active high

    input  logic        in_valid,
    input  logic [7:0]  in_pixel,
    input  logic [7:0]  thr,

    output logic        fs_valid,
    output logic [11:0] fs_score,

    input  logic        kp_rd_en,         // pop (BRIEF side)
    output logic        kp_valid,         // kp_data is the head keypoint
    output logic [35:0] kp_data,
    output logic        kp_overflow       // a keypoint was dropped because the FIFO was full
);

    logic         win_valid;
    logic [391:0] win_data;
    logic         nms_valid;
    logic [35:0]  nms_kp;

    fast_linebuf #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_linebuf (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_pixel(in_pixel),
        .win_valid(win_valid), .win_data(win_data)
    );

    fast_core u_core (
        .clk(clk), .rst(rst),
        .in_valid(win_valid), .win(win_data), .thr(thr),
        .out_valid(fs_valid), .out_score(fs_score)
    );

    fast_nms #(.MAP_W(IMG_W - 6), .MAP_H(IMG_H - 6)) u_nms (
        .clk(clk), .rst(rst),
        .in_valid(fs_valid), .in_score(fs_score),
        .out_valid(nms_valid), .out_kp(nms_kp)
    );

    fast_kp_fifo #(.W(36), .DEPTH(KP_FIFO_DEPTH)) u_fifo (
        .clk(clk), .rst(rst),
        .wr_en(nms_valid), .wr_data(nms_kp),
        .rd_en(kp_rd_en), .rd_data(kp_data), .rd_valid(kp_valid),
        .overflow(kp_overflow)
    );

endmodule
