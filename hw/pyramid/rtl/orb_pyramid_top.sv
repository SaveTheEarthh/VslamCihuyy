// ============================================================================
// orb_pyramid_top.v
// AXI-Stream pixels in -> Input FIFO -> 4-level image pyramid.
//
//   s_axis_*  : AXI-Stream slave from the AXI DMA (S_DATA_W bits per beat,
//               little-endian pixel order, TLAST not needed).
//   l0_*      : the original image stream as it leaves the FIFO (level 0)
//   l1_..l3_* : pyramid levels 1..3 (see orb_image_pyramid.v)
//
// rst is active HIGH.  With an AXI reset (aresetn, active low) use:
//     .rst(~aresetn)
// ============================================================================
`timescale 1ns/1ps

module orb_pyramid_top #(
    parameter S_DATA_W   = 32,
    parameter FIFO_DEPTH = 1024,
    parameter IMG_W      = 640,
    parameter IMG_H      = 480,
    parameter ROUND      = 1
)(
    input  wire                 clk,
    input  wire                 rst,

    input  wire [S_DATA_W-1:0]  s_axis_tdata,
    input  wire                 s_axis_tvalid,
    output wire                 s_axis_tready,

    output wire [15:0]          fifo_level,

    // level 0
    output wire                 l0_valid,
    output wire [7:0]           l0_pixel,

    // level 1
    output wire                 l1_valid,
    output wire [7:0]           l1_pixel,
    output wire [12:0]          l1_acc,
    output wire                 l1_eol,
    output wire                 l1_eof,

    // level 2
    output wire                 l2_valid,
    output wire [7:0]           l2_pixel,
    output wire [12:0]          l2_acc,
    output wire                 l2_eol,
    output wire                 l2_eof,

    // level 3
    output wire                 l3_valid,
    output wire [7:0]           l3_pixel,
    output wire [12:0]          l3_acc,
    output wire                 l3_eol,
    output wire                 l3_eof
);

    orb_input_fifo #(.S_DATA_W(S_DATA_W), .DEPTH(FIFO_DEPTH)) u_fifo (
        .clk(clk), .rst(rst),
        .s_tdata(s_axis_tdata), .s_tvalid(s_axis_tvalid), .s_tready(s_axis_tready),
        .pix_valid(l0_valid), .pix_data(l0_pixel),
        .level(fifo_level)
    );

    orb_image_pyramid #(.IMG_W(IMG_W), .IMG_H(IMG_H), .ROUND(ROUND)) u_pyramid (
        .clk(clk), .rst(rst),
        .in_valid(l0_valid), .in_pixel(l0_pixel),
        .l1_valid(l1_valid), .l1_pixel(l1_pixel), .l1_acc(l1_acc),
        .l1_eol(l1_eol), .l1_eof(l1_eof),
        .l2_valid(l2_valid), .l2_pixel(l2_pixel), .l2_acc(l2_acc),
        .l2_eol(l2_eol), .l2_eof(l2_eof),
        .l3_valid(l3_valid), .l3_pixel(l3_pixel), .l3_acc(l3_acc),
        .l3_eol(l3_eol), .l3_eof(l3_eof)
    );

endmodule
