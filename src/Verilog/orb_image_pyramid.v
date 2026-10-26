// ============================================================================
// orb_image_pyramid.v
// 4-level image pyramid: level 0 = the input stream itself, levels 1..3 come
// from three cascaded orb_pyr_scaler stages (level n built from level n-1).
//
// For 640x480 the levels are 640x480, 533x400, 444x333, 370x277.
// All level streams run concurrently, 1 pixel/cycle at their own rate.
// Each level has: valid, 8-bit pixel, raw accumulator, end-of-line, end-of-frame.
// ============================================================================
`timescale 1ns/1ps

module orb_image_pyramid #(
    parameter IMG_W = 640,
    parameter IMG_H = 480,
    parameter ROUND = 1
)(
    input  wire        clk,
    input  wire        rst,

    // level 0 (input)
    input  wire        in_valid,
    input  wire [7:0]  in_pixel,

    // level 1 : ~W/1.2
    output wire        l1_valid,
    output wire [7:0]  l1_pixel,
    output wire [12:0] l1_acc,
    output wire        l1_eol,
    output wire        l1_eof,

    // level 2 : ~W/1.44
    output wire        l2_valid,
    output wire [7:0]  l2_pixel,
    output wire [12:0] l2_acc,
    output wire        l2_eol,
    output wire        l2_eof,

    // level 3 : ~W/1.728
    output wire        l3_valid,
    output wire [7:0]  l3_pixel,
    output wire [12:0] l3_acc,
    output wire        l3_eol,
    output wire        l3_eof
);

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

    orb_pyr_scaler #(.IMG_W(IMG_W), .IMG_H(IMG_H), .ROUND(ROUND)) u_scaler1 (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_pixel(in_pixel),
        .out_valid(l1_valid), .out_pixel(l1_pixel), .out_acc(l1_acc),
        .out_eol(l1_eol), .out_eof(l1_eof)
    );

    orb_pyr_scaler #(.IMG_W(W1), .IMG_H(H1), .ROUND(ROUND)) u_scaler2 (
        .clk(clk), .rst(rst),
        .in_valid(l1_valid), .in_pixel(l1_pixel),
        .out_valid(l2_valid), .out_pixel(l2_pixel), .out_acc(l2_acc),
        .out_eol(l2_eol), .out_eof(l2_eof)
    );

    orb_pyr_scaler #(.IMG_W(W2), .IMG_H(H2), .ROUND(ROUND)) u_scaler3 (
        .clk(clk), .rst(rst),
        .in_valid(l2_valid), .in_pixel(l2_pixel),
        .out_valid(l3_valid), .out_pixel(l3_pixel), .out_acc(l3_acc),
        .out_eol(l3_eol), .out_eof(l3_eof)
    );

endmodule

