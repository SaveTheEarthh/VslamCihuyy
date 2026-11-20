// ============================================================================
// orb_window7.v
// Streaming 7x7 window generator (VSLAM2022 Sec. 4.1 line buffer + window
// buffer, and the "line buffer 7 rows -> window 7x7" box of Fig. 8).
// It is shared by FAST and the Gaussian filter.
//
//  * Input : raster-order 8-bit pixel stream of a IMG_W x IMG_H image,
//            in_valid may have gaps (the pyramid output has them).
//  * Memory: ONE line buffer, IMG_W words x 48 bit. Word at column x holds the
//            six previous rows of that column: {y-6, y-5, y-4, y-3, y-2, y-1}.
//  * Output: when a full 7x7 window is available (cx >= 6 && cy >= 6)
//            win_valid pulses and win_data holds the window:
//                pixel(r,c) = win_data[(r*7+c)*8 +: 8]
//                r = 0 .. 6 : rows y_c-3 .. y_c+3   (r=0 top / oldest)
//                c = 0 .. 6 : cols x_c-3 .. x_c+3   (c=0 left / oldest)
//                centre pixel = (r,c) = (3,3) = bits [199:192]
//            win_x / win_y = image coordinates of the window centre.
//  * Windows come out in raster order of the centres, (IMG_W-6)*(IMG_H-6)
//    per frame. Latency: win_valid appears 2 cycles after the input pixel
//    that completes the window (pixel at centre + (3,3)).
//
// Synchronous active-high reset. Plain Verilog-2001.
// ============================================================================
`timescale 1ns/1ps

module orb_window7 #(
    parameter IMG_W = 640,
    parameter IMG_H = 480
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         in_valid,
    input  wire [7:0]   in_pixel,

    output reg          win_valid,
    output reg  [11:0]  win_x,
    output reg  [11:0]  win_y,
    output wire [391:0] win_data
);

    // ---- stage 0 : input coordinates --------------------------------------
    reg [15:0] cx, cy;
    always @(posedge clk) begin
        if (rst) begin
            cx <= 16'd0;
            cy <= 16'd0;
        end else if (in_valid) begin
            if (cx == IMG_W - 1) begin
                cx <= 16'd0;
                cy <= (cy == IMG_H - 1) ? 16'd0 : cy + 16'd1;
            end else begin
                cx <= cx + 16'd1;
            end
        end
    end

    // ---- line buffer: read at t, write back at t+1 (needs the read data) ---
    reg [47:0] lb [0:IMG_W-1];

    reg        s1_v;
    reg [7:0]  s1_pix;
    reg [47:0] s1_old;      // rows y-6 .. y-1 of column cx
    reg [15:0] s1_cx, s1_cy;

    always @(posedge clk) begin
        if (rst) s1_v <= 1'b0;
        else     s1_v <= in_valid;
        if (in_valid) begin
            s1_old <= lb[cx];
            s1_pix <= in_pixel;
            s1_cx  <= cx;
            s1_cy  <= cy;
        end
        if (s1_v)
            lb[s1_cx] <= {s1_old[39:0], s1_pix};
    end

    // ---- 7 columns of 56 bit, newest column on top -------------------------
    // column c occupies [c*56 +: 56]; inside a column [55:48] is the oldest row
    reg [391:0] cols;

    always @(posedge clk) begin
        if (rst) win_valid <= 1'b0;
        else     win_valid <= s1_v && (s1_cx >= 16'd6) && (s1_cy >= 16'd6);
        if (s1_v) begin
            cols  <= {{s1_old, s1_pix}, cols[391:56]};
            win_x <= s1_cx[11:0] - 12'd3;
            win_y <= s1_cy[11:0] - 12'd3;
        end
    end

    genvar r, c;
    generate
        for (r = 0; r < 7; r = r + 1) begin : g_row
            for (c = 0; c < 7; c = c + 1) begin : g_col
                assign win_data[(r*7+c)*8 +: 8] = cols[c*56 + (6-r)*8 +: 8];
            end
        end
    endgenerate

endmodule
