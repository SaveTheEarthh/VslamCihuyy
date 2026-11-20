// ============================================================================
// orb_pyr_scaler.v
// One image-pyramid stage of the VSLAM2022 ORB-SLAM FPGA design (Sec. 4.2):
// streaming 6/5 (scale factor 1.2) bilinear downscaler.
//
//  * Input : raster-order 8-bit pixel stream, in_valid may have gaps.
//  * Output: raster-order 8-bit pixels, 5 of every 6 columns / rows kept.
//  * Throughput: 1 pixel / cycle, no back-pressure (no ready signal).
//  * Memory  : ONE line buffer (IMG_W bytes) + a 2x2 window.
//
// For an anchor pixel (x,y) with x6 = x mod 6, y6 = y mod 6 (both 0..4):
//     acc = (5-x6)(5-y6)*I(x,y)   + x6(5-y6)*I(x+1,y)
//         + (5-x6)*y6    *I(x,y+1) + x6*y6   *I(x+1,y+1)          (sum = 25)
//     out = ROUND ? (acc + 12)/25 : acc/25
//
// The window completes when pixel (cx,cy) = (x+1,y+1) arrives, so the anchor
// mod-6 values are derived from the arriving pixel's own mod-6 counters:
//     x6 = (cx mod 6) - 1,  invalid when (cx mod 6) == 0   (this also removes
//     cx == 0), and the same for y.  No subtractors on the coordinates needed.
//
// Latency: out_valid appears 4 clock cycles after the input cycle of the pixel
// that completes the 2x2 window.
//
// Divide by 25 is exact for 0 <= n <= 6387:  n/25 == (n * 5243) >> 17
// (checked exhaustively).
//
// Plain Verilog-2001.  Synchronous active-high reset.
// ============================================================================
`timescale 1ns/1ps

module orb_pyr_scaler #(
    parameter IMG_W = 640,   // input width  (>= 2)
    parameter IMG_H = 480,   // input height (>= 2)
    parameter ROUND = 1      // 1: round-half-up (acc+12)/25, 0: floor acc/25
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        in_valid,
    input  wire [7:0]  in_pixel,

    output reg         out_valid,
    output reg  [7:0]  out_pixel,
    output reg  [12:0] out_acc,    // raw weighted sum, 0..6375 (debug / golden)
    output reg         out_eol,    // last output pixel of an output row
    output reg         out_eof     // last output pixel of the output frame
);

    // ---- output size: valid anchors x in [0,W-2] with x mod 6 != 5 --------
    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam OUT_W = osz(IMG_W);
    localparam OUT_H = osz(IMG_H);

    // ---- stage 0 : input coordinate counters (advance on in_valid) --------
    reg [15:0] cx, cy;       // position of the arriving pixel
    reg [2:0]  cx6, cy6;     // cx mod 6, cy mod 6

    always @(posedge clk) begin
        if (rst) begin
            cx <= 16'd0; cy <= 16'd0; cx6 <= 3'd0; cy6 <= 3'd0;
        end else if (in_valid) begin
            if (cx == IMG_W - 1) begin
                cx  <= 16'd0;
                cx6 <= 3'd0;
                if (cy == IMG_H - 1) begin
                    cy  <= 16'd0;
                    cy6 <= 3'd0;
                end else begin
                    cy  <= cy + 16'd1;
                    cy6 <= (cy6 == 3'd5) ? 3'd0 : cy6 + 3'd1;
                end
            end else begin
                cx  <= cx + 16'd1;
                cx6 <= (cx6 == 3'd5) ? 3'd0 : cx6 + 3'd1;
            end
        end
    end

    // ---- line buffer (read-first, infers block RAM) -----------------------
    reg [7:0] lb [0:IMG_W-1];
    reg [7:0] s1_top;                    // I(cx, cy-1), aligned with stage 1

    always @(posedge clk) begin
        if (in_valid) begin
            s1_top <= lb[cx];
            lb[cx] <= in_pixel;
        end
    end

    // ---- stage 1 : align pixel, coordinates; build the 2x2 window ---------
    reg        s1_v;
    reg [7:0]  s1_pix;                   // I(cx, cy)
    reg [2:0]  s1_cx6, s1_cy6;

    always @(posedge clk) begin
        if (rst) s1_v <= 1'b0;
        else     s1_v <= in_valid;
        if (in_valid) begin
            s1_pix <= in_pixel;
            s1_cx6 <= cx6;
            s1_cy6 <= cy6;
        end
    end

    reg [7:0] wp_pix;                    // I(cx-1, cy)
    reg [7:0] wp_top;                    // I(cx-1, cy-1)
    always @(posedge clk) begin
        if (s1_v) begin
            wp_pix <= s1_pix;
            wp_top <= s1_top;
        end
    end

    // ---- stage 2 : weights and the four products --------------------------
    wire       v2_in = s1_v && (s1_cx6 != 3'd0) && (s1_cy6 != 3'd0);

    wire [2:0] wx0 = 3'd6 - s1_cx6;      // 5 - x6
    wire [2:0] wx1 = s1_cx6 - 3'd1;      //     x6
    wire [2:0] wy0 = 3'd6 - s1_cy6;      // 5 - y6
    wire [2:0] wy1 = s1_cy6 - 3'd1;      //     y6

    wire [5:0] w00 = wx0 * wy0;          // x6,y6 weights, max 25
    wire [5:0] w01 = wx1 * wy0;
    wire [5:0] w10 = wx0 * wy1;
    wire [5:0] w11 = wx1 * wy1;

    reg        v2;
    reg [12:0] p00, p01, p10, p11;

    always @(posedge clk) begin
        if (rst) v2 <= 1'b0;
        else     v2 <= v2_in;
        p00 <= wp_top * w00;             // I(x  ,y  )
        p01 <= s1_top * w01;             // I(x+1,y  )
        p10 <= wp_pix * w10;             // I(x  ,y+1)
        p11 <= s1_pix * w11;             // I(x+1,y+1)
    end

    // ---- stage 3 : sum -----------------------------------------------------
    reg        v3;
    reg [12:0] acc3;

    always @(posedge clk) begin
        if (rst) v3 <= 1'b0;
        else     v3 <= v2;
        acc3 <= p00 + p01 + p10 + p11;
    end

    // ---- stage 4 : normalise by 25 (multiply-shift) and output flags ------
    localparam [12:0] RND = ROUND ? 13'd12 : 13'd0;

    wire [12:0] nrm  = acc3 + RND;               // <= 6387
    wire [25:0] prod = nrm * 13'd5243;           // ~ nrm * 2^17 / 25
    wire [7:0]  pix  = prod[24:17];

    reg [15:0] ox, oy;

    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0;
            out_eol   <= 1'b0;
            out_eof   <= 1'b0;
            ox        <= 16'd0;
            oy        <= 16'd0;
        end else begin
            out_valid <= v3;
            out_eol   <= v3 && (ox == OUT_W - 1);
            out_eof   <= v3 && (ox == OUT_W - 1) && (oy == OUT_H - 1);
            if (v3) begin
                if (ox == OUT_W - 1) begin
                    ox <= 16'd0;
                    oy <= (oy == OUT_H - 1) ? 16'd0 : oy + 16'd1;
                end else begin
                    ox <= ox + 16'd1;
                end
            end
        end
        if (v3) begin
            out_pixel <= pix;
            out_acc   <= acc3;
        end
    end

endmodule
