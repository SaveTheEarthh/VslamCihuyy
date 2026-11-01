// ============================================================================
// orb_gauss7.v
// 7x7 binomial Gaussian filter (VSLAM2022 Sec. 4.4, Fig. 9) - DSP-free.
//
//  * Input : 7x7 window from orb_window7 (pixel(r,c) at [(r*7+c)*8 +: 8]).
//  * Kernel: [1 6 15 20 15 6 1] (x) [1 6 15 20 15 6 1], sum = 4096.
//            out = floor(sum(k*p) / 4096) = sum >> 12   (no rounding, same as
//            gaussian7() in orb_frontend_golden.py).
//  * Structure (separable, exploits kernel symmetry, constants as shift-add):
//        stage 1 : 7 row filters   h_r = (p0+p6) + 6(p1+p5) + 15(p2+p4) + 20 p3
//                  max 64*255 = 16320 -> 14 bit
//        stage 2 : 1 column filter over h_0..h_6, then >> 12
//                  max 4096*255 = 1044480 -> 20 bit, result fits 8 bit
//    6x = 4x+2x     15x = 16x-x     20x = 16x+4x   (no multipliers inferred)
//  * Latency: 2 cycles, 1 window per cycle.
//
// Synchronous active-high reset (only the valid chain is reset).
// ============================================================================
`timescale 1ns/1ps

module orb_gauss7 (
    input  wire         clk,
    input  wire         rst,
    input  wire         in_valid,
    input  wire [391:0] win,

    output reg          out_valid,
    output reg  [7:0]   out_pixel
);

    // 7-tap row filter on 8-bit samples -> 14 bit
    function [13:0] hfilt;
        input [7:0] a0, a1, a2, a3, a4, a5, a6;
        reg   [8:0] s0, s1, s2;
        begin
            s0 = {1'b0, a0} + {1'b0, a6};
            s1 = {1'b0, a1} + {1'b0, a5};
            s2 = {1'b0, a2} + {1'b0, a4};
            // all operands are extended to 14 bit by the assignment context
            hfilt = s0 + (s1 << 2) + (s1 << 1) + (s2 << 4) - s2
                       + (a3 << 4) + (a3 << 2);
        end
    endfunction

    // 7-tap column filter on 14-bit samples -> 20 bit
    function [19:0] vfilt;
        input [13:0] h0, h1, h2, h3, h4, h5, h6;
        reg   [14:0] s0, s1, s2;
        begin
            s0 = {1'b0, h0} + {1'b0, h6};
            s1 = {1'b0, h1} + {1'b0, h5};
            s2 = {1'b0, h2} + {1'b0, h4};
            vfilt = s0 + (s1 << 2) + (s1 << 1) + (s2 << 4) - s2
                       + (h3 << 4) + (h3 << 2);
        end
    endfunction

    // ---- stage 1 : horizontal filter of each of the 7 window rows ----------
    reg         v1;
    reg [97:0]  h_r;                      // 7 x 14 bit
    integer r;
    always @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else     v1 <= in_valid;
        for (r = 0; r < 7; r = r + 1)
            h_r[r*14 +: 14] <= hfilt(win[(r*7+0)*8 +: 8], win[(r*7+1)*8 +: 8],
                                     win[(r*7+2)*8 +: 8], win[(r*7+3)*8 +: 8],
                                     win[(r*7+4)*8 +: 8], win[(r*7+5)*8 +: 8],
                                     win[(r*7+6)*8 +: 8]);
    end

    // ---- stage 2 : vertical filter + floor division by 4096 ----------------
    wire [19:0] vsum = vfilt(h_r[0*14 +: 14], h_r[1*14 +: 14], h_r[2*14 +: 14],
                             h_r[3*14 +: 14], h_r[4*14 +: 14], h_r[5*14 +: 14],
                             h_r[6*14 +: 14]);

    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= v1;
        out_pixel <= vsum[19:12];
    end

endmodule
