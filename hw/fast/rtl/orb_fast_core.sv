// ============================================================================
// orb_fast_core.v
// FAST-16 corner detector + score (VSLAM2022 Sec. 3.5 and Sec. 4.3).
//
//  * Input : 7x7 window from orb_window7 (win_data layout: pixel(r,c) at
//            [(r*7+c)*8 +: 8], centre = (3,3)).
//  * Ring  : 16 pixels of the radius-3 Bresenham circle, clockwise from the top
//            (dx,dy) = (0,-3) (1,-3) (2,-2) (3,-1) (3,0) (3,1) (2,2) (1,3)
//                      (0,3) (-1,3) (-2,2) (-3,1) (-3,0) (-3,-1) (-2,-2) (-1,-3)
//  * brighter_i = ring_i > p + thr      darker_i = p > ring_i + thr
//    (9-bit arithmetic: no wrap-around when p + thr > 255 or p < thr)
//  * corner = 9 circularly contiguous brighter OR 9 contiguous darker.
//    All 16 contiguous-9 patterns are tested in parallel on the wrapped
//    vector, like the "bright/dark vectors ANDed with 16 bitmasks" of Sec. 4.3.
//  * score  = sum of |ring_i - p| over all 16 pixels (Sec. 3.5), 0 if the
//             pixel is not a corner. Max 16*255 = 4080 -> 12 bit.
//  * Latency: 3 cycles. Fully pipelined, 1 window per cycle.
//
// Synchronous active-high reset (only the valid chain is reset).
// ============================================================================
`timescale 1ns/1ps

module orb_fast_core (
    input  wire         clk,
    input  wire         rst,
    input  wire         in_valid,
    input  wire [391:0] win,
    input  wire [7:0]   thr,

    output reg          out_valid,
    output reg  [11:0]  out_score
);

    // index of ring pixel i inside the flattened 7x7 window: (3+dy)*7 + (3+dx)
    function integer ring_idx;
        input integer i;
        begin
            case (i)
                0:  ring_idx = 3;
                1:  ring_idx = 4;
                2:  ring_idx = 12;
                3:  ring_idx = 20;
                4:  ring_idx = 27;
                5:  ring_idx = 34;
                6:  ring_idx = 40;
                7:  ring_idx = 46;
                8:  ring_idx = 45;
                9:  ring_idx = 44;
                10: ring_idx = 36;
                11: ring_idx = 28;
                12: ring_idx = 21;
                13: ring_idx = 14;
                14: ring_idx = 8;
                default: ring_idx = 2;
            endcase
        end
    endfunction

    wire [7:0] p = win[24*8 +: 8];            // centre pixel (3,3) -> index 24

    wire [7:0]  ring [0:15];
    wire [7:0]  ad   [0:15];
    wire [15:0] bri;
    wire [15:0] dar;

    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : g_ring
            assign ring[i] = win[ring_idx(i)*8 +: 8];
            assign bri[i]  = ({1'b0, ring[i]} > ({1'b0, p} + {1'b0, thr}));
            assign dar[i]  = ({1'b0, p} > ({1'b0, ring[i]} + {1'b0, thr}));
            assign ad[i]   = (ring[i] > p) ? (ring[i] - p) : (p - ring[i]);
        end
    endgenerate

    // 9 contiguous ones anywhere on the circle (wrap-around via 25-bit vector)
    wire [24:0] bw = {bri[8:0], bri};
    wire [24:0] dw = {dar[8:0], dar};

    reg corner;
    integer j;
    always @(*) begin
        corner = 1'b0;
        for (j = 0; j < 16; j = j + 1) begin
            if ((&bw[j +: 9]) || (&dw[j +: 9]))
                corner = 1'b1;
        end
    end

    // ---- stage 1 : corner flag + 16 absolute differences -------------------
    reg         v1, c1;
    reg [127:0] ad_r;
    integer m;
    always @(posedge clk) begin
        if (rst) v1 <= 1'b0;
        else     v1 <= in_valid;
        c1 <= corner;
        for (m = 0; m < 16; m = m + 1)
            ad_r[m*8 +: 8] <= ad[m];
    end

    // ---- stage 2 : four partial sums of 4 ----------------------------------
    reg        v2, c2;
    reg [39:0] ps_r;
    integer n;
    always @(posedge clk) begin
        if (rst) v2 <= 1'b0;
        else     v2 <= v1;
        c2 <= c1;
        for (n = 0; n < 4; n = n + 1)
            ps_r[n*10 +: 10] <= ad_r[(4*n)*8 +: 8]     + ad_r[(4*n+1)*8 +: 8]
                              + ad_r[(4*n+2)*8 +: 8]   + ad_r[(4*n+3)*8 +: 8];
    end

    // ---- stage 3 : final sum, zero if not a corner -------------------------
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= v2;
        out_score <= c2 ? (ps_r[9:0] + ps_r[19:10] + ps_r[29:20] + ps_r[39:30])
                        : 12'd0;
    end

endmodule
