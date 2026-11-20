// ============================================================================
// orb_nms3x3.v
// 3x3 non-maximum suppression on the FAST score map (VSLAM2022 Sec. 3.5/4.3).
//
//  * Input : raster-order stream of the score map, MAP_W x MAP_H words of
//            12 bit (MAP_W = IMG_W-6, MAP_H = IMG_H-6). in_valid may have gaps;
//            positions are tracked by counting valid beats.
//  * Memory: ONE line buffer, MAP_W words x 24 bit = the two previous rows of
//            that column {y-2, y-1}  (same scheme as orb_window7).
//  * Keypoint rule (identical to nms3x3() in orb_frontend_golden.py):
//        score > 0
//        score >  up-left, up, up-right, left      (earlier in raster order)
//        score >= right, down-left, down, down-right (later in raster order)
//  * Output: out_valid pulses only for keypoints, in raster order.
//        out_kp = {score[11:0], y[11:0], x[11:0]}   (36 bit, = levelN_kp.hex)
//        x, y are IMAGE coordinates: map coordinate + 3 (X_OFF / Y_OFF).
//        (MAP_W-2)*(MAP_H-2) centres are examined per frame.
//  * Latency: 3 cycles after the score that completes the 3x3 window
//             (the bottom-right neighbour of the centre).
//
// Synchronous active-high reset. Plain Verilog-2001.
// ============================================================================
`timescale 1ns/1ps

module orb_nms3x3 #(
    parameter MAP_W = 474,
    parameter MAP_H = 634,
    parameter X_OFF = 3,          // image x of map column 0
    parameter Y_OFF = 3           // image y of map row 0
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         in_valid,
    input  wire [11:0]  in_score,

    output reg          out_valid,
    output reg  [35:0]  out_kp
);

    // ---- stage 0 : input coordinates in the score map ----------------------
    reg [15:0] cx, cy;
    always @(posedge clk) begin
        if (rst) begin
            cx <= 16'd0;
            cy <= 16'd0;
        end else if (in_valid) begin
            if (cx == MAP_W - 1) begin
                cx <= 16'd0;
                cy <= (cy == MAP_H - 1) ? 16'd0 : cy + 16'd1;
            end else begin
                cx <= cx + 16'd1;
            end
        end
    end

    // ---- line buffer: read at t, write back at t+1 -------------------------
    reg [23:0] lb [0:MAP_W-1];

    reg        s1_v;
    reg [11:0] s1_pix;
    reg [23:0] s1_old;            // rows y-2, y-1 of column cx : {y-2, y-1}
    reg [15:0] s1_cx, s1_cy;

    always @(posedge clk) begin
        if (rst) s1_v <= 1'b0;
        else     s1_v <= in_valid;
        if (in_valid) begin
            s1_old <= lb[cx];
            s1_pix <= in_score;
            s1_cx  <= cx;
            s1_cy  <= cy;
        end
        if (s1_v)
            lb[s1_cx] <= {s1_old[11:0], s1_pix};
    end

    // ---- 3 columns of 36 bit, newest column on top -------------------------
    // column c occupies [c*36 +: 36]; inside a column [35:24] is the oldest row
    // pixel(r,c) = cols[c*36 + (2-r)*12 +: 12]   r=0 top, c=0 left
    reg [107:0] cols;
    reg         w_valid;
    reg [11:0]  w_x, w_y;

    always @(posedge clk) begin
        if (rst) w_valid <= 1'b0;
        else     w_valid <= s1_v && (s1_cx >= 16'd2) && (s1_cy >= 16'd2);
        if (s1_v) begin
            cols <= {{s1_old, s1_pix}, cols[107:36]};
            w_x  <= s1_cx[11:0] + X_OFF - 1;    // centre = (cx-1, cy-1)
            w_y  <= s1_cy[11:0] + Y_OFF - 1;
        end
    end

    wire [11:0] n_ul = cols[24 +: 12];
    wire [11:0] n_u  = cols[60 +: 12];
    wire [11:0] n_ur = cols[96 +: 12];
    wire [11:0] n_l  = cols[12 +: 12];
    wire [11:0] n_c  = cols[48 +: 12];
    wire [11:0] n_r  = cols[84 +: 12];
    wire [11:0] n_dl = cols[0  +: 12];
    wire [11:0] n_d  = cols[36 +: 12];
    wire [11:0] n_dr = cols[72 +: 12];

    wire is_kp = (n_c != 12'd0) &&
                 (n_c >  n_ul) && (n_c >  n_u)  && (n_c >  n_ur) && (n_c >  n_l) &&
                 (n_c >= n_r)  && (n_c >= n_dl) && (n_c >= n_d)  && (n_c >= n_dr);

    // ---- stage 3 : keypoint output ------------------------------------------
    always @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= w_valid && is_kp;
        out_kp <= {n_c, w_y, w_x};
    end

endmodule
