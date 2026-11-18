// fast_nms.sv - 3x3 non-maximum suppression on the FAST score map (MAP_W x MAP_H = (IMG_W-6) x (IMG_H-6)).
// Keypoint: score > 0, > up-left/up/up-right/left, >= right/down-left/down/down-right
// (strict on earlier raster neighbours, >= on later ones, so equal adjacent scores keep one keypoint).
// out_kp = {score[11:0], y[11:0], x[11:0]}, image coordinates (map coord + 3). Pulses only on keypoints.
// Own x/y counters advance on in_valid beats. Latency 3 clk after the 3x3 window completes.
module fast_nms #(
    parameter int MAP_W = 634,
    parameter int MAP_H = 474
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        in_valid,
    input  logic [11:0] in_score,
    output logic        out_valid,
    output logic [35:0] out_kp
);

    // score-map coordinates
    logic [15:0] cx, cy;
    always_ff @(posedge clk) begin
        if (rst) begin
            cx <= '0;
            cy <= '0;
        end else if (in_valid) begin
            if (cx == MAP_W - 1) begin
                cx <= '0;
                cy <= (cy == MAP_H - 1) ? '0 : cy + 16'd1;
            end else begin
                cx <= cx + 16'd1;
            end
        end
    end

    // line buffer: two previous rows per column {y-2, y-1}
    logic [23:0] lb [0:MAP_W-1];
    logic        s1_v;
    logic [11:0] s1_pix;
    logic [23:0] s1_old;
    logic [15:0] s1_cx, s1_cy;

    always_ff @(posedge clk) begin
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

    // 3 columns x 36 bit, newest on top; pixel(r,c) = cols[c*36 + (2-r)*12 +: 12]
    logic [107:0] cols;
    logic         w_valid;
    logic [11:0]  w_x, w_y;

    always_ff @(posedge clk) begin
        if (rst) w_valid <= 1'b0;
        else     w_valid <= s1_v && (s1_cx >= 16'd2) && (s1_cy >= 16'd2);
        if (s1_v) begin
            cols <= {{s1_old, s1_pix}, cols[107:36]};
            w_x  <= s1_cx[11:0] + 12'd2;     // centre = (cx-1, cy-1), +3 border of the 7x7 window
            w_y  <= s1_cy[11:0] + 12'd2;
        end
    end

    logic [11:0] n_ul, n_u, n_ur, n_l, n_c, n_r, n_dl, n_d, n_dr;
    assign n_ul = cols[24 +: 12];
    assign n_u  = cols[60 +: 12];
    assign n_ur = cols[96 +: 12];
    assign n_l  = cols[12 +: 12];
    assign n_c  = cols[48 +: 12];
    assign n_r  = cols[84 +: 12];
    assign n_dl = cols[0  +: 12];
    assign n_d  = cols[36 +: 12];
    assign n_dr = cols[72 +: 12];

    logic is_kp;
    assign is_kp = (n_c != 12'd0) &&
                   (n_c >  n_ul) && (n_c >  n_u)  && (n_c >  n_ur) && (n_c >  n_l) &&
                   (n_c >= n_r)  && (n_c >= n_dl) && (n_c >= n_d)  && (n_c >= n_dr);

    always_ff @(posedge clk) begin
        if (rst) out_valid <= 1'b0;
        else     out_valid <= w_valid && is_kp;
        out_kp <= {n_c, w_y, w_x};
    end

endmodule
