// fast_linebuf.sv - 7-row line buffer + 7x7 window. Raster pixels in, in_valid may have gaps.
// win_data[(r*7+c)*8 +: 8] = pixel(r,c); r=0 top/oldest, c=0 left/oldest; centre (3,3) = [199:192].
// One 48-bit RAM word per column holds the 6 rows above it (infers BRAM).
// win_valid comes 2 clk after the pixel at centre+(3,3); (IMG_W-6)*(IMG_H-6) windows per frame.
module fast_linebuf #(
    parameter int IMG_W = 640,
    parameter int IMG_H = 480
)(
    input  logic         clk,
    input  logic         rst,
    input  logic         in_valid,
    input  logic [7:0]   in_pixel,
    output logic         win_valid,
    output logic [391:0] win_data
);

    // input pixel coordinates
    logic [15:0] cx, cy;
    always_ff @(posedge clk) begin
        if (rst) begin
            cx <= '0;
            cy <= '0;
        end else if (in_valid) begin
            if (cx == IMG_W - 1) begin
                cx <= '0;
                cy <= (cy == IMG_H - 1) ? '0 : cy + 16'd1;
            end else begin
                cx <= cx + 16'd1;
            end
        end
    end

    // line buffer: read at t, write back at t+1
    logic [47:0] lb [0:IMG_W-1];
    logic        s1_v;
    logic [7:0]  s1_pix;
    logic [47:0] s1_old;          // rows y-6..y-1 of column cx
    logic [15:0] s1_cx, s1_cy;

    always_ff @(posedge clk) begin
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

    // 7 columns x 56 bit, newest on top; inside a column [55:48] is the oldest row
    logic [391:0] cols;
    always_ff @(posedge clk) begin
        if (rst) win_valid <= 1'b0;
        else     win_valid <= s1_v && (s1_cx >= 16'd6) && (s1_cy >= 16'd6);
        if (s1_v)
            cols <= {{s1_old, s1_pix}, cols[391:56]};
    end

    always_comb begin
        for (int r = 0; r < 7; r++)
            for (int c = 0; c < 7; c++)
                win_data[(r*7+c)*8 +: 8] = cols[c*56 + (6-r)*8 +: 8];
    end

endmodule
