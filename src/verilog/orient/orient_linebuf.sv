// -----------------------------------------------------------------------------
// orient_linebuf.sv  --  37-row line buffer that outputs one 37x1 column per pixel
//
// Pixels arrive in raster order.  A single wide RAM (depth IMG_W, width 36 pixels)
// stores, for every image column, the 36 pixels above the current row.
//   stage 1: read  word = mem[col]               (rows r-36 .. r-1 of this column)
//   stage 2: col_data  = {new pixel, word}       (rows r-36 .. r)
//            mem[col]  = {new pixel, word>>1px}  (drop oldest row, keep r-35 .. r)
// Consecutive pixels use different addresses, so the delayed write never collides
// with the next read.  Maps onto block RAM (read-first, simple dual port).
//
// col_data element i = col_data[i*PIX_W +: PIX_W]:  i = 0 -> row r-36 (y = -18, top)
//                                                    i = 36 -> row r   (y = +18, newest)
// Latency: 2 clocks.  `tag` is a sideband that travels with the column.
// -----------------------------------------------------------------------------
module orient_linebuf #(
    parameter int PIX_W = 8,
    parameter int IMG_W = 640,
    parameter int CW    = $clog2(IMG_W),
    parameter int TAG_W = 1
)(
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   pix_valid,
    input  logic [PIX_W-1:0]       pix_data,
    input  logic [CW-1:0]          pix_col,
    input  logic [TAG_W-1:0]       tag_in,
    output logic                   col_valid,
    output logic [37*PIX_W-1:0]    col_data,
    output logic [TAG_W-1:0]       tag_out
);
    localparam int RW = 36 * PIX_W;

    logic [RW-1:0] mem [0:IMG_W-1];
    initial begin
        for (int k = 0; k < IMG_W; k++) mem[k] = '0;   // sim / BRAM init
    end

    // stage 1
    logic [RW-1:0]     rd;
    logic [PIX_W-1:0]  pix_q;
    logic [CW-1:0]     col_q;
    logic              v_q;
    logic [TAG_W-1:0]  tag_q;

    always_ff @(posedge clk) begin
        v_q   <= rst_n & pix_valid;
        pix_q <= pix_data;
        col_q <= pix_col;
        tag_q <= tag_in;
        if (pix_valid) rd <= mem[pix_col];
    end

    // stage 2
    always_ff @(posedge clk) begin
        col_valid <= rst_n & v_q;
        col_data  <= {pix_q, rd};
        tag_out   <= tag_q;
        if (v_q) mem[col_q] <= {pix_q, rd[RW-1:PIX_W]};
    end
endmodule
