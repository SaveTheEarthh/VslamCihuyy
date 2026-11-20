// ============================================================================
// orb_input_fifo.v
// "Input FIFO" stage in front of the image pyramid (VSLAM2022 Fig. 6 / Sec. 4).
//
//   AXI DMA --AXI-Stream (S_DATA_W bits)--> [ FIFO ] --> byte serializer
//                                                        --> 8-bit pixel stream
//
//  * Slave side  : AXI-Stream (tdata/tvalid/tready).  TLAST is not needed and
//                  is ignored - leave it unconnected.
//  * Master side : 1 pixel/cycle, pix_valid + pix_data (no ready: the pyramid
//                  accepts a pixel every cycle).  Gaps appear only when the FIFO
//                  runs empty (DMA slower than 1 pixel/cycle).
//  * Byte order  : AXI little-endian - the FIRST pixel of a beat is tdata[7:0],
//                  the next is tdata[15:8], ...
//  * Frames      : the stream is treated as one continuous pixel sequence, so
//                  back-to-back frames work as long as the DMA transfer holds
//                  a whole number of beats (e.g. W*H a multiple of S_DATA_W/8).
//                  Padding bytes in a partial last beat are NOT removed.
//  * Memory      : single-clock, first-word-fall-through FIFO using a
//                  synchronous-read RAM (infers block RAM), DEPTH words.
//
// Reset is synchronous, active HIGH (invert aresetn at the top level).
// DEPTH must be a power of two, 4 <= DEPTH <= 32768.
// ============================================================================
`timescale 1ns/1ps

module orb_input_fifo #(
    parameter S_DATA_W = 32,      // AXI-Stream width: 8, 16, 32, 64 ...
    parameter DEPTH    = 1024     // FIFO depth in beats (words), power of two
)(
    input  wire                  clk,
    input  wire                  rst,

    // AXI-Stream slave (from AXI DMA MM2S)
    input  wire [S_DATA_W-1:0]   s_tdata,
    input  wire                  s_tvalid,
    output wire                  s_tready,

    // pixel stream to the pyramid
    output wire                  pix_valid,
    output wire [7:0]            pix_data,

    // occupancy in beats (zero-extended), for debug / flow monitoring
    output wire [15:0]           level
);

    function integer clog2;
        input integer v;
        integer i;
        begin
            clog2 = 0;
            for (i = v - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam BYTES = S_DATA_W / 8;
    localparam AW    = clog2(DEPTH);

    reg [S_DATA_W-1:0] mem [0:DEPTH-1];
    reg [S_DATA_W-1:0] q;                 // FWFT head word (registered RAM output)

    reg [AW:0] wr_ptr;                    // extra MSB distinguishes full / empty
    reg [AW:0] wr_ptr_d;                  // wr_ptr delayed 1 cycle (RAM read latency)
    reg [AW:0] rd_ptr;
    reg [7:0]  bidx;                      // byte index inside the head word

    wire [AW:0] used  = wr_ptr - rd_ptr;
    wire        full  = (used == DEPTH);
    wire        empty = (rd_ptr == wr_ptr_d);

    assign s_tready  = ~full;
    assign level     = used;

    wire wr_en     = s_tvalid & s_tready;
    wire last_byte = (bidx == BYTES - 1);
    wire pop       = ~empty & last_byte;

    wire [AW:0] rd_ptr_n = pop ? (rd_ptr + 1'b1) : rd_ptr;

    // RAM: write port + read-first synchronous read. Reading at the NEXT read
    // pointer every cycle makes q always show the current head word.
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_ptr[AW-1:0]] <= s_tdata;
        q <= mem[rd_ptr_n[AW-1:0]];
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr   <= {(AW+1){1'b0}};
            wr_ptr_d <= {(AW+1){1'b0}};
            rd_ptr   <= {(AW+1){1'b0}};
            bidx     <= 8'd0;
        end else begin
            wr_ptr_d <= wr_ptr;
            if (wr_en)
                wr_ptr <= wr_ptr + 1'b1;
            rd_ptr <= rd_ptr_n;
            if (~empty)
                bidx <= last_byte ? 8'd0 : bidx + 8'd1;
        end
    end

    assign pix_valid = ~empty;
    assign pix_data  = q[{bidx, 3'b000} +: 8];

endmodule
