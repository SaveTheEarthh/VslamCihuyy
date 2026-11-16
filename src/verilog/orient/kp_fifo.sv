// -----------------------------------------------------------------------------
// kp_fifo.sv  --  small synchronous first-word-fall-through FIFO (keypoint FIFO)
// Depth = 2**AW.  rd_data always shows the head entry while !empty.
// -----------------------------------------------------------------------------
module kp_fifo #(
    parameter int DW = 16,
    parameter int AW = 6
)(
    input  logic           clk,
    input  logic           rst_n,
    input  logic           flush,          // synchronous clear (e.g. at end of frame)
    input  logic           wr_en,
    input  logic [DW-1:0]  wr_data,
    output logic           full,
    input  logic           rd_en,
    output logic [DW-1:0]  rd_data,
    output logic           empty
);
    logic [DW-1:0] mem [0:(1<<AW)-1];
    logic [AW:0]   wptr, rptr;

    assign empty   = (wptr == rptr);
    assign full    = (wptr[AW-1:0] == rptr[AW-1:0]) && (wptr[AW] != rptr[AW]);
    assign rd_data = mem[rptr[AW-1:0]];

    always_ff @(posedge clk) begin
        if (!rst_n || flush) begin
            wptr <= '0;
            rptr <= '0;
        end else begin
            if (wr_en && !full) begin
                mem[wptr[AW-1:0]] <= wr_data;
                wptr <= wptr + 1'b1;
            end
            if (rd_en && !empty) begin
                rptr <= rptr + 1'b1;
            end
        end
    end
endmodule
