// fast_kp_fifo.sv - keypoint FIFO, first-word-fall-through, BRAM-inferred (same scheme as orb_input_fifo).
// Write : wr_en + wr_data. If full the keypoint is DROPPED (paper: SLAM tolerates a few lost keypoints)
//         and `overflow` latches until reset.
// Read  : rd_valid = head word present on rd_data; pulse rd_en to pop it.
// DEPTH must be a power of two.
module fast_kp_fifo #(
    parameter int W     = 36,
    parameter int DEPTH = 256
)(
    input  logic         clk,
    input  logic         rst,
    input  logic         wr_en,
    input  logic [W-1:0] wr_data,
    input  logic         rd_en,
    output logic [W-1:0] rd_data,
    output logic         rd_valid,
    output logic         overflow
);

    localparam int AW = $clog2(DEPTH);

    logic [W-1:0] mem [0:DEPTH-1];
    logic [AW:0]  wr_ptr, wr_ptr_d, rd_ptr;   // extra MSB tells full from empty; _d = RAM write latency
    logic [AW:0]  used;
    logic         full, empty, wr, pop;
    logic [AW:0]  rd_ptr_n;

    assign used     = wr_ptr - rd_ptr;
    assign full     = (used == DEPTH);
    assign empty    = (rd_ptr == wr_ptr_d);
    assign wr       = wr_en & ~full;
    assign pop      = rd_en & ~empty;
    assign rd_ptr_n = pop ? rd_ptr + 1'b1 : rd_ptr;
    assign rd_valid = ~empty;

    // read the NEXT head every cycle so rd_data always shows the current head
    always_ff @(posedge clk) begin
        if (wr) mem[wr_ptr[AW-1:0]] <= wr_data;
        rd_data <= mem[rd_ptr_n[AW-1:0]];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr   <= '0;
            wr_ptr_d <= '0;
            rd_ptr   <= '0;
            overflow <= 1'b0;
        end else begin
            wr_ptr_d <= wr_ptr;
            if (wr) wr_ptr <= wr_ptr + 1'b1;
            rd_ptr <= rd_ptr_n;
            if (wr_en & full) overflow <= 1'b1;
        end
    end

endmodule
