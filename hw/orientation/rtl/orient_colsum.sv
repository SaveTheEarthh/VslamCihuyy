// -----------------------------------------------------------------------------
// orient_colsum.sv  --  column adder trees (Section 4.5.3)
//   S(C)   = sum of the 37 pixels of the column                (unsigned)
//   m01(C) = sum over y=-18..18 of y * pixel                   (signed)
//
// m01(C) uses the symmetry  sum_y y*I(y) = sum_{y=1..18} y * (I(+y) - I(-y)),
// which halves the number of multiplies (constant multiplies -> shift/add).
// Both results are aligned to the same latency.
//   latency = 2 (subtract, multiply) + 5 (18-input tree) = 7 clocks
// -----------------------------------------------------------------------------
module orient_colsum #(
    parameter int PIX_W = 8,
    parameter int TAG_W = 1,
    parameter int SW    = PIX_W + 6,      // width of S      (unsigned)
    parameter int MCW   = PIX_W + 11      // width of m01(C) (signed)
)(
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     valid_in,
    input  logic [37*PIX_W-1:0]      col_data,
    input  logic [TAG_W-1:0]         tag_in,
    output logic                     valid_out,
    output logic [SW-1:0]            s_out,
    output logic signed [MCW-1:0]    m01c_out,
    output logic [TAG_W-1:0]         tag_out
);
    localparam int TW    = PIX_W + 6;               // width of y * d
    localparam int LS    = $clog2(37);              // S tree latency (6)
    localparam int LM    = 2 + $clog2(18);          // m01 path latency (7)
    localparam int ALIGN = LM - LS;                 // extra delay on S (1)

    // ---- S(C): 37-input unsigned tree -------------------------------------
    logic [SW-1:0] s_tree;
    pipelined_adder_tree #(.N(37), .W(PIX_W), .SIGNED(1'b0))
        u_s (.clk(clk), .din(col_data), .dout(s_tree));

    // ---- m01(C): d_y = I(+y) - I(-y), t_y = y * d_y, 18-input signed tree ----
    logic signed [PIX_W:0] d [0:17];
    logic signed [TW-1:0]  t [0:17];
    logic [18*TW-1:0]      t_bus;

    genvar y;
    generate
        for (y = 1; y <= 18; y++) begin : g_m
            always_ff @(posedge clk) begin
                d[y-1] <= $signed({1'b0, col_data[(18+y)*PIX_W +: PIX_W]})
                        - $signed({1'b0, col_data[(18-y)*PIX_W +: PIX_W]});
                t[y-1] <= d[y-1] * y;
            end
            assign t_bus[(y-1)*TW +: TW] = t[y-1];
        end
    endgenerate

    logic signed [MCW-1:0] m_tree;
    pipelined_adder_tree #(.N(18), .W(TW), .SIGNED(1'b1))
        u_m (.clk(clk), .din(t_bus), .dout(m_tree));

    // ---- align S with m01(C), and delay valid + tag ------------------------
    logic [SW-1:0]    s_dly [0:ALIGN-1];
    logic [LM-1:0]    vp;
    logic [TAG_W-1:0] tp [0:LM-1];

    always_ff @(posedge clk) begin
        s_dly[0] <= s_tree;
        for (int j = 1; j < ALIGN; j++) begin
            s_dly[j] <= s_dly[j-1];
        end
        vp    <= rst_n ? {vp[LM-2:0], valid_in} : '0;
        tp[0] <= tag_in;
        for (int j = 1; j < LM; j++) begin
            tp[j] <= tp[j-1];
        end
    end

    assign s_out     = s_dly[ALIGN-1];
    assign m01c_out  = m_tree;
    assign valid_out = vp[LM-1];
    assign tag_out   = tp[LM-1];
endmodule
