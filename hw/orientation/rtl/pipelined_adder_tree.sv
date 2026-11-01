// -----------------------------------------------------------------------------
// pipelined_adder_tree.sv
// Generic pipelined adder tree: one register stage per tree level.
//   N      number of inputs (>= 2)
//   W      width of each input
//   SIGNED 1: inputs are two's complement, 0: unsigned
// Latency = L = ceil(log2(N)) clocks, output width OW = W + L (no overflow).
// din packs the inputs: element i is din[i*W +: W].   (L and OW: do not override)
// -----------------------------------------------------------------------------
module pipelined_adder_tree #(
    parameter int N      = 37,
    parameter int W      = 8,
    parameter bit SIGNED = 1'b0,
    parameter int L      = $clog2(N),
    parameter int OW     = W + L
)(
    input  logic                 clk,
    input  logic [N*W-1:0]       din,
    output logic signed [OW-1:0] dout
);
    localparam int P = 1 << L;                 // inputs padded to a power of two
    logic signed [OW-1:0] st [0:P-2];          // registers of all levels (P-1 in total)

    genvar l, i;
    generate
        for (l = 1; l <= L; l++) begin : g_lvl
            localparam int OFF  = P - (P >> (l - 1));                   // first reg of level l
            localparam int PREV = (l == 1) ? 0 : (P - (P >> (l - 2)));  // first reg of level l-1
            for (i = 0; i < (P >> l); i++) begin : g_add
                logic signed [OW-1:0] a, b;
                if (l == 1) begin : g_in
                    if (2*i < N) begin : g_a
                        logic [W-1:0] r;
                        assign r = din[(2*i)*W +: W];
                        assign a = SIGNED ? {{L{r[W-1]}}, r} : {{L{1'b0}}, r};
                    end else begin : g_a0
                        assign a = '0;
                    end
                    if (2*i + 1 < N) begin : g_b
                        logic [W-1:0] r;
                        assign r = din[(2*i+1)*W +: W];
                        assign b = SIGNED ? {{L{r[W-1]}}, r} : {{L{1'b0}}, r};
                    end else begin : g_b0
                        assign b = '0;
                    end
                end else begin : g_mid
                    assign a = st[PREV + 2*i];
                    assign b = st[PREV + 2*i + 1];
                end
                always_ff @(posedge clk) begin
                    st[OFF + i] <= a + b;
                end
            end
        end
    endgenerate

    assign dout = st[P-2];                 // single register of the last level
endmodule
