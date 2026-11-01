// -----------------------------------------------------------------------------
// orient_moments.sv  --  delay line + recursive moment registers (4.5.2 / 4.5.3)
//
//   m00' = m00 + S(Cin) - S(Cout)
//   m01' = m01 + m01(Cin) - m01(Cout)
//   m10' = m10 + m00 - 18*S(Cin) - 19*S(Cout)
//
// C_out terms are C_in terms delayed by 37 valid columns (shift register), so the
// adder trees are not duplicated.  Window frame: newest column at x=-18, i.e. m10
// is mirrored w.r.t. the image (m10_hw = -m10_image).
//
// Two stages keep the feedback loop short:
//   U0: read delay line, push new terms, form  corr10 = 18*(Sin+Sout)+Sout,
//       d00 = Sin-Sout, d01 = m01in-m01out    (no dependence on the registers)
//   U1: m00 += d00;  m01 += d01;  m10 += m00 - corr10
// Latency 2 clocks.  Registers and delay line are cleared by rst_n (they must
// always be consistent with each other).
// -----------------------------------------------------------------------------
module orient_moments #(
    parameter int PIX_W = 8,
    parameter int TAG_W = 1,
    parameter int SW    = PIX_W + 6,
    parameter int MCW   = PIX_W + 11,
    parameter int M0W   = PIX_W + 11,     // m00 width (unsigned)
    parameter int MW    = PIX_W + 14      // m10 / m01 width (signed)
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    valid_in,
    input  logic [SW-1:0]           s_in,
    input  logic signed [MCW-1:0]   m01c_in,
    input  logic [TAG_W-1:0]        tag_in,
    output logic                    valid_out,
    output logic [M0W-1:0]          m00_out,
    output logic signed [MW-1:0]    m10_out,
    output logic signed [MW-1:0]    m01_out,
    output logic [TAG_W-1:0]        tag_out
);
    localparam int D = 37;                                   // delay-line depth

    logic [SW-1:0]          dl_s [0:D-1];
    logic signed [MCW-1:0]  dl_m [0:D-1];

    logic signed [MW-1:0] s_in_x, s_old_x, m_in_x, m_old_x, t;
    assign s_in_x  = {{(MW-SW){1'b0}}, s_in};
    assign s_old_x = {{(MW-SW){1'b0}}, dl_s[D-1]};   // S(Cout)
    assign m_in_x  = m01c_in;
    assign m_old_x = dl_m[D-1];                      // m01(Cout)
    assign t       = s_in_x + s_old_x;

    // ---- U0 -----------------------------------------------------------------
    logic signed [MW-1:0] corr10, d00, d01;
    logic                 v_u0;
    logic [TAG_W-1:0]     tag_u0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int k = 0; k < D; k++) begin
                dl_s[k] <= '0;
                dl_m[k] <= '0;
            end
            v_u0 <= 1'b0;
        end else begin
            v_u0 <= valid_in;
            if (valid_in) begin
                dl_s[0] <= s_in;
                dl_m[0] <= m01c_in;
                for (int k = 1; k < D; k++) begin
                    dl_s[k] <= dl_s[k-1];
                    dl_m[k] <= dl_m[k-1];
                end
            end
        end
        tag_u0 <= tag_in;
        corr10 <= (t <<< 4) + (t <<< 1) + s_old_x;      // 18*Sin + 19*Sout
        d00    <= s_in_x - s_old_x;
        d01    <= m_in_x - m_old_x;
    end

    // ---- U1 -----------------------------------------------------------------
    logic signed [MW-1:0] r00, r10, r01;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r00       <= '0;
            r10       <= '0;
            r01       <= '0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= v_u0;
            if (v_u0) begin
                r00 <= r00 + d00;
                r10 <= r10 + r00 - corr10;              // uses the *old* m00
                r01 <= r01 + d01;
            end
        end
        tag_out <= tag_u0;
    end

    assign m00_out = r00[M0W-1:0];
    assign m10_out = r10;
    assign m01_out = r01;
endmodule
