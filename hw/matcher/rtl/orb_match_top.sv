// orb_match_top.sv - ORB matcher IP: AXI-Lite registers + AXI-Stream data.
//
// Block design on the KV260: the PS writes the registers over AXI-Lite and an
// AXI DMA streams the input words in (MM2S) and the results out (S2MM).
//
// Registers (32-bit, byte addresses):
//   0x00 CTRL       W: bit0 = start          R: bit0 = busy, bit1 = done (sticky)
//   0x04 N_TRAIN    number of train keypoints (0..8192)
//   0x08 N_QUERY    number of query keypoints (0..8192)
//   0x0C TH_DIST    distance threshold (ORB-SLAM3 init: 50)
//   0x10 FLAGS      bit0 = one_to_one (STEP 5 rule), bit1 = check_orient (STEP 6)
//   0x14 N_MATCHES  R: number of matches of the last run
module orb_match_top
    import orb_match_pkg::*;
(
    input  logic        aclk,
    input  logic        aresetn,

    // ---- AXI4-Lite slave (control)
    input  logic [5:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [5:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    // ---- AXI4-Stream in (64-bit) and out (32-bit)
    input  logic [63:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    output logic [31:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,
    output logic        m_axis_tlast
);

    // ------------------------------------------------------------ registers
    logic [CNT_W-1:0]  n_train, n_query;
    logic [DIST_W-1:0] th_dist;
    logic              one_to_one, check_orient;
    logic              start, busy, done, done_sticky;
    logic [15:0]       n_matches;

    // ---- write channel: accept address and data together, then respond
    logic aw_ok, w_ok;
    assign aw_ok = s_axi_awvalid && !s_axi_bvalid;
    assign w_ok  = s_axi_wvalid  && !s_axi_bvalid;
    assign s_axi_awready = aw_ok && w_ok;
    assign s_axi_wready  = aw_ok && w_ok;
    assign s_axi_bresp   = 2'b00;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_bvalid <= 1'b0;
            start        <= 1'b0;
            n_train      <= '0;
            n_query      <= '0;
            th_dist      <= 9'd50;
            one_to_one       <= 1'b1;
            check_orient <= 1'b1;
            done_sticky  <= 1'b0;
        end else begin
            start <= 1'b0;
            if (done)
                done_sticky <= 1'b1;
            if (aw_ok && w_ok) begin
                s_axi_bvalid <= 1'b1;
                case (s_axi_awaddr[5:2])
                    4'h0: if (s_axi_wdata[0] && !busy) begin
                              start       <= 1'b1;
                              done_sticky <= 1'b0;
                          end
                    4'h1: n_train      <= s_axi_wdata[CNT_W-1:0];
                    4'h2: n_query      <= s_axi_wdata[CNT_W-1:0];
                    4'h3: th_dist      <= s_axi_wdata[DIST_W-1:0];
                    4'h4: {check_orient, one_to_one} <= s_axi_wdata[1:0];
                    default: ;
                endcase
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // ---- read channel
    assign s_axi_arready = !s_axi_rvalid;
    assign s_axi_rresp   = 2'b00;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= '0;
        end else if (s_axi_arvalid && s_axi_arready) begin
            s_axi_rvalid <= 1'b1;
            case (s_axi_araddr[5:2])
                4'h0:    s_axi_rdata <= {30'b0, done_sticky, busy};
                4'h1:    s_axi_rdata <= 32'(n_train);
                4'h2:    s_axi_rdata <= 32'(n_query);
                4'h3:    s_axi_rdata <= 32'(th_dist);
                4'h4:    s_axi_rdata <= {30'b0, check_orient, one_to_one};
                4'h5:    s_axi_rdata <= {16'b0, n_matches};
                default: s_axi_rdata <= '0;
            endcase
        end else if (s_axi_rvalid && s_axi_rready) begin
            s_axi_rvalid <= 1'b0;
        end
    end

    // ------------------------------------------------------------ the matcher
    orb_match_core u_core (
        .clk         (aclk),
        .rst_n       (aresetn),
        .start       (start),
        .n_train     (n_train),
        .n_query     (n_query),
        .th_dist     (th_dist),
        .one_to_one      (one_to_one),
        .check_orient(check_orient),
        .busy        (busy),
        .done        (done),
        .n_matches   (n_matches),
        .s_tdata     (s_axis_tdata),
        .s_tvalid    (s_axis_tvalid),
        .s_tready    (s_axis_tready),
        .m_tdata     (m_axis_tdata),
        .m_tvalid    (m_axis_tvalid),
        .m_tready    (m_axis_tready),
        .m_tlast     (m_axis_tlast)
    );

endmodule
