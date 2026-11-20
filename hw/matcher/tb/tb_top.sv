// tb_top.sv - testbench for orb_match_top.
//
// For each test (prepared in C by tb_dpi.c):
//   1. write N_TRAIN, N_QUERY, TH_DIST, FLAGS over AXI-Lite, then CTRL.start
//   2. stream the input words in (AXI-Stream, with random tvalid gaps)
//   3. collect the output words (random tready stalls)
//   4. wait for CTRL.done, read N_MATCHES, let C compare with the golden model
// Real EuRoC pairs run without stalls, so their cycle counts are the real ones.
`timescale 1ns / 1ps

module tb_top;

    import "DPI-C" function int      tb_num_tests();
    import "DPI-C" function int      tb_prepare(input int t);
    import "DPI-C" function int      tb_cfg(input int which);
    import "DPI-C" function int      tb_stream_len();
    import "DPI-C" function longint  tb_stream_word(input int k);
    import "DPI-C" function void     tb_out(input int k, input int word);
    import "DPI-C" function int      tb_check(input int nm, input int n_out, input int cycles);

    // ------------------------------------------------------------ DUT
    logic        aclk = 0, aresetn = 0;
    always #1.667 aclk = ~aclk;                 // 300 MHz

    logic [5:0]  awaddr = 0, araddr = 0;
    logic        awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    logic [31:0] wdata = 0;
    logic        awready, wready, bvalid, arready, rvalid;
    logic [1:0]  bresp, rresp;
    logic [31:0] rdata;

    logic [63:0] s_tdata = 0;
    logic        s_tvalid = 0, s_tready;
    logic [31:0] m_tdata;
    logic        m_tvalid, m_tlast;
    logic        m_tready = 0;

    orb_match_top dut (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready),
        .m_axis_tlast(m_tlast)
    );

    // ------------------------------------------------------------ AXI-Lite
    // Inputs change on the falling edge, the DUT samples on the rising edge.
    task automatic axil_write(input logic [5:0] a, input logic [31:0] d);
        @(negedge aclk);
        awaddr = a;  wdata = d;  awvalid = 1;  wvalid = 1;
        do @(negedge aclk); while (!bvalid);      // accepted at the previous rising edge
        awvalid = 0;  wvalid = 0;  bready = 1;
        @(negedge aclk);
        bready = 0;
    endtask

    task automatic axil_read(input logic [5:0] a, output logic [31:0] d);
        @(negedge aclk);
        araddr = a;  arvalid = 1;
        do @(negedge aclk); while (!rvalid);
        d = rdata;  arvalid = 0;  rready = 1;
        @(negedge aclk);
        rready = 0;
    endtask

    // ------------------------------------------------------------ streams
    int stall_pct;                              // % of cycles with tvalid/tready low

    task automatic drive_stream(input int len);
        int k = 0;
        while (k < len) begin
            @(negedge aclk);
            if (!s_tvalid && $urandom_range(99) >= stall_pct) begin
                s_tdata  = tb_stream_word(k);
                s_tvalid = 1;
            end
            if (s_tvalid && s_tready) begin     // taken at the next rising edge
                @(posedge aclk);
                #0.1;
                s_tvalid = 0;
                k++;
            end
        end
    endtask

    int n_out, last_at, busy_cycles;
    always @(posedge aclk) if (dut.busy) busy_cycles++;
    always @(negedge aclk) m_tready <= ($urandom_range(99) >= stall_pct);
    always @(posedge aclk)
        if (aresetn && m_tvalid && m_tready) begin
            tb_out(n_out, m_tdata);
            if (m_tlast) last_at = n_out;
            n_out++;
        end

    // ------------------------------------------------------------ test loop
    initial begin
        int n_tests, fails, cycles, nq;
        logic [31:0] r;

        repeat (10) @(posedge aclk);
        aresetn = 1;
        n_tests = tb_num_tests();
        fails = 0;

        for (int t = 0; t < n_tests; t++) begin
            if (tb_prepare(t) != 0) begin
                $display("[rtl] test %0d: could not prepare", t);
                fails++;
                continue;
            end
            nq = tb_cfg(1);
            stall_pct = (nq > 1000) ? 0 : 30;   // real pairs at full speed
            axil_write(6'h04, tb_cfg(0));
            axil_write(6'h08, nq);
            axil_write(6'h0C, tb_cfg(2));
            axil_write(6'h10, tb_cfg(3));
            n_out = 0;
            last_at = -1;
            busy_cycles = 0;
            axil_write(6'h00, 1);               // start
            drive_stream(tb_stream_len());
            do axil_read(6'h00, r); while (!r[1]);   // wait for done
            cycles = busy_cycles;
            axil_read(6'h14, r);
            if (nq > 0 && last_at != nq - 1) begin
                $display("[rtl] tlast on output %0d, expected %0d", last_at, nq - 1);
                fails++;
            end
            fails += tb_check(int'(signed'(r[15:0])), n_out, cycles);
        end

        $display("\n%s (%0d tests)", fails ? "*** RTL TEST FAILED ***" : "*** ALL RTL TESTS PASSED ***", n_tests);
        $finish;
    end

endmodule
