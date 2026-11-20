// ============================================================================
// tb_record_writer.sv
// Self-checking testbench for orb_record_writer.sv:
//   Test 1: Empty frame (0 records -> 8 header words, TLAST on h7)
//   Test 2: Buffer overflow + 50% random backpressure + AXI4-Stream stability
//   Test 3: Mid-frame reset during ST_EMIT_REC + clean recovery
//   Test 4: Real RTL dump (459 features from hw_sys_features.txt) -> checks all
//           2303 words and writes sw/host/frame0_rtl.bin (18424 bytes)
//   Test 5: Live pairwise integration (orb_frontend_top -> orb_record_writer)
//           on a 96x84 corner frame with input bubbles and AXI backpressure
// ============================================================================
`timescale 1ns/1ps

module tb_record_writer;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    localparam logic [63:0] EXP_HDR0     = {16'd0, vslam_cfg_pkg::FMT_VER, vslam_cfg_pkg::MAGIC_ORBK};
    localparam int          B_REC_AW     = vslam_cfg_pkg::REC_AW;
    localparam int          B_MAX_REC    = 1 << B_REC_AW;
    localparam int          MAX_RX_WORDS = 8 + 5 * B_MAX_REC;

    int total_err = 0;

    // Helper: compute expected angle16 matching orb_record_writer
    function automatic logic [15:0] exp_angle16(input logic [1:0] q, input logic [3:0] s);
        case (q)
            2'd1:    exp_angle16 = {2'b00,  s, 10'd512};
            2'd0:    exp_angle16 = {2'b01, ~s, 10'd512};
            2'd3:    exp_angle16 = {2'b10,  s, 10'd512};
            default: exp_angle16 = {2'b11, ~s, 10'd512};
        endcase
    endfunction

    // ========================================================================
    // DUT A: Small capacity (REC_AW=3 -> 8 records) for Tests 1, 2, 3
    // ========================================================================
    logic         a_rst = 1'b1;
    logic         a_fstart = 1'b0, a_fdone = 1'b0;
    logic [31:0]  a_fid = 32'd0;
    logic [63:0]  a_fts = 64'd0;
    logic         a_fvalid = 1'b0;
    logic [1:0]   a_flvl = 2'd0;
    logic [11:0]  a_fx = 12'd0, a_fy = 12'd0, a_fsc = 12'd0;
    logic [1:0]   a_fq = 2'd0;
    logic [3:0]   a_fsec = 4'd0;
    logic [255:0] a_fdesc = 256'd0;
    logic         a_tvalid, a_tready = 1'b1, a_tlast, a_busy;
    logic [63:0]  a_tdata;
    logic [15:0]  a_ndrop;

    orb_record_writer #(.IMG_W(320), .IMG_H(240), .REC_AW(3)) dut_a (
        .clk(clk), .rst(a_rst),
        .frame_start(a_fstart), .frame_done(a_fdone),
        .frame_id(a_fid), .frame_timestamp_ns(a_fts),
        .feat_valid(a_fvalid), .feat_level(a_flvl),
        .feat_x(a_fx), .feat_y(a_fy), .feat_score(a_fsc),
        .feat_quad(a_fq), .feat_sector(a_fsec), .feat_desc(a_fdesc),
        .m_axis_tvalid(a_tvalid), .m_axis_tready(a_tready),
        .m_axis_tdata(a_tdata), .m_axis_tlast(a_tlast),
        .busy(a_busy), .n_dropped(a_ndrop)
    );

    // AXI4-Stream stability monitor on DUT A (valid never drops, data/last stable while !ready)
    logic        a_prev_v = 1'b0, a_prev_r = 1'b0, a_prev_last = 1'b0;
    logic [63:0] a_prev_data = 64'd0;
    logic        a_chk_axis = 1'b0;

    always @(posedge clk) begin
        if (a_rst || !a_chk_axis) begin
            a_prev_v <= 1'b0;
        end else begin
            if (a_prev_v && !a_prev_r) begin
                if (!a_tvalid) begin
                    $display("ERR: AXI tvalid dropped while tready=0");
                    total_err++;
                end
                if (a_tdata !== a_prev_data || a_tlast !== a_prev_last) begin
                    $display("ERR: AXI tdata/tlast changed while tvalid && !tready");
                    total_err++;
                end
            end
            a_prev_v    <= a_tvalid;
            a_prev_r    <= a_tready;
            a_prev_data <= a_tdata;
            a_prev_last <= a_tlast;
        end
    end

    // ========================================================================
    // DUT B: Full capacity (REC_AW=10 -> 1024 records) for Real RTL Dump (Test 4)
    // ========================================================================
    logic         b_rst = 1'b1;
    logic         b_fstart = 1'b0, b_fdone = 1'b0;
    logic [31:0]  b_fid = 32'd0;
    logic [63:0]  b_fts = 64'd0;
    logic         b_fvalid = 1'b0;
    logic [1:0]   b_flvl = 2'd0;
    logic [11:0]  b_fx = 12'd0, b_fy = 12'd0, b_fsc = 12'd0;
    logic [1:0]   b_fq = 2'd0;
    logic [3:0]   b_fsec = 4'd0;
    logic [255:0] b_fdesc = 256'd0;
    logic         b_tvalid, b_tready = 1'b1, b_tlast, b_busy;
    logic [63:0]  b_tdata;
    logic [15:0]  b_ndrop;

    orb_record_writer #(
        .IMG_W  (vslam_cfg_pkg::SIM_IMG_W),
        .IMG_H  (vslam_cfg_pkg::SIM_IMG_H),
        .REC_AW (B_REC_AW)
    ) dut_b (
        .clk(clk), .rst(b_rst),
        .frame_start(b_fstart), .frame_done(b_fdone),
        .frame_id(b_fid), .frame_timestamp_ns(b_fts),
        .feat_valid(b_fvalid), .feat_level(b_flvl),
        .feat_x(b_fx), .feat_y(b_fy), .feat_score(b_fsc),
        .feat_quad(b_fq), .feat_sector(b_fsec), .feat_desc(b_fdesc),
        .m_axis_tvalid(b_tvalid), .m_axis_tready(b_tready),
        .m_axis_tdata(b_tdata), .m_axis_tlast(b_tlast),
        .busy(b_busy), .n_dropped(b_ndrop)
    );

    // ========================================================================
    // DUT C: Live integration orb_frontend_top (96x84) -> orb_record_writer
    // ========================================================================
    localparam int CW = 96;
    localparam int CH = 84;
    logic       c_rst = 1'b1;
    logic       c_in_valid = 1'b0;
    logic [7:0] c_in_pixel = 8'd0;
    logic       c_fe_valid, c_fe_busy;
    logic [1:0] c_fe_lvl, c_fe_q;
    logic [11:0] c_fe_x, c_fe_y, c_fe_sc;
    logic [3:0]  c_fe_sec;
    logic [255:0] c_fe_desc;

    logic        c_fstart = 1'b0, c_fdone = 1'b0;
    logic        c_tvalid, c_tready = 1'b1, c_tlast, c_rw_busy;
    logic [63:0] c_tdata;
    logic [15:0] c_ndrop;

    orb_frontend_top #(
        .IMG_W            (CW),
        .IMG_H            (CH),
        .ROUND            (vslam_cfg_pkg::PYR_ROUND),
        .ENABLE_EXTRACTOR (1),
        .N_SECT           (vslam_cfg_pkg::N_SECT),
        .KP_FIFO_AW       (8),
        .WIN_FIFO_AW      (vslam_cfg_pkg::WIN_FIFO_AW),
        .OUT_FIFO_AW      (vslam_cfg_pkg::OUT_FIFO_AW),
        .PATTERN_FILE     (vslam_cfg_pkg::PATTERN_FILE)
    ) dut_c_fe (
        .clk(clk), .rst(c_rst),
        .in_valid(c_in_valid), .in_pixel(c_in_pixel), .thr(8'(vslam_cfg_pkg::FAST_THR)),
        .l0_g_valid(), .l0_g_pixel(), .l0_fs_valid(), .l0_fs_score(), .l0_kp_valid(), .l0_kp_data(),
        .l1_g_valid(), .l1_g_pixel(), .l1_fs_valid(), .l1_fs_score(), .l1_kp_valid(), .l1_kp_data(),
        .l2_g_valid(), .l2_g_pixel(), .l2_fs_valid(), .l2_fs_score(), .l2_kp_valid(), .l2_kp_data(),
        .l3_g_valid(), .l3_g_pixel(), .l3_fs_valid(), .l3_fs_score(), .l3_kp_valid(), .l3_kp_data(),
        .l0_feat_valid(), .l0_feat_x(), .l0_feat_y(), .l0_feat_score(), .l0_feat_quad(), .l0_feat_sector(), .l0_feat_desc(),
        .l1_feat_valid(), .l1_feat_x(), .l1_feat_y(), .l1_feat_score(), .l1_feat_quad(), .l1_feat_sector(), .l1_feat_desc(),
        .l2_feat_valid(), .l2_feat_x(), .l2_feat_y(), .l2_feat_score(), .l2_feat_quad(), .l2_feat_sector(), .l2_feat_desc(),
        .l3_feat_valid(), .l3_feat_x(), .l3_feat_y(), .l3_feat_score(), .l3_feat_quad(), .l3_feat_sector(), .l3_feat_desc(),
        .feat_valid(c_fe_valid), .feat_level(c_fe_lvl),
        .feat_x(c_fe_x), .feat_y(c_fe_y), .feat_score(c_fe_sc),
        .feat_quad(c_fe_q), .feat_sector(c_fe_sec), .feat_desc(c_fe_desc),
        .busy(c_fe_busy)
    );

    orb_record_writer #(.IMG_W(CW), .IMG_H(CH), .REC_AW(8)) dut_c_rw (
        .clk(clk), .rst(c_rst),
        .frame_start(c_fstart), .frame_done(c_fdone),
        .frame_id(32'd77), .frame_timestamp_ns(64'd987654321),
        .feat_valid(c_fe_valid), .feat_level(c_fe_lvl),
        .feat_x(c_fe_x), .feat_y(c_fe_y), .feat_score(c_fe_sc),
        .feat_quad(c_fe_q), .feat_sector(c_fe_sec), .feat_desc(c_fe_desc),
        .m_axis_tvalid(c_tvalid), .m_axis_tready(c_tready),
        .m_axis_tdata(c_tdata), .m_axis_tlast(c_tlast),
        .busy(c_rw_busy), .n_dropped(c_ndrop)
    );

    // ========================================================================
    // Test helpers & buffers
    // ========================================================================
    logic [63:0] rx_words [0:MAX_RX_WORDS-1];
    int          rx_cnt;
    int          seed = 32'h1357_2468;

    task automatic collect_a(input int max_words, input bit backpressure);
        int cyc;
        begin
            rx_cnt = 0;
            cyc    = 0;
            while (cyc < 2000) begin
                a_tready <= backpressure ? (($random(seed) & 1) == 1) : 1'b1;
                @(posedge clk);
                if (a_tvalid && a_tready) begin
                    if (rx_cnt < max_words) rx_words[rx_cnt] = a_tdata;
                    rx_cnt++;
                    if (a_tlast) begin
                        a_tready <= 1'b1;
                        @(posedge clk);
                        cyc = 2000;
                    end
                end
                cyc++;
            end
        end
    endtask

    // Feature vectors loaded from hw_sys_features.txt
    int           n_sys = 0;
    logic [1:0]   sys_lvl  [0:B_MAX_REC-1];
    logic [11:0]  sys_x    [0:B_MAX_REC-1], sys_y [0:B_MAX_REC-1], sys_sc [0:B_MAX_REC-1];
    logic [1:0]   sys_q    [0:B_MAX_REC-1];
    logic [3:0]   sys_sec  [0:B_MAX_REC-1];
    logic [255:0] sys_desc [0:B_MAX_REC-1];

    task automatic write_u64_le(input int fd, input logic [63:0] w);
        begin
            $fwrite(fd, "%c%c%c%c%c%c%c%c",
                    w[7:0], w[15:8], w[23:16], w[31:24],
                    w[39:32], w[47:40], w[55:48], w[63:56]);
        end
    endtask

    initial begin
        // ---- Reset DUT A & DUT B ----
        a_rst = 1'b1; b_rst = 1'b1; c_rst = 1'b1;
        a_chk_axis = 1'b0;
        repeat (5) @(posedge clk);
        a_rst <= 1'b0; b_rst <= 1'b0;
        repeat (2) @(posedge clk);
        a_chk_axis <= 1'b1;

        // ====================================================================
        // Test 1: Empty frame
        // ====================================================================
        a_fid <= 32'd1; a_fts <= 64'h11223344_55667788; a_fstart <= 1'b1;
        @(posedge clk);
        a_fstart <= 1'b0; a_fdone <= 1'b1;
        @(posedge clk);
        a_fdone <= 1'b0;
        collect_a(64, 1'b0);

        if (rx_cnt !== 8) begin
            $display("T1 ERR: expected 8 words, got %0d", rx_cnt); total_err++;
        end
        if (rx_words[0] !== EXP_HDR0) begin $display("T1 ERR h0=%016x", rx_words[0]); total_err++; end
        if (rx_words[1] !== 64'h0000_0000_0000_0001) begin $display("T1 ERR h1=%016x", rx_words[1]); total_err++; end
        if (rx_words[2] !== 64'h1122_3344_5566_7788) begin $display("T1 ERR h2=%016x", rx_words[2]); total_err++; end
        if (rx_words[3] !== {16'd0, 16'd0, 16'd240, 16'd320}) begin $display("T1 ERR h3=%016x", rx_words[3]); total_err++; end
        $display("Test 1 (empty frame): words=%0d err=%0d", rx_cnt, total_err);

        // ====================================================================
        // Test 2: Overflow (11 pushed, cap=8) + 50% random backpressure
        // ====================================================================
        a_fid <= 32'd42; a_fts <= 64'hAABBCCDD_EEFF0011; a_fstart <= 1'b1;
        @(posedge clk);
        a_fstart <= 1'b0;

        for (int i = 0; i < 11; i++) begin
            a_fvalid <= 1'b1;
            a_flvl   <= i[1:0];
            a_fx     <= 12'(20 + i);
            a_fy     <= 12'(30 + i);
            a_fsc    <= 12'(500 + i);
            a_fq     <= i[1:0];
            a_fsec   <= i[3:0];
            a_fdesc  <= {8{32'(32'hA500_0000 + i)}};
            @(posedge clk);
        end
        a_fvalid <= 1'b0;
        @(posedge clk);
        if (a_ndrop !== 16'd3) begin $display("T2 ERR n_dropped=%0d exp 3", a_ndrop); total_err++; end
        a_fdone <= 1'b1;
        @(posedge clk);
        a_fdone <= 1'b0;

        collect_a(128, 1'b1);
        if (rx_cnt !== 8 + 8 * 5) begin $display("T2 ERR rx_cnt=%0d exp 48", rx_cnt); total_err++; end
        if (rx_words[1] !== {32'd8, 32'd42}) begin $display("T2 ERR h1=%016x", rx_words[1]); total_err++; end
        if (rx_words[3] !== {16'd1, 16'd3, 16'd240, 16'd320}) begin $display("T2 ERR h3=%016x", rx_words[3]); total_err++; end
        if (rx_words[4] !== {16'd2, 16'd2, 16'd2, 16'd2}) begin $display("T2 ERR h4=%016x", rx_words[4]); total_err++; end
        for (int i = 0; i < 8; i++) begin
            logic [63:0] exp_w4;
            exp_w4 = {4'd0, exp_angle16(i[1:0], i[3:0]), 4'd0, 12'(500 + i),
                      1'b1, 1'b0, i[1:0], 12'(30 + i), 12'(20 + i)};
            if (rx_words[8 + i*5 + 0] !== {2{32'(32'hA500_0000 + i)}}) total_err++;
            if (rx_words[8 + i*5 + 4] !== exp_w4) begin
                $display("T2 ERR rec %0d w4=%016x exp=%016x", i, rx_words[8 + i*5 + 4], exp_w4);
                total_err++;
            end
        end
        $display("Test 2 (overflow + backpressure): words=%0d err=%0d", rx_cnt, total_err);

        // ====================================================================
        // Test 3: Mid-frame reset during ST_EMIT_REC + clean recovery
        // ====================================================================
        a_fid <= 32'd99; a_fstart <= 1'b1; @(posedge clk); a_fstart <= 1'b0;
        for (int i = 0; i < 4; i++) begin
            a_fvalid <= 1'b1; a_fx <= 12'(i); a_fy <= 12'(i); a_fsc <= 12'd100;
            a_fdesc  <= {4{64'hDEAD_BEEF_0000_0000}};
            @(posedge clk);
        end
        a_fvalid <= 1'b0; a_fdone <= 1'b1; @(posedge clk); a_fdone <= 1'b0;
        a_tready <= 1'b1;
        repeat (10) @(posedge clk); // inside ST_EMIT_REC
        a_chk_axis <= 1'b0;
        a_rst      <= 1'b1;
        repeat (3) @(posedge clk);
        a_rst      <= 1'b0;
        @(posedge clk);
        if (a_tvalid !== 1'b0 || a_busy !== 1'b0) begin
            $display("T3 ERR not cleared on reset: tvalid=%0b busy=%0b", a_tvalid, a_busy);
            total_err++;
        end
        a_chk_axis <= 1'b1;
        a_fid <= 32'd100; a_fstart <= 1'b1; @(posedge clk); a_fstart <= 1'b0;
        for (int i = 0; i < 2; i++) begin
            a_fvalid <= 1'b1; a_flvl <= 2'd1; a_fx <= 12'(50 + i); a_fy <= 12'(60 + i);
            a_fsc <= 12'd777; a_fq <= 2'd2; a_fsec <= 4'd5; a_fdesc <= 256'hCAFE;
            @(posedge clk);
        end
        a_fvalid <= 1'b0; a_fdone <= 1'b1; @(posedge clk); a_fdone <= 1'b0;
        collect_a(64, 1'b0);
        if (rx_cnt !== 18 || rx_words[1] !== {32'd2, 32'd100}) begin
            $display("T3 ERR recovery rx_cnt=%0d h1=%016x", rx_cnt, rx_words[1]);
            total_err++;
        end
        $display("Test 3 (mid-frame reset + recovery): words=%0d err=%0d", rx_cnt, total_err);

        // ====================================================================
        // Test 4: Real RTL dump (hw_sys_features.txt) -> sw/host/frame0_rtl.bin
        // ====================================================================
        begin
            int fd_in, fd_bin, rc;
            int lv, x, y, sc, q, sec;
            logic [255:0] dsc;
            int exp_lc [0:3];
            int cyc_b;

            exp_lc[0] = 0; exp_lc[1] = 0; exp_lc[2] = 0; exp_lc[3] = 0;
            fd_in = $fopen("hw_sys_features.txt", "r");
            if (fd_in == 0) begin
                $display("T4 ERR: cannot open hw_sys_features.txt");
                total_err++;
            end else begin
                while (!$feof(fd_in)) begin
                    rc = $fscanf(fd_in, "%d %d %d %d %d %d %h\n", lv, x, y, sc, q, sec, dsc);
                    if (rc == 7 && n_sys < B_MAX_REC) begin
                        sys_lvl[n_sys]  = lv[1:0];
                        sys_x[n_sys]    = x[11:0];
                        sys_y[n_sys]    = y[11:0];
                        sys_sc[n_sys]   = sc[11:0];
                        sys_q[n_sys]    = q[1:0];
                        sys_sec[n_sys]  = sec[3:0];
                        sys_desc[n_sys] = dsc;
                        exp_lc[lv[1:0]]++;
                        n_sys++;
                    end
                end
                $fclose(fd_in);
            end

            b_fid <= 32'd0; b_fts <= 64'd1000000; b_fstart <= 1'b1;
            @(posedge clk);
            b_fstart <= 1'b0;
            for (int i = 0; i < n_sys; i++) begin
                b_fvalid <= 1'b1;
                b_flvl   <= sys_lvl[i];
                b_fx     <= sys_x[i];
                b_fy     <= sys_y[i];
                b_fsc    <= sys_sc[i];
                b_fq     <= sys_q[i];
                b_fsec   <= sys_sec[i];
                b_fdesc  <= sys_desc[i];
                @(posedge clk);
            end
            b_fvalid <= 1'b0;
            b_fdone  <= 1'b1;
            @(posedge clk);
            b_fdone  <= 1'b0;

            fd_bin = $fopen("../../../sw/host/frame0_rtl.bin", "wb");
            rx_cnt = 0;
            cyc_b  = 0;
            while (cyc_b < MAX_RX_WORDS * 4) begin
                b_tready <= (($random(seed) & 3) != 0); // 75% ready
                @(posedge clk);
                if (b_tvalid && b_tready) begin
                    rx_words[rx_cnt] = b_tdata;
                    if (fd_bin != 0) write_u64_le(fd_bin, b_tdata);
                    rx_cnt++;
                    if (b_tlast) begin
                        b_tready <= 1'b1;
                        @(posedge clk);
                        cyc_b = MAX_RX_WORDS * 4;
                    end
                end
                cyc_b++;
            end
            if (fd_bin != 0) $fclose(fd_bin);

            if (rx_cnt !== 8 + n_sys * 5) begin
                $display("T4 ERR: rx_cnt=%0d exp=%0d", rx_cnt, 8 + n_sys * 5);
                total_err++;
            end
            if (rx_words[1] !== {32'(n_sys), 32'd0}) begin $display("T4 ERR h1"); total_err++; end
            if (rx_words[4] !== {exp_lc[3][15:0], exp_lc[2][15:0], exp_lc[1][15:0], exp_lc[0][15:0]}) begin
                $display("T4 ERR h4=%016x", rx_words[4]); total_err++;
            end
            for (int i = 0; i < n_sys; i++) begin
                logic [63:0] exp_w4;
                exp_w4 = {4'd0, exp_angle16(sys_q[i], sys_sec[i]), 4'd0, sys_sc[i],
                          1'b1, 1'b0, sys_lvl[i], sys_y[i], sys_x[i]};
                if (rx_words[8 + i*5 + 0] !== sys_desc[i][63:0])    total_err++;
                if (rx_words[8 + i*5 + 1] !== sys_desc[i][127:64])  total_err++;
                if (rx_words[8 + i*5 + 2] !== sys_desc[i][191:128]) total_err++;
                if (rx_words[8 + i*5 + 3] !== sys_desc[i][255:192]) total_err++;
                if (rx_words[8 + i*5 + 4] !== exp_w4)               total_err++;
            end
            $display("Test 4 (real RTL dump %0d feats -> sw/host/frame0_rtl.bin): words=%0d err=%0d",
                     n_sys, rx_cnt, total_err);
        end

        // ====================================================================
        // Test 5: Live Pairwise RTL (orb_frontend_top 96x84 -> orb_record_writer)
        // ====================================================================
        begin
            int fe_cnt;
            int cyc_c;
            logic [7:0] pix;
            fe_cnt = 0;
            cyc_c  = 0;

            c_rst <= 1'b1;
            repeat (5) @(posedge clk);
            c_rst <= 1'b0;
            repeat (2) @(posedge clk);

            c_fstart <= 1'b1; @(posedge clk); c_fstart <= 1'b0;

            for (int y = 0; y < CH; y++) begin
                for (int x = 0; x < CW; x++) begin
                    while ((($random(seed) & 32'h7fffffff) % 8) == 0) begin
                        c_in_valid <= 1'b0;
                        @(posedge clk);
                        if (c_fe_valid) fe_cnt++;
                    end
                    // High-contrast blocks inside valid [21..74, 21..62] region
                    pix = (((x >= 32 && x <= 48 && y >= 28 && y <= 44) ||
                            (x >= 52 && x <= 68 && y >= 36 && y <= 54)) ? 8'd230 : 8'd25);
                    c_in_valid <= 1'b1;
                    c_in_pixel <= pix;
                    @(posedge clk);
                    if (c_fe_valid) fe_cnt++;
                end
            end
            c_in_valid <= 1'b0;

            cyc_c = 0;
            while (c_fe_busy && cyc_c < 80000) begin
                @(posedge clk);
                if (c_fe_valid) fe_cnt++;
                cyc_c++;
            end
            repeat (5) begin
                @(posedge clk);
                if (c_fe_valid) fe_cnt++;
            end

            c_fdone <= 1'b1; @(posedge clk); c_fdone <= 1'b0;

            rx_cnt = 0;
            cyc_c  = 0;
            while (cyc_c < 20000) begin
                c_tready <= (($random(seed) & 1) == 1);
                @(posedge clk);
                if (c_tvalid && c_tready) begin
                    rx_words[rx_cnt] = c_tdata;
                    rx_cnt++;
                    if (c_tlast) begin
                        c_tready <= 1'b1;
                        @(posedge clk);
                        cyc_c = 20000;
                    end
                end
                cyc_c++;
            end

            if (fe_cnt <= 0 || rx_cnt !== 8 + fe_cnt * 5) begin
                $display("T5 ERR: fe_cnt=%0d rx_cnt=%0d", fe_cnt, rx_cnt);
                total_err++;
            end
            if (rx_words[0] !== EXP_HDR0 ||
                rx_words[1] !== {32'(fe_cnt), 32'd77} ||
                c_ndrop !== 16'd0) begin
                $display("T5 ERR header mismatch h0=%016x h1=%016x ndrop=%0d",
                         rx_words[0], rx_words[1], c_ndrop);
                total_err++;
            end
            for (int i = 0; i < fe_cnt; i++) begin
                logic [63:0] w4;
                w4 = rx_words[8 + i*5 + 4];
                if (w4[27] !== 1'b1) begin $display("T5 ERR invalid V bit"); total_err++; end
            end
            $display("Test 5 (live orb_frontend_top -> orb_record_writer): feats=%0d words=%0d err=%0d",
                     fe_cnt, rx_cnt, total_err);
        end

        if (total_err == 0) begin
            $display("TB_RESULT: PASS");
        end else begin
            $display("TB_RESULT: FAIL (%0d errors)", total_err);
        end
        $finish;
    end

endmodule
