// ============================================================================
// tb_system_top.sv
// Self-checking end-to-end system testbench for orb_frontend_top:
//   Phase 1: Mid-frame reset test (stream partial frame, assert rst, verify
//            counters/FIFOs clear and no spurious features leak).
//   Phase 2: Full 144x108 multi-level ORB extraction with periodic input gaps,
//            checking all 4 levels' Gaussian pixel counts, per-level feature
//            streams, and the unified arbitrated feature stream bit-exact
//            against Python golden vectors (orb_system_golden.py).
// Exports hw_sys_features.txt and hw_sys_l0..l3_gauss.hex for visual output.
// ============================================================================
`timescale 1ns/1ps

module tb_system_top;

    parameter int IMG_W       = vslam_cfg_pkg::SIM_IMG_W;
    parameter int IMG_H       = vslam_cfg_pkg::SIM_IMG_H;
    parameter int THR         = vslam_cfg_pkg::FAST_THR;
    parameter int MAX_FEAT    = vslam_cfg_pkg::SIM_MAX_FEAT;
    parameter int TIMEOUT_CYC = vslam_cfg_pkg::SIM_TIMEOUT_CYC;

    localparam int MID_RST_PIX = (IMG_W * IMG_H > 4500) ? 4500 : (IMG_W * IMG_H / 3);

    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam int W0 = IMG_W;   localparam int H0 = IMG_H;
    localparam int W1 = osz(W0); localparam int H1 = osz(H0);
    localparam int W2 = osz(W1); localparam int H2 = osz(H1);
    localparam int W3 = osz(W2); localparam int H3 = osz(H2);

    localparam int GN0 = (W0 - 6) * (H0 - 6);
    localparam int GN1 = (W1 - 6) * (H1 - 6);
    localparam int GN2 = (W2 - 6) * (H2 - 6);
    localparam int GN3 = (W3 - 6) * (H3 - 6);

    localparam int FEAT_W = 298; // {score[11:0], y[11:0], x[11:0], quad[1:0], sector[3:0], desc[255:0]}

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic       rst = 1'b1;
    logic       in_valid = 1'b0;
    logic [7:0] in_pixel = 8'd0;
    logic       capture_en = 1'b0;
    logic       post_reset_chk = 1'b0;

    // Intermediate streams
    logic        l0_g_valid,  l1_g_valid,  l2_g_valid,  l3_g_valid;
    logic [7:0]  l0_g_pixel,  l1_g_pixel,  l2_g_pixel,  l3_g_pixel;
    logic        l0_fs_valid, l1_fs_valid, l2_fs_valid, l3_fs_valid;
    logic [11:0] l0_fs_score, l1_fs_score, l2_fs_score, l3_fs_score;
    logic        l0_kp_valid, l1_kp_valid, l2_kp_valid, l3_kp_valid;
    logic [35:0] l0_kp_data,  l1_kp_data,  l2_kp_data,  l3_kp_data;

    // Per-level feature outputs
    logic         l0_feat_valid, l1_feat_valid, l2_feat_valid, l3_feat_valid;
    logic [11:0]  l0_feat_x, l0_feat_y, l0_feat_score;
    logic [1:0]   l0_feat_quad;
    logic [3:0]   l0_feat_sector;
    logic [255:0] l0_feat_desc;

    logic [11:0]  l1_feat_x, l1_feat_y, l1_feat_score;
    logic [1:0]   l1_feat_quad;
    logic [3:0]   l1_feat_sector;
    logic [255:0] l1_feat_desc;

    logic [11:0]  l2_feat_x, l2_feat_y, l2_feat_score;
    logic [1:0]   l2_feat_quad;
    logic [3:0]   l2_feat_sector;
    logic [255:0] l2_feat_desc;

    logic [11:0]  l3_feat_x, l3_feat_y, l3_feat_score;
    logic [1:0]   l3_feat_quad;
    logic [3:0]   l3_feat_sector;
    logic [255:0] l3_feat_desc;

    // Unified arbitrated feature output
    logic         feat_valid;
    logic [1:0]   feat_level;
    logic [11:0]  feat_x, feat_y, feat_score;
    logic [1:0]   feat_quad;
    logic [3:0]   feat_sector;
    logic [255:0] feat_desc;
    logic         busy;

    orb_frontend_top #(
        .IMG_W            (IMG_W),
        .IMG_H            (IMG_H),
        .ROUND            (vslam_cfg_pkg::PYR_ROUND),
        .ENABLE_EXTRACTOR (vslam_cfg_pkg::ENABLE_EXTRACTOR),
        .N_SECT           (vslam_cfg_pkg::N_SECT),
        .KP_FIFO_AW       (vslam_cfg_pkg::KP_FIFO_AW),
        .WIN_FIFO_AW      (vslam_cfg_pkg::SIM_WIN_FIFO_AW),
        .OUT_FIFO_AW      (vslam_cfg_pkg::OUT_FIFO_AW),
        .PATTERN_FILE     (vslam_cfg_pkg::PATTERN_FILE)
    ) dut (
        .clk           (clk),
        .rst           (rst),
        .in_valid      (in_valid),
        .in_pixel      (in_pixel),
        .thr           (THR[7:0]),

        .l0_g_valid    (l0_g_valid),   .l0_g_pixel  (l0_g_pixel),
        .l0_fs_valid   (l0_fs_valid),  .l0_fs_score (l0_fs_score),
        .l0_kp_valid   (l0_kp_valid),  .l0_kp_data  (l0_kp_data),
        .l1_g_valid    (l1_g_valid),   .l1_g_pixel  (l1_g_pixel),
        .l1_fs_valid   (l1_fs_valid),  .l1_fs_score (l1_fs_score),
        .l1_kp_valid   (l1_kp_valid),  .l1_kp_data  (l1_kp_data),
        .l2_g_valid    (l2_g_valid),   .l2_g_pixel  (l2_g_pixel),
        .l2_fs_valid   (l2_fs_valid),  .l2_fs_score (l2_fs_score),
        .l2_kp_valid   (l2_kp_valid),  .l2_kp_data  (l2_kp_data),
        .l3_g_valid    (l3_g_valid),   .l3_g_pixel  (l3_g_pixel),
        .l3_fs_valid   (l3_fs_valid),  .l3_fs_score (l3_fs_score),
        .l3_kp_valid   (l3_kp_valid),  .l3_kp_data  (l3_kp_data),

        .l0_feat_valid (l0_feat_valid), .l0_feat_x (l0_feat_x), .l0_feat_y (l0_feat_y),
        .l0_feat_score (l0_feat_score), .l0_feat_quad (l0_feat_quad), .l0_feat_sector (l0_feat_sector),
        .l0_feat_desc  (l0_feat_desc),

        .l1_feat_valid (l1_feat_valid), .l1_feat_x (l1_feat_x), .l1_feat_y (l1_feat_y),
        .l1_feat_score (l1_feat_score), .l1_feat_quad (l1_feat_quad), .l1_feat_sector (l1_feat_sector),
        .l1_feat_desc  (l1_feat_desc),

        .l2_feat_valid (l2_feat_valid), .l2_feat_x (l2_feat_x), .l2_feat_y (l2_feat_y),
        .l2_feat_score (l2_feat_score), .l2_feat_quad (l2_feat_quad), .l2_feat_sector (l2_feat_sector),
        .l2_feat_desc  (l2_feat_desc),

        .l3_feat_valid (l3_feat_valid), .l3_feat_x (l3_feat_x), .l3_feat_y (l3_feat_y),
        .l3_feat_score (l3_feat_score), .l3_feat_quad (l3_feat_quad), .l3_feat_sector (l3_feat_sector),
        .l3_feat_desc  (l3_feat_desc),

        .feat_valid    (feat_valid),
        .feat_level    (feat_level),
        .feat_x        (feat_x),
        .feat_y        (feat_y),
        .feat_score    (feat_score),
        .feat_quad     (feat_quad),
        .feat_sector   (feat_sector),
        .feat_desc     (feat_desc),
        .busy          (busy)
    );

    // ---- Reference memories ------------------------------------------------
    logic [7:0]        img     [0:IMG_W*IMG_H-1];
    logic [FEAT_W-1:0] exp_l0  [0:MAX_FEAT-1];
    logic [FEAT_W-1:0] exp_l1  [0:MAX_FEAT-1];
    logic [FEAT_W-1:0] exp_l2  [0:MAX_FEAT-1];
    logic [FEAT_W-1:0] exp_l3  [0:MAX_FEAT-1];
    logic [31:0]       exp_cnt [0:3];

    int fd_feat, fd_g0, fd_g1, fd_g2, fd_g3;
    int g_cnt [0:3];

    initial begin
        $readmemh("sys_level0_pixels.hex",   img);
        $readmemh("sys_expected_l0.hex",     exp_l0);
        $readmemh("sys_expected_l1.hex",     exp_l1);
        $readmemh("sys_expected_l2.hex",     exp_l2);
        $readmemh("sys_expected_l3.hex",     exp_l3);
        $readmemh("sys_expected_counts.hex", exp_cnt);

        fd_feat = $fopen("hw_sys_features.txt", "w");
        fd_g0   = $fopen("hw_sys_l0_gauss.hex", "w");
        fd_g1   = $fopen("hw_sys_l1_gauss.hex", "w");
        fd_g2   = $fopen("hw_sys_l2_gauss.hex", "w");
        fd_g3   = $fopen("hw_sys_l3_gauss.hex", "w");
    end

    // ---- Gaussian dump & count (only during Phase 2 capture_en) ------------
    always @(posedge clk) begin
        if (rst || !capture_en) begin
            g_cnt[0] = 0; g_cnt[1] = 0; g_cnt[2] = 0; g_cnt[3] = 0;
        end else begin
            if (l0_g_valid) begin $fdisplay(fd_g0, "%02x", l0_g_pixel); g_cnt[0] = g_cnt[0] + 1; end
            if (l1_g_valid) begin $fdisplay(fd_g1, "%02x", l1_g_pixel); g_cnt[1] = g_cnt[1] + 1; end
            if (l2_g_valid) begin $fdisplay(fd_g2, "%02x", l2_g_pixel); g_cnt[2] = g_cnt[2] + 1; end
            if (l3_g_valid) begin $fdisplay(fd_g3, "%02x", l3_g_pixel); g_cnt[3] = g_cnt[3] + 1; end
        end
    end

    // ---- Per-level & unified feature checkers ------------------------------
    int lvl_cnt [0:3];
    int lvl_err [0:3];
    int arb_cnt [0:3];
    int arb_err = 0;
    int arb_total = 0;
    int reset_leak_err = 0;

    function automatic logic [FEAT_W-1:0] get_exp(input int lvl, input int idx);
        case (lvl)
            0: get_exp = exp_l0[idx];
            1: get_exp = exp_l1[idx];
            2: get_exp = exp_l2[idx];
            default: get_exp = exp_l3[idx];
        endcase
    endfunction

    task automatic check_feat(
        input int           lvl,
        input logic [11:0]  fx,
        input logic [11:0]  fy,
        input logic [11:0]  fsc,
        input logic [1:0]   fq,
        input logic [3:0]   fsec,
        input logic [255:0] fdesc
    );
        logic [FEAT_W-1:0] got_w, exp_w;
        int idx;
        begin
            idx = lvl_cnt[lvl];
            got_w = {fsc, fy, fx, fq, fsec, fdesc};
            if (idx >= exp_cnt[lvl]) begin
                lvl_err[lvl] = lvl_err[lvl] + 1;
                if (lvl_err[lvl] <= 5)
                    $display("[L%0d] EXTRA feature #%0d at (%0d,%0d)", lvl, idx, fx, fy);
            end else begin
                exp_w = get_exp(lvl, idx);
                if (got_w !== exp_w) begin
                    lvl_err[lvl] = lvl_err[lvl] + 1;
                    if (lvl_err[lvl] <= 5) begin
                        $display("[L%0d] MISMATCH #%0d: got (x=%0d y=%0d sc=%0d q=%0d s=%0d) exp (x=%0d y=%0d sc=%0d q=%0d s=%0d)",
                                 lvl, idx, fx, fy, fsc, fq, fsec,
                                 exp_w[273:262], exp_w[285:274], exp_w[297:286], exp_w[261:260], exp_w[259:256]);
                        $display("  got desc = %064x", fdesc);
                        $display("  exp desc = %064x", exp_w[255:0]);
                    end
                end
            end
            lvl_cnt[lvl] = lvl_cnt[lvl] + 1;
        end
    endtask

    logic [FEAT_W-1:0] got_arb_w, exp_arb_w;
    int aidx;

    always @(posedge clk) begin
        if (rst || !capture_en) begin
            for (int l = 0; l < 4; l++) begin
                lvl_cnt[l] = 0;
                lvl_err[l] = 0;
                arb_cnt[l] = 0;
            end
            arb_err   = 0;
            arb_total = 0;
            if (!rst && post_reset_chk && (feat_valid || l0_feat_valid || l1_feat_valid || l2_feat_valid || l3_feat_valid)) begin
                reset_leak_err = reset_leak_err + 1;
            end
        end else begin
            if (l0_feat_valid) check_feat(0, l0_feat_x, l0_feat_y, l0_feat_score, l0_feat_quad, l0_feat_sector, l0_feat_desc);
            if (l1_feat_valid) check_feat(1, l1_feat_x, l1_feat_y, l1_feat_score, l1_feat_quad, l1_feat_sector, l1_feat_desc);
            if (l2_feat_valid) check_feat(2, l2_feat_x, l2_feat_y, l2_feat_score, l2_feat_quad, l2_feat_sector, l2_feat_desc);
            if (l3_feat_valid) check_feat(3, l3_feat_x, l3_feat_y, l3_feat_score, l3_feat_quad, l3_feat_sector, l3_feat_desc);

            if (feat_valid) begin
                $fdisplay(fd_feat, "%0d %0d %0d %0d %0d %0d %064x",
                          feat_level, feat_x, feat_y, feat_score, feat_quad, feat_sector, feat_desc);
                aidx = arb_cnt[feat_level];
                got_arb_w = {feat_score, feat_y, feat_x, feat_quad, feat_sector, feat_desc};
                if (aidx >= exp_cnt[feat_level]) begin
                    arb_err = arb_err + 1;
                end else begin
                    exp_arb_w = get_exp(feat_level, aidx);
                    if (got_arb_w !== exp_arb_w) begin
                        arb_err = arb_err + 1;
                        if (arb_err <= 5)
                            $display("[ARB L%0d] MISMATCH #%0d", feat_level, aidx);
                    end
                end
                arb_cnt[feat_level] = aidx + 1;
                arb_total = arb_total + 1;
            end
        end
    end

    // ---- Main stimulus & completion check ----------------------------------
    int cyc;
    int total_err;
    int exp_total;
    int seed = 32'hCAFE_BABE;

    initial begin
        $display("tb_system_top: %0dx%0d (%0dx%0d, %0dx%0d, %0dx%0d), thr=%0d",
                 W0, H0, W1, H1, W2, H2, W3, H3, THR);

        // ---- Phase 1: Mid-frame reset test ---------------------------------
        capture_en     <= 1'b0;
        post_reset_chk <= 1'b0;
        rst            <= 1'b1;
        in_valid       <= 1'b0;
        in_pixel       <= 8'd0;
        repeat (5) @(posedge clk);
        rst <= 1'b0;
        repeat (2) @(posedge clk);

        for (int i = 0; i < MID_RST_PIX; i++) begin
            in_valid <= 1'b1;
            in_pixel <= img[i];
            @(posedge clk);
        end
        in_valid <= 1'b0;
        rst      <= 1'b1;
        repeat (5) @(posedge clk);
        rst            <= 1'b0;
        post_reset_chk <= 1'b1;
        repeat (20) @(posedge clk);
        post_reset_chk <= 1'b0;
        if (busy !== 1'b0) begin
            $display("ERROR: busy remained high after mid-frame reset");
            reset_leak_err = reset_leak_err + 1;
        end

        // ---- Phase 2: Full-frame extraction with periodic input gaps -------
        capture_en <= 1'b1;
        @(posedge clk);

        for (int i = 0; i < IMG_W * IMG_H; i++) begin
            while ((($random(seed) & 32'h7fffffff) % 10) == 0) begin
                in_valid <= 1'b0;
                @(posedge clk);
            end
            in_valid <= 1'b1;
            in_pixel <= img[i];
            @(posedge clk);
        end
        in_valid <= 1'b0;

        repeat (5) @(posedge clk);
        cyc = 0;
        while (busy && cyc < TIMEOUT_CYC) begin
            @(posedge clk);
            cyc++;
        end
        repeat (50) @(posedge clk);

        exp_total = exp_cnt[0] + exp_cnt[1] + exp_cnt[2] + exp_cnt[3];
        total_err = arb_err + reset_leak_err;

        $display("---- System Results (%0d drain cycles, busy=%0b, rst_leak=%0d) ----",
                 cyc, busy, reset_leak_err);
        for (int l = 0; l < 4; l++) begin
            $display("  Level %0d : feat %0d/%0d (err %0d) | arb %0d/%0d",
                     l, lvl_cnt[l], exp_cnt[l], lvl_err[l], arb_cnt[l], exp_cnt[l]);
            total_err = total_err + lvl_err[l];
            if (lvl_cnt[l] !== exp_cnt[l]) total_err = total_err + 1;
            if (arb_cnt[l] !== exp_cnt[l]) total_err = total_err + 1;
        end
        if (g_cnt[0] !== GN0) total_err = total_err + 1;
        if (g_cnt[1] !== GN1) total_err = total_err + 1;
        if (g_cnt[2] !== GN2) total_err = total_err + 1;
        if (g_cnt[3] !== GN3) total_err = total_err + 1;
        if (arb_total !== exp_total) total_err = total_err + 1;
        if (busy !== 1'b0) total_err = total_err + 1;

        $display("  Gauss counts: L0=%0d/%0d L1=%0d/%0d L2=%0d/%0d L3=%0d/%0d",
                 g_cnt[0], GN0, g_cnt[1], GN1, g_cnt[2], GN2, g_cnt[3], GN3);
        $display("  Unified stream: %0d/%0d features, arb_err=%0d", arb_total, exp_total, arb_err);

        $fclose(fd_feat);
        $fclose(fd_g0); $fclose(fd_g1); $fclose(fd_g2); $fclose(fd_g3);

        if (total_err == 0) begin
            $display("TB_RESULT: PASS");
        end else begin
            $display("TB_RESULT: FAIL (%0d errors)", total_err);
        end
        $finish;
    end

endmodule
