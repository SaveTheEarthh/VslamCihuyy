// -----------------------------------------------------------------------------
// tb_orient.sv  --  self-checking SystemVerilog testbench
//   DUT1 orient_core : every window's moments, quadrant and sector vs expected_core.txt
//   DUT2 orient_top  : keypoint-gated results vs expected_top.txt (border keypoints and
//                      duplicates are pushed on purpose and must not produce output)
// Run twice:  vvp sim            -> back-to-back pixels
//             vvp sim +gap=40    -> 40 % random bubbles (pix_valid gaps)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_orient;
`include "sim_params.svh"      // IMG_W IMG_H N_SECT N_WIN N_KP N_TOP

    localparam int PIX_W = 8;
    localparam int CW    = $clog2(IMG_W);
    localparam int RW    = $clog2(IMG_H);
    localparam int SEC_W = $clog2(N_SECT / 4);
    localparam int M0W   = PIX_W + 11;
    localparam int MW    = PIX_W + 14;
    localparam int NPIX  = IMG_W * IMG_H;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    // ---- stimulus data -----------------------------------------------------------------
    logic [PIX_W-1:0] img [0:NPIX-1];
    int kx [0:N_KP-1];
    int ky [0:N_KP-1];
    int ex_x [0:N_WIN-1], ex_y [0:N_WIN-1], ex_m00 [0:N_WIN-1], ex_m10 [0:N_WIN-1];
    int ex_m01 [0:N_WIN-1], ex_q [0:N_WIN-1], ex_s [0:N_WIN-1];
    int tx [0:N_TOP-1], ty [0:N_TOP-1], tq [0:N_TOP-1], ts [0:N_TOP-1];
    int gap, seed, fd, rc;

    initial begin
        if (!$value$plusargs("gap=%d", gap)) gap = 0;
        seed = 12345;
        $readmemh("img.hex", img);
        fd = $fopen("kps.txt", "r");
        for (int i = 0; i < N_KP; i++) rc = $fscanf(fd, "%d %d\n", kx[i], ky[i]);
        $fclose(fd);
        fd = $fopen("expected_core.txt", "r");
        for (int i = 0; i < N_WIN; i++)
            rc = $fscanf(fd, "%d %d %d %d %d %d %d\n",
                         ex_x[i], ex_y[i], ex_m00[i], ex_m10[i], ex_m01[i], ex_q[i], ex_s[i]);
        $fclose(fd);
        fd = $fopen("expected_top.txt", "r");
        for (int i = 0; i < N_TOP; i++) rc = $fscanf(fd, "%d %d %d %d\n", tx[i], ty[i], tq[i], ts[i]);
        $fclose(fd);
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_orient.vcd");
            $dumpvars(0, tb_orient);
        end
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
    end

    // ---- pixel driver ------------------------------------------------------------------
    logic             pix_valid = 1'b0, pix_sof = 1'b0;
    logic [PIX_W-1:0] pix_data  = '0;
    int               idx = 0;
    always_ff @(posedge clk) begin
        if (!rst_n || idx >= NPIX) begin
            pix_valid <= 1'b0;
            pix_sof   <= 1'b0;
        end else if ((($random(seed) & 32'h7fffffff) % 100) >= gap) begin
            pix_valid <= 1'b1;
            pix_data  <= img[idx];
            pix_sof   <= (idx == 0);
            idx       <= idx + 1;
        end else begin
            pix_valid <= 1'b0;
            pix_sof   <= 1'b0;
        end
    end

    // ---- keypoint driver (valid/ready) -----------------------------------------------------
    logic         kp_valid = 1'b0;
    logic [CW-1:0] kp_x = '0;
    logic [RW-1:0] kp_y = '0;
    logic         kp_ready;
    int           kidx = 0;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            kp_valid <= 1'b0;
            kidx     <= 0;
        end else if (!kp_valid) begin
            if (kidx < N_KP) begin
                kp_valid <= 1'b1;
                kp_x     <= CW'(kx[kidx]);
                kp_y     <= RW'(ky[kidx]);
            end
        end else if (kp_ready) begin               // current keypoint accepted on this edge
            kidx <= kidx + 1;
            if (kidx + 1 < N_KP) begin
                kp_x <= CW'(kx[kidx+1]);
                kp_y <= RW'(ky[kidx+1]);
            end else begin
                kp_valid <= 1'b0;
            end
        end
    end

    // ---- DUT 1: all windows ----------------------------------------------------------------
    logic                c_valid;
    logic [CW-1:0]       c_x;
    logic [RW-1:0]       c_y;
    logic [1:0]          c_quad;
    logic [SEC_W-1:0]    c_sec;
    logic [M0W-1:0]      c_m00;
    logic signed [MW-1:0] c_m10, c_m01;
    orient_core #(.PIX_W(PIX_W), .IMG_W(IMG_W), .IMG_H(IMG_H), .N_SECT(N_SECT)) dut_core (
        .clk(clk), .rst_n(rst_n), .pix_valid(pix_valid), .pix_sof(pix_sof), .pix_data(pix_data),
        .res_valid(c_valid), .res_x(c_x), .res_y(c_y), .res_quad(c_quad), .res_sector(c_sec),
        .res_m00(c_m00), .res_m10(c_m10), .res_m01(c_m01));

    // ---- DUT 2: keypoint-gated ----------------------------------------------------------------
    logic             t_valid;
    logic [CW-1:0]    t_x;
    logic [RW-1:0]    t_y;
    logic [1:0]       t_quad;
    logic [SEC_W-1:0] t_sec;
    orient_top #(.PIX_W(PIX_W), .IMG_W(IMG_W), .IMG_H(IMG_H), .N_SECT(N_SECT)) dut_top (
        .clk(clk), .rst_n(rst_n), .pix_valid(pix_valid), .pix_sof(pix_sof), .pix_data(pix_data),
        .kp_in_valid(kp_valid), .kp_in_x(kp_x), .kp_in_y(kp_y), .kp_in_ready(kp_ready),
        .kp_flush(1'b0),
        .out_valid(t_valid), .out_x(t_x), .out_y(t_y), .out_quad(t_quad), .out_sector(t_sec));

    // ---- checkers ----------------------------------------------------------------------------
    int n_core = 0, err_core = 0, n_top = 0, err_top = 0;
    always_ff @(posedge clk) if (c_valid) begin
        if (n_core >= N_WIN) err_core <= err_core + 1;
        else if (c_x !== ex_x[n_core] || c_y !== ex_y[n_core] || c_m00 !== ex_m00[n_core] ||
                 c_m10 !== ex_m10[n_core] || c_m01 !== ex_m01[n_core] ||
                 c_quad !== ex_q[n_core] || c_sec !== ex_s[n_core]) begin
            err_core <= err_core + 1;
            if (err_core < 5)
                $display("CORE MISMATCH #%0d (%0d,%0d): got m00=%0d m10=%0d m01=%0d q=%0d s=%0d | exp %0d,%0d,%0d,%0d,%0d,%0d,%0d",
                         n_core, c_x, c_y, c_m00, c_m10, c_m01, c_quad, c_sec,
                         ex_x[n_core], ex_y[n_core], ex_m00[n_core], ex_m10[n_core],
                         ex_m01[n_core], ex_q[n_core], ex_s[n_core]);
        end
        n_core <= n_core + 1;
    end

    always_ff @(posedge clk) if (t_valid) begin
        if (n_top >= N_TOP) err_top <= err_top + 1;
        else if (t_x !== tx[n_top] || t_y !== ty[n_top] || t_quad !== tq[n_top] || t_sec !== ts[n_top]) begin
            err_top <= err_top + 1;
            if (err_top < 5)
                $display("TOP MISMATCH #%0d: got (%0d,%0d) q=%0d s=%0d | exp (%0d,%0d) q=%0d s=%0d",
                         n_top, t_x, t_y, t_quad, t_sec, tx[n_top], ty[n_top], tq[n_top], ts[n_top]);
        end
        n_top <= n_top + 1;
    end

    // ---- end of test -----------------------------------------------------------------------------
    initial begin
        wait (rst_n);
        wait (idx >= NPIX);
        repeat (200) @(posedge clk);
        $display("----------------------------------------------------------------");
        $display("image %0dx%0d, N_SECT=%0d, bubbles=%0d%%", IMG_W, IMG_H, N_SECT, gap);
        $display("core: %0d/%0d windows checked, %0d errors", n_core, N_WIN, err_core);
        $display("top : %0d/%0d keypoints out,   %0d errors", n_top, N_TOP, err_top);
        if (n_core == N_WIN && n_top == N_TOP && err_core == 0 && err_top == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL");
        $finish;
    end

    initial begin                                   // watchdog
        #(10 * (NPIX * 4 + 5000));
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
