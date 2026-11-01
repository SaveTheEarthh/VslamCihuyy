// ============================================================================
// TBFrontend.v (Updated with File Export)
// Self-checking testbench yang juga MENGEKSPOR hasil hardware ke file .hex
// ============================================================================
`timescale 1ns/1ps

// ----------------------------------------------------------------------------
// Generic in-order stream checker
// ----------------------------------------------------------------------------
module orb_stream_chk #(
    parameter W     = 8,
    parameter DEPTH = 16,
    parameter FILE  = "none.hex",
    parameter TAG   = "stream"
)(
    input  wire         clk,
    input  wire         valid,
    input  wire [W-1:0] data,
    output reg  [31:0]  cnt,
    output reg  [31:0]  err
);
    reg [W-1:0] mem [0:DEPTH-1];

    initial begin
        cnt = 0;
        err = 0;
        $readmemh(FILE, mem);
    end

    always @(posedge clk) begin
        if (valid) begin
            if (cnt < DEPTH) begin
                if (data !== mem[cnt]) begin
                    if (err < 10)
                        $display("[%0t] MISMATCH %0s #%0d : got %h expected %h",
                                 $time, TAG, cnt, data, mem[cnt]);
                    err <= err + 1;
                end
            end else begin
                if (err < 10)
                    $display("[%0t] EXTRA beat %0s #%0d : %h (expected file has %0d entries)",
                             $time, TAG, cnt, data, DEPTH);
                err <= err + 1;
            end
            cnt <= cnt + 1;
        end
    end
endmodule


// ----------------------------------------------------------------------------
module TB_Frontend;

    parameter IMG_W       = 480;        
    parameter IMG_H       = 640;
    parameter THR         = 20;         
    parameter KP_DEPTH    = 65536;      
    parameter TIMEOUT_CYC = 100000;     

    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam W0 = IMG_W;      localparam H0 = IMG_H;
    localparam W1 = osz(W0);    localparam H1 = osz(H0);
    localparam W2 = osz(W1);    localparam H2 = osz(H1);
    localparam W3 = osz(W2);    localparam H3 = osz(H2);

    localparam N0 = (W0 - 6) * (H0 - 6);
    localparam N1 = (W1 - 6) * (H1 - 6);
    localparam N2 = (W2 - 6) * (H2 - 6);
    localparam N3 = (W3 - 6) * (H3 - 6);

    // ---- clock / reset / stimulus ------------------------------------------
    reg         clk = 1'b0;
    reg         rst = 1'b1;
    reg         in_valid = 1'b0;
    reg  [7:0]  in_pixel = 8'd0;
    always #5 clk = ~clk;

    reg [7:0] img [0:IMG_W*IMG_H-1];

    // ---- DUT ------------------------------------------------------------------
    wire        l0_g_valid,  l1_g_valid,  l2_g_valid,  l3_g_valid;
    wire [7:0]  l0_g_pixel,  l1_g_pixel,  l2_g_pixel,  l3_g_pixel;
    wire        l0_fs_valid, l1_fs_valid, l2_fs_valid, l3_fs_valid;
    wire [11:0] l0_fs_score, l1_fs_score, l2_fs_score, l3_fs_score;
    wire        l0_kp_valid, l1_kp_valid, l2_kp_valid, l3_kp_valid;
    wire [35:0] l0_kp_data,  l1_kp_data,  l2_kp_data,  l3_kp_data;

    orb_frontend_top #(.IMG_W(IMG_W), .IMG_H(IMG_H), .ROUND(1)) dut (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_pixel(in_pixel), .thr(THR[7:0]),
        .l0_g_valid(l0_g_valid),   .l0_g_pixel(l0_g_pixel),
        .l0_fs_valid(l0_fs_valid), .l0_fs_score(l0_fs_score),
        .l0_kp_valid(l0_kp_valid), .l0_kp_data(l0_kp_data),
        .l1_g_valid(l1_g_valid),   .l1_g_pixel(l1_g_pixel),
        .l1_fs_valid(l1_fs_valid), .l1_fs_score(l1_fs_score),
        .l1_kp_valid(l1_kp_valid), .l1_kp_data(l1_kp_data),
        .l2_g_valid(l2_g_valid),   .l2_g_pixel(l2_g_pixel),
        .l2_fs_valid(l2_fs_valid), .l2_fs_score(l2_fs_score),
        .l2_kp_valid(l2_kp_valid), .l2_kp_data(l2_kp_data),
        .l3_g_valid(l3_g_valid),   .l3_g_pixel(l3_g_pixel),
        .l3_fs_valid(l3_fs_valid), .l3_fs_score(l3_fs_score),
        .l3_kp_valid(l3_kp_valid), .l3_kp_data(l3_kp_data)
    );

    // ========================================================================
    // FITUR BARU: Menulis Output ke File Hex
    // ========================================================================
    integer f_g0, f_g1, f_g2, f_g3;
    integer f_f0, f_f1, f_f2, f_f3;
    integer f_k0, f_k1, f_k2, f_k3;

    initial begin
        f_g0 = $fopen("hw_l0_gauss.hex", "w"); f_g1 = $fopen("hw_l1_gauss.hex", "w");
        f_g2 = $fopen("hw_l2_gauss.hex", "w"); f_g3 = $fopen("hw_l3_gauss.hex", "w");
        
        f_f0 = $fopen("hw_l0_fastscore.hex", "w"); f_f1 = $fopen("hw_l1_fastscore.hex", "w");
        f_f2 = $fopen("hw_l2_fastscore.hex", "w"); f_f3 = $fopen("hw_l3_fastscore.hex", "w");

        f_k0 = $fopen("hw_l0_kp.hex", "w"); f_k1 = $fopen("hw_l1_kp.hex", "w");
        f_k2 = $fopen("hw_l2_kp.hex", "w"); f_k3 = $fopen("hw_l3_kp.hex", "w");
    end

    always @(posedge clk) begin
        // Tulis Gaussian (8-bit)
        if (l0_g_valid) $fdisplay(f_g0, "%02x", l0_g_pixel);
        if (l1_g_valid) $fdisplay(f_g1, "%02x", l1_g_pixel);
        if (l2_g_valid) $fdisplay(f_g2, "%02x", l2_g_pixel);
        if (l3_g_valid) $fdisplay(f_g3, "%02x", l3_g_pixel);
        
        // Tulis FAST Score (12-bit)
        if (l0_fs_valid) $fdisplay(f_f0, "%03x", l0_fs_score);
        if (l1_fs_valid) $fdisplay(f_f1, "%03x", l1_fs_score);
        if (l2_fs_valid) $fdisplay(f_f2, "%03x", l2_fs_score);
        if (l3_fs_valid) $fdisplay(f_f3, "%03x", l3_fs_score);

        // Tulis Keypoint (36-bit)
        if (l0_kp_valid) $fdisplay(f_k0, "%09x", l0_kp_data);
        if (l1_kp_valid) $fdisplay(f_k1, "%09x", l1_kp_data);
        if (l2_kp_valid) $fdisplay(f_k2, "%09x", l2_kp_data);
        if (l3_kp_valid) $fdisplay(f_k3, "%09x", l3_kp_data);
    end
    // ========================================================================

    // ---- expected keypoint counts ---------------------------------------------
    reg [31:0] kc0 [0:0];
    reg [31:0] kc1 [0:0];
    reg [31:0] kc2 [0:0];
    reg [31:0] kc3 [0:0];

    // ---- checkers (12 streams) ------------------------------------------------
    wire [31:0] g0_c, g1_c, g2_c, g3_c,  g0_e, g1_e, g2_e, g3_e;
    wire [31:0] f0_c, f1_c, f2_c, f3_c,  f0_e, f1_e, f2_e, f3_e;
    wire [31:0] k0_c, k1_c, k2_c, k3_c,  k0_e, k1_e, k2_e, k3_e;

    orb_stream_chk #(.W(8),  .DEPTH(N0), .FILE("../pyCodes/imageTest/level0_gauss.hex"),     .TAG("L0 gauss"))
        c_g0 (.clk(clk), .valid(l0_g_valid),  .data(l0_g_pixel),  .cnt(g0_c), .err(g0_e));
    orb_stream_chk #(.W(8),  .DEPTH(N1), .FILE("../pyCodes/imageTest/level1_gauss.hex"),     .TAG("L1 gauss"))
        c_g1 (.clk(clk), .valid(l1_g_valid),  .data(l1_g_pixel),  .cnt(g1_c), .err(g1_e));
    orb_stream_chk #(.W(8),  .DEPTH(N2), .FILE("../pyCodes/imageTest/level2_gauss.hex"),     .TAG("L2 gauss"))
        c_g2 (.clk(clk), .valid(l2_g_valid),  .data(l2_g_pixel),  .cnt(g2_c), .err(g2_e));
    orb_stream_chk #(.W(8),  .DEPTH(N3), .FILE("../pyCodes/imageTest/level3_gauss.hex"),     .TAG("L3 gauss"))
        c_g3 (.clk(clk), .valid(l3_g_valid),  .data(l3_g_pixel),  .cnt(g3_c), .err(g3_e));

    orb_stream_chk #(.W(12), .DEPTH(N0), .FILE("../pyCodes/imageTest/level0_fastscore.hex"), .TAG("L0 fast"))
        c_f0 (.clk(clk), .valid(l0_fs_valid), .data(l0_fs_score), .cnt(f0_c), .err(f0_e));
    orb_stream_chk #(.W(12), .DEPTH(N1), .FILE("../pyCodes/imageTest/level1_fastscore.hex"), .TAG("L1 fast"))
        c_f1 (.clk(clk), .valid(l1_fs_valid), .data(l1_fs_score), .cnt(f1_c), .err(f1_e));
    orb_stream_chk #(.W(12), .DEPTH(N2), .FILE("../pyCodes/imageTest/level2_fastscore.hex"), .TAG("L2 fast"))
        c_f2 (.clk(clk), .valid(l2_fs_valid), .data(l2_fs_score), .cnt(f2_c), .err(f2_e));
    orb_stream_chk #(.W(12), .DEPTH(N3), .FILE("../pyCodes/imageTest/level3_fastscore.hex"), .TAG("L3 fast"))
        c_f3 (.clk(clk), .valid(l3_fs_valid), .data(l3_fs_score), .cnt(f3_c), .err(f3_e));

    orb_stream_chk #(.W(36), .DEPTH(KP_DEPTH), .FILE("../pyCodes/imageTest/level0_kp.hex"), .TAG("L0 kp"))
        c_k0 (.clk(clk), .valid(l0_kp_valid), .data(l0_kp_data), .cnt(k0_c), .err(k0_e));
    orb_stream_chk #(.W(36), .DEPTH(KP_DEPTH), .FILE("../pyCodes/imageTest/level1_kp.hex"), .TAG("L1 kp"))
        c_k1 (.clk(clk), .valid(l1_kp_valid), .data(l1_kp_data), .cnt(k1_c), .err(k1_e));
    orb_stream_chk #(.W(36), .DEPTH(KP_DEPTH), .FILE("../pyCodes/imageTest/level2_kp.hex"), .TAG("L2 kp"))
        c_k2 (.clk(clk), .valid(l2_kp_valid), .data(l2_kp_data), .cnt(k2_c), .err(k2_e));
    orb_stream_chk #(.W(36), .DEPTH(KP_DEPTH), .FILE("../pyCodes/imageTest/level3_kp.hex"), .TAG("L3 kp"))
        c_k3 (.clk(clk), .valid(l3_kp_valid), .data(l3_kp_data), .cnt(k3_c), .err(k3_e));

    wire all_done =
        (g0_c === N0) && (g1_c === N1) && (g2_c === N2) && (g3_c === N3) &&
        (f0_c === N0) && (f1_c === N1) && (f2_c === N2) && (f3_c === N3) &&
        (k0_c === kc0[0]) && (k1_c === kc1[0]) && (k2_c === kc2[0]) && (k3_c === kc3[0]);

    // ---- main sequence ------------------------------------------------------
    integer i;
    integer cyc;
    integer total_err;

    task report_level;
        input integer lvl;
        input integer n_exp;
        input [31:0] gc, ge, fc, fe, kc, ke, kexp;
        begin
            $display("  level %0d : gauss %0d/%0d (err %0d) | fast %0d/%0d (err %0d) | kp %0d/%0d (err %0d)",
                     lvl, gc, n_exp, ge, fc, n_exp, fe, kc, kexp, ke);
            total_err = total_err + ge + fe + ke;
            if (gc !== n_exp) total_err = total_err + 1;
            if (fc !== n_exp) total_err = total_err + 1;
            if (kc !== kexp)  total_err = total_err + 1;
            if (kexp > KP_DEPTH) begin
                $display("  level %0d : %0d keypoints exceed KP_DEPTH=%0d, increase the parameter",
                         lvl, kexp, KP_DEPTH);
                total_err = total_err + 1;
            end
        end
    endtask

    initial begin
        $readmemh("../pyCodes/imageTest/level0_pixels.hex",   img);
        $readmemh("../pyCodes/imageTest/level0_kp_count.hex", kc0);
        $readmemh("../pyCodes/imageTest/level1_kp_count.hex", kc1);
        $readmemh("../pyCodes/imageTest/level2_kp_count.hex", kc2);
        $readmemh("../pyCodes/imageTest/level3_kp_count.hex", kc3);

        $display("TB_Frontend: level sizes %0dx%0d  %0dx%0d  %0dx%0d  %0dx%0d, thr=%0d",
                 W0, H0, W1, H1, W2, H2, W3, H3, THR);

        rst = 1'b1;
        repeat (5) @(posedge clk);
        rst <= 1'b0;
        repeat (3) @(posedge clk);

        for (i = 0; i < IMG_W*IMG_H; i = i + 1) begin
            in_valid <= 1'b1;
            in_pixel <= img[i];
            @(posedge clk);
        end
        in_valid <= 1'b0;

        cyc = 0;
        while (!all_done && cyc < TIMEOUT_CYC) begin
            @(posedge clk);
            cyc = cyc + 1;
        end
        repeat (200) @(posedge clk);          

        total_err = 0;
        $display("---- results (%0d cycles after last input) ----", cyc);
        report_level(0, N0, g0_c, g0_e, f0_c, f0_e, k0_c, k0_e, kc0[0]);
        report_level(1, N1, g1_c, g1_e, f1_c, f1_e, k1_c, k1_e, kc1[0]);
        report_level(2, N2, g2_c, g2_e, f2_c, f2_e, k2_c, k2_e, kc2[0]);
        report_level(3, N3, g3_c, g3_e, f3_c, f3_e, k3_c, k3_e, kc3[0]);

        if (total_err == 0) $display("*** PASS: all levels match the golden model ***");
        else                $display("*** FAIL: %0d error(s) ***", total_err);
        
        // Menutup semua file setelah simulasi selesai
        $fclose(f_g0); $fclose(f_g1); $fclose(f_g2); $fclose(f_g3);
        $fclose(f_f0); $fclose(f_f1); $fclose(f_f2); $fclose(f_f3);
        $fclose(f_k0); $fclose(f_k1); $fclose(f_k2); $fclose(f_k3);

        $finish;
    end

endmodule