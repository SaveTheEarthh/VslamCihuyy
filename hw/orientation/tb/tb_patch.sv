// -----------------------------------------------------------------------------
// tb_patch.sv  --  streams N_PATCH separate 37x37 images (patches.hex) through orient_core.
// Each 37x37 image contains exactly one complete window (centre 18,18), so every
// patch gives one result.  Results are written to rtl_results.txt:
//     x y m00 m10 m01 quadrant sector
// Optional plusarg  +gap=NN  inserts NN % random bubbles on pix_valid.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_patch;
`include "patch_params.svh"          // N_PATCH, N_SECT

    localparam int PIX_W = 8;
    localparam int IMG_W = 37;
    localparam int IMG_H = 37;
    localparam int NFRM  = IMG_W * IMG_H;
    localparam int NPIX  = NFRM * N_PATCH;
    localparam int CW    = $clog2(IMG_W);
    localparam int RW    = $clog2(IMG_H);
    localparam int SEC_W = $clog2(N_SECT / 4);
    localparam int M0W   = PIX_W + 11;
    localparam int MW    = PIX_W + 14;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    logic [PIX_W-1:0] pix [0:NPIX-1];
    int gap, seed, fd, n_res;
    int idx = 0;

    initial begin
        if (!$value$plusargs("gap=%d", gap)) gap = 0;
        seed  = 777;
        n_res = 0;
        $readmemh("patches.hex", pix);
        fd = $fopen("rtl_results.txt", "w");
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
    end

    logic             pix_valid = 1'b0, pix_sof = 1'b0;
    logic [PIX_W-1:0] pix_data  = '0;
    always @(posedge clk) begin
        if (!rst_n || idx >= NPIX) begin
            pix_valid <= 1'b0;
            pix_sof   <= 1'b0;
        end else if ((($random(seed) & 32'h7fffffff) % 100) >= gap) begin
            pix_valid <= 1'b1;
            pix_data  <= pix[idx];
            pix_sof   <= ((idx % NFRM) == 0);              // first pixel of every patch
            idx       <= idx + 1;
        end else begin
            pix_valid <= 1'b0;
            pix_sof   <= 1'b0;
        end
    end

    logic                 res_valid;
    logic [CW-1:0]        res_x;
    logic [RW-1:0]        res_y;
    logic [1:0]           res_quad;
    logic [SEC_W-1:0]     res_sector;
    logic [M0W-1:0]       res_m00;
    logic signed [MW-1:0] res_m10, res_m01;
    orient_core #(.PIX_W(PIX_W), .IMG_W(IMG_W), .IMG_H(IMG_H), .N_SECT(N_SECT)) dut (
        .clk(clk), .rst_n(rst_n), .pix_valid(pix_valid), .pix_sof(pix_sof), .pix_data(pix_data),
        .res_valid(res_valid), .res_x(res_x), .res_y(res_y), .res_quad(res_quad),
        .res_sector(res_sector), .res_m00(res_m00), .res_m10(res_m10), .res_m01(res_m01));

    always @(posedge clk) if (res_valid) begin
        $fdisplay(fd, "%0d %0d %0d %0d %0d %0d %0d",
                  res_x, res_y, res_m00, res_m10, res_m01, res_quad, res_sector);
        n_res <= n_res + 1;
    end

    initial begin
        wait (rst_n);
        wait (idx >= NPIX);
        repeat (200) @(posedge clk);
        $fclose(fd);
        $display("tb_patch: %0d results written to rtl_results.txt (expected %0d)", n_res, N_PATCH);
        $finish;
    end
endmodule
