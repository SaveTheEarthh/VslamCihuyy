// ============================================================================
// tb_orb_pyramid_top.v - AXI-Stream -> Input FIFO -> pyramid, vs golden model
//
// Run with the simulator's working directory = golden_out (same hex files as
// tb_orb_pyramid.v).  IMG_W*IMG_H must be a multiple of S_DATA_W/8.
//
//   GAPS=1       random TVALID bubbles from the "DMA"
//   FIFO_DEPTH   small by default so the FIFO really fills up (TREADY drops)
//   S_DATA_W     AXI-Stream width: 8, 16, 32, 64
// ============================================================================
`timescale 1ns/1ps

module tb_orb_pyramid_top;

    parameter IMG_W      = 480;
    parameter IMG_H      = 640;
    parameter ROUND      = 1;
    parameter S_DATA_W   = 32;
    parameter FIFO_DEPTH = 16;
    parameter GAPS       = 0;
    parameter NFRAMES    = 2;

    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam BYTES = S_DATA_W / 8;
    localparam W1 = osz(IMG_W), H1 = osz(IMG_H);
    localparam W2 = osz(W1),    H2 = osz(H1);
    localparam W3 = osz(W2),    H3 = osz(H2);
    localparam N0 = IMG_W * IMG_H;
    localparam N1 = W1 * H1;
    localparam N2 = W2 * H2;
    localparam N3 = W3 * H3;
    localparam NBEATS = (NFRAMES * N0) / BYTES;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    reg  [S_DATA_W-1:0] tdata  = {S_DATA_W{1'b0}};
    reg                 tvalid = 1'b0;
    wire                tready;
    wire [15:0]         fifo_level;

    wire        l0_valid, l1_valid, l2_valid, l3_valid;
    wire [7:0]  l0_pixel, l1_pixel, l2_pixel, l3_pixel;
    wire [12:0] l1_acc, l2_acc, l3_acc;
    wire        l1_eol, l1_eof, l2_eol, l2_eof, l3_eol, l3_eof;

    orb_pyramid_top #(.S_DATA_W(S_DATA_W), .FIFO_DEPTH(FIFO_DEPTH),
                      .IMG_W(IMG_W), .IMG_H(IMG_H), .ROUND(ROUND)) dut (
        .clk(clk), .rst(rst),
        .s_axis_tdata(tdata), .s_axis_tvalid(tvalid), .s_axis_tready(tready),
        .fifo_level(fifo_level),
        .l0_valid(l0_valid), .l0_pixel(l0_pixel),
        .l1_valid(l1_valid), .l1_pixel(l1_pixel), .l1_acc(l1_acc),
        .l1_eol(l1_eol), .l1_eof(l1_eof),
        .l2_valid(l2_valid), .l2_pixel(l2_pixel), .l2_acc(l2_acc),
        .l2_eol(l2_eol), .l2_eof(l2_eof),
        .l3_valid(l3_valid), .l3_pixel(l3_pixel), .l3_acc(l3_acc),
        .l3_eol(l3_eol), .l3_eof(l3_eof)
    );

    reg [7:0]  img [0:N0-1];
    reg [7:0]  e1  [0:N1-1];   reg [12:0] a1 [0:N1-1];
    reg [7:0]  e2  [0:N2-1];   reg [12:0] a2 [0:N2-1];
    reg [7:0]  e3  [0:N3-1];   reg [12:0] a3 [0:N3-1];

    integer c0 = 0, c1 = 0, c2 = 0, c3 = 0;
    integer err = 0;
    integer stall_cycles = 0;           // cycles where the FIFO refused data

    always @(posedge clk) if (tvalid && !tready) stall_cycles = stall_cycles + 1;

    // ---------------- checkers ---------------------------------------------
    always @(posedge clk) if (l0_valid) begin
        if (c0 < NFRAMES * N0) begin
            if (l0_pixel !== img[c0 % N0]) begin
                err = err + 1;
                if (err < 20) $display("L0 mismatch #%0d: got %0d exp %0d", c0, l0_pixel, img[c0 % N0]);
            end
        end
        c0 = c0 + 1;
    end

    always @(posedge clk) if (l1_valid) begin
        if (c1 < NFRAMES * N1) begin
            if (l1_pixel !== e1[c1 % N1]) begin
                err = err + 1;
                if (err < 20) $display("L1 pix mismatch #%0d: got %0d exp %0d", c1, l1_pixel, e1[c1 % N1]);
            end
            if (l1_acc !== a1[c1 % N1]) begin
                err = err + 1;
                if (err < 20) $display("L1 acc mismatch #%0d: got %0d exp %0d", c1, l1_acc, a1[c1 % N1]);
            end
            if (l1_eol !== (((c1 % W1) == W1 - 1))) begin err = err + 1; $display("L1 eol wrong #%0d", c1); end
            if (l1_eof !== (((c1 % N1) == N1 - 1))) begin err = err + 1; $display("L1 eof wrong #%0d", c1); end
        end
        c1 = c1 + 1;
    end
    always @(posedge clk) begin
        if (l0_valid && (c0 % 100000 == 0) && (c0 > 0))
            $display("Progress: Processed %0d pixels...", c0);
    end

    always @(posedge clk) if (l2_valid) begin
        if (c2 < NFRAMES * N2) begin
            if (l2_pixel !== e2[c2 % N2]) begin
                err = err + 1;
                if (err < 20) $display("L2 pix mismatch #%0d: got %0d exp %0d", c2, l2_pixel, e2[c2 % N2]);
            end
            if (l2_acc !== a2[c2 % N2]) begin
                err = err + 1;
                if (err < 20) $display("L2 acc mismatch #%0d: got %0d exp %0d", c2, l2_acc, a2[c2 % N2]);
            end
            if (l2_eol !== (((c2 % W2) == W2 - 1))) begin err = err + 1; $display("L2 eol wrong #%0d", c2); end
            if (l2_eof !== (((c2 % N2) == N2 - 1))) begin err = err + 1; $display("L2 eof wrong #%0d", c2); end
        end
        c2 = c2 + 1;
    end

    always @(posedge clk) if (l3_valid) begin
        if (c3 < NFRAMES * N3) begin
            if (l3_pixel !== e3[c3 % N3]) begin
                err = err + 1;
                if (err < 20) $display("L3 pix mismatch #%0d: got %0d exp %0d", c3, l3_pixel, e3[c3 % N3]);
            end
            if (l3_acc !== a3[c3 % N3]) begin
                err = err + 1;
                if (err < 20) $display("L3 acc mismatch #%0d: got %0d exp %0d", c3, l3_acc, a3[c3 % N3]);
            end
            if (l3_eol !== (((c3 % W3) == W3 - 1))) begin err = err + 1; $display("L3 eol wrong #%0d", c3); end
            if (l3_eof !== (((c3 % N3) == N3 - 1))) begin err = err + 1; $display("L3 eof wrong #%0d", c3); end
        end
        c3 = c3 + 1;
    end

    // ---------------- AXI-Stream source -------------------------------------
    integer k, b, idx, seed;
    reg [S_DATA_W-1:0] word;

    initial begin
        seed = 32'h0BAD_CAFE;
        $readmemh("../pyCodes/imageTest/level0_pixels.hex", img);
        $readmemh("../pyCodes/imageTest/level1_pixels.hex", e1);  
        $readmemh("../pyCodes/imageTest/level1_acc.hex", a1);
        $readmemh("../pyCodes/imageTest/level2_pixels.hex", e2);  
        $readmemh("../pyCodes/imageTest/level2_acc.hex", a2);
        $readmemh("../pyCodes/imageTest/level3_pixels.hex", e3);  
        $readmemh("../pyCodes/imageTest/level3_acc.hex", a3);

        repeat (5) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);

        for (k = 0; k < NBEATS; k = k + 1) begin
            for (b = 0; b < BYTES; b = b + 1) begin
                idx = (k * BYTES + b) % N0;           // frames repeat back-to-back
                word[8*b +: 8] = img[idx];            // little-endian pixel order
            end
            if (GAPS) begin
                while ((($random(seed)) & 3) == 0) begin
                    tvalid <= 1'b0;
                    @(posedge clk);
                end
            end
            tvalid <= 1'b1;
            tdata  <= word;
            @(posedge clk);
            while (!tready) @(posedge clk);           // hold until accepted
        end
        tvalid <= 1'b0;
        repeat (60 + 2 * FIFO_DEPTH * BYTES) @(posedge clk);

        if (c0 !== NFRAMES * N0) begin err = err + 1; $display("L0 count: got %0d exp %0d", c0, NFRAMES * N0); end
        if (c1 !== NFRAMES * N1) begin err = err + 1; $display("L1 count: got %0d exp %0d", c1, NFRAMES * N1); end
        if (c2 !== NFRAMES * N2) begin err = err + 1; $display("L2 count: got %0d exp %0d", c2, NFRAMES * N2); end
        if (c3 !== NFRAMES * N3) begin err = err + 1; $display("L3 count: got %0d exp %0d", c3, NFRAMES * N3); end

        if (err == 0)
            $display("PASS: %0dx%0d, S_DATA_W=%0d, FIFO_DEPTH=%0d, %0d frame(s), ROUND=%0d, GAPS=%0d (FIFO back-pressure cycles: %0d)",
                     IMG_W, IMG_H, S_DATA_W, FIFO_DEPTH, NFRAMES, ROUND, GAPS, stall_cycles);
        else
            $display("FAIL: %0d error(s)", err);
        $finish;
    end

    // watchdog
    initial begin
        #(10 * (NFRAMES * N0 * 6 + 5000));
        $display("FAIL: timeout");
        $finish;
    end

endmodule
