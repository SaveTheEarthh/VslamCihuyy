// ============================================================================
// TBImagePyramid.v  -  self-checking testbench against the Python golden model
// UPDATED: Now writes hardware-generated scaled hex pixels to output files!
// ============================================================================
`timescale 1ns/1ps

module TBImagePyramid;

    parameter IMG_W   = 640;   // Make sure this matches your test image width
    parameter IMG_H   = 480;   // Make sure this matches your test image height
    parameter ROUND   = 1;     // 1 = golden --rounding round, 0 = floor
    parameter GAPS    = 0;
    parameter NFRAMES = 1;     // Testing 1 frame is usually enough for a full image

    function integer osz;
        input integer n;
        integer m;
        begin
            m   = n - 1;
            osz = m - (m / 6);
        end
    endfunction

    localparam W1 = osz(IMG_W), H1 = osz(IMG_H);
    localparam W2 = osz(W1),    H2 = osz(H1);
    localparam W3 = osz(W2),    H3 = osz(H2);
    localparam N0 = IMG_W * IMG_H;
    localparam N1 = W1 * H1;
    localparam N2 = W2 * H2;
    localparam N3 = W3 * H3;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    reg        in_valid = 1'b0;
    reg  [7:0] in_pixel = 8'd0;

    wire        l1_valid, l2_valid, l3_valid;
    wire [7:0]  l1_pixel, l2_pixel, l3_pixel;
    wire [12:0] l1_acc,   l2_acc,   l3_acc;
    wire        l1_eol, l1_eof, l2_eol, l2_eof, l3_eol, l3_eof;

    // File handles for hex output
    integer file_l1, file_l2, file_l3;

    orb_image_pyramid #(.IMG_W(IMG_W), .IMG_H(IMG_H), .ROUND(ROUND)) dut (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_pixel(in_pixel),
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

    integer c1 = 0, c2 = 0, c3 = 0;
    integer err = 0;

    // ---------------- checkers (sample DUT outputs at each clock edge) ------
    always @(posedge clk) if (l1_valid) begin
        // Write the generated pixel to the hex file
        $fdisplay(file_l1, "%02x", l1_pixel);
        
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

    always @(posedge clk) if (l2_valid) begin
        // Write the generated pixel to the hex file
        $fdisplay(file_l2, "%02x", l2_pixel);

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
        // Write the generated pixel to the hex file
        $fdisplay(file_l3, "%02x", l3_pixel);

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

    // ---------------- stimulus ----------------------------------------------
    integer f, i, seed;
    initial begin
        // Open files to write the Verilog outputs
        file_l1 = $fopen("hardware_out_level1.hex", "w");
        file_l2 = $fopen("hardware_out_level2.hex", "w");
        file_l3 = $fopen("hardware_out_level3.hex", "w");

        seed = 32'h1234_5678;
        // Make sure these match the files generated by your Python script!
        $readmemh("level0_pixels.hex", img);
        $readmemh("level1_pixels.hex", e1);  $readmemh("level1_acc.hex", a1);
        $readmemh("level2_pixels.hex", e2);  $readmemh("level2_acc.hex", a2);
        $readmemh("level3_pixels.hex", e3);  $readmemh("level3_acc.hex", a3);

        repeat (5) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);

        for (f = 0; f < NFRAMES; f = f + 1) begin
            for (i = 0; i < N0; i = i + 1) begin
                if (GAPS) begin
                    while ((($random(seed)) & 3) == 0) begin
                        in_valid <= 1'b0;
                        @(posedge clk);
                    end
                end
                in_valid <= 1'b1;
                in_pixel <= img[i];
                @(posedge clk);
            end
        end
        in_valid <= 1'b0;
        repeat (40) @(posedge clk);

        if (c1 !== NFRAMES * N1) begin err = err + 1; $display("L1 count: got %0d exp %0d", c1, NFRAMES * N1); end
        if (c2 !== NFRAMES * N2) begin err = err + 1; $display("L2 count: got %0d exp %0d", c2, NFRAMES * N2); end
        if (c3 !== NFRAMES * N3) begin err = err + 1; $display("L3 count: got %0d exp %0d", c3, NFRAMES * N3); end

        if (err == 0)
            $display("PASS: %0dx%0d, %0d frame(s), ROUND=%0d, GAPS=%0d  (L1=%0d L2=%0d L3=%0d px/frame)",
                     IMG_W, IMG_H, NFRAMES, ROUND, GAPS, N1, N2, N3);
        else
            $display("FAIL: %0d error(s)", err);
            
        // Close the files before finishing
        $fclose(file_l1);
        $fclose(file_l2);
        $fclose(file_l3);
        $finish;
    end

    // watchdog
    initial begin
        #(10 * (NFRAMES * N0 * 4 + 2000));
        $display("FAIL: timeout");
        // Close files here as well just in case of a timeout
        $fclose(file_l1);
        $fclose(file_l2);
        $fclose(file_l3);
        $finish;
    end

endmodule