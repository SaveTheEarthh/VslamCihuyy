`timescale 1ns / 1ps

module hw_gaussian (
    input  wire        clk,
    input  wire        rst_n,
    
    // 49 piksel x 6-bit = 294 bit (Flattened 7x7 Patch)
    input  wire [293:0] patch_7x7, 
    
    output reg  [5:0]  smoothed_center
);

    // Unpack 294-bit ke array 2D internal [Y][X]
    wire [5:0] patch [0:6][0:6];
    genvar y, x;
    generate
        for (y = 0; y < 7; y = y + 1) begin : row
            for (x = 0; x < 7; x = x + 1) begin : col
                assign patch[y][x] = patch_7x7[((y*7 + x)*6) +: 6];
            end
        end
    endgenerate

    // 1. GAUSSIAN FILTER 7x7 (DSP-FREE)
    // Catatan: Untuk sintesis sesungguhnya, 49 perkalian konstan ini
    // akan diterjemahkan menjadi jaringan adder-tree (shift-and-add)
    reg [19:0] gauss_sum;
    
    always @(*) begin
        gauss_sum = 20'd0;
        // Simulasi Center Pixel untuk penyederhanaan contoh kode
        // Di hardware asli, ini dijumlahkan dari seluruh 49 piksel
        gauss_sum = patch[3][3] * 8'd200 + patch[3][2] * 8'd120 + patch[3][4] * 8'd120; 
    end

    // 2. REGISTER OUTPUT
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            smoothed_center <= 6'd0;
        end else begin
            // Shift kanan 12 bit (membagi 4096) untuk DSP-Free division
            smoothed_center <= gauss_sum[17:12]; 
        end
    end

endmodule
