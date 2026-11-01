`timescale 1ns / 1ps

module hw_fast (
    input  wire        clk,
    input  wire        rst_n,
    
    // Kuantisasi 6-bit
    input  wire [5:0]  center_pixel,
    input  wire [95:0] ring_pixels, // 16 piksel x 6-bit = 96 bit
    input  wire [5:0]  threshold,
    
    output reg         is_corner
);

    // Array internal untuk mempermudah akses 16 piksel cincin
    wire [5:0] ring [0:15];
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : unpack_ring
            assign ring[i] = ring_pixels[(i*6) +: 6];
        end
    endgenerate

    // Vektor kondisi terang (brighter) dan gelap (darker)
    wire [15:0] brighter;
    wire [15:0] darker;
    
    // Komparator Paralel (1 Siklus Clock)
    generate
        for (i = 0; i < 16; i = i + 1) begin : fast_compare
            assign brighter[i] = (ring[i] > (center_pixel + threshold)) ? 1'b1 : 1'b0;
            assign darker[i]   = (ring[i] < (center_pixel - threshold)) ? 1'b1 : 1'b0;
        end
    endgenerate

    // Logika deteksi 9 piksel berurutan (Contiguous)
    // Diperluas menjadi 24-bit untuk menangani *wrap-around* lingkaran
    wire [24:0] brighter_wrap = {brighter[8:0], brighter};
    wire [24:0] darker_wrap   = {darker[8:0], darker};
    
    reg corner_found;
    integer j;
    
    always @(*) begin
        corner_found = 1'b0;
        // Cek apakah ada pola 9-bit '111111111' yang berurutan
        for (j = 0; j < 16; j = j + 1) begin
            if ((brighter_wrap[j +: 9] == 9'h1FF) || (darker_wrap[j +: 9] == 9'h1FF)) begin
                corner_found = 1'b1;
            end
        end
    end

    // Sequential Logic (Register Output)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_corner <= 1'b0;
        end else begin
            is_corner <= corner_found;
        end
    end

endmodule
