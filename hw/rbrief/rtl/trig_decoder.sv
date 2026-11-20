// =============================================================================
// trig_decoder.sv : penerjemah (kuadran, sektor) -> (cos_val, sin_val)
//
// Tugas modul:
//   Modul orientasi mengirim arah keypoint sebagai kuadran dan sektor. Modul
//   rbrief butuh arah itu sebagai dua bilangan cos_val dan sin_val. Modul ini
//   menerjemahkannya dengan tabel kecil. Murni rangkaian kombinasional (tanpa clock).
//
// Port:
//   quadrant : 0..3, ditentukan dari tanda m10 dan m01 milik modul orientasi
//   sector   : 0..15, sektor di dalam kuadran (tiap sektor 5,625 derajat)
//   cos_val  : cos sudut, signed 9 bit, dikali 128 (-128..+128)
//   sin_val  : sin sudut, signed 9 bit, dikali 128 (-128..+128)
//
// Arti kuadran dan sektor (dari patch_visual.html milik modul orientasi):
//   Sudut dihitung pada bingkai gambar: 0 derajat = kanan, 90 derajat = bawah.
//   Sektor k adalah sudut referensi a = (k + 0,5) * 5,625 derajat yang diukur dari
//   SUMBU HORIZONTAL, jadi bukan berlanjut terus berlawanan arah jarum jam.
//   Kuadran menentukan di mana sudut sebenarnya berada:
//
//     kuadran | tanda m10, m01  | sudut sebenarnya | cos | sin
//     --------+-----------------+------------------+-----+-----
//        0    | m10 >= 0, m01 >= 0 | 180 - a  (90..180)  | -  | +
//        1    | m10 <  0, m01 >= 0 |       a  ( 0.. 90)  | +  | +
//        2    | m10 <  0, m01 <  0 | 360 - a  (270..360) | +  | -
//        3    | m10 >= 0, m01 <  0 | 180 + a  (180..270) | -  | -
//
//   Catatan: sumbu x modul orientasi terbalik terhadap gambar (blob di kanan
//   menghasilkan m10 negatif). Tabel di atas sudah memperhitungkannya.
// =============================================================================
module trig_decoder (
    input  logic [1:0]        quadrant,
    input  logic [3:0]        sector,
    output logic signed [8:0] cos_val,
    output logic signed [8:0] sin_val
);

    // -------------------------------------------------------------------------
    // Tabel cos untuk 16 sektor di kuadran pertama, dikali 128.
    // Nilainya sama dengan LUT yang dicetak rbrief_only.py.
    // -------------------------------------------------------------------------
    function automatic logic [7:0] cos_lut(input logic [3:0] k);
        case (k)
            4'd0:  cos_lut = 8'd128;
            4'd1:  cos_lut = 8'd127;
            4'd2:  cos_lut = 8'd124;
            4'd3:  cos_lut = 8'd121;
            4'd4:  cos_lut = 8'd116;
            4'd5:  cos_lut = 8'd110;
            4'd6:  cos_lut = 8'd103;
            4'd7:  cos_lut = 8'd95;
            4'd8:  cos_lut = 8'd86;
            4'd9:  cos_lut = 8'd76;
            4'd10: cos_lut = 8'd66;
            4'd11: cos_lut = 8'd55;
            4'd12: cos_lut = 8'd43;
            4'd13: cos_lut = 8'd31;
            4'd14: cos_lut = 8'd19;
            default: cos_lut = 8'd6;
        endcase
    endfunction

    // -------------------------------------------------------------------------
    // Nilai positif cos(a) dan sin(a). sin(a) = cos(90 - a), yaitu sektor
    // ke-(15 - k). Untuk 4 bit, 15 - k sama dengan ~k, jadi satu tabel cukup.
    // -------------------------------------------------------------------------
    logic signed [8:0] c_pos, s_pos;
    assign c_pos = {1'b0, cos_lut(sector)};
    assign s_pos = {1'b0, cos_lut(~sector)};

    // -------------------------------------------------------------------------
    // Pasang tanda sesuai kuadran (lihat tabel di atas). Tidak ada pertukaran
    // cos dan sin, karena sektor sudah dihitung dari sumbu horizontal.
    // -------------------------------------------------------------------------
    always_comb begin
        case (quadrant)
            2'd0:    begin cos_val = -c_pos; sin_val =  s_pos; end
            2'd1:    begin cos_val =  c_pos; sin_val =  s_pos; end
            2'd2:    begin cos_val =  c_pos; sin_val = -s_pos; end
            default: begin cos_val = -c_pos; sin_val = -s_pos; end
        endcase
    end

endmodule