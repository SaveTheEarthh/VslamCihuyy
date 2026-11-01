// =============================================================================
// rbrief_top.sv : pembungkus yang menyambung trig_decoder ke rbrief
//
// Tugas modul:
//   Menerima arah keypoint dalam bentuk (kuadran, sektor) dari modul orientasi,
//   menerjemahkannya jadi cos_val dan sin_val dengan trig_decoder, lalu
//   meneruskannya ke rbrief. Kedua modul di dalamnya tidak diubah.
//
//   quadrant, sector ──► [trig_decoder] ──cos_val, sin_val──► [rbrief] ──► descriptor
//
// Port:
//   clk, rst_n, start, window_flat, busy, done, descriptor : sama seperti rbrief
//   quadrant : 0..3, dari modul orientasi
//   sector   : 0..15, dari modul orientasi
//
// Catatan:
//   - rbrief menyimpan cos_val dan sin_val saat start, jadi quadrant dan sector
//     cukup valid pada saat start.
//   - Kalau nanti ada beberapa rbrief paralel, satu trig_decoder bisa dipakai
//     bersama (modul ini cukup untuk satu rbrief).
// =============================================================================
module rbrief_top #(
    parameter PATTERN_FILE = "vectors/pattern_rom.hex"
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 start,
    input  logic [1369*6-1:0]    window_flat,
    input  logic [1:0]           quadrant,
    input  logic [3:0]           sector,
    output logic                 busy,
    output logic                 done,
    output logic [255:0]         descriptor
);

    // Kabel penghubung antara penerjemah dan rbrief
    logic signed [8:0] cos_val;
    logic signed [8:0] sin_val;

    // Langkah 1: terjemahkan (kuadran, sektor) menjadi cos dan sin
    trig_decoder u_trig_decoder (
        .quadrant(quadrant),
        .sector  (sector),
        .cos_val (cos_val),
        .sin_val (sin_val)
    );

    // Langkah 2: hitung descriptor dengan cos dan sin tersebut
    rbrief #(.PATTERN_FILE(PATTERN_FILE)) u_rbrief (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .window_flat(window_flat),
        .cos_val    (cos_val),
        .sin_val    (sin_val),
        .busy       (busy),
        .done       (done),
        .descriptor (descriptor)
    );

endmodule