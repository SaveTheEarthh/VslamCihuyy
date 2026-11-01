// =============================================================================
// rbrief.sv : modul rBRIEF (versi dasar, 1 pasang titik per clock)
//
// Tugas modul:
//   Untuk satu keypoint, hasilkan descriptor 256-bit. Tiap clock memproses satu
//   pasang titik (A, B): putar kedua titik sesuai sudut keypoint, ambil piksel
//   di posisi hasil putaran, lalu bandingkan. Kalau A lebih terang dari B, bit
//   descriptor ke-i = 1.
//   Setara dengan fungsi rbrief() dan rotate() di rbrief_only.py.
//
// Port:
//   clk, rst_n   : clock dan reset aktif-rendah
//   start        : pulsa 1 clock untuk memulai satu perhitungan
//   window_flat  : window 37x37, tiap piksel 6 bit (1369 x 6 = 8214 bit).
//                  Piksel (baris r, kolom c) ada di bit [(r*37 + c)*6 +: 6].
//                  Baris 0 = paling atas (y = -18), kolom 0 = paling kiri (x = -18).
//   cos_val      : cos sudut keypoint, signed 9 bit, dikali 128 (-128..+128)
//   sin_val      : sin sudut keypoint, signed 9 bit, dikali 128 (-128..+128)
//   busy         : 1 selama perhitungan berjalan
//   done         : pulsa 1 clock saat descriptor sudah selesai
//   descriptor   : hasil 256 bit, valid saat done = 1
//
// Asumsi:
//   - window_flat harus tetap stabil dari start sampai done (di paper, window
//     "dibekukan" selama descriptor dihitung). Modul tidak menyalin window.
//   - cos_val dan sin_val disimpan (latch) saat start, jadi boleh berubah setelahnya.
//
// Waktu: 256 clock untuk 256 pasangan, ditambah 1 clock sebelum done muncul.
// =============================================================================
module rbrief #(
    // File isi ROM pola titik, dibuat oleh rbrief_only.py
    parameter PATTERN_FILE = "vectors/pattern_rom.hex"
) (
    input  logic                 clk,
    input  logic                 rst_n,
    input  logic                 start,
    input  logic [1369*6-1:0]    window_flat,
    input  logic signed [8:0]    cos_val,
    input  logic signed [8:0]    sin_val,
    output logic                 busy,
    output logic                 done,
    output logic [255:0]         descriptor
);

    // -------------------------------------------------------------------------
    // Konstanta
    // -------------------------------------------------------------------------
    localparam [10:0]        WIN          = 11'd37;    // lebar window
    localparam [5:0]         HALF         = 6'd18;     // jarak pusat ke tepi window
    localparam               SCALE_BITS   = 7;         // cos/sin dikali 2^7 = 128
    localparam signed [15:0] ROUND_OFFSET = 16'sd64;   // 0,5 dalam skala 128, untuk pembulatan

    // -------------------------------------------------------------------------
    // ROM pola titik
    // Satu baris = satu pasangan, 20 bit: {x1, y1, x2, y2}, tiap koordinat
    // 5 bit signed (-13..+13). Diisi dari file saat simulasi dimulai.
    // -------------------------------------------------------------------------
    logic [19:0] pattern_rom [0:255];
    initial $readmemh(PATTERN_FILE, pattern_rom);

    // -------------------------------------------------------------------------
    // Register state
    // -------------------------------------------------------------------------
    logic [7:0]        idx;      // nomor pasangan yang sedang diproses (0..255)
    logic signed [8:0] cos_q;    // salinan cos_val yang disimpan saat start
    logic signed [8:0] sin_q;    // salinan sin_val yang disimpan saat start
    logic              running;  // 1 selama perhitungan berjalan

    assign busy = running;

    // -------------------------------------------------------------------------
    // Langkah 1: baca satu pasangan titik dari ROM
    // idx menentukan baris ROM. Hasilnya dipecah menjadi 4 koordinat.
    // -------------------------------------------------------------------------
    logic [19:0]       pair_word;
    logic signed [4:0] x1, y1;   // titik A (sebelum diputar)
    logic signed [4:0] x2, y2;   // titik B (sebelum diputar)

    assign pair_word = pattern_rom[idx];
    assign x1 = pair_word[19:15];
    assign y1 = pair_word[14:10];
    assign x2 = pair_word[9:5];
    assign y2 = pair_word[4:0];

    // -------------------------------------------------------------------------
    // Langkah 2: putar titik A dan B (rotator)
    //   xr = (x*cos - y*sin + 64) >>> 7
    //   yr = (x*sin + y*cos + 64) >>> 7
    // Semua operand diperlebar menjadi 16 bit bertanda supaya perkaliannya
    // tidak meluap. Nilai terbesarnya sekitar 13*128*2 = 3328, muat di 16 bit.
    // -------------------------------------------------------------------------
    logic signed [15:0] x1e, y1e, x2e, y2e, ce, se;
    assign x1e = x1;       // perlebaran 5 bit -> 16 bit, tanda ikut diperluas
    assign y1e = y1;
    assign x2e = x2;
    assign y2e = y2;
    assign ce  = cos_q;    // perlebaran 9 bit -> 16 bit
    assign se  = sin_q;

    // Hasil perkalian dan penjumlahan, masih dalam skala 128
    logic signed [15:0] xa_acc, ya_acc, xb_acc, yb_acc;
    assign xa_acc = x1e * ce - y1e * se + ROUND_OFFSET;
    assign ya_acc = x1e * se + y1e * ce + ROUND_OFFSET;
    assign xb_acc = x2e * ce - y2e * se + ROUND_OFFSET;
    assign yb_acc = x2e * se + y2e * ce + ROUND_OFFSET;

    // Bagi 128 dengan geser kanan aritmetik (>>>). Hasilnya koordinat titik
    // setelah diputar, dalam satuan piksel (-18..+18).
    logic signed [15:0] xa, ya, xb, yb;
    assign xa = xa_acc >>> SCALE_BITS;
    assign ya = ya_acc >>> SCALE_BITS;
    assign xb = xb_acc >>> SCALE_BITS;
    assign yb = yb_acc >>> SCALE_BITS;

    // -------------------------------------------------------------------------
    // Langkah 3: geser titik pusat dari (0,0) ke (18,18)
    // Koordinat -18..+18 menjadi indeks 0..36. Cukup 6 bit, dan hitungan
    // modulo 64 tetap menghasilkan nilai yang benar karena hasilnya selalu <= 36.
    // -------------------------------------------------------------------------
    logic [5:0] kolom_a, baris_a, kolom_b, baris_b;
    assign kolom_a = xa[5:0] + HALF;
    assign baris_a = ya[5:0] + HALF;
    assign kolom_b = xb[5:0] + HALF;
    assign baris_b = yb[5:0] + HALF;

    // -------------------------------------------------------------------------
    // Langkah 4: ambil piksel A dan B dari window
    // Posisi piksel (baris, kolom) dalam window yang di-flatten:
    //     pos = baris*37 + kolom     (0..1368)
    // Tiap piksel 6 bit, jadi bit awalnya pos*6.
    // -------------------------------------------------------------------------
    logic [10:0] pos_a, pos_b;
    assign pos_a = {5'b0, baris_a} * WIN + {5'b0, kolom_a};
    assign pos_b = {5'b0, baris_b} * WIN + {5'b0, kolom_b};

    logic [5:0] pix_a, pix_b;
    assign pix_a = window_flat[pos_a * 6 +: 6];
    assign pix_b = window_flat[pos_b * 6 +: 6];

    // -------------------------------------------------------------------------
    // Langkah 5: FSM (mesin keadaan) yang mengatur urutan 256 pasangan
    //
    //   Menunggu : running = 0, menunggu start.
    //   Start    : simpan cos/sin, kosongkan descriptor, idx = 0, running = 1.
    //   Berjalan : tiap clock isi bit ke-idx dengan hasil (pix_a > pix_b),
    //              lalu idx naik 1.
    //   Selesai  : setelah idx = 255, running = 0 dan done = 1 selama 1 clock.
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset: semua kembali ke keadaan awal
            running    <= 1'b0;
            done       <= 1'b0;
            idx        <= 8'd0;
            cos_q      <= 9'sd0;
            sin_q      <= 9'sd0;
            descriptor <= 256'd0;
        end else begin
            // done hanya menyala 1 clock, jadi default-nya 0 tiap clock.
            done <= 1'b0;

            if (!running) begin
                // Menunggu: mulai kalau ada pulsa start
                if (start) begin
                    cos_q      <= cos_val;     // simpan sudut untuk keypoint ini
                    sin_q      <= sin_val;
                    descriptor <= 256'd0;      // mulai dari 256 bit nol
                    idx        <= 8'd0;        // mulai dari pasangan ke-0
                    running    <= 1'b1;
                end
            end else begin
                // Berjalan: isi satu bit descriptor tiap clock.
                // Bit bernilai 1 kalau piksel A lebih terang dari piksel B.
                descriptor[idx] <= (pix_a > pix_b);

                if (idx == 8'd255) begin
                    // Pasangan terakhir sudah diproses
                    running <= 1'b0;
                    done    <= 1'b1;
                end
                // Lanjut ke pasangan berikutnya. Setelah 255 kembali ke 0.
                idx <= idx + 8'd1;
            end
        end
    end

endmodule