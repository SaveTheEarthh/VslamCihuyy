// =============================================================================
// tb_rbrief.sv : testbench untuk modul rbrief
//
// Cara kerja:
//   1. Baca vektor tes dari folder vectors/ (dibuat oleh rbrief_only.py).
//   2. Untuk tiap vektor: masukkan window dan sudut ke modul, tekan start,
//      tunggu done.
//   3. Bandingkan descriptor dari modul dengan jawaban yang benar
//      (expected.hex dari Python). Sama = lulus, beda = gagal.
//   4. Cetak ringkasan di akhir.
//
// Jalankan python rbrief_only.py lebih dulu supaya folder vectors/ ada, dan
// jalankan simulasi dari folder yang berisi vectors/ tersebut.
// =============================================================================
`timescale 1ns/1ps

module tb_rbrief;

    // -------------------------------------------------------------------------
    // Konstanta
    // -------------------------------------------------------------------------
    localparam int N_TESTS = 100;    // jumlah vektor tes (samakan dengan Python)
    localparam int PIX_PER = 1369;   // jumlah piksel per window = 37 x 37

    // -------------------------------------------------------------------------
    // Sinyal untuk menghubungkan modul yang diuji (DUT = device under test)
    // -------------------------------------------------------------------------
    logic                clk = 1'b0;
    logic                rst_n;
    logic                start;
    logic [1369*6-1:0]   window_flat;
    logic signed [8:0]   cos_val, sin_val;
    logic                busy, done;
    logic [255:0]        descriptor;

    rbrief #(.PATTERN_FILE("vectors/pattern_rom.hex")) dut (
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

    // Clock: berbalik tiap 5 ns, jadi periode 10 ns (100 MHz)
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Memori tempat file vektor dibaca
    // -------------------------------------------------------------------------
    logic [7:0]   win_mem [0:N_TESTS*PIX_PER-1];  // semua piksel semua tes, 1 piksel per entri
    logic [19:0]  par_mem [0:N_TESTS-1];          // {sin[8:0], cos[8:0]} per tes
    logic [255:0] exp_mem [0:N_TESTS-1];          // descriptor yang benar per tes

    int pass_cnt;    // jumlah tes lulus
    int fail_cnt;    // jumlah tes gagal

    // Baca ketiga file hex ke memori pada awal simulasi
    initial begin
        $readmemh("vectors/windows.hex",  win_mem);
        $readmemh("vectors/params.hex",   par_mem);
        $readmemh("vectors/expected.hex", exp_mem);
    end

    // Pengaman waktu: hentikan simulasi kalau terlalu lama (misalnya done tidak
    // pernah muncul karena ada bug).
    initial begin
        #(N_TESTS * 3000 * 10);
        $display("TIMEOUT: simulasi terlalu lama, kemungkinan done tidak pernah muncul");
        $finish;
    end

    // -------------------------------------------------------------------------
    // Program tes utama
    // -------------------------------------------------------------------------
    initial begin
        // Keadaan awal: reset aktif, semua input 0
        rst_n       = 1'b0;
        start       = 1'b0;
        window_flat = '0;
        cos_val     = '0;
        sin_val     = '0;
        pass_cnt    = 0;
        fail_cnt    = 0;

        // Tahan reset selama 4 clock, lepas, lalu tunggu 2 clock
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // Jalankan tes satu per satu
        for (int t = 0; t < N_TESTS; t++) begin

            // a) Susun window dari file. Piksel ke-p (urut baris demi baris)
            //    ditaruh di bit [p*6 +: 6], sesuai format window_flat.
            //    Hanya 6 bit bawah yang dipakai (nilai piksel 0..63).
            for (int p = 0; p < PIX_PER; p++)
                window_flat[p*6 +: 6] = win_mem[t*PIX_PER + p][5:0];

            // b) Ambil sudut dari file: cos di 9 bit bawah, sin di 9 bit atasnya.
            cos_val = par_mem[t][8:0];
            sin_val = par_mem[t][17:9];

            // c) Kirim pulsa start selama 1 clock.
            @(posedge clk); start <= 1'b1;
            @(posedge clk); start <= 1'b0;

            // d) Tunggu modul selesai (done = 1).
            @(posedge clk);
            while (!done) @(posedge clk);

            // e) Bandingkan descriptor dari modul dengan jawaban yang benar.
            //    Operator === membandingkan persis, termasuk nilai X atau Z.
            if (descriptor === exp_mem[t]) begin
                pass_cnt++;
            end else begin
                fail_cnt++;
                $display("GAGAL pada tes %0d", t);
                $display("  hasil RTL : %064h", descriptor);
                $display("  seharusnya: %064h", exp_mem[t]);
                // Bit yang berbeda bernilai 1 pada baris ini (membantu mencari bug)
                $display("  bit beda  : %064h", descriptor ^ exp_mem[t]);
            end
        end

        // Ringkasan akhir
        $display("========================================");
        $display("LULUS: %0d dari %0d   GAGAL: %0d", pass_cnt, N_TESTS, fail_cnt);
        if (fail_cnt == 0)
            $display("SEMUA TES LULUS (hasil RTL sama dengan Python)");
        $display("========================================");
        $finish;
    end

endmodule