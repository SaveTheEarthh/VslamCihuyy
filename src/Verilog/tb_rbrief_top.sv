// =============================================================================
// tb_rbrief_top.sv : testbench untuk rbrief_top (penerjemah + rbrief)
//
// Memakai vektor yang sama dengan tb_rbrief.sv (folder vectors/ dari
// rbrief_only.py). Bedanya, input sudut ke modul berupa (kuadran, sektor), bukan
// cos dan sin. Untuk tiap tes, testbench mencari (kuadran, sektor) yang
// menghasilkan cos dan sin dari params.hex, memasukkannya ke modul, lalu
// membandingkan descriptor dengan expected.hex.
//
// Yang diuji di sini adalah sambungan kedua modul (lebar bit, tanda, urutan
// sinyal). Kebenaran isi tabel penerjemah diuji terpisah oleh tb_trig_decoder.sv.
// =============================================================================
`timescale 1ns/1ps

module tb_rbrief_top;

    localparam int N_TESTS = 100;    // samakan dengan jumlah tes di Python
    localparam int PIX_PER = 1369;   // 37 x 37

    // -------------------------------------------------------------------------
    // Modul yang diuji
    // -------------------------------------------------------------------------
    logic                clk = 1'b0;
    logic                rst_n;
    logic                start;
    logic [1369*6-1:0]   window_flat;
    logic [1:0]          quadrant;
    logic [3:0]          sector;
    logic                busy, done;
    logic [255:0]        descriptor;

    rbrief_top #(.PATTERN_FILE("vectors/pattern_rom.hex")) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .window_flat(window_flat),
        .quadrant   (quadrant),
        .sector     (sector),
        .busy       (busy),
        .done       (done),
        .descriptor (descriptor)
    );

    always #5 clk = ~clk;            // periode 10 ns

    // -------------------------------------------------------------------------
    // Penerjemah kedua, hanya untuk membuat tabel pencari: cos dan sin untuk
    // setiap kombinasi (kuadran, sektor). Indeks tabel = kuadran*16 + sektor.
    // -------------------------------------------------------------------------
    logic [1:0]        ref_q;
    logic [3:0]        ref_k;
    logic signed [8:0] ref_cos, ref_sin;
    trig_decoder ref_dec (.quadrant(ref_q), .sector(ref_k),
                          .cos_val(ref_cos), .sin_val(ref_sin));

    logic signed [8:0] tab_cos [0:63];
    logic signed [8:0] tab_sin [0:63];

    // -------------------------------------------------------------------------
    // Memori vektor tes
    // -------------------------------------------------------------------------
    logic [7:0]   win_mem [0:N_TESTS*PIX_PER-1];
    logic [19:0]  par_mem [0:N_TESTS-1];       // {sin[8:0], cos[8:0]}
    logic [255:0] exp_mem [0:N_TESTS-1];

    int pass_cnt, fail_cnt;
    int found;                                  // indeks tabel yang cocok

    initial begin
        $readmemh("vectors/windows.hex",  win_mem);
        $readmemh("vectors/params.hex",   par_mem);
        $readmemh("vectors/expected.hex", exp_mem);
    end

    // Pengaman waktu
    initial begin
        #(N_TESTS * 3000 * 10);
        $display("TIMEOUT: simulasi terlalu lama");
        $finish;
    end

    // -------------------------------------------------------------------------
    // Program tes utama
    // -------------------------------------------------------------------------
    initial begin
        rst_n = 1'b0; start = 1'b0; window_flat = '0;
        quadrant = '0; sector = '0;
        pass_cnt = 0; fail_cnt = 0;

        // a) Isi tabel pencari dari penerjemah (64 kombinasi)
        for (int i = 0; i < 64; i++) begin
            ref_q = i / 16;
            ref_k = i % 16;
            #1;
            tab_cos[i] = ref_cos;
            tab_sin[i] = ref_sin;
        end

        // b) Reset
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // c) Jalankan tes satu per satu
        for (int t = 0; t < N_TESTS; t++) begin

            // Susun window dari file (urutan sama dengan tb_rbrief)
            for (int p = 0; p < PIX_PER; p++)
                window_flat[p*6 +: 6] = win_mem[t*PIX_PER + p][5:0];

            // Cari (kuadran, sektor) yang menghasilkan cos dan sin tes ini
            found = -1;
            for (int i = 0; i < 64; i++)
                if (tab_cos[i] === $signed(par_mem[t][8:0]) &&
                    tab_sin[i] === $signed(par_mem[t][17:9]))
                    found = i;

            if (found < 0) begin
                fail_cnt++;
                $display("GAGAL tes %0d: cos/sin tidak ada di tabel kuadran-sektor", t);
            end else begin
                quadrant = found / 16;
                sector   = found % 16;

                // Pulsa start 1 clock, lalu tunggu done
                @(posedge clk); start <= 1'b1;
                @(posedge clk); start <= 1'b0;
                @(posedge clk);
                while (!done) @(posedge clk);

                if (descriptor === exp_mem[t]) begin
                    pass_cnt++;
                end else begin
                    fail_cnt++;
                    $display("GAGAL tes %0d (kuadran %0d, sektor %0d)", t, quadrant, sector);
                    $display("  hasil RTL : %064h", descriptor);
                    $display("  seharusnya: %064h", exp_mem[t]);
                    $display("  bit beda  : %064h", descriptor ^ exp_mem[t]);
                end
            end
        end

        $display("========================================");
        $display("LULUS: %0d dari %0d   GAGAL: %0d", pass_cnt, N_TESTS, fail_cnt);
        if (fail_cnt == 0)
            $display("SEMUA TES LULUS (penerjemah + rbrief sama dengan Python)");
        $display("========================================");
        $finish;
    end

endmodule