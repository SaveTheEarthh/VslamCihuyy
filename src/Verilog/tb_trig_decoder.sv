// =============================================================================
// tb_trig_decoder.sv : testbench untuk trig_decoder
//
// Mencoba semua 64 kombinasi (kuadran x sektor) dan membandingkan cos_val dan
// sin_val dengan jawaban dari trig_decoder_golden.py (vectors/trig_decoder.hex).
// Jalankan trig_decoder_golden.py lebih dulu, dan jalankan simulasi dari folder
// yang berisi vectors/.
// =============================================================================
`timescale 1ns/1ps

module tb_trig_decoder;

    logic [1:0]        quadrant;
    logic [3:0]        sector;
    logic signed [8:0] cos_val, sin_val;

    trig_decoder dut (
        .quadrant(quadrant),
        .sector  (sector),
        .cos_val (cos_val),
        .sin_val (sin_val)
    );

    // Satu baris file = {quadrant[1:0], sector[3:0], cos[8:0], sin[8:0]} (24 bit)
    logic [23:0] vec [0:63];
    logic [8:0]  exp_cos, exp_sin;
    int          pass_cnt, fail_cnt;

    initial begin
        $readmemh("vectors/trig_decoder.hex", vec);
        pass_cnt = 0;
        fail_cnt = 0;

        for (int i = 0; i < 64; i++) begin
            // Masukkan kuadran dan sektor dari file, tunggu rangkaian settle
            quadrant = vec[i][23:22];
            sector   = vec[i][21:18];
            exp_cos  = vec[i][17:9];
            exp_sin  = vec[i][8:0];
            #1;

            if (cos_val === $signed(exp_cos) && sin_val === $signed(exp_sin)) begin
                pass_cnt++;
            end else begin
                fail_cnt++;
                $display("GAGAL: kuadran %0d sektor %0d -> RTL (cos=%0d, sin=%0d), seharusnya (cos=%0d, sin=%0d)",
                         quadrant, sector, cos_val, sin_val,
                         $signed(exp_cos), $signed(exp_sin));
            end
        end

        $display("========================================");
        $display("LULUS: %0d dari 64   GAGAL: %0d", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("SEMUA KOMBINASI BENAR");
        $display("========================================");
        $finish;
    end

endmodule