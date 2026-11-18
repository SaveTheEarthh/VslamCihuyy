// fast_core.sv - FAST-9 on the 16-pixel Bresenham ring (radius 3) of a 7x7 window.
// corner = 9 circularly contiguous brighter OR darker (> thr); all 16 positions tested in parallel.
// score  = SAD of the 16 ring pixels vs centre (0 if not a corner), max 16*255 -> 12 bit. No multipliers.
// Latency 3 clk, fully pipelined, 1 window/clk.
module fast_core (
    input  logic         clk,
    input  logic         rst,
    input  logic         in_valid,
    input  logic [391:0] win,        // layout of fast_linebuf
    input  logic [7:0]   thr,
    output logic         out_valid,
    output logic [11:0]  out_score
);

    // ring pixel i -> index in the flattened 7x7 window: (3+dy)*7 + (3+dx), clockwise from the top
    function automatic int ring_idx(input int i);
        case (i)
            0:  ring_idx = 3;
            1:  ring_idx = 4;
            2:  ring_idx = 12;
            3:  ring_idx = 20;
            4:  ring_idx = 27;
            5:  ring_idx = 34;
            6:  ring_idx = 40;
            7:  ring_idx = 46;
            8:  ring_idx = 45;
            9:  ring_idx = 44;
            10: ring_idx = 36;
            11: ring_idx = 28;
            12: ring_idx = 21;
            13: ring_idx = 14;
            14: ring_idx = 8;
            default: ring_idx = 2;
        endcase
    endfunction

    logic [7:0]   p;
    logic [7:0]   r;
    logic [15:0]  bri, dar;
    logic [127:0] ad;
    logic [24:0]  bw, dw;
    logic         corner;

    assign p = win[24*8 +: 8];       // centre (3,3)

    always_comb begin
        bri = '0;
        dar = '0;
        ad  = '0;
        for (int i = 0; i < 16; i++) begin
            r        = win[ring_idx(i)*8 +: 8];
            bri[i]   = ({1'b0, r} > ({1'b0, p} + {1'b0, thr}));   // 9 bit: no wrap
            dar[i]   = ({1'b0, p} > ({1'b0, r} + {1'b0, thr}));
            ad[i*8 +: 8] = (r > p) ? (r - p) : (p - r);
        end
    end

    // wrap the ring so a 9-wide slice covers every circular run
    assign bw = {bri[8:0], bri};
    assign dw = {dar[8:0], dar};

    always_comb begin
        corner = 1'b0;
        for (int j = 0; j < 16; j++)
            if ((&bw[j +: 9]) || (&dw[j +: 9]))
                corner = 1'b1;
    end

    // s1: corner flag + 16 |diff| ; s2: four partial sums of 4 ; s3: final sum
    logic         v1, v2, c1, c2;
    logic [127:0] ad_r;
    logic [39:0]  ps_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            v1        <= 1'b0;
            v2        <= 1'b0;
            out_valid <= 1'b0;
        end else begin
            v1        <= in_valid;
            v2        <= v1;
            out_valid <= v2;
        end

        c1   <= corner;
        ad_r <= ad;

        c2 <= c1;
        for (int n = 0; n < 4; n++)
            ps_r[n*10 +: 10] <= ad_r[(4*n)*8 +: 8]   + ad_r[(4*n+1)*8 +: 8]
                              + ad_r[(4*n+2)*8 +: 8] + ad_r[(4*n+3)*8 +: 8];

        out_score <= c2 ? (ps_r[9:0] + ps_r[19:10] + ps_r[29:20] + ps_r[39:30]) : 12'd0;
    end

endmodule
