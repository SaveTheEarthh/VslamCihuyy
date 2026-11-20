// orb_match_pkg.sv - constants and helper functions for the RTL ORB matcher.
//
// Same algorithm and data formats as the C versions (../hls/orb_match.h,
// ../model/orb_match_ref.c); the testbench checks the RTL against orb_match_ref.c.
package orb_match_pkg;

    // ------------------------------------------------------------ sizing
    localparam int LANES     = 8;                    // candidates compared per clock
    localparam int MAX_TRAIN = 8192;                 // frame-2 keypoints
    localparam int MAX_QUERY = 8192;                 // frame-1 keypoints
    localparam int ROWS      = MAX_TRAIN / LANES;    // depth of each lane's memory
    localparam int IDX_W     = $clog2(MAX_TRAIN);    // 13: train index
    localparam int QIDX_W    = $clog2(MAX_QUERY);    // 13: query index
    localparam int ROW_W     = $clog2(ROWS);         // 10: row = index / LANES
    localparam int CNT_W     = IDX_W + 1;            // 14: counts 0..8192

    // ------------------------------------------------- ORB-SLAM3 constants
    localparam int          DIST_W       = 9;        // distances 0..256, plus 511
    localparam logic [8:0]  DIST_NONE    = 9'd511;   // "no candidate"
    localparam int          HISTO_LENGTH = 30;       // rotation histogram bins
    localparam int          LIM_LEN      = 257;      // ratio table entries

    // ------------------------------------------------- packed word fields
    // Train meta word (one per train keypoint) and query meta words w0/w1:
    // same bit layout as orb_match.h.
    //   train: [19:0] x  [39:20] y  [55:40] angle  [58:56] level  [63] valid
    //   q w0 : [19:0] x  [39:20] y  [59:40] r (window half-size)
    //   q w1 : [15:0] angle [31:16] lo [47:32] hi [50:48] min lvl [53:51] max lvl
    //          [62] use window [63] valid
    // Coordinates are signed Q11.8 pixels (pixel * 256).

    typedef logic signed [21:0] coord_t;   // 20-bit field, sign-extended with headroom

    function automatic coord_t sext20(input logic [19:0] v);
        return coord_t'(signed'(v));
    endfunction

    // --------------------------------------------------------- popcount
    // Counts the 1s of 256 bits with an adder tree, the same idea as the SWAR
    // trick in ORB-SLAM3's DescriptorDistance: add neighbouring bits, then
    // neighbouring pairs, and so on (8 levels). Split 5 + 3 levels so the lane
    // can put a pipeline register in between (300 MHz).

    // Part 1: 256 bits -> 8 counts of 32 bits each (each 0..32, 6 bits).
    function automatic logic [8*6-1:0] popcount_part(input logic [255:0] v);
        logic [1:0] s1 [128];   // counts of 2 bits
        logic [2:0] s2 [64];    // counts of 4 bits
        logic [3:0] s3 [32];    // counts of 8 bits
        logic [4:0] s4 [16];    // counts of 16 bits
        logic [5:0] s5 [8];     // counts of 32 bits
        logic [8*6-1:0] out;
        for (int i = 0; i < 128; i++) s1[i] = {1'b0, v[2*i]} + {1'b0, v[2*i+1]};
        for (int i = 0; i < 64;  i++) s2[i] = {1'b0, s1[2*i]} + {1'b0, s1[2*i+1]};
        for (int i = 0; i < 32;  i++) s3[i] = {1'b0, s2[2*i]} + {1'b0, s2[2*i+1]};
        for (int i = 0; i < 16;  i++) s4[i] = {1'b0, s3[2*i]} + {1'b0, s3[2*i+1]};
        for (int i = 0; i < 8;   i++) s5[i] = {1'b0, s4[2*i]} + {1'b0, s4[2*i+1]};
        for (int i = 0; i < 8;   i++) out[6*i +: 6] = s5[i];
        return out;
    endfunction

    // Part 2: add the 8 partial counts -> 0..256.
    function automatic logic [8:0] popcount_sum(input logic [8*6-1:0] p);
        logic [6:0] s6 [4];
        logic [7:0] s7 [2];
        for (int i = 0; i < 4; i++) s6[i] = {1'b0, p[6*(2*i) +: 6]} + {1'b0, p[6*(2*i+1) +: 6]};
        for (int i = 0; i < 2; i++) s7[i] = {1'b0, s6[2*i]} + {1'b0, s6[2*i+1]};
        return {1'b0, s7[0]} + {1'b0, s7[1]};
    endfunction

    // ------------------------------------------------- rotation histogram bin
    // ORB-SLAM3: rot = angle1 - angle2 (+360 if negative); bin = round(rot/30).
    // With 16-bit angles (65536 = 360 deg): bin = (rot16 * 12 + 32768) >> 16.
    // *12 is done as *8 + *4 (two shifted adds, no multiplier). Result 0..12.
    function automatic logic [4:0] rot_bin(input logic [15:0] a1, input logic [15:0] a2);
        logic [15:0] rot16;
        logic [20:0] t;
        rot16 = a1 - a2;                                    // wraps mod 360 deg for free
        t = {2'b0, rot16, 3'b0} + {3'b0, rot16, 2'b0} + 21'd32768;
        return {1'b0, t[19:16]};
    endfunction

endpackage
