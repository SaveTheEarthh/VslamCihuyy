// orb_lane.sv - one of the LANES parallel compare units.
//
// Lane L owns train keypoints L, L+8, L+16, ... (row k holds index 8k+L), so
// all lanes together compare 8 candidates per clock. Each lane:
//   - stores its share of the train data (descriptor, meta, matched_dist)
//   - per clock, reads one candidate, checks it (STEP 1), computes the Hamming
//     distance (STEP 2) and updates its own best / second best (STEP 3)
// The core merges the 8 lane results after the scan.
//
// Pipeline (one candidate enters per clock while rd_en = 1):
//   s1  memory read             (descriptor, meta, matched_dist)
//   s2  memory output register  (lets Vivado use the URAM/BRAM output register)
//   s3  XOR (1 = differing bit), candidate checks (range, valid, level, window)
//   s4  popcount part 1 (5 adder levels)
//   s5  popcount part 2 (3 adder levels) -> distance
//   s6  one-to-one check, update best / second best
// So the lane result is final when pipe_busy goes low.
module orb_lane
    import orb_match_pkg::*;
#(
    parameter int LANE = 0
) (
    input  logic                 clk,

    // ---- load port: write train keypoint (8*ld_row + LANE) during train load
    input  logic                 ld_en,
    input  logic [ROW_W-1:0]     ld_row,
    input  logic [255:0]         ld_desc,
    input  logic [63:0]          ld_meta,

    // ---- matched_dist update from the core's decision (STEP 5)
    input  logic                 md_we,
    input  logic [ROW_W-1:0]     md_row,
    input  logic [DIST_W-1:0]    md_dist,

    // ---- the current query (held stable by the core during the scan)
    input  logic [255:0]         q_desc,
    input  coord_t               qx, qy,        // query position (Q11.8)
    input  coord_t               r,             // window half-size (Q11.8)
    input  logic [2:0]           min_lvl, max_lvl,
    input  logic                 use_win,
    input  logic                 one_to_one,
    input  logic [CNT_W-1:0]     lo, hi,        // scan train indices lo .. hi-1

    // ---- scan control
    input  logic                 clear,         // start of a query: forget best/second
    input  logic                 rd_en,         // read row rd_row this clock
    input  logic [ROW_W-1:0]     rd_row,
    output logic                 pipe_busy,     // candidates still in flight

    // ---- lane result (valid when pipe_busy = 0)
    output logic [DIST_W-1:0]    best,
    output logic [DIST_W-1:0]    second,
    output logic [IDX_W-1:0]     best_idx,
    output logic [15:0]          best_ang       // train angle of the best (for STEP 6)
);

    // ------------------------------------------------------------ memories
    // 8 lanes x 256 bits must be read every clock: UltraRAM (see the notes).
    (* ram_style = "ultra" *) logic [255:0]      desc_mem [ROWS];
    (* ram_style = "ultra" *) logic [63:0]       meta_mem [ROWS];
    (* ram_style = "block" *) logic [DIST_W-1:0] md_mem   [ROWS];   // matched_dist

    logic [255:0]      desc_s1, desc_s2;
    logic [63:0]       meta_s1, meta_s2;
    logic [DIST_W-1:0] md_s1,   md_s2;

    always_ff @(posedge clk) begin
        if (ld_en) begin
            desc_mem[ld_row] <= ld_desc;
            meta_mem[ld_row] <= ld_meta;
        end
        if (rd_en) begin
            desc_s1 <= desc_mem[rd_row];
            meta_s1 <= meta_mem[rd_row];
        end
        desc_s2 <= desc_s1;
        meta_s2 <= meta_s1;
    end

    // matched_dist: written with "nobody owns it" (511) at load, then by the
    // core's decision. The two never happen at the same time.
    always_ff @(posedge clk) begin
        if (ld_en)
            md_mem[ld_row] <= DIST_NONE;
        else if (md_we)
            md_mem[md_row] <= md_dist;
        if (rd_en)
            md_s1 <= md_mem[rd_row];
        md_s2 <= md_s1;
    end

    // ------------------------------------------------------ pipeline control
    logic             v1, v2, v3, v4, v5;      // "a candidate is in stage n"
    logic [ROW_W-1:0] row1, row2;

    always_ff @(posedge clk) begin
        v1   <= rd_en;
        v2   <= v1;
        row1 <= rd_row;
        row2 <= row1;
    end

    assign pipe_busy = v1 | v2 | v3 | v4 | v5;

    // ---------------------------------------------- s3: XOR + candidate checks
    // Index of the candidate in stage 2/3: j = 8 * row + LANE
    logic [CNT_W-1:0] j2;
    assign j2 = {1'b0, row2, 3'(LANE)};

    coord_t dx2, dy2;
    logic   cand_ok2;
    assign dx2 = sext20(meta_s2[19:0])  - qx;
    assign dy2 = sext20(meta_s2[39:20]) - qy;
    assign cand_ok2 = (j2 >= lo) && (j2 < hi)                       // inside this query's range
                   && meta_s2[63]                                   // valid keypoint
                   && (meta_s2[58:56] >= min_lvl) && (meta_s2[58:56] <= max_lvl)  // level
                   && (!use_win || (dx2 < r && dx2 > -r && dy2 < r && dy2 > -r)); // |dx|,|dy| < r

    logic [255:0]      x3;
    logic              ok3;
    logic [IDX_W-1:0]  j3;
    logic [15:0]       ang3;
    logic [DIST_W-1:0] md3;

    always_ff @(posedge clk) begin
        v3    <= v2;
        x3    <= desc_s2 ^ q_desc;                  // STEP 2: 1 where the bits differ
        ok3   <= cand_ok2;                          // STEP 1
        j3    <= j2[IDX_W-1:0];
        ang3  <= meta_s2[55:40];
        md3   <= md_s2;
    end

    // ---------------------------------------------- s4: popcount part 1
    logic [8*6-1:0]    part4;
    logic              ok4;
    logic [IDX_W-1:0]  j4;
    logic [15:0]       ang4;
    logic [DIST_W-1:0] md4;

    always_ff @(posedge clk) begin
        v4    <= v3;
        part4 <= popcount_part(x3);                 // STEP 2: 8 counts of 32 bits
        ok4   <= ok3;
        j4    <= j3;
        ang4  <= ang3;
        md4   <= md3;
    end

    // ---------------------------------------------- s5: popcount part 2
    logic [DIST_W-1:0] d5;
    logic              ok5;
    logic [IDX_W-1:0]  j5;
    logic [15:0]       ang5;
    logic [DIST_W-1:0] md5;

    always_ff @(posedge clk) begin
        v5   <= v4;
        d5   <= popcount_sum(part4);                // STEP 2: distance 0..256
        ok5  <= ok4;
        j5   <= j4;
        ang5 <= ang4;
        md5  <= md4;
    end

    // ---------------------------------------------- s6: best / second best
    // STEP 5 part 1: skip a candidate already owned with an equal/better distance.
    // STEP 3: strict "<" and increasing j per lane -> lowest index wins ties.
    logic take;
    assign take = v5 && ok5 && (!one_to_one || d5 < md5);

    always_ff @(posedge clk) begin
        if (clear) begin
            best     <= DIST_NONE;
            second   <= DIST_NONE;
            best_idx <= '0;
            best_ang <= '0;
        end else if (take) begin
            if (d5 < best) begin
                second   <= best;
                best     <= d5;
                best_idx <= j5;
                best_ang <= ang5;
            end else if (d5 < second) begin
                second   <= d5;
            end
        end
    end

endmodule
