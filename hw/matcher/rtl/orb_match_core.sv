// orb_match_core.sv - ORB feature matcher, RTL version.
//
// Same algorithm as ../model/orb_match_ref.c (ORB-SLAM3 SearchForInitialization),
// same results bit for bit. Structure:
//
//   input stream (64-bit AXI-Stream, e.g. from an AXI DMA), in this order:
//     1. ratio table : 257 words, word k = ratio_lim[k] in bits [8:0]
//     2. train       : n_train x 5 words  = desc[63:0], desc[127:64],
//                      desc[191:128], desc[255:192], meta
//     3. queries     : n_query x 6 words  = 4 desc words, meta w0, meta w1
//   output stream (32-bit), one word per query after all queries are done:
//     [15:0]  match: train index, or 0xFFFF (-1) for no match
//     [31:16] best distance (511 = no candidate)
//
//   FSM per query:  FETCH (6 words) -> SETUP -> SCAN (8 candidates/clock in the
//   lanes) -> DRAIN -> MERGE (8 lanes -> 1) -> DEC0..DEC2 (accept, steal,
//   histogram) -> next query. After all queries: MAX3 (three tallest
//   histogram bins) -> WB (filter by rotation, stream results out).
module orb_match_core
    import orb_match_pkg::*;
(
    input  logic               clk,
    input  logic               rst_n,

    // ---- control / config (from the AXI-Lite registers)
    input  logic               start,          // pulse: begin a run
    input  logic [CNT_W-1:0]   n_train,
    input  logic [CNT_W-1:0]   n_query,
    input  logic [DIST_W-1:0]  th_dist,        // accept best <= th_dist (TH_LOW = 50)
    input  logic               one_to_one,         // one-to-one rule (STEP 5)
    input  logic               check_orient,   // rotation check (STEP 6)
    output logic               busy,
    output logic               done,           // pulse at the end of a run
    output logic [15:0]        n_matches,

    // ---- input stream
    input  logic [63:0]        s_tdata,
    input  logic               s_tvalid,
    output logic               s_tready,

    // ---- output stream
    output logic [31:0]        m_tdata,
    output logic               m_tvalid,
    input  logic               m_tready,
    output logic               m_tlast
);

    // =====================================================================
    // FSM
    // =====================================================================
    typedef enum logic [4:0] {
        S_IDLE, S_LIM, S_TRAIN, S_FETCH, S_SETUP, S_SCAN, S_DRAIN, S_MERGE,
        S_DEC0, S_DEC0B, S_DEC1, S_DEC2, S_NEXT, S_MAX3, S_MAX3_FIN, S_WB, S_DONE
    } state_t;

    state_t state;

    assign busy = (state != S_IDLE);
    assign s_tready = (state == S_LIM) || (state == S_TRAIN) || (state == S_FETCH);
    wire beat = s_tvalid && s_tready;              // one input word accepted

    // counters
    logic [8:0]        lim_cnt;                    // ratio table word 0..256
    logic [CNT_W-1:0]  tj;                         // train keypoint being loaded
    logic [2:0]        wcnt;                       // word within a train/query record
    logic [CNT_W-1:0]  qi;                         // current query
    localparam int ROWP_W = ROW_W + 1;              // rows 0..ROWS (one past the end)
    logic [ROWP_W-1:0] row, row_end;               // scan rows [row, row_end)
    logic [3:0]        mcnt;                       // merge step 0..7
    logic [4:0]        hcnt;                       // histogram bin 0..29

    // =====================================================================
    // Small memories (one read port, one write port each)
    // =====================================================================
    // ratio_lim (STEP 4): accept only best < lim[best2]. Depth 512 so that
    // best2 = 511 ("none") is a legal address; that entry is never used.
    logic [DIST_W-1:0] lim_mem [512];
    logic [DIST_W-1:0] lim_q, lim_q2;            // q2 = RAM output register (timing)

    // matches21 (STEP 5): which query owns each train point. {valid, query}
    logic [QIDX_W:0]   m21_mem [MAX_TRAIN];
    logic [QIDX_W:0]   m21_q, m21_q2;

    // matches12 = the answer: which train point each query got. {valid, train}
    logic [IDX_W:0]    m12_mem [MAX_QUERY];
    logic [IDX_W:0]    m12_q;

    // best distance per query, and rotation bin per matched query (STEP 6)
    logic [DIST_W-1:0] qdist_mem [MAX_QUERY];
    logic [DIST_W-1:0] qdist_q;
    logic [4:0]        qbin_mem [MAX_QUERY];
    logic [4:0]        qbin_q;

    // write / read ports, driven by the FSM below
    logic              lim_we;   logic [8:0]        lim_wa;   logic [DIST_W-1:0] lim_wd;
    logic [8:0]        lim_ra;
    logic              m21_we;   logic [IDX_W-1:0]  m21_wa;   logic [QIDX_W:0]   m21_wd;
    logic [IDX_W-1:0]  m21_ra;
    logic              m12_we;   logic [QIDX_W-1:0] m12_wa;   logic [IDX_W:0]    m12_wd;
    logic              qd_we;    logic [QIDX_W-1:0] qd_wa;    logic [DIST_W-1:0] qd_wd;
    logic              qb_we;    logic [QIDX_W-1:0] qb_wa;    logic [4:0]        qb_wd;
    logic              wb_re;    logic [QIDX_W-1:0] wb_ra;    // write-back reads

    always_ff @(posedge clk) begin
        if (lim_we) lim_mem[lim_wa] <= lim_wd;
        lim_q  <= lim_mem[lim_ra];
        lim_q2 <= lim_q;
        if (m21_we) m21_mem[m21_wa] <= m21_wd;
        m21_q  <= m21_mem[m21_ra];
        m21_q2 <= m21_q;
        if (m12_we) m12_mem[m12_wa] <= m12_wd;
        if (qd_we)  qdist_mem[qd_wa] <= qd_wd;
        if (qb_we)  qbin_mem[qb_wa] <= qb_wd;
        if (wb_re) begin                           // hold output when not reading
            m12_q   <= m12_mem[wb_ra];
            qdist_q <= qdist_mem[wb_ra];
            qbin_q  <= qbin_mem[wb_ra];
        end
    end

    // =====================================================================
    // Current query registers
    // =====================================================================
    logic [63:0]  qd_w [4];                        // descriptor words
    logic [63:0]  qm0, qm1;                        // meta words
    wire  [255:0] q_desc  = {qd_w[3], qd_w[2], qd_w[1], qd_w[0]};
    coord_t       q_x, q_y, q_r;
    assign q_x = sext20(qm0[19:0]);
    assign q_y = sext20(qm0[39:20]);
    assign q_r = coord_t'({2'b00, qm0[59:40]});
    wire  [15:0]  q_ang   = qm1[15:0];
    wire  [15:0]  q_lo16  = qm1[31:16];
    wire  [15:0]  q_hi16  = qm1[47:32];
    wire  [2:0]   q_minl  = qm1[50:48];
    wire  [2:0]   q_maxl  = qm1[53:51];
    wire          q_win   = qm1[62];
    wire          q_valid = qm1[63];

    // hi = min(hi, n_train); scan only if valid and lo < hi
    wire  [15:0]  q_hi    = (q_hi16 > 16'(n_train)) ? 16'(n_train) : q_hi16;
    wire          q_skip  = !q_valid || (q_lo16 >= q_hi);
    logic [CNT_W-1:0] q_lo_r, q_hi_r;               // registered for the lanes
    logic             skip_r;                       // this query was skipped (no scan)

    // =====================================================================
    // Lanes
    // =====================================================================
    logic [63:0]      tdesc_w [4];                 // train descriptor being assembled
    logic             lane_clear, lane_rd;
    logic [LANES-1:0] lane_ld, lane_md_we, lane_busy;
    logic [DIST_W-1:0] l_best [LANES], l_second [LANES];
    logic [IDX_W-1:0]  l_idx  [LANES];
    logic [15:0]       l_ang  [LANES];

    // the merged result (STEP 3 across lanes)
    logic [DIST_W-1:0] best, best2;
    logic [IDX_W-1:0]  best_idx;
    logic [15:0]       best_ang;

    for (genvar l = 0; l < LANES; l++) begin : g_lane
        orb_lane #(.LANE(l)) u_lane (
            .clk      (clk),
            .ld_en    (lane_ld[l]),
            .ld_row   (tj[IDX_W-1:3]),
            .ld_desc  ({tdesc_w[3], tdesc_w[2], tdesc_w[1], tdesc_w[0]}),
            .ld_meta  (s_tdata),
            .md_we    (lane_md_we[l]),
            .md_row   (best_idx[IDX_W-1:3]),
            .md_dist  (best),
            .q_desc   (q_desc),
            .qx       (q_x),
            .qy       (q_y),
            .r        (q_r),
            .min_lvl  (q_minl),
            .max_lvl  (q_maxl),
            .use_win  (q_win),
            .one_to_one   (one_to_one),
            .lo       (q_lo_r),
            .hi       (q_hi_r),
            .clear    (lane_clear),
            .rd_en    (lane_rd),
            .rd_row   (row[ROW_W-1:0]),
            .pipe_busy(lane_busy[l]),
            .best     (l_best[l]),
            .second   (l_second[l]),
            .best_idx (l_idx[l]),
            .best_ang (l_ang[l])
        );
    end

    // =====================================================================
    // Rotation histogram (STEP 6) and its three tallest bins
    // =====================================================================
    logic [CNT_W-1:0] histo [HISTO_LENGTH];
    logic [CNT_W-1:0] max1, max2, max3;
    logic [4:0]       ind1, ind2, ind3;
    logic             ind1_v, ind2_v, ind3_v;      // "this bin is kept"
    // Rotation bin of the current match. best_ang is final after S_MERGE and
    // the bin is only used in S_DEC2, so compute it a few clocks early and
    // register it (keeps subtract + *12 + round + histogram increment out of
    // one clock).
    logic [4:0]       new_bin;
    always_ff @(posedge clk) new_bin <= rot_bin(q_ang, best_ang);

    // =====================================================================
    // Output (write-back) stage
    // =====================================================================
    logic             wb_have;                     // m12_q/qdist_q/qbin_q hold query wb_cur
    logic [CNT_W-1:0] wb_cur, wb_next;
    logic [15:0]      n_matches_r;

    wire m12_valid = m12_q[IDX_W];
    wire bin_kept  = (ind1_v && qbin_q == ind1) || (ind2_v && qbin_q == ind2)
                  || (ind3_v && qbin_q == ind3);
    wire wb_reject = check_orient && m12_valid && !bin_kept;   // unusual rotation
    wire out_free  = !m_tvalid || m_tready;

    assign n_matches = n_matches_r;

    // =====================================================================
    // Main FSM
    // =====================================================================
    always_comb begin
        // defaults: nothing written, nothing read
        lim_we = 0;  lim_wa = lim_cnt;            lim_wd = s_tdata[DIST_W-1:0];
        lim_ra = best2;
        m21_we = 0;  m21_wa = tj[IDX_W-1:0];      m21_wd = '0;
        m21_ra = best_idx;
        m12_we = 0;  m12_wa = qi[QIDX_W-1:0];     m12_wd = '0;
        qd_we  = 0;  qd_wa  = qi[QIDX_W-1:0];     qd_wd  = DIST_NONE;
        qb_we  = 0;  qb_wa  = qi[QIDX_W-1:0];     qb_wd  = new_bin;
        wb_re  = 0;  wb_ra  = wb_next[QIDX_W-1:0];
        lane_ld = '0;  lane_md_we = '0;
        lane_clear = 0;  lane_rd = 0;

        case (state)
            S_LIM: lim_we = beat;

            S_TRAIN: if (beat && wcnt == 4) begin   // 5th word = meta: store keypoint tj
                lane_ld[tj[2:0]] = 1'b1;
                m21_we = 1'b1;                       // nobody owns it yet (valid = 0)
            end

            S_SETUP: lane_clear = 1'b1;

            S_NEXT: if (skip_r) begin                // skipped query: no match, no distance
                m12_we = 1'b1;
                qd_we  = 1'b1;
            end

            S_SCAN: lane_rd = 1'b1;

            S_DEC0: begin                            // reads lim[best2], m21[best_idx] (always on)
                qd_we = 1'b1;
                qd_wd = best;
            end

            S_DEC1: begin
                if (!(best <= th_dist && (best2 == DIST_NONE || best < lim_q2))) begin
                    m12_we = 1'b1;                   // STEP 4 failed: no match
                end else if (one_to_one && m21_q2[QIDX_W]) begin
                    m12_we = 1'b1;                   // STEP 5: steal from older query
                    m12_wa = m21_q2[QIDX_W-1:0];
                end
            end

            S_DEC2: begin                            // accept: query qi owns best_idx
                m12_we = 1'b1;
                m12_wd = {1'b1, best_idx};
                if (one_to_one) begin
                    m21_we = 1'b1;
                    m21_wa = best_idx;
                    m21_wd = {1'b1, qi[QIDX_W-1:0]};
                    lane_md_we[best_idx[2:0]] = 1'b1;
                end
                qb_we = check_orient;
            end

            S_WB: wb_re = wb_have ? (out_free && wb_next < n_query) : (wb_next < n_query);

            default: ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            done        <= 1'b0;
            m_tvalid    <= 1'b0;
            m_tlast     <= 1'b0;
            m_tdata     <= '0;
            n_matches_r <= '0;
            wb_have     <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                // ---------------------------------------------- idle
                S_IDLE: if (start) begin
                    lim_cnt     <= '0;
                    tj          <= '0;
                    wcnt        <= '0;
                    qi          <= '0;
                    n_matches_r <= '0;
                    for (int b = 0; b < HISTO_LENGTH; b++) histo[b] <= '0;
                    state <= S_LIM;
                end

                // ---------------------------------------------- load ratio table
                S_LIM: if (beat) begin
                    lim_cnt <= lim_cnt + 1;
                    if (lim_cnt == 9'(LIM_LEN - 1))
                        state <= (n_train != 0) ? S_TRAIN
                               : (n_query != 0) ? S_FETCH : S_MAX3;
                end

                // ---------------------------------------------- load train keypoints
                S_TRAIN: if (beat) begin
                    if (wcnt != 4) begin
                        tdesc_w[wcnt[1:0]] <= s_tdata;
                        wcnt <= wcnt + 1;
                    end else begin                   // meta word: lane write happens now
                        wcnt <= '0;
                        tj   <= tj + 1;
                        if (tj + 1 == n_train)
                            state <= (n_query != 0) ? S_FETCH : S_MAX3;
                    end
                end

                // ---------------------------------------------- fetch one query
                S_FETCH: if (beat) begin
                    case (wcnt)
                        3'd0, 3'd1, 3'd2, 3'd3: qd_w[wcnt[1:0]] <= s_tdata;
                        3'd4: qm0 <= s_tdata;
                        default: qm1 <= s_tdata;
                    endcase
                    if (wcnt == 5) begin
                        wcnt  <= '0;
                        state <= S_SETUP;
                    end else begin
                        wcnt <= wcnt + 1;
                    end
                end

                // ---------------------------------------------- set up the scan
                S_SETUP: begin
                    skip_r  <= q_skip;
                    q_lo_r  <= CNT_W'(q_lo16);
                    q_hi_r  <= CNT_W'(q_hi);
                    row     <= ROWP_W'(q_lo16 >> 3);              // first row
                    row_end <= ROWP_W'((q_hi + 16'd7) >> 3);      // one past the last row
                    state   <= q_skip ? S_NEXT : S_SCAN;
                end

                // ---------------------------------------------- 8 candidates per clock
                S_SCAN: begin
                    row <= row + 1;
                    if (row + 1 == row_end)
                        state <= S_DRAIN;
                end

                S_DRAIN: if (lane_busy == '0) begin
                    mcnt  <= '0;
                    state <= S_MERGE;
                end

                // ---------------------------------------------- merge lanes 0..7
                // lowest distance wins, ties go to the lower index;
                // second = second smallest distance over all lanes
                S_MERGE: begin
                    if (mcnt == 0) begin
                        best     <= l_best[0];
                        best2    <= l_second[0];
                        best_idx <= l_idx[0];
                        best_ang <= l_ang[0];
                    end else if (l_best[mcnt[2:0]] < best ||
                                 (l_best[mcnt[2:0]] == best && l_idx[mcnt[2:0]] < best_idx)) begin
                        best2    <= (best < l_second[mcnt[2:0]]) ? best : l_second[mcnt[2:0]];
                        best     <= l_best[mcnt[2:0]];
                        best_idx <= l_idx[mcnt[2:0]];
                        best_ang <= l_ang[mcnt[2:0]];
                    end else begin
                        best2    <= (l_best[mcnt[2:0]] < best2) ? l_best[mcnt[2:0]] : best2;
                    end
                    mcnt <= mcnt + 1;
                    if (mcnt == LANES - 1)
                        state <= S_DEC0;
                end

                // ---------------------------------------------- decide (STEPS 4, 5)
                // reads of lim[best2] and m21[best_idx] take 2 clocks (RAM +
                // output register), so the decision logic starts from a register
                S_DEC0:  state <= S_DEC0B;
                S_DEC0B: state <= S_DEC1;

                S_DEC1: begin
                    if (!(best <= th_dist && (best2 == DIST_NONE || best < lim_q2)))
                        state <= S_NEXT;           // rejected
                    else begin
                        if (one_to_one && m21_q2[QIDX_W])
                            n_matches_r <= n_matches_r - 1;   // older query lost it
                        state <= S_DEC2;
                    end
                end

                S_DEC2: begin
                    n_matches_r <= n_matches_r + 1;
                    if (check_orient)
                        histo[new_bin] <= histo[new_bin] + 1;
                    state <= S_NEXT;
                end

                S_NEXT: begin
                    qi <= qi + 1;
                    if (qi + 1 == n_query) begin
                        hcnt <= '0;
                        max1 <= '0;  max2 <= '0;  max3 <= '0;
                        ind1_v <= 1'b0;  ind2_v <= 1'b0;  ind3_v <= 1'b0;
                        state <= S_MAX3;
                    end else
                        state <= S_FETCH;
                end

                // ---------------------------------------------- three tallest bins
                S_MAX3: begin
                    if (histo[hcnt] > max1) begin
                        max3 <= max2;  ind3 <= ind2;  ind3_v <= ind2_v;
                        max2 <= max1;  ind2 <= ind1;  ind2_v <= ind1_v;
                        max1 <= histo[hcnt];  ind1 <= hcnt;  ind1_v <= 1'b1;
                    end else if (histo[hcnt] > max2) begin
                        max3 <= max2;  ind3 <= ind2;  ind3_v <= ind2_v;
                        max2 <= histo[hcnt];  ind2 <= hcnt;  ind2_v <= 1'b1;
                    end else if (histo[hcnt] > max3) begin
                        max3 <= histo[hcnt];  ind3 <= hcnt;  ind3_v <= 1'b1;
                    end
                    hcnt <= hcnt + 1;
                    if (hcnt == HISTO_LENGTH - 1)
                        state <= S_MAX3_FIN;
                end

                // 10% rule: 10 * max < max1 -> drop that bin
                S_MAX3_FIN: begin
                    if (({max2, 3'b0} + {max2, 1'b0}) < {3'b0, max1}) begin
                        ind2_v <= 1'b0;
                        ind3_v <= 1'b0;
                    end else if (({max3, 3'b0} + {max3, 1'b0}) < {3'b0, max1}) begin
                        ind3_v <= 1'b0;
                    end
                    wb_have <= 1'b0;
                    wb_next <= '0;
                    state   <= (n_query != 0) ? S_WB : S_DONE;
                end

                // ---------------------------------------------- stream results out
                // wb_re reads query wb_next; its data is in m12_q.. one clock later.
                S_WB: begin
                    if (wb_re) begin
                        wb_cur  <= wb_next;
                        wb_next <= wb_next + 1;
                    end
                    if (wb_have && out_free) begin
                        m_tvalid <= 1'b1;
                        m_tlast  <= (wb_cur + 1 == n_query);
                        m_tdata  <= {7'b0, qdist_q,
                                     (m12_valid && !wb_reject) ? {3'b0, m12_q[IDX_W-1:0]} : 16'hFFFF};
                        if (wb_reject)
                            n_matches_r <= n_matches_r - 1;
                    end else if (m_tvalid && m_tready) begin
                        m_tvalid <= 1'b0;
                    end
                    if (wb_re)
                        wb_have <= 1'b1;
                    else if (wb_have && out_free)
                        wb_have <= 1'b0;
                    if (m_tvalid && m_tready && m_tlast) begin
                        m_tvalid <= 1'b0;
                        m_tlast  <= 1'b0;
                        state    <= S_DONE;
                    end
                end

                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
