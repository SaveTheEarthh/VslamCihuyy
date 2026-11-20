/*
 * ORB matcher kernel for Vitis HLS (top function: orb_match_hls).
 * Same results as orb_match_ref.c, bit for bit; see orb_match.h for the
 * data formats and the algorithm.
 *
 * Micro-architecture
 *   1. load   : burst-read train descriptors + meta into BRAM, split into
 *               ORB_LANES banks (cyclic), so lane l owns indices l, l+P, ...
 *   2. match  : per query, a pipelined scan (II=1) over its range [lo, hi).
 *               Each clock, ORB_LANES lanes each do 256 XOR + popcount + gating
 *               and keep their own best / second-best (no cross-lane logic in
 *               the loop-carried path). Lanes are merged after the scan.
 *               Then the accept / steal decision updates the one-to-one tables.
 *               Queries are fetched ORB_QBLOCK at a time to keep AXI bursts long.
 *   3. orient : 30-bin rotation histogram -> three maxima -> filter + write back.
 *
 * Cycles ~= n_train*5 (load) + sum over valid queries of (ceil(range/P) + ~20)
 *          + ~4 per invalid query + 2*n_query (write back).
 */
#include "orb_match.h"

/* SWAR popcount, the same steps as ORBmatcher::DescriptorDistance, except the
 * final "* 0x01010101 >> 24" byte sum: a multiplier would cost DSPs, an
 * adder tree costs a few LUTs. */
static unsigned popcount64(uint64_t v)
{
#pragma HLS INLINE
    v = v - ((v >> 1) & 0x5555555555555555ULL);                          /* 32 x 2-bit counts */
    v = (v & 0x3333333333333333ULL) + ((v >> 2) & 0x3333333333333333ULL); /* 16 x 4-bit */
    v = (v + (v >> 4)) & 0x0F0F0F0F0F0F0F0FULL;                          /*  8 x 8-bit */
    unsigned b0 = (unsigned)(v & 0xF),         b1 = (unsigned)((v >> 8) & 0xF);
    unsigned b2 = (unsigned)((v >> 16) & 0xF), b3 = (unsigned)((v >> 24) & 0xF);
    unsigned b4 = (unsigned)((v >> 32) & 0xF), b5 = (unsigned)((v >> 40) & 0xF);
    unsigned b6 = (unsigned)((v >> 48) & 0xF), b7 = (unsigned)((v >> 56) & 0xF);
    return ((b0 + b1) + (b2 + b3)) + ((b4 + b5) + (b6 + b7));
}

/* Each byte count is <= 8, so 4 bits suffice ("& 0xF"); that lets HLS size
 * the adder tree 4 -> 5 -> 6 -> 7 bits. */

int orb_match_hls(const uint64_t *t_desc, const uint64_t *t_meta, int n_train,
                  const uint64_t *q_desc, const uint64_t *q_meta, int n_query,
                  int th_dist, const uint16_t *ratio_lim, int unique, int check_orient,
                  int32_t *match_out, int32_t *dist_out)
{
#pragma HLS INTERFACE mode=m_axi port=t_desc    offset=slave bundle=gmem depth=32768 max_read_burst_length=256
#pragma HLS INTERFACE mode=m_axi port=t_meta    offset=slave bundle=gmem depth=8192  max_read_burst_length=256
#pragma HLS INTERFACE mode=m_axi port=q_desc    offset=slave bundle=gmem depth=32768 max_read_burst_length=256
#pragma HLS INTERFACE mode=m_axi port=q_meta    offset=slave bundle=gmem depth=16384 max_read_burst_length=256
#pragma HLS INTERFACE mode=m_axi port=ratio_lim offset=slave bundle=gmem depth=257   max_read_burst_length=256
#pragma HLS INTERFACE mode=m_axi port=match_out offset=slave bundle=gmem depth=8192  max_write_burst_length=256
#pragma HLS INTERFACE mode=m_axi port=dist_out  offset=slave bundle=gmem depth=8192  max_write_burst_length=256
#pragma HLS INTERFACE mode=s_axilite port=n_train
#pragma HLS INTERFACE mode=s_axilite port=n_query
#pragma HLS INTERFACE mode=s_axilite port=th_dist
#pragma HLS INTERFACE mode=s_axilite port=unique
#pragma HLS INTERFACE mode=s_axilite port=check_orient
#pragma HLS INTERFACE mode=s_axilite port=return

    /* train side, banked so ORB_LANES entries can be read per clock */
    uint64_t td[ORB_DESC_WORDS][ORB_MAX_TRAIN];
    uint64_t tm[ORB_MAX_TRAIN];
    uint16_t mdist[ORB_MAX_TRAIN];   /* vMatchedDistance */
    int16_t  m21[ORB_MAX_TRAIN];     /* vnMatches21 (only one access per query: not banked) */
#pragma HLS ARRAY_PARTITION variable=td    dim=1 type=complete
#pragma HLS ARRAY_PARTITION variable=td    dim=2 type=cyclic factor=ORB_LANES
#pragma HLS ARRAY_PARTITION variable=tm    dim=1 type=cyclic factor=ORB_LANES
#pragma HLS ARRAY_PARTITION variable=mdist dim=1 type=cyclic factor=ORB_LANES
#if ORB_TRAIN_URAM
    /* 8 lanes x 256 bit = 2048 bit/clock -> 32 banks whatever the depth: 128
     * BRAM18 (44% of K26) or 32 URAM (each up to 4096 deep, so this also
     * allows ORB_MAX_TRAIN up to 32k for free). The ORB extractor wants BRAM. */
#pragma HLS BIND_STORAGE variable=td type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=tm type=ram_2p impl=uram
#endif

    /* query side */
    uint64_t qd[ORB_DESC_WORDS][ORB_QBLOCK];
    uint64_t qm[2][ORB_QBLOCK];
#pragma HLS ARRAY_PARTITION variable=qd dim=1 type=complete
#pragma HLS ARRAY_PARTITION variable=qm dim=1 type=complete
    int16_t  m12[ORB_MAX_QUERY];     /* vnMatches12 */
    uint16_t qdist[ORB_MAX_QUERY];
    uint8_t  qbin[ORB_MAX_QUERY];
    uint16_t histo[ORB_HISTO_LENGTH];
    uint16_t lim[ORB_RATIO_LIM_LEN];
#pragma HLS ARRAY_PARTITION variable=histo type=complete

    int nmatches = 0;

    /* ---------------------------------------------------------- 1. load */
load_meta:
    for (int j = 0; j < n_train; j++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=ORB_MAX_TRAIN
        tm[j] = t_meta[j];
        mdist[j] = ORB_DIST_NONE;
        m21[j] = ORB_NO_MATCH;
    }
load_desc:
    for (int k = 0; k < n_train * ORB_DESC_WORDS; k++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=ORB_MAX_TRAIN*ORB_DESC_WORDS
        td[k % ORB_DESC_WORDS][k / ORB_DESC_WORDS] = t_desc[k];
    }
load_ratio:
    for (int s = 0; s < ORB_RATIO_LIM_LEN; s++) {
#pragma HLS PIPELINE II=1
        lim[s] = ratio_lim[s];
    }
init_hist:
    for (int b = 0; b < ORB_HISTO_LENGTH; b++) {
#pragma HLS UNROLL
        histo[b] = 0;
    }

    /* --------------------------------------------------------- 2. match */
query_blocks:
    for (int q0 = 0; q0 < n_query; q0 += ORB_QBLOCK) {
#pragma HLS LOOP_TRIPCOUNT max=ORB_MAX_QUERY/ORB_QBLOCK
        const int nb = (n_query - q0 < ORB_QBLOCK) ? n_query - q0 : ORB_QBLOCK;

    load_qdesc:
        for (int k = 0; k < nb * ORB_DESC_WORDS; k++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=ORB_QBLOCK*ORB_DESC_WORDS
            qd[k % ORB_DESC_WORDS][k / ORB_DESC_WORDS] = q_desc[q0 * ORB_DESC_WORDS + k];
        }
    load_qmeta:
        for (int k = 0; k < nb * 2; k++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=ORB_QBLOCK*2
            qm[k % 2][k / 2] = q_meta[q0 * 2 + k];
        }

    queries:
        for (int b = 0; b < nb; b++) {
#pragma HLS LOOP_TRIPCOUNT max=ORB_QBLOCK
            const int i = q0 + b;
            const uint64_t w0 = qm[0][b], w1 = qm[1][b];
            const int32_t qx = orb_sext20(ORB_FIELD(w0, 0, 20));
            const int32_t qy = orb_sext20(ORB_FIELD(w0, 20, 20));
            const int32_t r = (int32_t)ORB_FIELD(w0, 40, 20);
            const uint32_t qang = ORB_FIELD(w1, 0, 16);
            const int lo = (int)ORB_FIELD(w1, 16, 16);
            const int hi_raw = (int)ORB_FIELD(w1, 32, 16);
            const int hi = (hi_raw < n_train) ? hi_raw : n_train;
            const uint32_t min_lvl = ORB_FIELD(w1, 48, 3), max_lvl = ORB_FIELD(w1, 51, 3);
            const int use_win = (int)ORB_FIELD(w1, 62, 1), qvalid = (int)ORB_FIELD(w1, 63, 1);
            const uint64_t qd0 = qd[0][b], qd1 = qd[1][b], qd2 = qd[2][b], qd3 = qd[3][b];

            /* fast path: e.g. mono init only matches level-0 queries (~22%) */
            qdist[i] = ORB_DIST_NONE;
            m12[i] = ORB_NO_MATCH;
            if (!qvalid || lo >= hi)
                continue;

            unsigned lbest[ORB_LANES], lsecond[ORB_LANES];
            int lidx[ORB_LANES];
#pragma HLS ARRAY_PARTITION variable=lbest   type=complete
#pragma HLS ARRAY_PARTITION variable=lsecond type=complete
#pragma HLS ARRAY_PARTITION variable=lidx    type=complete
            for (int l = 0; l < ORB_LANES; l++) {
#pragma HLS UNROLL
                lbest[l] = ORB_DIST_NONE;
                lsecond[l] = ORB_DIST_NONE;
                lidx[l] = -1;
            }

            /* scan rows of ORB_LANES; partial first/last rows are masked */
            const int k_lo = lo / ORB_LANES;
            const int k_hi = (hi + ORB_LANES - 1) / ORB_LANES;
        scan:
            for (int k = k_lo; k < k_hi; k++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=ORB_MAX_TRAIN/ORB_LANES
                for (int l = 0; l < ORB_LANES; l++) {
#pragma HLS UNROLL
                    const int j = k * ORB_LANES + l;
                    const uint64_t m = tm[j];
                    const int32_t dx = orb_sext20(ORB_FIELD(m, 0, 20)) - qx;
                    const int32_t dy = orb_sext20(ORB_FIELD(m, 20, 20)) - qy;
                    const uint32_t lvl = ORB_FIELD(m, 56, 3);
                    const unsigned d = popcount64(qd0 ^ td[0][j]) + popcount64(qd1 ^ td[1][j])
                                     + popcount64(qd2 ^ td[2][j]) + popcount64(qd3 ^ td[3][j]);

                    const int ok = (j >= lo) && (j < hi) && ORB_FIELD(m, 63, 1)
                                && (lvl >= min_lvl) && (lvl <= max_lvl)
                                && (!use_win || (dx < r && dx > -r && dy < r && dy > -r))
                                && (!unique || d < mdist[j]);
                    if (ok) {
                        if (d < lbest[l]) {        /* strict <: lowest index wins ties */
                            lsecond[l] = lbest[l];
                            lbest[l] = d;
                            lidx[l] = j;
                        } else if (d < lsecond[l]) {
                            lsecond[l] = d;
                        }
                    }
                }
            }

            /* merge lanes: best = min (ties -> lower index), second = 2nd smallest overall */
            unsigned best = lbest[0], best2 = lsecond[0];
            int best_idx = lidx[0];
            for (int l = 1; l < ORB_LANES; l++) {
#pragma HLS UNROLL
                const int win = lbest[l] < best || (lbest[l] == best && lidx[l] < best_idx);
                if (win) {
                    best2 = (best < lsecond[l]) ? best : lsecond[l];
                    best = lbest[l];
                    best_idx = lidx[l];
                } else {
                    best2 = (lbest[l] < best2) ? lbest[l] : best2;
                }
            }

            qdist[i] = (uint16_t)best;

            /* accept? best2 is 0..256 or NONE, so lim[] needs no bounds logic */
            const int pass_th = best <= (unsigned)th_dist;
            const int pass_ratio = best2 == ORB_DIST_NONE || best < lim[best2 & 0x1FF];
            if (pass_th && pass_ratio) {
                if (unique) {
                    const int prev = m21[best_idx];
                    if (prev >= 0) {                   /* steal from the older match */
                        m12[prev] = ORB_NO_MATCH;
                        nmatches--;
                    }
                    m21[best_idx] = (int16_t)i;
                    mdist[best_idx] = (uint16_t)best;
                }
                m12[i] = (int16_t)best_idx;
                nmatches++;
                if (check_orient) {
                    const uint32_t bin = orb_rot_bin(qang, ORB_FIELD(tm[best_idx], 40, 16));
                    qbin[i] = (uint8_t)bin;
                    histo[bin]++;
                }
            }
        }
    }

    /* -------------------------------------------------------- 3. orient */
    int ind1 = -1, ind2 = -1, ind3 = -1;
    if (check_orient) {
        int max1 = 0, max2 = 0, max3 = 0;
    three_maxima:
        for (int h = 0; h < ORB_HISTO_LENGTH; h++) {
#pragma HLS PIPELINE II=1
            const int s = histo[h];
            if (s > max1) {
                max3 = max2; max2 = max1; max1 = s;
                ind3 = ind2; ind2 = ind1; ind1 = h;
            } else if (s > max2) {
                max3 = max2; max2 = s;
                ind3 = ind2; ind2 = h;
            } else if (s > max3) {
                max3 = s;
                ind3 = h;
            }
        }
        if (10 * max2 < max1) {
            ind2 = -1;
            ind3 = -1;
        } else if (10 * max3 < max1) {
            ind3 = -1;
        }
    }

write_out:
    for (int i = 0; i < n_query; i++) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=ORB_MAX_QUERY
        int m = m12[i];
        if (check_orient && m >= 0) {
            const int bin = qbin[i];
            if (bin != ind1 && bin != ind2 && bin != ind3) {
                m = ORB_NO_MATCH;
                nmatches--;
            }
        }
        match_out[i] = m;
        dist_out[i] = qdist[i];
    }
    return nmatches;
}
