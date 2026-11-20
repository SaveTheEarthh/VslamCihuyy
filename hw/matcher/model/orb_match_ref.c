/*
 * orb_match_ref.c - the feature matcher, written as plainly as possible.
 *
 * This is the "golden model": the readable version of the matching algorithm.
 * orb_match_hls.c does exactly the same thing but is rearranged for hardware.
 * The testbench checks that both give identical results.
 * The structure follows ORB-SLAM3's ORBmatcher::SearchForInitialization
 * (~/vslam_ws/ORB_SLAM3/src/ORBmatcher.cc, line 648), so you can read them
 * side by side.
 *
 * ---------------------------------------------------------------------------
 * THE PROBLEM
 *   Frame 1 and frame 2 are two camera images taken a moment apart.
 *   Each has thousands of keypoints (corners). Each keypoint has:
 *     - a position x, y      (where it is in the image, in pixels)
 *     - a pyramid level 0-7  (at which image size it was detected)
 *     - an angle 0-360 deg   (which way the patch around it "points")
 *     - a descriptor         (256 bits = a fingerprint of the patch)
 *   Goal: for every keypoint in frame 1, find the keypoint in frame 2 that is
 *   the same physical point, or say "no match".
 *
 *   QUERY = a keypoint of frame 1 (the one we are looking for a partner for)
 *   TRAIN = a keypoint of frame 2 (the pool we search in)
 *
 * THE ALGORITHM (each step is marked in the code below)
 *   For each query i:
 *     STEP 1  pick candidates: only train points near the query's position
 *             and on the same pyramid level
 *     STEP 2  score each candidate: Hamming distance (number of differing bits)
 *     STEP 3  remember the best and second-best scores
 *     STEP 4  accept the best only if it is good (<= th_dist) and clearly
 *             better than the second best (ratio test)
 *     STEP 5  one-to-one: a train point can belong to only one query; a
 *             better query "steals" it from a worse one
 *   After all queries:
 *     STEP 6  rotation check: correct matches all turn by about the same
 *             angle; throw away matches whose angle change is unusual
 * ---------------------------------------------------------------------------
 */
#include <string.h>
#include "orb_match.h"

/*
 * STEP 2 helper: Hamming distance between two 256-bit descriptors.
 *
 *   a, b : each points to one descriptor = 4 x 64-bit words = 256 bits
 *   returns how many of the 256 bits differ (0 = identical, 256 = opposite)
 *
 * Example with 8 bits:  a = 1011 0010
 *                       b = 1001 1010
 *                   a ^ b = 0010 1000   (XOR: 1 wherever they differ)
 *                   count the 1s -> distance 2
 */
static int hamming_naive(const uint64_t *a, const uint64_t *b)
{
    int dist = 0;
    for (int w = 0; w < ORB_DESC_WORDS; w++) {      /* the 4 words of the descriptor */
        uint64_t x = a[w] ^ b[w];                   /* 1 where the bits differ */
        for (int bit = 0; bit < 64; bit++)          /* count the 1s, one bit at a time */
            dist += (int)((x >> bit) & 1);
    }
    return dist;
}

/*
 * STEP 6 helper: find the three tallest bars of the rotation histogram.
 * (Copy of ORB-SLAM3's ORBmatcher::ComputeThreeMaxima.)
 *
 *   histo[b]  : how many matches fell into rotation bin b (b = 0..29)
 *   max1/2/3  : the counts of the 1st, 2nd and 3rd tallest bins
 *   *ind1/2/3 : OUTPUT - the bin numbers of those bins; -1 means "none"
 *               (pointers because C can only return one value directly)
 *
 * Example: histo[12] = 359, histo[0] = 337, histo[11] = 1, all others 0
 *   -> ind1 = 12, ind2 = 0, ind3 = 11
 *   -> then the 10% rule removes bin 11 (1 is less than 10% of 359): ind3 = -1
 */
void orb_three_maxima_ref(const int *histo, int *ind1, int *ind2, int *ind3)
{
    int max1 = 0, max2 = 0, max3 = 0;
    *ind1 = *ind2 = *ind3 = -1;

    /* Walk over all bins, keeping a "podium" of the 3 tallest seen so far. */
    for (int i = 0; i < ORB_HISTO_LENGTH; i++) {
        const int s = histo[i];               /* height of bin i */
        if (s > max1) {                       /* new 1st place: old 1st->2nd, old 2nd->3rd */
            max3 = max2; max2 = max1; max1 = s;
            *ind3 = *ind2; *ind2 = *ind1; *ind1 = i;
        } else if (s > max2) {                /* new 2nd place: old 2nd->3rd */
            max3 = max2; max2 = s;
            *ind3 = *ind2; *ind2 = i;
        } else if (s > max3) {                /* new 3rd place */
            max3 = s;
            *ind3 = i;
        }
    }

    /* 10% rule: a runner-up bin only counts if it has at least 10% of the
     * tallest bin's matches; otherwise its matches are probably wrong ones.
     * ORB-SLAM3 writes "max2 < 0.1f*max1"; "10*max2 < max1" is the same test
     * in integers (checked for every possible value in the testbench). */
    if (10 * max2 < max1) {          /* 2nd bin too small -> drop 2nd and 3rd */
        *ind2 = -1;
        *ind3 = -1;
    } else if (10 * max3 < max1) {   /* only the 3rd bin too small -> drop it */
        *ind3 = -1;
    }
}

/*
 * The matcher.
 *
 * INPUTS - frame 2 (train), prepared by orb_host.c:
 *   t_desc   : descriptors of all train points; point j's descriptor is the
 *              4 words t_desc[4*j .. 4*j+3]
 *   t_meta   : one 64-bit word per train point with x, y, angle, level and a
 *              valid bit packed inside (bit layout in orb_match.h; read with
 *              ORB_FIELD(word, first_bit, number_of_bits) = word[first+n-1:first])
 *   n_train  : number of train points
 *
 * INPUTS - frame 1 (query):
 *   q_desc   : descriptors of all query points (4 words each, like t_desc)
 *   q_meta   : two 64-bit words per query point, q_meta[2*i] and q_meta[2*i+1]:
 *              position, search-window size, angle, which train indices to
 *              scan, allowed levels, valid bit (layout in orb_match.h)
 *   n_query  : number of query points
 *
 * INPUTS - settings (ORB-SLAM3 initialisation uses 50, table for 0.9, 1, 1):
 *   th_dist      : accept only matches with distance <= th_dist
 *   ratio_lim    : ratio-test table, 257 entries. ratio_lim[s] is the
 *                  smallest distance that does NOT pass "best < 0.9 * s".
 *                  Example: ratio_lim[40] = 36, so with a second best of 40
 *                  the best must be 0..35. (A table instead of "* 0.9" so the
 *                  FPGA needs no floating point; see orb_ratio_table.)
 *   unique       : 1 = each train point can be matched only once (STEP 5)
 *   check_orient : 1 = do the rotation check (STEP 6)
 *
 * OUTPUTS:
 *   match_out[i] : the train index matched to query i, or -1 for no match
 *   dist_out[i]  : the best distance found for query i (511 = no candidates)
 *   return value : number of matches
 */
int orb_match_ref(const uint64_t *t_desc, const uint64_t *t_meta, int n_train,
                  const uint64_t *q_desc, const uint64_t *q_meta, int n_query,
                  int th_dist, const uint16_t *ratio_lim, int unique, int check_orient,
                  int32_t *match_out, int32_t *dist_out)
{
    /* Bookkeeping arrays ("static" only because they are too big for the stack). */

    /* For each TRAIN point j: the distance of the match that currently owns it
     * (511 = nobody owns it yet). ORB-SLAM3 calls this vMatchedDistance. */
    static int matched_dist[ORB_MAX_TRAIN];

    /* For each TRAIN point j: which QUERY currently owns it (-1 = nobody).
     * The reverse direction of match_out. ORB-SLAM3: vnMatches21
     * ("matches from frame 2 to frame 1"). */
    static int matches21[ORB_MAX_TRAIN];

    /* For each QUERY i that got a match: the rotation bin of that match,
     * so STEP 6 can later look up which bin each match fell into. */
    static int qbin[ORB_MAX_QUERY];

    /* Rotation histogram: histo[b] = number of accepted matches whose angle
     * change falls into bin b (bins are 30 degrees wide). ORB-SLAM3: rotHist. */
    int histo[ORB_HISTO_LENGTH] = {0};

    int nmatches = 0;                        /* running count of matches */

    /* At the start, no train point is owned by anyone. */
    for (int j = 0; j < n_train; j++) {
        matched_dist[j] = ORB_DIST_NONE;
        matches21[j] = ORB_NO_MATCH;
    }

    /* ================================================================
     * Main loop: one query (frame-1 keypoint) at a time
     * ================================================================ */
    for (int i = 0; i < n_query; i++) {

        /* --- Unpack query i's information from its two packed words. --- */
        const uint64_t w0 = q_meta[2 * i], w1 = q_meta[2 * i + 1];
        const int32_t qx = orb_sext20(ORB_FIELD(w0, 0, 20));   /* where the query was (x), in 1/256 px */
        const int32_t qy = orb_sext20(ORB_FIELD(w0, 20, 20));  /* where the query was (y), in 1/256 px */
        const int32_t r = (int32_t)ORB_FIELD(w0, 40, 20);      /* search window half-size (100 px = 25600) */
        const uint32_t qang = ORB_FIELD(w1, 0, 16);            /* query's angle (65536 = 360 deg) */
        const int lo = (int)ORB_FIELD(w1, 16, 16);             /* scan train indices lo .. hi-1 only */
        const int hi = (int)ORB_FIELD(w1, 32, 16);             /*   (ORB-SLAM3's grid lookup, done on the ARM) */
        const int min_lvl = (int)ORB_FIELD(w1, 48, 3);         /* allowed pyramid levels of a candidate */
        const int max_lvl = (int)ORB_FIELD(w1, 51, 3);
        const int use_win = (int)ORB_FIELD(w1, 62, 1);         /* 1 = apply the position window */
        const int qvalid = (int)ORB_FIELD(w1, 63, 1);          /* 0 = skip this query entirely */

        /* Default answer: no match. */
        match_out[i] = ORB_NO_MATCH;
        dist_out[i] = ORB_DIST_NONE;

        /* ORB-SLAM3 initialisation only matches level-0 keypoints; the others
         * are marked invalid ("if(level1>0) continue;" in ORB-SLAM3). */
        if (!qvalid)
            continue;

        /* STEP 3 state: the two smallest distances seen so far.
         * They start at 511 ("nothing yet"), which is bigger than any real
         * distance (max 256), so the first real candidate always wins. */
        int best = ORB_DIST_NONE;     /* smallest distance so far          */
        int best2 = ORB_DIST_NONE;    /* second smallest distance so far   */
        int best_idx = -1;            /* train index that has `best`       */

        /* ------------------------------------------------------------
         * Loop over this query's candidates: train points lo .. hi-1
         * ------------------------------------------------------------ */
        for (int j = lo; j < hi && j < n_train; j++) {

            /* Unpack train point j. */
            const uint64_t m = t_meta[j];
            const int32_t dx = orb_sext20(ORB_FIELD(m, 0, 20)) - qx;   /* how far right of the query */
            const int32_t dy = orb_sext20(ORB_FIELD(m, 20, 20)) - qy;  /* how far below the query */
            const int lvl = (int)ORB_FIELD(m, 56, 3);                  /* its pyramid level */

            /* STEP 1: is j a candidate at all? Skip it if ... */
            if (!ORB_FIELD(m, 63, 1))                    /* ... it is marked invalid */
                continue;
            if (lvl < min_lvl || lvl > max_lvl)          /* ... it is on the wrong pyramid level */
                continue;
            if (use_win && !(dx < r && dx > -r && dy < r && dy > -r))  /* ... it is outside the window */
                continue;                                /*     (|dx| < r and |dy| < r) */

            /* STEP 2: score it. */
            const int dist = hamming_naive(q_desc + ORB_DESC_WORDS * i,   /* query i's descriptor */
                                           t_desc + ORB_DESC_WORDS * j);  /* train j's descriptor */

            /* STEP 5 (part 1): if j already belongs to another query with an
             * equal or better distance, this query cannot take it - skip.
             * Example: query 5 owns j with distance 30. Now this query
             * gets distance 35 -> skip. Distance 20 -> keep going. */
            if (unique && matched_dist[j] <= dist)
                continue;

            /* STEP 3: update the best / second best. */
            if (dist < best) {            /* new best: the old best becomes second */
                best2 = best;
                best = dist;
                best_idx = j;
            } else if (dist < best2) {    /* not the best, but better than second */
                best2 = dist;
            }
        }
        dist_out[i] = best;

        /* STEP 4: accept or reject the best candidate.
         *   (a) good enough?  best <= 50
         *   (b) clear winner? best < 0.9 * second best (Lowe's ratio test).
         *       If best = 30 and second = 32, two candidates look almost the
         *       same, so we cannot tell which is right: reject.
         *       If only one candidate existed (best2 is still 511), skip (b). */
        if (best > th_dist)
            continue;
        if (best2 != ORB_DIST_NONE && !(best < ratio_lim[best2]))
            continue;

        /* STEP 5 (part 2): take ownership of train point best_idx. */
        if (unique) {
            if (matches21[best_idx] >= 0) {
                /* Someone else owned it, with a worse distance (otherwise we
                 * would have skipped it above). They lose their match. */
                match_out[matches21[best_idx]] = ORB_NO_MATCH;
                nmatches--;
            }
            matches21[best_idx] = i;          /* now query i owns it */
            matched_dist[best_idx] = best;    /* with this distance  */
        }
        match_out[i] = best_idx;              /* record the match */
        nmatches++;

        /* STEP 6 (part 1): put this match into the rotation histogram.
         *   rotation = query angle - train angle  (how much the patch turned)
         *   bin      = round(rotation / 30 deg)   -> 0..12
         * The bin is remembered per query so part 2 can find it again. */
        if (check_orient) {
            const int bin = (int)orb_rot_bin(qang, ORB_FIELD(t_meta[best_idx], 40, 16));
            qbin[i] = bin;
            histo[bin]++;
        }
    }

    /* ================================================================
     * STEP 6 (part 2): after all queries, keep only matches in the
     * (up to) three most popular rotation bins.
     * ================================================================ */
    if (check_orient) {
        int ind1, ind2, ind3;                            /* bins to keep (-1 = none) */
        orb_three_maxima_ref(histo, &ind1, &ind2, &ind3);
        for (int i = 0; i < n_query; i++) {
            if (match_out[i] < 0)                        /* query i has no match: nothing to check */
                continue;
            if (qbin[i] == ind1 || qbin[i] == ind2 || qbin[i] == ind3)
                continue;                                /* its bin is popular: keep it */
            match_out[i] = ORB_NO_MATCH;                 /* unusual rotation: probably wrong */
            nmatches--;
        }
    }
    return nmatches;
}
