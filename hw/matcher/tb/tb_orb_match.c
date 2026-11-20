/*
 * Testbench for orb_match_hls (works for gcc, Vitis HLS C-sim and C/RTL co-sim).
 *
 *   tb [--quick] [pair.bin ...]
 *
 * 1. Exhaustive checks that each float expression in ORB-SLAM3 equals the
 *    integer form used in hardware (ratio test, three-maxima, rotation bin).
 * 2. Random tests: kernel vs golden model, must be bit-exact.
 * 3. Real EuRoC pairs from sw/orbslam3/dump_orbslam3: kernel vs golden (bit-exact)
 *    and vs ORB-SLAM3's own SearchForInitialization (agreement reported).
 * --quick runs a short version for co-simulation. Exit code 0 = pass.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "orb_match.h"
#include "orb_host.h"

void orb_three_maxima_ref(const int *histo, int *ind1, int *ind2, int *ind3);

static uint64_t t_desc[ORB_MAX_TRAIN * ORB_DESC_WORDS], t_meta[ORB_MAX_TRAIN];
static uint64_t q_desc[ORB_MAX_QUERY * ORB_DESC_WORDS], q_meta[ORB_MAX_QUERY * 2];
static int32_t m_hw[ORB_MAX_QUERY], d_hw[ORB_MAX_QUERY], m_ref[ORB_MAX_QUERY], d_ref[ORB_MAX_QUERY];

static uint64_t rng = 0x9E3779B97F4A7C15ULL;
static uint32_t rnd(void)                       /* xorshift64*, reproducible */
{
    rng ^= rng >> 12;
    rng ^= rng << 25;
    rng ^= rng >> 27;
    return (uint32_t)((rng * 0x2545F4914F6CDD1DULL) >> 32);
}
static int rnd_range(int lo, int hi) { return lo + (int)(rnd() % (uint32_t)(hi - lo + 1)); }

/* ------------------------------------------------- 1. arithmetic checks */
static int check_arithmetic(void)
{
    int fails = 0;

    /* ratio: ORB-SLAM3 "bestDist < (float)bestDist2 * mfNNratio", every ratio
     * ORB-SLAM3 constructs an ORBmatcher with. Also count how often a naive
     * integer num/den would have been wrong (0.6f is the trap). */
    static const struct { float f; int num, den; } ratios[] = {
        {0.6f, 6, 10}, {0.7f, 7, 10}, {0.75f, 3, 4}, {0.8f, 8, 10}, {0.9f, 9, 10}};
    for (unsigned k = 0; k < sizeof ratios / sizeof ratios[0]; k++) {
        uint16_t lim[ORB_RATIO_LIM_LEN];
        int naive_wrong = 0;
        orb_ratio_table(ratios[k].f, lim);
        for (int b = 0; b <= 256; b++)
            for (int s = b; s <= 256; s++) {
                const int orbslam = b < (float)s * ratios[k].f;
                fails += orbslam != (b < lim[s]);
                naive_wrong += orbslam != (b * ratios[k].den < ratios[k].num * s);
            }
        if (naive_wrong)
            printf("[arith] note: ratio %.2f as %d/%d would differ from ORB-SLAM3 in %d cases -> table used\n",
                   ratios[k].f, ratios[k].num, ratios[k].den, naive_wrong);
    }

    /* ComputeThreeMaxima: "max2 < 0.1f*(float)max1" */
    for (int m1 = 0; m1 <= ORB_MAX_QUERY; m1++)
        for (int m2 = 0; m2 <= m1; m2++)
            if ((m2 < 0.1f * (float)m1) != (10 * m2 < m1))
                fails++;

    /* rotation bin on every possible quantised rotation */
    for (uint32_t rot16 = 0; rot16 < 65536; rot16++) {
        float rot = (float)rot16 * (360.0f / 65536.0f);
        int bin = (int)roundf(rot * (1.0f / ORB_HISTO_LENGTH));
        if (bin == ORB_HISTO_LENGTH)
            bin = 0;
        if ((uint32_t)bin != orb_rot_bin(rot16, 0))
            fails++;
    }
    printf("[arith] ratio / three-maxima / rotation-bin integer forms: %s\n",
           fails ? "MISMATCH" : "identical to ORB-SLAM3 float code");
    return fails;
}

/* --------------------------------------------------- 2. random tests */
static void random_desc(uint64_t *d)
{
    for (int w = 0; w < ORB_DESC_WORDS; w++)
        d[w] = ((uint64_t)rnd() << 32) | rnd();
}

static void noisy_copy(const uint64_t *src, uint64_t *dst, int nflip)
{
    memcpy(dst, src, ORB_DESC_WORDS * sizeof(uint64_t));
    for (int f = 0; f < nflip; f++) {
        int bit = rnd_range(0, 255);
        dst[bit / 64] ^= 1ULL << (bit % 64);
    }
}

static int run_compare(int nt, int nq, int th, float ratio, int uq, int ori, const char *tag, int verbose)
{
    uint16_t lim[ORB_RATIO_LIM_LEN];
    orb_ratio_table(ratio, lim);
    int nm_hw = orb_match_hls(t_desc, t_meta, nt, q_desc, q_meta, nq, th, lim, uq, ori, m_hw, d_hw);
    int nm_ref = orb_match_ref(t_desc, t_meta, nt, q_desc, q_meta, nq, th, lim, uq, ori, m_ref, d_ref);
    int bad = (nm_hw != nm_ref);
    for (int i = 0; i < nq; i++)
        if (m_hw[i] != m_ref[i] || d_hw[i] != d_ref[i]) {
            if (!bad && verbose)
                printf("  first diff at query %d: hw (%d,%d) ref (%d,%d)\n", i, m_hw[i], d_hw[i], m_ref[i], d_ref[i]);
            bad = 1;
        }
    if (bad || verbose)
        printf("  %-28s nt=%4d nq=%4d th=%3d ratio=%.2f uniq=%d ori=%d  matches hw=%d ref=%d  %s\n",
               tag, nt, nq, th, ratio, uq, ori, nm_hw, nm_ref, bad ? "FAIL" : "ok");
    return bad;
}

/* Scene: train features spread over a 752x480 image; a fraction of queries are
 * noisy, slightly displaced copies of train features, the rest random. */
static int random_case(int nt, int nq, int brute, const char *tag, int verbose)
{
    for (int j = 0; j < nt; j++) {
        random_desc(t_desc + ORB_DESC_WORDS * j);
        t_meta[j] = orb_pack_train(rnd_range(0, 752 << 8), rnd_range(0, 480 << 8),
                                   rnd() & 0xFFFF, rnd_range(0, 7), rnd_range(0, 9) != 0);
    }
    for (int i = 0; i < nq; i++) {
        int x, y, lvl = rnd_range(0, 7);
        uint32_t ang = rnd() & 0xFFFF;
        if (nt > 0 && rnd_range(0, 3) != 0) {      /* planted match */
            const int j = rnd_range(0, nt - 1);
            noisy_copy(t_desc + ORB_DESC_WORDS * j, q_desc + ORB_DESC_WORDS * i, rnd_range(0, 70));
            x = orb_sext20(ORB_FIELD(t_meta[j], 0, 20)) + rnd_range(-3000, 3000);
            y = orb_sext20(ORB_FIELD(t_meta[j], 20, 20)) + rnd_range(-3000, 3000);
            lvl = (int)ORB_FIELD(t_meta[j], 56, 3);
            /* most rotations agree (+-10 deg), some do not */
            ang = (ORB_FIELD(t_meta[j], 40, 16) + (rnd_range(0, 4) ? rnd_range(-1820, 1820) : (int)rnd())) & 0xFFFF;
        } else {
            random_desc(q_desc + ORB_DESC_WORDS * i);
            x = rnd_range(0, 752 << 8);
            y = rnd_range(0, 480 << 8);
        }
        /* scan range: whole list, or a random sub-range (unaligned to lanes) */
        int lo = 0, hi = nt;
        if (!brute && nt > 0) {
            lo = rnd_range(0, nt - 1);
            hi = rnd_range(lo, nt + 3);           /* hi > n_train must be clamped */
        }
        const int min_l = brute ? 0 : (lvl > 0 ? lvl - rnd_range(0, 1) : 0);
        const int max_l = brute ? 7 : (lvl < 7 ? lvl + rnd_range(0, 1) : 7);
        q_meta[2 * i] = orb_pack_query0(x, y, (uint32_t)rnd_range(1, 120) << 8);
        q_meta[2 * i + 1] = orb_pack_query1(ang, lo, hi, min_l, max_l, !brute, rnd_range(0, 19) != 0);
    }
    const int th = rnd_range(0, 1) ? ORB_TH_LOW : ORB_TH_HIGH;
    static const float ratios[] = {0.0f /* off */, 0.6f, 0.7f, 0.75f, 0.8f, 0.9f};
    const float ratio = ratios[rnd_range(0, 5)];
    return run_compare(nt, nq, th, ratio, rnd_range(0, 3) != 0, rnd_range(0, 3) != 0, tag, verbose);
}

static int random_tests(int quick)
{
    int fails = 0;
    /* edge sizes: empty, < one lane row, exactly a row, row + 1, query-block edges */
    static const int edge[][2] = {{0, 5}, {5, 0}, {1, 1}, {ORB_LANES - 1, 7}, {ORB_LANES, 9},
                                  {ORB_LANES + 1, ORB_QBLOCK}, {37, ORB_QBLOCK + 1}, {100, 2 * ORB_QBLOCK - 1}};
    for (unsigned e = 0; e < sizeof edge / sizeof edge[0]; e++)
        fails += random_case(edge[e][0], edge[e][1], (int)(e & 1), "edge size", 0);

    const int n = quick ? 4 : 300;
    for (int t = 0; t < n; t++) {
        int nt = quick ? rnd_range(1, 200) : rnd_range(1, 1500);
        int nq = quick ? rnd_range(1, 300) : rnd_range(1, 1200);
        fails += random_case(nt, nq, t % 4 == 0, "random", 0);
    }
    if (!quick) {   /* full-size buffers, both modes */
        fails += random_case(ORB_MAX_TRAIN, 600, 0, "max train, windowed", 0);
        fails += random_case(2000, ORB_MAX_QUERY, 1, "max query, brute force", 0);
    }
    printf("[random] %d cases kernel vs golden model: %s\n",
           (int)(sizeof edge / sizeof edge[0]) + n + (quick ? 0 : 2), fails ? "FAILED" : "all bit-exact");
    return fails;
}

/* ------------------------------------------------- 3. ORB-SLAM3 pairs */
static int pair_test(const char *path)
{
    orb_pair p;
    orb_job job;
    int fails = 0;

    if (orb_pair_load(path, &p) || orb_prepare_init(&p, &job)) {
        printf("[pair] %s: could not load\n", path);
        return 1;
    }
    memcpy(t_desc, job.t_desc, sizeof(uint64_t) * ORB_DESC_WORDS * job.n_train);
    memcpy(t_meta, job.t_meta, sizeof(uint64_t) * job.n_train);
    memcpy(q_desc, job.q_desc, sizeof(uint64_t) * ORB_DESC_WORDS * job.n_query);
    memcpy(q_meta, job.q_meta, sizeof(uint64_t) * 2 * job.n_query);

    /* SearchForInitialization: ORBmatcher(0.9, true), TH_LOW */
    fails += run_compare(job.n_train, job.n_query, ORB_TH_LOW, 0.9f, 1, 1, "pair (init rules)", 0);

    int nm_hw = 0, same = 0, only_hw = 0, only_slam = 0, differ = 0, scanned = 0, valid_q = 0;
    for (int i = 0; i < job.n_query; i++) {
        const int a = m_hw[i] >= 0 ? job.perm[m_hw[i]] : -1, b = p.m12[i];
        nm_hw += a >= 0;
        if (a == b)
            same += a >= 0;
        else if (a >= 0 && b >= 0)
            differ++;
        else if (a >= 0)
            only_hw++;
        else
            only_slam++;
        if (ORB_FIELD(job.q_meta[2 * i + 1], 63, 1)) {
            valid_q++;
            scanned += (int)ORB_FIELD(job.q_meta[2 * i + 1], 32, 16) - (int)ORB_FIELD(job.q_meta[2 * i + 1], 16, 16);
        }
    }
    const char *name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
    printf("[pair] %-18s N1=%d N2=%d (in grid %d)  ORB-SLAM3 %3d matches, FPGA %3d: "
           "identical %3d, other idx %d, only-FPGA %d, only-ORB-SLAM3 %d | %d level-0 queries, "
           "avg scan %.0f -> %.0f cycles\n",
           name, p.n1, p.n2, job.n_train, p.nm_orbslam, nm_hw, same, differ, only_hw, only_slam,
           valid_q, valid_q ? (double)scanned / valid_q : 0.0,
           valid_q ? (double)scanned / valid_q / ORB_LANES : 0.0);
    orb_job_free(&job);
    orb_pair_free(&p);
    return fails;
}

int main(int argc, char **argv)
{
    int quick = 0, fails = 0;
    for (int a = 1; a < argc; a++)
        quick |= !strcmp(argv[a], "--quick");

    fails += check_arithmetic();
    fails += random_tests(quick);
    for (int a = 1; a < argc; a++)
        if (strcmp(argv[a], "--quick"))
            fails += pair_test(argv[a]);

    printf("\n%s\n", fails ? "*** TEST FAILED ***" : "*** ALL TESTS PASSED ***");
    return fails ? 1 : 0;
}
