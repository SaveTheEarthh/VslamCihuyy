/*
 * C side of the RTL testbench (called from tb_top.sv through DPI-C).
 *
 * For each test it builds the inputs (random scenes, or a real EuRoC pair
 * from ../sim/data), serialises them into the 64-bit input stream the RTL
 * expects, runs the golden model orb_match_ref() on the same inputs, and
 * compares the RTL's output words against it bit for bit.
 *
 * Environment:
 *   ORB_RANDOM  number of random tests (default 60)
 *   ORB_PAIRS   space-separated list of golden pair files (.bin)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "orb_match.h"
#include "orb_host.h"

static uint64_t t_desc[ORB_MAX_TRAIN * ORB_DESC_WORDS], t_meta[ORB_MAX_TRAIN];
static uint64_t q_desc[ORB_MAX_QUERY * ORB_DESC_WORDS], q_meta[ORB_MAX_QUERY * 2];
static int32_t m_ref[ORB_MAX_QUERY], d_ref[ORB_MAX_QUERY];
static uint32_t out_word[ORB_MAX_QUERY];
static uint64_t stream[ORB_RATIO_LIM_LEN + 5 * ORB_MAX_TRAIN + 6 * ORB_MAX_QUERY];

static int nt, nq, th, uniq, orient, stream_len, nm_ref;
static float ratio;
static char name[96];

/* real pair, kept to compare with ORB-SLAM3 */
static int have_pair;
static orb_pair pair;
static orb_job job;

static char *pair_list[64];
static int n_pairs = -1, n_random;

static uint64_t rng = 0x9E3779B97F4A7C15ULL;
static uint32_t rnd(void)
{
    rng ^= rng >> 12;
    rng ^= rng << 25;
    rng ^= rng >> 27;
    return (uint32_t)((rng * 0x2545F4914F6CDD1DULL) >> 32);
}
static int rnd_range(int lo, int hi) { return lo + (int)(rnd() % (uint32_t)(hi - lo + 1)); }

static void read_env(void)
{
    if (n_pairs >= 0)
        return;
    const char *r = getenv("ORB_RANDOM");
    n_random = r ? atoi(r) : 60;
    n_pairs = 0;
    const char *p = getenv("ORB_PAIRS");
    if (p) {
        char *copy = strdup(p);
        for (char *tok = strtok(copy, " "); tok && n_pairs < 64; tok = strtok(NULL, " "))
            pair_list[n_pairs++] = tok;
    }
}

int tb_num_tests(void)
{
    read_env();
    return n_random + n_pairs;
}

/* Random scene, same idea as tb_orb_match.c: some queries are noisy copies of
 * train keypoints placed nearby, the rest random. Small sizes first. */
static void make_random(int t)
{
    static const int edge[][2] = {{0, 5}, {5, 0}, {1, 1}, {7, 7}, {8, 9}, {9, 20}, {37, 40}};
    const int n_edge = (int)(sizeof edge / sizeof edge[0]);
    const int brute = (t % 4 == 0);

    if (t < n_edge) {
        nt = edge[t][0];
        nq = edge[t][1];
    } else {
        nt = rnd_range(1, 600);
        nq = rnd_range(1, 300);
    }
    for (int j = 0; j < nt; j++) {
        for (int w = 0; w < ORB_DESC_WORDS; w++)
            t_desc[ORB_DESC_WORDS * j + w] = ((uint64_t)rnd() << 32) | rnd();
        t_meta[j] = orb_pack_train(rnd_range(0, 752 << 8), rnd_range(0, 480 << 8),
                                   rnd() & 0xFFFF, rnd_range(0, 7), rnd_range(0, 9) != 0);
    }
    for (int i = 0; i < nq; i++) {
        uint64_t *qd = q_desc + ORB_DESC_WORDS * i;
        int x, y, lvl = rnd_range(0, 7);
        uint32_t ang = rnd() & 0xFFFF;
        if (nt > 0 && rnd_range(0, 3) != 0) {
            const int j = rnd_range(0, nt - 1);
            memcpy(qd, t_desc + ORB_DESC_WORDS * j, ORB_DESC_BYTES);
            for (int f = rnd_range(0, 70); f > 0; f--) {
                int bit = rnd_range(0, 255);
                qd[bit / 64] ^= 1ULL << (bit % 64);
            }
            x = orb_sext20(ORB_FIELD(t_meta[j], 0, 20)) + rnd_range(-3000, 3000);
            y = orb_sext20(ORB_FIELD(t_meta[j], 20, 20)) + rnd_range(-3000, 3000);
            lvl = (int)ORB_FIELD(t_meta[j], 56, 3);
            ang = (ORB_FIELD(t_meta[j], 40, 16) + (rnd_range(0, 4) ? rnd_range(-1820, 1820) : (int)rnd())) & 0xFFFF;
        } else {
            for (int w = 0; w < ORB_DESC_WORDS; w++)
                qd[w] = ((uint64_t)rnd() << 32) | rnd();
            x = rnd_range(0, 752 << 8);
            y = rnd_range(0, 480 << 8);
        }
        int lo = 0, hi = nt;
        if (!brute && nt > 0) {
            lo = rnd_range(0, nt - 1);
            hi = rnd_range(lo, nt + 3);
        }
        const int min_l = brute ? 0 : (lvl > 0 ? lvl - rnd_range(0, 1) : 0);
        const int max_l = brute ? 7 : (lvl < 7 ? lvl + rnd_range(0, 1) : 7);
        q_meta[2 * i] = orb_pack_query0(x, y, (uint32_t)rnd_range(1, 120) << 8);
        q_meta[2 * i + 1] = orb_pack_query1(ang, lo, hi, min_l, max_l, !brute, rnd_range(0, 19) != 0);
    }
    static const float ratios[] = {0.0f, 0.6f, 0.7f, 0.75f, 0.8f, 0.9f};
    th = rnd_range(0, 1) ? ORB_TH_LOW : ORB_TH_HIGH;
    ratio = ratios[rnd_range(0, 5)];
    uniq = rnd_range(0, 3) != 0;
    orient = rnd_range(0, 3) != 0;
    snprintf(name, sizeof name, "random %d%s", t, brute ? " (brute force)" : "");
}

/* Prepare test t. Returns 0 on success. */
int tb_prepare(int t)
{
    uint16_t lim[ORB_RATIO_LIM_LEN];

    read_env();
    if (have_pair) {
        orb_job_free(&job);
        orb_pair_free(&pair);
        have_pair = 0;
    }
    if (t < n_random) {
        make_random(t);
    } else {
        const char *path = pair_list[t - n_random];
        if (orb_pair_load(path, &pair) || orb_prepare_init(&pair, &job))
            return 1;
        have_pair = 1;
        nt = job.n_train;
        nq = job.n_query;
        memcpy(t_desc, job.t_desc, sizeof(uint64_t) * ORB_DESC_WORDS * nt);
        memcpy(t_meta, job.t_meta, sizeof(uint64_t) * nt);
        memcpy(q_desc, job.q_desc, sizeof(uint64_t) * ORB_DESC_WORDS * nq);
        memcpy(q_meta, job.q_meta, sizeof(uint64_t) * 2 * nq);
        th = ORB_TH_LOW;            /* SearchForInitialization: ORBmatcher(0.9, true) */
        ratio = 0.9f;
        uniq = 1;
        orient = 1;
        const char *base = strrchr(path, '/');
        snprintf(name, sizeof name, "%s", base ? base + 1 : path);
    }

    /* golden answer */
    orb_ratio_table(ratio, lim);
    nm_ref = orb_match_ref(t_desc, t_meta, nt, q_desc, q_meta, nq, th, lim, uniq, orient, m_ref, d_ref);

    /* input stream: ratio table, train records, query records */
    int k = 0;
    for (int s = 0; s < ORB_RATIO_LIM_LEN; s++)
        stream[k++] = lim[s];
    for (int j = 0; j < nt; j++) {
        for (int w = 0; w < ORB_DESC_WORDS; w++)
            stream[k++] = t_desc[ORB_DESC_WORDS * j + w];
        stream[k++] = t_meta[j];
    }
    for (int i = 0; i < nq; i++) {
        for (int w = 0; w < ORB_DESC_WORDS; w++)
            stream[k++] = q_desc[ORB_DESC_WORDS * i + w];
        stream[k++] = q_meta[2 * i];
        stream[k++] = q_meta[2 * i + 1];
    }
    stream_len = k;
    memset(out_word, 0xEE, sizeof out_word);
    return 0;
}

/* 0 n_train, 1 n_query, 2 th_dist, 3 flags (bit0 unique, bit1 orient) */
int tb_cfg(int which)
{
    switch (which) {
    case 0: return nt;
    case 1: return nq;
    case 2: return th;
    default: return uniq | (orient << 1);
    }
}

int tb_stream_len(void) { return stream_len; }
long long tb_stream_word(int k) { return (long long)stream[k]; }

void tb_out(int k, int word)
{
    if (k >= 0 && k < ORB_MAX_QUERY)
        out_word[k] = (uint32_t)word;
}

/* Compare the RTL's results with the golden model. Returns 1 on failure. */
int tb_check(int nm_hw, int n_out, int cycles)
{
    int bad = (nm_hw != nm_ref) || (n_out != nq);
    int first = -1;
    for (int i = 0; i < nq; i++) {
        const int m = (int16_t)(out_word[i] & 0xFFFF);
        const int d = (int)(out_word[i] >> 16);
        if (m != m_ref[i] || d != d_ref[i]) {
            if (first < 0)
                first = i;
            bad = 1;
        }
    }
    printf("[rtl] %-26s nt=%4d nq=%4d matches rtl=%4d ref=%4d  %8d cycles  %s",
           name, nt, nq, nm_hw, nm_ref, cycles, bad ? "FAIL" : "bit-exact");
    if (first >= 0)
        printf("  (first diff query %d: rtl %d/%d ref %d/%d)", first,
               (int16_t)(out_word[first] & 0xFFFF), (int)(out_word[first] >> 16), m_ref[first], d_ref[first]);
    if (n_out != nq)
        printf("  (got %d outputs, expected %d)", n_out, nq);

    if (have_pair && !bad) {   /* also compare with ORB-SLAM3 itself */
        int same = 0, diff = 0;
        for (int i = 0; i < nq; i++) {
            const int16_t m = (int16_t)(out_word[i] & 0xFFFF);
            const int a = m >= 0 ? job.perm[m] : -1;
            if (a == pair.m12[i])
                same += a >= 0;
            else
                diff++;
        }
        printf("  | ORB-SLAM3 %d matches, identical %d, different %d", pair.nm_orbslam, same, diff);
        bad |= diff != 0;
    }
    printf("\n");
    fflush(stdout);
    return bad;
}
