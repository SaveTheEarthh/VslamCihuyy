/*
 * ORB feature matcher for FPGA (Vitis HLS, plain C).
 *
 * Implements ORB-SLAM3's ORBmatcher::SearchForInitialization rule set
 * (src/ORBmatcher.cc) as a hardware kernel:
 *
 *   for each query i (frame 1):
 *     candidates j = train features (frame 2) inside a window around the query,
 *                    on an allowed pyramid level, not already claimed by a
 *                    better match                               (gating)
 *     best, second = two smallest Hamming distances, best index (scan)
 *     accept if best <= th_dist and best < ratio * second          (tests)
 *     one-to-one: if train j was already taken, the newer match steals it
 *   reject accepted matches whose rotation (angle1 - angle2) is not in the
 *   three most popular histogram bins                       (orientation)
 *
 * The ARM side does what ORB-SLAM3's grid does: it sorts train features by
 * grid cell and gives each query a contiguous index range [lo, hi) to scan.
 * The kernel then scans that range ORB_LANES descriptors per clock and applies
 * the exact window test itself. Setting lo=0, hi=n_train and no window gives
 * eSLAM-style brute-force matching with the same kernel.
 *
 * All inputs are fixed point, so kernel and golden model agree bit for bit.
 */
#ifndef ORB_MATCH_H
#define ORB_MATCH_H

#include <stdint.h>

/* ------------------------------------------------------------ sizing */
#define ORB_DESC_BYTES   32           /* 256-bit rBRIEF descriptor        */
#define ORB_DESC_WORDS   4            /* stored as 4 x uint64             */
#define ORB_MAX_TRAIN    8192         /* mono init uses 5 x 1000 features */
#define ORB_MAX_QUERY    8192
#define ORB_LANES        8            /* descriptors compared per clock   */
#define ORB_QBLOCK       256          /* queries fetched per AXI burst    */
#ifndef ORB_TRAIN_URAM
#define ORB_TRAIN_URAM   1            /* train descriptors in UltraRAM    */
#endif

/* ------------------------------------------------- ORB-SLAM3 constants */
#define ORB_TH_LOW       50           /* ORBmatcher::TH_LOW               */
#define ORB_TH_HIGH      100          /* ORBmatcher::TH_HIGH              */
#define ORB_HISTO_LENGTH 30           /* ORBmatcher::HISTO_LENGTH         */
#define ORB_DIST_NONE    511          /* "INT_MAX": no candidate (9 bits) */
#define ORB_NO_MATCH     (-1)

/* --------------------------------------------------- fixed-point format
 * Coordinates: signed Q11.8 pixels in 20 bits (+-2048 px, 1/256 px step).
 * Angles:      16-bit binary angle, 65536 = 360 deg (wraps for free).
 */
#define ORB_XY_FRAC      8
#define ORB_XY_BITS      20

/* Train meta word (one uint64 per train feature):
 *   [19:0]  x  (signed Q11.8)    [39:20] y (signed Q11.8)
 *   [55:40] angle                [58:56] pyramid level (octave)
 *   [63]    valid (0 = never a candidate, e.g. not in ORB-SLAM3's grid)
 *
 * Query meta, two uint64 per query:
 *   w0: [19:0] x   [39:20] y   [59:40] window half-size r (unsigned Q12.8)
 *   w1: [15:0] angle   [31:16] scan lo   [47:32] scan hi (exclusive)
 *       [50:48] min level   [53:51] max level
 *       [62] use window (0 = accept any position)   [63] valid
 *
 * Descriptor: 32 bytes copied little-endian into 4 x uint64 (memcpy on ARM).
 */
#define ORB_FIELD(w, lsb, bits)  ((uint32_t)(((w) >> (lsb)) & ((1ULL << (bits)) - 1)))

static inline int32_t orb_sext20(uint32_t v)       /* sign-extend 20 bits */
{
    return (int32_t)(v ^ 0x80000u) - 0x80000;
}

static inline uint64_t orb_pack_train(int32_t x, int32_t y, uint32_t angle, uint32_t level, int valid)
{
    return  ((uint64_t)(x & 0xFFFFF))
          | ((uint64_t)(y & 0xFFFFF) << 20)
          | ((uint64_t)(angle & 0xFFFF) << 40)
          | ((uint64_t)(level & 7) << 56)
          | ((uint64_t)(valid != 0) << 63);
}

static inline uint64_t orb_pack_query0(int32_t x, int32_t y, uint32_t r)
{
    return  ((uint64_t)(x & 0xFFFFF))
          | ((uint64_t)(y & 0xFFFFF) << 20)
          | ((uint64_t)(r & 0xFFFFF) << 40);
}

static inline uint64_t orb_pack_query1(uint32_t angle, uint32_t lo, uint32_t hi,
                                       uint32_t min_lvl, uint32_t max_lvl, int use_window, int valid)
{
    return  ((uint64_t)(angle & 0xFFFF))
          | ((uint64_t)(lo & 0xFFFF) << 16)
          | ((uint64_t)(hi & 0xFFFF) << 32)
          | ((uint64_t)(min_lvl & 7) << 48)
          | ((uint64_t)(max_lvl & 7) << 51)
          | ((uint64_t)(use_window != 0) << 62)
          | ((uint64_t)(valid != 0) << 63);
}

/* Rotation-histogram bin, integer form of ORB-SLAM3's
 *   rot = angle1 - angle2; if (rot < 0) rot += 360; bin = round(rot / 30);
 * Note factor = 1/HISTO_LENGTH, so only bins 0..12 are ever used (30 deg
 * wide), and 345..360 deg lands in bin 12, not bin 0. Reproduced as is.
 * rot16 * 360/65536 / 30 = rot16 * 12 / 65536;  +0.5 then floor = round. */
static inline uint32_t orb_rot_bin(uint32_t angle1, uint32_t angle2)
{
    uint32_t rot16 = (angle1 - angle2) & 0xFFFF;
    return (rot16 * 12 + 32768) >> 16;
}

/* Ratio test as a table. ORB-SLAM3 tests "best < (float)second * ratio" and
 * the float rounding matters: 0.6f = 0.60000002, so 15 vs 25 passes while some
 * other exact-0.6 pairs fail. No integer num/den reproduces that for every
 * ratio, so the ARM fills lim[s] = number of b in 0..256 with b < (float)s*ratio
 * and the kernel checks best < lim[second]. ratio <= 0 disables the test. */
#define ORB_RATIO_LIM_LEN 257
static inline void orb_ratio_table(float ratio, uint16_t lim[ORB_RATIO_LIM_LEN])
{
    for (int s = 0; s < ORB_RATIO_LIM_LEN; s++) {
        int n = 0;
        while (n <= 256 && (ratio <= 0.0f || (float)n < (float)s * ratio))
            n++;
        lim[s] = (uint16_t)n;
    }
}

/* ------------------------------------------------------------ kernel API
 * Returns the number of matches (ORB-SLAM3's nmatches).
 *   th_dist          accept only best <= th_dist (TH_LOW = 50 for init)
 *   ratio_lim[257]   accept only best < ratio_lim[second] (orb_ratio_table);
 *                    a second best of "none" always passes
 *   unique           ORB-SLAM3's vMatchedDistance / vnMatches21 one-to-one rule
 *   check_orient     rotation-consistency histogram
 *   match_out[i]     train index or -1;  dist_out[i] best distance (511 = none)
 */
int orb_match_hls(const uint64_t *t_desc, const uint64_t *t_meta, int n_train,
                  const uint64_t *q_desc, const uint64_t *q_meta, int n_query,
                  int th_dist, const uint16_t *ratio_lim, int unique, int check_orient,
                  int32_t *match_out, int32_t *dist_out);

/* Golden model: same interface, written like ORBmatcher.cc, no HW tricks. */
int orb_match_ref(const uint64_t *t_desc, const uint64_t *t_meta, int n_train,
                  const uint64_t *q_desc, const uint64_t *q_meta, int n_query,
                  int th_dist, const uint16_t *ratio_lim, int unique, int check_orient,
                  int32_t *match_out, int32_t *dist_out);

#endif
