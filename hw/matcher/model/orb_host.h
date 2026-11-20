/*
 * ARM-side helpers: load golden pairs written by sw/orbslam3/dump_orbslam3, and turn
 * ORB-SLAM3 keypoints into the kernel's packed fixed-point buffers.
 * This is the code that would run on the KV260's Cortex-A53 before it starts
 * the kernel. Not synthesised.
 */
#ifndef ORB_HOST_H
#define ORB_HOST_H

#include <stdint.h>
#include "orb_match.h"

/* one keypoint, as written by dump_orbslam3 (56 bytes) */
typedef struct {
    float x, y, angle;          /* undistorted pixel position, angle in degrees */
    int32_t octave;             /* pyramid level */
    int32_t cell_x, cell_y;     /* ORB-SLAM3 grid cell, -1 if not in the grid */
    uint8_t desc[ORB_DESC_BYTES];
} orb_kp;

/* two frames + ORB-SLAM3's SearchForInitialization result */
typedef struct {
    int n1, n2, window, nm_orbslam, grid_cols, grid_rows;
    float min_x, min_y, grid_w_inv, grid_h_inv;
    orb_kp *f1, *f2;
    float *prev;                /* n1 x (x, y): window centres (vbPrevMatched) */
    int32_t *m12;               /* ORB-SLAM3's vnMatches12 */
} orb_pair;

/* kernel buffers + the permutation back to ORB-SLAM3 indices */
typedef struct {
    int n_train, n_query;
    uint64_t *t_desc, *t_meta, *q_desc, *q_meta;
    int32_t *perm;              /* kernel train index -> original frame-2 index */
} orb_job;

int  orb_pair_load(const char *path, orb_pair *p);
void orb_pair_free(orb_pair *p);

/* Mono initialisation (SearchForInitialization): queries = frame 1, level 0
 * only, window = p->window around prev; train = frame 2 sorted by grid cell. */
int  orb_prepare_init(const orb_pair *p, orb_job *job);
void orb_job_free(orb_job *job);

static inline int32_t orb_to_fix(float v)     /* pixels -> Q11.8 */
{
    float s = v * (float)(1 << ORB_XY_FRAC);
    return (int32_t)(s < 0 ? s - 0.5f : s + 0.5f);
}

static inline uint32_t orb_angle16(float deg)  /* degrees -> binary angle */
{
    return (uint32_t)(int32_t)(deg * (65536.0f / 360.0f) + 0.5f) & 0xFFFF;
}

#endif
