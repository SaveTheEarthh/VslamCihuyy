#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "orb_host.h"

int orb_pair_load(const char *path, orb_pair *p)
{
    FILE *f = fopen(path, "rb");
    int32_t hdr[8];
    float grid[4];

    memset(p, 0, sizeof *p);
    if (!f) {
        perror(path);
        return -1;
    }
    if (fread(hdr, sizeof hdr, 1, f) != 1 || hdr[0] != 0x4D42524F || hdr[1] != 1
        || fread(grid, sizeof grid, 1, f) != 1) {
        fprintf(stderr, "%s: bad header\n", path);
        fclose(f);
        return -1;
    }
    p->n1 = hdr[2];
    p->n2 = hdr[3];
    p->window = hdr[4];
    p->nm_orbslam = hdr[5];
    p->grid_cols = hdr[6];
    p->grid_rows = hdr[7];
    p->min_x = grid[0];
    p->min_y = grid[1];
    p->grid_w_inv = grid[2];
    p->grid_h_inv = grid[3];

    p->f1 = malloc(sizeof(orb_kp) * p->n1);
    p->f2 = malloc(sizeof(orb_kp) * p->n2);
    p->prev = malloc(sizeof(float) * 2 * p->n1);
    p->m12 = malloc(sizeof(int32_t) * p->n1);
    if (fread(p->f1, sizeof(orb_kp), p->n1, f) != (size_t)p->n1
        || fread(p->f2, sizeof(orb_kp), p->n2, f) != (size_t)p->n2
        || fread(p->prev, sizeof(float) * 2, p->n1, f) != (size_t)p->n1
        || fread(p->m12, sizeof(int32_t), p->n1, f) != (size_t)p->n1) {
        fprintf(stderr, "%s: truncated\n", path);
        fclose(f);
        orb_pair_free(p);
        return -1;
    }
    fclose(f);
    return 0;
}

void orb_pair_free(orb_pair *p)
{
    free(p->f1);
    free(p->f2);
    free(p->prev);
    free(p->m12);
    memset(p, 0, sizeof *p);
}

int orb_prepare_init(const orb_pair *p, orb_job *job)
{
    const int ncells = p->grid_cols * p->grid_rows;
    int *cell_count = calloc(ncells + 1, sizeof(int));
    int *col_start = calloc(p->grid_cols + 1, sizeof(int));
    int n = 0;

    memset(job, 0, sizeof *job);
    if (p->n2 > ORB_MAX_TRAIN || p->n1 > ORB_MAX_QUERY) {
        fprintf(stderr, "frame too large for kernel (%d, %d)\n", p->n1, p->n2);
        free(cell_count);
        free(col_start);
        return -1;
    }

    /* Counting sort of frame-2 features by (cell_x, cell_y), stable in index.
     * That is exactly the order Frame::GetFeaturesInArea visits them (ix outer,
     * iy inner, mGrid lists in index order), so even ties resolve the same. */
    for (int j = 0; j < p->n2; j++)
        if (p->f2[j].cell_x >= 0)
            cell_count[p->f2[j].cell_x * p->grid_rows + p->f2[j].cell_y + 1]++;
    for (int c = 0; c < ncells; c++)
        cell_count[c + 1] += cell_count[c];
    for (int cx = 0; cx <= p->grid_cols; cx++)
        col_start[cx] = cell_count[cx * p->grid_rows];
    n = cell_count[ncells];

    job->n_train = n;
    job->n_query = p->n1;
    job->t_desc = malloc(sizeof(uint64_t) * ORB_DESC_WORDS * (n ? n : 1));
    job->t_meta = malloc(sizeof(uint64_t) * (n ? n : 1));
    job->perm = malloc(sizeof(int32_t) * (n ? n : 1));
    job->q_desc = malloc(sizeof(uint64_t) * ORB_DESC_WORDS * (p->n1 ? p->n1 : 1));
    job->q_meta = malloc(sizeof(uint64_t) * 2 * (p->n1 ? p->n1 : 1));

    for (int j = 0; j < p->n2; j++) {
        const orb_kp *k = &p->f2[j];
        if (k->cell_x < 0)
            continue;               /* not in mGrid: ORB-SLAM3 can never return it */
        const int s = cell_count[k->cell_x * p->grid_rows + k->cell_y]++;
        job->perm[s] = j;
        memcpy(job->t_desc + ORB_DESC_WORDS * s, k->desc, ORB_DESC_BYTES);
        job->t_meta[s] = orb_pack_train(orb_to_fix(k->x), orb_to_fix(k->y), orb_angle16(k->angle),
                                        (uint32_t)k->octave, 1);
    }

    /* Queries. The cell range is computed exactly like GetFeaturesInArea; the
     * columns [min_cx, max_cx] form one contiguous block of the sorted train
     * list. The kernel's window test then removes the rows outside the window. */
    const float r = (float)p->window;
    for (int i = 0; i < p->n1; i++) {
        const orb_kp *k = &p->f1[i];
        const float x = p->prev[2 * i], y = p->prev[2 * i + 1];
        int valid = (k->octave == 0);        /* "if(level1>0) continue;" */
        int lo = 0, hi = 0;

        const int min_cx = (int)fmaxf(0.f, floorf((x - p->min_x - r) * p->grid_w_inv));
        const int max_cx = (int)fminf((float)(p->grid_cols - 1), ceilf((x - p->min_x + r) * p->grid_w_inv));
        const int min_cy = (int)fmaxf(0.f, floorf((y - p->min_y - r) * p->grid_h_inv));
        const int max_cy = (int)fminf((float)(p->grid_rows - 1), ceilf((y - p->min_y + r) * p->grid_h_inv));
        if (min_cx >= p->grid_cols || max_cx < 0 || min_cy >= p->grid_rows || max_cy < 0 || max_cx < min_cx)
            valid = 0;
        else {
            lo = col_start[min_cx];
            hi = col_start[max_cx + 1];
        }

        memcpy(job->q_desc + ORB_DESC_WORDS * i, k->desc, ORB_DESC_BYTES);
        job->q_meta[2 * i] = orb_pack_query0(orb_to_fix(x), orb_to_fix(y), (uint32_t)orb_to_fix(r));
        job->q_meta[2 * i + 1] = orb_pack_query1(orb_angle16(k->angle), (uint32_t)lo, (uint32_t)hi,
                                                 (uint32_t)k->octave, (uint32_t)k->octave, 1, valid);
    }

    free(cell_count);
    free(col_start);
    return 0;
}

void orb_job_free(orb_job *job)
{
    free(job->t_desc);
    free(job->t_meta);
    free(job->q_desc);
    free(job->q_meta);
    free(job->perm);
    memset(job, 0, sizeof *job);
}
