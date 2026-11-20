/*
 * Golden-data generator for the FPGA matcher.
 *
 * Runs ORB-SLAM3's own ORBextractor + Frame + ORBmatcher::SearchForInitialization
 * on two EuRoC cam0 images, exactly as monocular initialisation does
 * (5 x 1000 features, scale 1.2, 8 levels, FAST 20/7, window 100 px, ratio 0.9,
 * orientation check on), and writes everything the matcher needs plus
 * ORB-SLAM3's answer to a binary file (format: see hw/matcher/model/orb_host.h).
 *
 * Build/run (inside the ros2-box distrobox): ./build_dump.sh, then
 *   ./dump_orbslam3 <cam0/data dir> <frame idx> <offset> <out.bin>
 */
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>
#include <dirent.h>

#include <opencv2/imgcodecs.hpp>

#include "Frame.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "CameraModels/Pinhole.h"

using namespace ORB_SLAM3;

/* One keypoint record, 56 bytes, little-endian (matches orb_dataset.h). */
struct Rec {
    float x, y, angle;
    int32_t octave, cell_x, cell_y; /* cell = -1 if not in Frame::mGrid */
    uint8_t desc[32];
};

static std::vector<Rec> records(const Frame &F)
{
    std::vector<Rec> r(F.N);
    for (int i = 0; i < F.N; i++) {
        const cv::KeyPoint &k = F.mvKeysUn[i];
        r[i].x = k.pt.x;
        r[i].y = k.pt.y;
        r[i].angle = k.angle;
        r[i].octave = k.octave;
        r[i].cell_x = r[i].cell_y = -1;
        memcpy(r[i].desc, F.mDescriptors.ptr<uint8_t>(i), 32);
    }
    for (int ix = 0; ix < FRAME_GRID_COLS; ix++)
        for (int iy = 0; iy < FRAME_GRID_ROWS; iy++)
            for (size_t j : F.mGrid[ix][iy]) {
                r[j].cell_x = ix;
                r[j].cell_y = iy;
            }
    return r;
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: %s <cam0/data dir> <frame idx> <offset> <out.bin>\n", argv[0]);
        return 1;
    }
    const std::string dir = argv[1];
    const int idx = atoi(argv[2]), off = atoi(argv[3]);

    std::vector<std::string> files;
    DIR *d = opendir(dir.c_str());
    if (!d) { perror(dir.c_str()); return 1; }
    while (dirent *e = readdir(d))
        if (strstr(e->d_name, ".png")) files.push_back(e->d_name);
    closedir(d);
    std::sort(files.begin(), files.end());
    if (idx + off >= (int)files.size()) { fprintf(stderr, "index out of range\n"); return 1; }

    cv::Mat im1 = cv::imread(dir + "/" + files[idx], cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(dir + "/" + files[idx + off], cv::IMREAD_GRAYSCALE);

    /* EuRoC cam0 intrinsics + radial-tangential distortion (Examples/Monocular/EuRoC.yaml).
     * We keep the native 752x480 instead of the yaml's 600x350 resize: the
     * matcher logic is identical, only the numbers change. */
    GeometricCamera *cam = new Pinhole({458.654f, 457.296f, 367.215f, 248.375f});
    cv::Mat dist = (cv::Mat_<float>(4, 1) << -0.28340811f, 0.07395907f, 0.00019359f, 1.76187114e-05f);

    ORBextractor ext(5 * 1000, 1.2f, 8, 20, 7); /* = Tracking::mpIniORBextractor */
    Frame F1(im1, 0.0, &ext, nullptr, cam, dist, 0.f, 0.f);
    Frame F2(im2, 0.05, &ext, nullptr, cam, dist, 0.f, 0.f);

    const int window = 100;
    std::vector<cv::Point2f> prev(F1.N);
    for (int i = 0; i < F1.N; i++) prev[i] = F1.mvKeysUn[i].pt;
    std::vector<cv::Point2f> prev_in = prev; /* SearchForInitialization overwrites it */

    ORBmatcher matcher(0.9f, true);
    std::vector<int> m12;
    int nm = matcher.SearchForInitialization(F1, F2, prev, m12, window);

    /* CPU baseline: median of 50 runs (prev is an in/out argument, so reset it) */
    std::vector<double> us;
    for (int rep = 0; rep < 50; rep++) {
        std::vector<cv::Point2f> p2 = prev_in;
        std::vector<int> m;
        auto t0 = std::chrono::steady_clock::now();
        matcher.SearchForInitialization(F1, F2, p2, m, window);
        us.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
    }
    std::sort(us.begin(), us.end());

    FILE *f = fopen(argv[4], "wb");
    if (!f) { perror(argv[4]); return 1; }
    int32_t hdr[8] = {0x4D42524F /* "ORBM" */, 1, F1.N, F2.N, window, nm, FRAME_GRID_COLS, FRAME_GRID_ROWS};
    float grid[4] = {Frame::mnMinX, Frame::mnMinY, Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv};
    fwrite(hdr, sizeof hdr, 1, f);
    fwrite(grid, sizeof grid, 1, f);
    std::vector<Rec> r1 = records(F1), r2 = records(F2);
    fwrite(r1.data(), sizeof(Rec), r1.size(), f);
    fwrite(r2.data(), sizeof(Rec), r2.size(), f);
    for (const cv::Point2f &p : prev_in) fwrite(&p, sizeof(float), 2, f);
    fwrite(m12.data(), sizeof(int32_t), m12.size(), f);
    fclose(f);

    printf("%s: %s -> %s  N1=%d N2=%d  ORB-SLAM3 matches=%d  SearchForInitialization %.0f us (median, this CPU)\n",
           argv[4], files[idx].c_str(), files[idx + off].c_str(), F1.N, F2.N, nm, us[us.size() / 2]);
    return 0;
}
