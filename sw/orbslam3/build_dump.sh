#!/usr/bin/env bash
# Build dump_orbslam3 against the ORB-SLAM3 checkout and generate the matcher's
# golden pairs in hw/matcher/sim/data. Run on the host; it re-enters the ros2-box distrobox.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ORB=${ORB:-$HOME/vslam_ws/ORB_SLAM3}
SEQ=${SEQ:-$HOME/vslam_ws/datasets/EuRoC/MH_01_easy/mav0/cam0/data}

if [ -z "${CONTAINER_ID:-}" ]; then
    exec distrobox enter ros2-box -- bash "$0" "$@"
fi

# -march=native + COMPILEDWITHC11 must match the ORB-SLAM3 build, or Frame/Eigen layouts differ (segfault)
g++ -O3 -march=native -std=c++17 -DCOMPILEDWITHC11 "$HERE/dump_orbslam3.cc" -o "$HERE/dump_orbslam3" \
    -I"$ORB" -I"$ORB/include" -I"$ORB/include/CameraModels" -I"$ORB/Thirdparty/Sophus" \
    -I/usr/include/eigen3 -DHAVE_EIGEN -DHAVE_GLEW -D_LINUX_ $(pkg-config --cflags --libs opencv4) \
    -L"$ORB/lib" -lORB_SLAM3 "$ORB/Thirdparty/DBoW2/lib/libDBoW2.so" "$ORB/Thirdparty/g2o/lib/libg2o.so" \
    -L/usr/local/lib -lpango_display -lpango_opengl -lpango_core -lGLEW -lGL -lboost_serialization -lcrypto -Wl,-rpath,"$ORB/lib:$ORB/Thirdparty/DBoW2/lib:$ORB/Thirdparty/g2o/lib"

# (start frame, offset): small, medium and large motion at different places in MH_01
for p in "300 1" "300 5" "800 3" "1500 10" "2500 2"; do
    set -- $p
    "$HERE/dump_orbslam3" "$SEQ" "$1" "$2" "$HERE/../../hw/matcher/sim/data/mh01_${1}_${2}.bin"
done
