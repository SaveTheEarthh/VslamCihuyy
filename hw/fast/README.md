# fast (owner: Fadhly)

FAST-9 corner detection (`cv::FAST`) per 35 px cell, threshold 20 with a retry at 7, plus non-maximum suppression.

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model; must agree with ORB-SLAM3 on EuRoC |
| `sim/` | `run.sh`: vectors into `build/`, simulate, exit 0 only if bit-exact |
