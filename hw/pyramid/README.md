# pyramid (owner: Timothy)

Image pyramid: 8 levels, scale factor 1.2, bilinear resize of the 600×350 input (ORB-SLAM3 `ComputePyramid`).

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model; must agree with ORB-SLAM3 on EuRoC |
| `sim/` | `run.sh`: vectors into `build/`, simulate, exit 0 only if bit-exact |
