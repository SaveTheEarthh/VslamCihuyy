# rbrief (owner: Kenny)

rBRIEF descriptor: 256 learned point pairs rotated by the keypoint angle, bit = I(a) < I(b).

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model; must agree with ORB-SLAM3 on EuRoC |
| `sim/` | `run.sh`: vectors into `build/`, simulate, exit 0 only if bit-exact |
