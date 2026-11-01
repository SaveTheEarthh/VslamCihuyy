# orientation (owner: Raegrand)

Keypoint orientation by intensity centroid over a radius-15 circular patch (ORB-SLAM3 `IC_Angle`).

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model; must agree with ORB-SLAM3 on EuRoC |
| `sim/` | `run.sh`: vectors into `build/`, simulate, exit 0 only if bit-exact |
