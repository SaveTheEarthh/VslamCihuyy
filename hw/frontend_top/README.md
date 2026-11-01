# frontend_top (owner: TBD)

Integration of all stages into the ORB front-end top level, with an end-to-end testbench against the full golden model.

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model; must agree with ORB-SLAM3 on EuRoC |
| `sim/` | `run.sh`: vectors into `build/`, simulate, exit 0 only if bit-exact |
