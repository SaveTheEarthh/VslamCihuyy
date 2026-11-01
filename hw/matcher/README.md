# matcher (owner: Zaki)

Descriptor matcher: bit-exact port of ORB-SLAM3 `SearchForInitialization`. `hls/` holds the Vitis HLS version; `rtl/` the SystemVerilog one.

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable source only |
| `tb/` | Testbenches |
| `model/` | Golden model; must agree with ORB-SLAM3 on EuRoC |
| `sim/` | `run.sh`: vectors into `build/`, simulate, exit 0 only if bit-exact |
