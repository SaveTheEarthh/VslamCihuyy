# Test data

`frames/` holds a few EuRoC MH_01_easy frames, already resized to **600×350** (the monocular
`EuRoC.yaml` size) and saved as 8-bit PGM. Every stage tests on these.

Keep this folder small. Full EuRoC sequences stay outside the repo (`data/euroc/` is ignored);
fetch them with `tools/fetch_euroc.py` (to be added).
