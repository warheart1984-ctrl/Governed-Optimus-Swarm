#!/usr/bin/env bash
# One-command, rerun-ready harness for the OmniSim seam adapter.
#
# Default (offline, CI-safe — no OmniSim, no network):
#   ./omnisim_seam/rerun.sh
#
# After OmniLink signs off on a physics-layer fix, live replay is:
#   ./omnisim_seam/rerun.sh --live [-- extra args to omnisim_seam/__init__.py]
#
# Do not pass --live against a known-failing turn-control path.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

export PYTHONPATH="${ROOT}${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1

echo "== environment reset =="
rm -f "${ROOT}/omnisim_seam_evidence.json"
rm -rf "${ROOT}/.pytest_cache" "${ROOT}/omnisim_seam/__pycache__"
find "${ROOT}/omnisim_seam" -name '*.pyc' -delete

echo "== route-geometry harness (no pytest required) =="
python3 "${ROOT}/omnisim_seam/test_route_geometry.py"

echo "== seam + narrow invariant tests =="
python3 -m pytest \
  "${ROOT}/omnisim_seam/test_route_geometry.py" \
  "${ROOT}/omnisim_seam/test_omnisim_seam.py" \
  "${ROOT}/omnisim_seam/test_omnisim_narrow.py" \
  "${ROOT}/omnisim_seam/test_geometry_attribution.py" \
  -q

if [[ "${1:-}" == "--live" ]]; then
  echo "== live OmniSim seam (opt-in) =="
  echo "NOTE: skip this until OmniLink gives a green light on the physics fix."
  shift
  python3 "${ROOT}/omnisim_seam/__init__.py" "$@"
else
  echo "== live OmniSim skipped (pass --live only after OmniLink sign-off) =="
fi

echo "== rerun complete =="
