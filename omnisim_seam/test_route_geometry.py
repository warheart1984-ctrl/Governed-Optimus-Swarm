"""Tiny harness for route-distance math.

Run either way (no live OmniSim, no network):

    python3 omnisim_seam/test_route_geometry.py
    python3 -m pytest omnisim_seam/test_route_geometry.py -q

Covers the OmniLink-validated rule: bill HTTP wait from the observed
starting pose, fall back to spawn only when that pose is unusable, and
stay robust against NaN / Inf / truncated / already-at-goal poses.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

# Allow `python3 omnisim_seam/test_route_geometry.py` from any cwd.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from omnisim_seam.route_geometry import (
    DEFAULT_MAX_PLAUSIBLE_DISTANCE_M,
    euclidean_m,
    heading_error_rad,
    plan_route_budget,
    remaining_to_goal_m,
    wrap_heading_rad,
    xy_from_observation,
    yaw_from_observation,
)


def _budget(start_obs, goal, fallback=(-4.0, -2.0), **kwargs):
    defaults = dict(
        cruise_speed_mps=1.0,
        settle_timeout_s=3.0,
        min_timeout_s=5.0,
    )
    defaults.update(kwargs)
    return plan_route_budget(start_obs, goal, fallback, **defaults)


def test_observed_pose_beats_spawn():
    """A robot already 10 m from spawn must not be billed as an 8 m spawn-to-goal hop."""
    # spawn (-4, -2) -> goal (4, -2) would be 8 m. Observed start is (0, -2).
    budget = _budget({"pose": [0.0, -2.0], "yaw": 0.0}, goal=(4.0, -2.0))
    assert budget.source == "observed"
    assert budget.distance_m == 4.0
    assert budget.timeout_s == 7.0  # 4 m / 1 m/s + 3 s settle, above min 5
    assert budget.abort_reason is None
    assert budget.spawn_drift_m == 4.0


def test_fallback_when_observation_missing():
    budget = _budget(None, goal=(4.0, -2.0))
    assert budget.source == "fallback_spawn"
    assert budget.distance_m == 8.0  # spawn (-4,-2) to (4,-2)
    assert budget.timeout_s == 11.0  # 8 + 3
    assert any("unusable" in n for n in budget.notes)


def test_nan_pose_falls_back():
    budget = _budget({"pose": [float("nan"), 1.0]}, goal=(4.0, -2.0))
    assert budget.source == "fallback_spawn"
    assert budget.abort_reason is None


def test_inf_pose_falls_back():
    budget = _budget({"pose": [float("inf"), 0.0]}, goal=(4.0, -2.0))
    assert budget.source == "fallback_spawn"


def test_truncated_pose_falls_back():
    budget = _budget({"pose": [1.0]}, goal=(4.0, -2.0))
    assert budget.source == "fallback_spawn"


def test_string_pose_falls_back():
    budget = _budget({"pose": "nope"}, goal=(4.0, -2.0))
    assert budget.source == "fallback_spawn"


def test_already_at_goal_is_settle_only():
    budget = _budget({"pose": [4.0, -2.0], "yaw": 0.0}, goal=(4.0, -2.0))
    assert budget.distance_m == 0.0
    assert budget.timeout_s == 5.0  # min_timeout_s floors the settle-only wait
    assert budget.heading_to_goal_rad is None
    assert any("coincides" in n for n in budget.notes)
    assert budget.abort_reason is None


def test_implausible_distance_aborts():
    far = {"pose": [0.0, 0.0]}
    budget = _budget(far, goal=(DEFAULT_MAX_PLAUSIBLE_DISTANCE_M + 10.0, 0.0))
    assert budget.abort_reason is not None
    assert "implausible" in budget.abort_reason
    assert budget.distance_m > DEFAULT_MAX_PLAUSIBLE_DISTANCE_M


def test_non_finite_goal_aborts():
    budget = _budget({"pose": [0.0, 0.0]}, goal=(float("nan"), 1.0))
    assert budget.abort_reason is not None
    assert budget.source == "invalid_goal"


def test_heading_wraps_weird_yaw():
    # 3pi wraps to +pi (atan2(0, -1)); the range is (-pi, pi].
    assert wrap_heading_rad(3 * math.pi) == pytest_approx(math.pi)
    assert wrap_heading_rad(float("nan")) is None
    assert wrap_heading_rad(float("inf")) is None
    # 1e9 rad is a real (if absurd) number; wrap must still return (-pi, pi].
    wrapped = wrap_heading_rad(1e9)
    assert wrapped is not None
    assert -math.pi < wrapped <= math.pi


def pytest_approx(value, rel=1e-9):
    # Local helper so this file runs without pytest as `python3 ...py`.
    class _Approx:
        def __init__(self, expected):
            self.expected = expected

        def __eq__(self, other):
            if other is None or self.expected is None:
                return other is self.expected
            return math.isclose(other, self.expected, rel_tol=rel, abs_tol=1e-9)

        def __repr__(self):
            return f"approx({self.expected})"

    return _Approx(value)


def test_large_heading_error_still_dispatches():
    """Turn-control belongs to OmniSim; a 180 deg yaw is logged, not aborted."""
    budget = _budget(
        {"pose": [0.0, -2.0], "yaw": math.pi},  # facing west, goal is east
        goal=(4.0, -2.0),
    )
    assert budget.abort_reason is None
    assert budget.heading_error_rad is not None
    assert abs(budget.heading_error_rad) > 2.0  # ~pi
    assert any("heading error" in n for n in budget.notes)


def test_zero_cruise_does_not_divide_by_zero():
    budget = _budget(
        {"pose": [0.0, 0.0]},
        goal=(1.0, 0.0),
        cruise_speed_mps=0.0,
        settle_timeout_s=0.0,
        min_timeout_s=0.0,
    )
    assert math.isfinite(budget.timeout_s)
    assert budget.timeout_s > 0.0


def test_xy_and_yaw_extractors():
    assert xy_from_observation({"pose": [1.5, 2.5]}) == (1.5, 2.5)
    assert xy_from_observation({"pose": [float("nan"), 0.0]}) is None
    assert xy_from_observation({}) is None
    assert yaw_from_observation({"yaw": 0.0}) == 0.0
    assert yaw_from_observation({"yaw": "bad"}) is None


def test_remaining_and_euclidean():
    assert euclidean_m((0.0, 0.0), (3.0, 4.0)) == 5.0
    assert remaining_to_goal_m({"pose": [3.0, 4.0]}, (0.0, 0.0)) == 5.0
    assert remaining_to_goal_m({"pose": None}, (0.0, 0.0)) is None


def test_heading_error_shortest_path():
    # +350 deg vs 0 is a -10 deg error, not +350.
    err = heading_error_rad(math.radians(350.0), 0.0)
    assert err is not None
    assert math.isclose(err, math.radians(10.0), abs_tol=1e-9)


def test_min_timeout_floors_short_hops():
    budget = _budget(
        {"pose": [0.0, 0.0]},
        goal=(0.1, 0.0),
        cruise_speed_mps=1.0,
        settle_timeout_s=0.0,
        min_timeout_s=45.0,
    )
    assert budget.timeout_s == 45.0


# ------------------------------------------------------------------------- #
# Standalone runner (no pytest required)                                    #
# ------------------------------------------------------------------------- #
_CASES = [
    test_observed_pose_beats_spawn,
    test_fallback_when_observation_missing,
    test_nan_pose_falls_back,
    test_inf_pose_falls_back,
    test_truncated_pose_falls_back,
    test_string_pose_falls_back,
    test_already_at_goal_is_settle_only,
    test_implausible_distance_aborts,
    test_non_finite_goal_aborts,
    test_heading_wraps_weird_yaw,
    test_large_heading_error_still_dispatches,
    test_zero_cruise_does_not_divide_by_zero,
    test_xy_and_yaw_extractors,
    test_remaining_and_euclidean,
    test_heading_error_shortest_path,
    test_min_timeout_floors_short_hops,
]


def main() -> int:
    failed = 0
    for case in _CASES:
        try:
            case()
            print(f"PASS  {case.__name__}")
        except Exception as exc:  # noqa: BLE001 — harness prints and continues
            failed += 1
            print(f"FAIL  {case.__name__}: {exc}")
    print(f"\n{len(_CASES) - failed}/{len(_CASES)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
