"""Pure geometry for the OmniSim seam adapter.

Why this module exists
----------------------
The HTTP budget for `/drive_to_waypoint` used to be billed from the
*configured spawn*, not from where the Husky actually stood. OmniLink
validated the fix: derive route distance from the **observed starting
pose**. Spawn is only a fallback when the observation is unusable.

This file is deliberately physics-free. It does not command turns, does
not estimate slip, and does not second-guess OmniSim's completion gate.
It answers four adapter-side questions:

  1. What (x, y) can we trust as the start of this route?
  2. How far is that from the goal, in metres?
  3. How long should the HTTP wait be at the configured cruise speed?
  4. Is the observation sane enough to dispatch, or should we abort?

Weird starting poses (NaN, Inf, truncated lists, already-at-goal,
absurdly large coordinates) are handled here so the adapter stays
fail-closed without crashing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

# Minimum cruise used when the caller passes 0 or a negative speed so
# distance / speed cannot explode into inf.
MIN_CRUISE_SPEED_MPS = 0.01

# Diagnostic windows. These do NOT override OmniSim's arrived/settled/
# timed_out gate; they are recorded next to it so a later run can show
# "gate said arrived, but remaining distance was 1.8 m".
DEFAULT_ARRIVAL_TOLERANCE_M = 0.15
DEFAULT_YAW_TOLERANCE_RAD = 0.35  # ~20 degrees

# Hard cap: if a finite-but-insane pose implies a multi-kilometre drive,
# abort rather than sit on an HTTP socket for hours. 500 m is far beyond
# the Husky example world (~8 m waypoint span).
DEFAULT_MAX_PLAUSIBLE_DISTANCE_M = 500.0

# Cap the derived wait so a slightly-long route cannot exceed a human
# CI budget even when we do dispatch.
DEFAULT_MAX_TIMEOUT_S = 3600.0

XY = Tuple[float, float]


def wrap_heading_rad(yaw: float) -> Optional[float]:
    """Wrap a heading into (-pi, pi].

    OmniSim yaw can arrive as any real: 0, 2pi, -7pi, 1e-12. Wrapping
    lets heading-error math stay continuous. Non-finite yaw is not a
    heading we can sanity-check, so we return None (caller logs it).
    """
    if not math.isfinite(yaw):
        return None
    # atan2(sin, cos) is the numerically stable 2-pi wrap.
    return math.atan2(math.sin(yaw), math.cos(yaw))


def heading_error_rad(current_yaw: Optional[float], desired_yaw: Optional[float]) -> Optional[float]:
    """Smallest signed rotation from current heading to desired, in (-pi, pi]."""
    if current_yaw is None or desired_yaw is None:
        return None
    wrapped_current = wrap_heading_rad(current_yaw)
    wrapped_desired = wrap_heading_rad(desired_yaw)
    if wrapped_current is None or wrapped_desired is None:
        return None
    return wrap_heading_rad(wrapped_desired - wrapped_current)


def xy_from_observation(obs: Optional[Mapping[str, Any]]) -> Optional[XY]:
    """Extract a finite (x, y) from an observe() dict.

    Accepts the seam's ``{"pose": [x, y], ...}`` shape and is robust
    against None, missing keys, 1-element lists, strings, NaN, and Inf.
    Returns None when the observation cannot be used as a start pose.
    """
    if not obs:
        return None
    pose = obs.get("pose")
    if pose is None:
        return None
    try:
        x = float(pose[0])
        y = float(pose[1])
    except (TypeError, IndexError, ValueError, KeyError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return (x, y)


def yaw_from_observation(obs: Optional[Mapping[str, Any]]) -> Optional[float]:
    """Extract a wrappable yaw, or None if missing/non-finite."""
    if not obs:
        return None
    raw = obs.get("yaw")
    if raw is None:
        return None
    try:
        yaw = float(raw)
    except (TypeError, ValueError):
        return None
    return wrap_heading_rad(yaw)


def euclidean_m(a: XY, b: XY) -> float:
    """Planar metres between two ENU points. Callers must pass finite xy."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def heading_to_goal_rad(start: XY, goal: XY) -> Optional[float]:
    """World-frame heading from start toward goal, or None if coincident."""
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return None
    return math.atan2(dy, dx)


def finite_xy(point: Optional[Sequence[float]]) -> Optional[XY]:
    """Coerce a 2-vector to finite (x, y), else None."""
    if point is None:
        return None
    try:
        x = float(point[0])
        y = float(point[1])
    except (TypeError, IndexError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return (x, y)


@dataclass(frozen=True)
class RouteBudget:
    """Result of billing a drive from an observed (or fallback) start."""

    distance_m: float
    timeout_s: float
    source: str  # "observed" | "fallback_spawn"
    start_xy: Optional[XY]
    goal_xy: Optional[XY]
    abort_reason: Optional[str] = None
    heading_to_goal_rad: Optional[float] = None
    current_yaw_rad: Optional[float] = None
    heading_error_rad: Optional[float] = None
    spawn_drift_m: Optional[float] = None
    notes: Tuple[str, ...] = field(default_factory=tuple)

    def to_json(self) -> Dict[str, Any]:
        return {
            "distance_m": self.distance_m,
            "timeout_s": self.timeout_s,
            "source": self.source,
            "start_xy": list(self.start_xy) if self.start_xy else None,
            "goal_xy": list(self.goal_xy) if self.goal_xy else None,
            "abort_reason": self.abort_reason,
            "heading_to_goal_rad": self.heading_to_goal_rad,
            "current_yaw_rad": self.current_yaw_rad,
            "heading_error_rad": self.heading_error_rad,
            "spawn_drift_m": self.spawn_drift_m,
            "notes": list(self.notes),
        }


def plan_route_budget(
    start_obs: Optional[Mapping[str, Any]],
    goal_xy: Sequence[float],
    fallback_xy: Sequence[float],
    cruise_speed_mps: float,
    settle_timeout_s: float,
    min_timeout_s: float,
    max_plausible_distance_m: float = DEFAULT_MAX_PLAUSIBLE_DISTANCE_M,
    max_timeout_s: float = DEFAULT_MAX_TIMEOUT_S,
) -> RouteBudget:
    """Derive route distance and HTTP wait from the observed start pose.

    Logic (in order)
    ----------------
    1. Goal must be a finite (x, y). A non-finite waypoint is an adapter
       bug / table error; abort. We never send OmniSim NaN coordinates.
    2. Prefer the observed pose. Spawn is a configured origin, not ground
       truth — a Husky that already moved, or spawned with an offset,
       would get an undersized wait if we billed from spawn.
    3. If the observation is missing, truncated, NaN, or Inf, fall back
       to spawn so the wait is still bounded. Record the source so the
       evidence stream can show *why* the number was chosen.
    4. Distance is planar Euclidean. The seam does not plan around
       obstacles; OmniSim's `/drive_to_waypoint` owns the path.
    5. Timeout = distance / cruise + settle, floored at min_timeout_s
       and capped at max_timeout_s. Cruise is clamped to a small positive
       floor so a 0 m/s config cannot divide by zero.
    6. Early abort if distance exceeds max_plausible_distance_m. That is
       a sanity check on the pose, not a navigation decision.
    7. Heading is diagnostic. A large initial heading error is exactly
       what turn-control (OmniSim physics) must handle; we log it and
       still dispatch unless the pose itself is unusable.

    Already-at-goal (distance ~ 0) is valid: timeout collapses to the
    settle window, and we do not abort.
    """
    notes = []
    goal = finite_xy(goal_xy)
    fallback = finite_xy(fallback_xy)
    observed = xy_from_observation(start_obs)
    current_yaw = yaw_from_observation(start_obs)

    if goal is None:
        return RouteBudget(
            distance_m=0.0,
            timeout_s=min_timeout_s,
            source="invalid_goal",
            start_xy=observed or fallback,
            goal_xy=None,
            abort_reason="non-finite or malformed goal waypoint",
            current_yaw_rad=current_yaw,
            notes=("abort: goal is not a finite (x, y)",),
        )

    if observed is not None:
        start = observed
        source = "observed"
    elif fallback is not None:
        start = fallback
        source = "fallback_spawn"
        notes.append("observed start pose unusable; billed from configured spawn")
    else:
        return RouteBudget(
            distance_m=0.0,
            timeout_s=min_timeout_s,
            source="invalid_start",
            start_xy=None,
            goal_xy=goal,
            abort_reason="no finite start pose and no finite spawn fallback",
            current_yaw_rad=current_yaw,
            notes=("abort: cannot form a route origin",),
        )

    spawn_drift_m = None
    if observed is not None and fallback is not None:
        spawn_drift_m = euclidean_m(fallback, observed)
        if spawn_drift_m > 1e-6:
            notes.append(
                f"observed start is {spawn_drift_m:.3f} m from configured spawn"
            )

    distance_m = euclidean_m(start, goal)
    desired_yaw = heading_to_goal_rad(start, goal)
    yaw_err = heading_error_rad(current_yaw, desired_yaw)

    if distance_m > max_plausible_distance_m:
        notes.append(
            f"abort: distance {distance_m:.3f} m exceeds "
            f"{max_plausible_distance_m:.3f} m plausibility cap"
        )
        return RouteBudget(
            distance_m=distance_m,
            timeout_s=min_timeout_s,
            source=source,
            start_xy=start,
            goal_xy=goal,
            abort_reason=(
                f"implausible route distance {distance_m:.3f} m "
                f"(cap {max_plausible_distance_m:.3f} m)"
            ),
            heading_to_goal_rad=desired_yaw,
            current_yaw_rad=current_yaw,
            heading_error_rad=yaw_err,
            spawn_drift_m=spawn_drift_m,
            notes=tuple(notes),
        )

    speed = max(float(cruise_speed_mps), MIN_CRUISE_SPEED_MPS)
    settle = max(float(settle_timeout_s), 0.0)
    derived = distance_m / speed + settle
    timeout_s = min(max(float(min_timeout_s), derived), float(max_timeout_s))
    if timeout_s == float(max_timeout_s) and derived > float(max_timeout_s):
        notes.append(f"timeout capped at {max_timeout_s:.1f} s")

    if desired_yaw is None:
        notes.append("start coincides with goal; wait is settle-only")
    elif yaw_err is not None and abs(yaw_err) > DEFAULT_YAW_TOLERANCE_RAD:
        notes.append(
            f"initial heading error {math.degrees(abs(yaw_err)):.1f} deg "
            "(turn-control is OmniSim's; adapter will still dispatch)"
        )

    return RouteBudget(
        distance_m=distance_m,
        timeout_s=timeout_s,
        source=source,
        start_xy=start,
        goal_xy=goal,
        heading_to_goal_rad=desired_yaw,
        current_yaw_rad=current_yaw,
        heading_error_rad=yaw_err,
        spawn_drift_m=spawn_drift_m,
        notes=tuple(notes),
    )


def pose_delta_m(before: Optional[Mapping[str, Any]], after: Optional[Mapping[str, Any]]) -> Optional[float]:
    """Metres travelled between two observations, or None if either is unusable."""
    a = xy_from_observation(before)
    b = xy_from_observation(after)
    if a is None or b is None:
        return None
    return euclidean_m(a, b)


def remaining_to_goal_m(obs: Optional[Mapping[str, Any]], goal_xy: Sequence[float]) -> Optional[float]:
    """Remaining planar metres from an observation to the goal."""
    start = xy_from_observation(obs)
    goal = finite_xy(goal_xy)
    if start is None or goal is None:
        return None
    return euclidean_m(start, goal)


def within_tolerance(value: Optional[float], window: float) -> Optional[bool]:
    """True if |value| <= window. None if value is missing (not a fail)."""
    if value is None:
        return None
    if not math.isfinite(value):
        return False
    return abs(value) <= window
