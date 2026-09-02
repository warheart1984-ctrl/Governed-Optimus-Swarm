"""Geometry-attribution logger and diagnostic for the OmniSim seam.

Why this module exists
----------------------
OmniLink (email 1 Sep 2026) asked for a synchronized, per-sample trace so
that measured world velocity can be compared with
``[vx * cos(yaw), vx * sin(yaw)]``. That comparison is meant to distinguish
command dispatch, bridge integration, and pose composition — not to patch
Husky turn-control, and not to flip the arrived/settled/timed_out gate.

The 9 fields on each timestamped sample:

  1. raw_world_root      world-frame root (x, y)
  2. bridge_x
  3. bridge_y
  4. bridge_yaw
  5. odometry_pose       robot odom (x, y[, yaw])
  6. cmd_vel_linear_x    commanded body linear.x
  7. cmd_vel_angular_z   commanded body angular.z
  8. world_dx_dt         pose-derived world vx
  9. world_dy_dt         pose-derived world vy

Honesty: missing telemetry stays None. This file never invents zeros, never
talks HTTP, and never claims a live Husky fix. ``--live`` stays off until
OmniLink sends a physics fix commit. The last live four-Husky result
(5.1111 m / 7.0711 m misses) remains governing.

Recovery hysteresis (adapter-side): a single complete sample with R ≥ 1.1
does **not** warrant ``recover_robot()``. ``diagnose_trace`` requires
``RECOVER_FAIL_STREAK_N`` (default 3) consecutive *complete, time-aligned*
failing samples. Incomplete or temporally misaligned ticks hold the streak
(neither increment nor reset). A clean complete sample resets it to 0.

Alongside R, each complete diagnosis records:

  * ``vector_residual_m_s`` — ||v_measured − v_expected||
  * ``heading_error_rad`` — wrapped atan2(v_measured) − atan2(v_expected),
    i.e. the OmniLink comparison of measured world velocity vs
    ``[vx * cos(yaw), vx * sin(yaw)]``. This is **not** yaw vs a commanded
    heading. Either near-zero vector leaves heading error None.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:
    from omnisim_seam.route_geometry import wrap_heading_rad
except ImportError:  # `python3 omnisim_seam/*.py` (script, not package)
    from route_geometry import wrap_heading_rad

# OmniLink synchronized-sample window: constituent stamps must land inside
# this many seconds of each other or the sample is fail-closed.
ALIGNMENT_DT_MAX_S = 0.010

# Recover the in-flight envelope when the measured/expected speed ratio
# meets or exceeds this. Double-frame (R ≈ √2) is above this line.
R_RECOVER_THRESHOLD = 1.1

# Consecutive complete, time-aligned failing samples required before the
# adapter calls recover_robot(). Configurable; Adapter threads this through.
RECOVER_FAIL_STREAK_N = 3

# Clean: R near 1 and the world-velocity vectors match.
R_CLEAN_LOW = 0.9
R_CLEAN_HIGH = 1.1  # R == 1.1 is recover territory, not clean.

DOUBLE_FRAME_R = math.sqrt(2.0)
DOUBLE_FRAME_R_ABS_TOL = 0.08
COMPONENT_MATCH_ABS = 0.05
ZERO_VEL_EPS = 1e-9
PRE_DRIFT_EPS = 1e-6

# The 9 diagnostic fields OmniLink asked for on each sample.
SAMPLE_FIELD_NAMES: Tuple[str, ...] = (
    "raw_world_root",
    "bridge_x",
    "bridge_y",
    "bridge_yaw",
    "odometry_pose",
    "cmd_vel_linear_x",
    "cmd_vel_angular_z",
    "world_dx_dt",
    "world_dy_dt",
)

XY = Tuple[float, float]
CmdVel = Tuple[Optional[float], Optional[float]]


class AttributionClass(str, Enum):
    """Fail-closed classification of one synchronized sample."""

    SAMPLE_INCOMPLETE = "sample_incomplete"
    TEMPORAL_MISALIGNED = "temporal_misaligned"
    CLEAN = "clean"
    DOUBLE_FRAME = "double_frame"
    INTEGRATION_TICK_RATE = "integration_tick_rate"
    ZERO_EXPECTED_VELOCITY = "zero_expected_velocity"
    EMPTY_TRACE = "empty_trace"


def _finite(value: Any) -> Optional[float]:
    """Coerce to a finite float, else None. Never substitutes 0."""
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _finite_xy(value: Any) -> Optional[XY]:
    """Accept [x, y], (x, y), or {x, y}. Missing/non-finite -> None."""
    if value is None:
        return None
    if isinstance(value, Mapping):
        x = _finite(value.get("x"))
        y = _finite(value.get("y"))
        if x is None or y is None:
            return None
        return (x, y)
    try:
        x = _finite(value[0])
        y = _finite(value[1])
    except (TypeError, IndexError, KeyError):
        return None
    if x is None or y is None:
        return None
    return (x, y)


def _finite_pose(value: Any) -> Optional[Tuple[float, ...]]:
    """Odometry pose as (x, y) or (x, y, yaw). Incomplete -> None."""
    if value is None:
        return None
    if isinstance(value, Mapping):
        if "pose" in value:
            xy = _finite_xy(value.get("pose"))
            if xy is None:
                return None
            yaw = _finite(value.get("yaw"))
            return xy if yaw is None else (xy[0], xy[1], yaw)
        xy = _finite_xy(value)
        if xy is None:
            return None
        yaw = _finite(value.get("yaw"))
        return xy if yaw is None else (xy[0], xy[1], yaw)
    xy = _finite_xy(value)
    if xy is None:
        return None
    try:
        yaw = _finite(value[2])
    except (TypeError, IndexError, KeyError):
        yaw = None
    return xy if yaw is None else (xy[0], xy[1], yaw)


def _hypot(x: float, y: float) -> float:
    return math.hypot(x, y)


def extract_cmd_vel(
    obs: Optional[Mapping[str, Any]] = None,
    dispatch: Optional[Mapping[str, Any]] = None,
) -> CmdVel:
    """Pull commanded body (linear.x, angular.z) if present.

    Looks at observe() / dispatch dicts. ROS Twist (linear.x / angular.z),
    flat linear/angular, v_linear/v_angular, and cmd_vel wrappers are
    accepted. Absent keys stay None — a missing command is not 0.0.
    """
    for source in (obs, dispatch):
        vx, wz = _cmd_vel_from_mapping(source)
        if vx is not None or wz is not None:
            return (vx, wz)
    return (None, None)


def _cmd_vel_from_mapping(data: Optional[Mapping[str, Any]]) -> CmdVel:
    if not data:
        return (None, None)

    wrapped = data.get("cmd_vel")
    if isinstance(wrapped, Mapping):
        vx, wz = _cmd_vel_from_mapping(wrapped)
        if vx is not None or wz is not None:
            return (vx, wz)

    vx = _finite(data.get("cmd_vel_linear_x"))
    wz = _finite(data.get("cmd_vel_angular_z"))
    if vx is None:
        vx = _finite(data.get("v_linear"))
    if wz is None:
        wz = _finite(data.get("v_angular"))
    if vx is None:
        vx = _twist_component(data.get("linear"), "x")
    if wz is None:
        wz = _twist_component(data.get("angular"), "z")
    if vx is None:
        vx = _finite(data.get("linear_x"))
    if wz is None:
        wz = _finite(data.get("angular_z"))
    return (vx, wz)


def _twist_component(value: Any, key: str) -> Optional[float]:
    if isinstance(value, Mapping):
        return _finite(value.get(key))
    return _finite(value)


def extract_raw_world_root(obs: Optional[Mapping[str, Any]]) -> Optional[XY]:
    """World-frame root if the payload actually carries one."""
    if not obs:
        return None
    for key in ("raw_world_root", "world_root"):
        xy = _finite_xy(obs.get(key))
        if xy is not None:
            return xy
    x = _finite(obs.get("world_x"))
    y = _finite(obs.get("world_y"))
    if x is not None and y is not None:
        return (x, y)
    return None


def extract_odometry_pose(obs: Optional[Mapping[str, Any]]) -> Optional[Tuple[float, ...]]:
    if not obs:
        return None
    for key in ("odometry_pose", "odometry", "odom"):
        pose = _finite_pose(obs.get(key))
        if pose is not None:
            return pose
    return None


def extract_bridge_xy_yaw(
    obs: Optional[Mapping[str, Any]],
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """Bridge pose from observe() without inventing a missing axis.

    Prefer explicit x/y/yaw. Fall back to pose[0]/pose[1] only when those
    slots are actually present (a 1-element pose does not become y=0).
    """
    if not obs:
        return (None, None, None)
    x = _finite(obs.get("x"))
    y = _finite(obs.get("y"))
    yaw = _finite(obs.get("yaw"))
    pose = obs.get("pose")
    if x is None and pose is not None:
        try:
            x = _finite(pose[0])
        except (TypeError, IndexError, KeyError):
            x = None
    if y is None and pose is not None:
        try:
            y = _finite(pose[1])
        except (TypeError, IndexError, KeyError):
            y = None
    return (x, y, yaw)


def world_velocity_from_poses(
    prev_xy: Optional[Sequence[float]],
    curr_xy: Optional[Sequence[float]],
    dt_s: Optional[float],
) -> Tuple[Optional[float], Optional[float]]:
    """Pose-derived world (dx/dt, dy/dt). None when dt or a pose is unusable."""
    prev = _finite_xy(prev_xy)
    curr = _finite_xy(curr_xy)
    dt = _finite(dt_s)
    if prev is None or curr is None or dt is None or dt <= 0.0:
        return (None, None)
    return ((curr[0] - prev[0]) / dt, (curr[1] - prev[1]) / dt)


def _dt_between(
    prev_sim: Optional[float],
    curr_sim: Optional[float],
    prev_wall: Optional[float],
    curr_wall: Optional[float],
) -> Optional[float]:
    """Prefer sim_time delta; wall clock is the fallback. dt <= 0 is unusable."""
    if prev_sim is not None and curr_sim is not None:
        dt = curr_sim - prev_sim
        if math.isfinite(dt) and dt > 0.0:
            return dt
    if prev_wall is not None and curr_wall is not None:
        dt = curr_wall - prev_wall
        if math.isfinite(dt) and dt > 0.0:
            return dt
    return None


@dataclass
class AttributionSample:
    """One timestamped, synchronized geometry-attribution sample."""

    raw_world_root: Optional[XY] = None
    bridge_x: Optional[float] = None
    bridge_y: Optional[float] = None
    bridge_yaw: Optional[float] = None
    odometry_pose: Optional[Tuple[float, ...]] = None
    cmd_vel_linear_x: Optional[float] = None
    cmd_vel_angular_z: Optional[float] = None
    world_dx_dt: Optional[float] = None
    world_dy_dt: Optional[float] = None
    wall_time: Optional[float] = None
    wall_time_iso: Optional[str] = None
    sim_time: Optional[float] = None
    robot_id: Optional[str] = None
    source_stamps: Tuple[float, ...] = field(default_factory=tuple)

    def missing_fields(self) -> List[str]:
        missing = []
        for name in SAMPLE_FIELD_NAMES:
            value = getattr(self, name)
            if value is None:
                missing.append(name)
                continue
            if name in ("raw_world_root", "odometry_pose"):
                if not isinstance(value, tuple) or len(value) < 2:
                    missing.append(name)
                    continue
                if any(_finite(part) is None for part in value[:2]):
                    missing.append(name)
                continue
            if _finite(value) is None:
                missing.append(name)
        return missing

    def to_json(self) -> Dict[str, Any]:
        return {
            "raw_world_root": (
                list(self.raw_world_root) if self.raw_world_root is not None else None
            ),
            "bridge_x": self.bridge_x,
            "bridge_y": self.bridge_y,
            "bridge_yaw": self.bridge_yaw,
            "odometry_pose": (
                list(self.odometry_pose) if self.odometry_pose is not None else None
            ),
            "cmd_vel_linear_x": self.cmd_vel_linear_x,
            "cmd_vel_angular_z": self.cmd_vel_angular_z,
            "world_dx_dt": self.world_dx_dt,
            "world_dy_dt": self.world_dy_dt,
            "wall_time": self.wall_time,
            "wall_time_iso": self.wall_time_iso,
            "sim_time": self.sim_time,
            "robot_id": self.robot_id,
            "source_stamps": list(self.source_stamps),
        }


def attribution_sample_from_observation(
    obs: Optional[Mapping[str, Any]],
    prev_sample: Optional[AttributionSample] = None,
    dispatch: Optional[Mapping[str, Any]] = None,
    wall_time: Optional[float] = None,
    extra_stamps: Sequence[float] = (),
    robot_id: Optional[str] = None,
) -> AttributionSample:
    """Build one sample from observe()/dispatch. Missing keys stay None."""
    obs = obs or {}
    stamp = _finite(wall_time)
    if stamp is None:
        stamp = _finite(obs.get("wall_time"))
    iso = obs.get("wall_time_iso") if isinstance(obs.get("wall_time_iso"), str) else None
    if iso is None and stamp is not None:
        iso = datetime.fromtimestamp(stamp, timezone.utc).isoformat()

    bridge_x, bridge_y, bridge_yaw = extract_bridge_xy_yaw(obs)
    vx, wz = extract_cmd_vel(obs, dispatch)
    sim_time = _finite(obs.get("sim_time"))

    stamps: List[float] = []
    if stamp is not None:
        stamps.append(stamp)
    for extra in extra_stamps:
        extra_f = _finite(extra)
        if extra_f is not None:
            stamps.append(extra_f)
    for key in ("odom_time", "cmd_vel_time", "world_root_time"):
        extra_f = _finite(obs.get(key))
        if extra_f is not None:
            stamps.append(extra_f)

    dx_dt, dy_dt = (None, None)
    if prev_sample is not None:
        dt = _dt_between(
            prev_sample.sim_time, sim_time, prev_sample.wall_time, stamp,
        )
        dx_dt, dy_dt = world_velocity_from_poses(
            (prev_sample.bridge_x, prev_sample.bridge_y),
            (bridge_x, bridge_y),
            dt,
        )

    return AttributionSample(
        raw_world_root=extract_raw_world_root(obs),
        bridge_x=bridge_x,
        bridge_y=bridge_y,
        bridge_yaw=bridge_yaw,
        odometry_pose=extract_odometry_pose(obs),
        cmd_vel_linear_x=vx,
        cmd_vel_angular_z=wz,
        world_dx_dt=dx_dt,
        world_dy_dt=dy_dt,
        wall_time=stamp,
        wall_time_iso=iso,
        sim_time=sim_time,
        robot_id=robot_id,
        source_stamps=tuple(stamps),
    )


class AttributionLogger:
    """Per-tick logger. Pure: the caller feeds observe()/dispatch dicts."""

    def __init__(self, robot_id: Optional[str] = None):
        self.robot_id = robot_id
        self.samples: List[AttributionSample] = []

    def record(
        self,
        obs: Optional[Mapping[str, Any]],
        dispatch: Optional[Mapping[str, Any]] = None,
        wall_time: Optional[float] = None,
        extra_stamps: Sequence[float] = (),
    ) -> AttributionSample:
        prev = self.samples[-1] if self.samples else None
        sample = attribution_sample_from_observation(
            obs,
            prev_sample=prev,
            dispatch=dispatch,
            wall_time=wall_time,
            extra_stamps=extra_stamps,
            robot_id=self.robot_id,
        )
        self.samples.append(sample)
        return sample

    def to_json(self) -> List[Dict[str, Any]]:
        return [sample.to_json() for sample in self.samples]


@dataclass
class AttributionDiagnosis:
    """Structured result of steps 1–5 for one sample."""

    ok: bool
    classification: AttributionClass
    reasons: List[str]
    r_ratio: Optional[float] = None
    delta_pre: Optional[Tuple[Optional[float], Optional[float]]] = None
    delta_odom: Optional[Tuple[Optional[float], ...]] = None
    v_expected: Optional[Tuple[Optional[float], Optional[float]]] = None
    v_measured: Optional[Tuple[Optional[float], Optional[float]]] = None
    recover_recommended: bool = False
    stamp_spread_s: Optional[float] = None
    pre_dispatch_drift: bool = False
    missing_fields: Tuple[str, ...] = field(default_factory=tuple)
    vector_residual_m_s: Optional[float] = None
    heading_error_rad: Optional[float] = None

    def to_json(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "passed": self.ok,
            "classification": self.classification.value,
            "reasons": list(self.reasons),
            "r_ratio": self.r_ratio,
            "delta_pre": list(self.delta_pre) if self.delta_pre is not None else None,
            "delta_odom": list(self.delta_odom) if self.delta_odom is not None else None,
            "v_expected": (
                list(self.v_expected) if self.v_expected is not None else None
            ),
            "v_measured": (
                list(self.v_measured) if self.v_measured is not None else None
            ),
            "recover_recommended": self.recover_recommended,
            "stamp_spread_s": self.stamp_spread_s,
            "pre_dispatch_drift": self.pre_dispatch_drift,
            "missing_fields": list(self.missing_fields),
            "vector_residual_m_s": self.vector_residual_m_s,
            "heading_error_rad": self.heading_error_rad,
        }


@dataclass
class TraceDiagnosis:
    """Aggregate of diagnose_sample over a synchronized trace."""

    ok: bool
    classification: AttributionClass
    reasons: List[str]
    samples: List[AttributionDiagnosis] = field(default_factory=list)
    r_ratio: Optional[float] = None
    recover_recommended: bool = False
    fail_streak: int = 0
    fail_streak_peak: int = 0
    recover_fail_streak_n: int = RECOVER_FAIL_STREAK_N

    def to_json(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "passed": self.ok,
            "classification": self.classification.value,
            "reasons": list(self.reasons),
            "r_ratio": self.r_ratio,
            "recover_recommended": self.recover_recommended,
            "fail_streak": self.fail_streak,
            "fail_streak_peak": self.fail_streak_peak,
            "recover_fail_streak_n": self.recover_fail_streak_n,
            "samples": [sample.to_json() for sample in self.samples],
        }


def _stamp_spread_s(sample: AttributionSample) -> Optional[float]:
    stamps = [s for s in sample.source_stamps if math.isfinite(s)]
    if sample.wall_time is not None and math.isfinite(sample.wall_time):
        if sample.wall_time not in stamps:
            stamps.append(sample.wall_time)
    if len(stamps) < 2:
        return 0.0 if stamps else None
    return max(stamps) - min(stamps)


def _delta_pre(
    sample: AttributionSample,
) -> Optional[Tuple[Optional[float], Optional[float]]]:
    if sample.raw_world_root is None:
        return None
    if sample.bridge_x is None or sample.bridge_y is None:
        return None
    return (
        sample.bridge_x - sample.raw_world_root[0],
        sample.bridge_y - sample.raw_world_root[1],
    )


def _delta_odom(sample: AttributionSample) -> Optional[Tuple[Optional[float], ...]]:
    odom = sample.odometry_pose
    if odom is None or len(odom) < 2:
        return None
    if sample.bridge_x is None or sample.bridge_y is None:
        return None
    dx = sample.bridge_x - odom[0]
    dy = sample.bridge_y - odom[1]
    if len(odom) >= 3 and sample.bridge_yaw is not None:
        wrapped = wrap_heading_rad(sample.bridge_yaw - odom[2])
        return (dx, dy, wrapped)
    return (dx, dy)


def _vectors_match(
    measured: Tuple[float, float],
    expected: Tuple[float, float],
    abs_tol: float = COMPONENT_MATCH_ABS,
) -> bool:
    return (
        math.isclose(measured[0], expected[0], rel_tol=0.05, abs_tol=abs_tol)
        and math.isclose(measured[1], expected[1], rel_tol=0.05, abs_tol=abs_tol)
    )


def expected_world_velocity(
    cmd_vel_linear_x: float, bridge_yaw: float,
) -> Tuple[float, float]:
    """v_world_expected = [vx * cos(θ), vx * sin(θ)] from body linear.x."""
    return (
        cmd_vel_linear_x * math.cos(bridge_yaw),
        cmd_vel_linear_x * math.sin(bridge_yaw),
    )


def vector_residual_m_s(
    v_measured: Optional[Tuple[Optional[float], Optional[float]]],
    v_expected: Optional[Tuple[Optional[float], Optional[float]]],
) -> Optional[float]:
    """||v_measured − v_expected||. None when either vector is incomplete."""
    if v_measured is None or v_expected is None:
        return None
    mx, my = v_measured
    ex, ey = v_expected
    if mx is None or my is None or ex is None or ey is None:
        return None
    if not all(math.isfinite(part) for part in (mx, my, ex, ey)):
        return None
    return _hypot(mx - ex, my - ey)


def heading_error_from_world_velocities(
    v_measured: Optional[Tuple[Optional[float], Optional[float]]],
    v_expected: Optional[Tuple[Optional[float], Optional[float]]],
) -> Optional[float]:
    """Wrapped atan2(v_measured) − atan2(v_expected).

    This is the OmniLink comparison: direction of measured world velocity
    versus expected ``[vx * cos(yaw), vx * sin(yaw)]``. It is not yaw
    versus a commanded heading. Near-zero either vector → None (direction
    undefined; not invented).
    """
    if v_measured is None or v_expected is None:
        return None
    mx, my = v_measured
    ex, ey = v_expected
    if mx is None or my is None or ex is None or ey is None:
        return None
    if not all(math.isfinite(part) for part in (mx, my, ex, ey)):
        return None
    if _hypot(mx, my) <= ZERO_VEL_EPS or _hypot(ex, ey) <= ZERO_VEL_EPS:
        return None
    return wrap_heading_rad(math.atan2(my, mx) - math.atan2(ey, ex))


def _kinematics_residuals(
    v_measured: Optional[Tuple[Optional[float], Optional[float]]],
    v_expected: Optional[Tuple[Optional[float], Optional[float]]],
) -> Tuple[Optional[float], Optional[float]]:
    return (
        vector_residual_m_s(v_measured, v_expected),
        heading_error_from_world_velocities(v_measured, v_expected),
    )


def _is_unusable_for_streak(item: AttributionDiagnosis) -> bool:
    """Incomplete / misaligned ticks hold the fail streak (no increment, no reset)."""
    return item.classification in (
        AttributionClass.SAMPLE_INCOMPLETE,
        AttributionClass.TEMPORAL_MISALIGNED,
    )


def _is_failing_complete(item: AttributionDiagnosis) -> bool:
    """Complete + time-aligned + recover_recommended / R ≥ 1.1.

    double_frame and integration_tick_rate with R ≥ 1.1 already set
    recover_recommended on diagnose_sample.
    """
    if _is_unusable_for_streak(item):
        return False
    if item.recover_recommended:
        return True
    if item.classification in (
        AttributionClass.DOUBLE_FRAME,
        AttributionClass.INTEGRATION_TICK_RATE,
    ) and item.r_ratio is not None and item.r_ratio >= R_RECOVER_THRESHOLD:
        return True
    return False


def _normalize_fail_streak_n(value: Any) -> int:
    try:
        n = int(value)
    except (TypeError, ValueError):
        return RECOVER_FAIL_STREAK_N
    return n if n >= 1 else 1


def diagnose_sample(sample: AttributionSample) -> AttributionDiagnosis:
    """Steps 1–5 on one synchronized sample. Pure; no Adapter, no HTTP."""
    reasons: List[str] = []
    missing = tuple(sample.missing_fields())
    spread = _stamp_spread_s(sample)
    delta_pre = _delta_pre(sample)
    delta_odom = _delta_odom(sample)
    pre_drift = False
    if delta_pre is not None and delta_pre[0] is not None and delta_pre[1] is not None:
        pre_drift = _hypot(delta_pre[0], delta_pre[1]) > PRE_DRIFT_EPS
        if pre_drift:
            reasons.append(
                f"pre-dispatch world-to-bridge drift "
                f"dx={delta_pre[0]!r} dy={delta_pre[1]!r}"
            )

    if missing:
        reasons.append(
            "fail-closed: sample incomplete; missing "
            + ", ".join(missing)
        )
        return AttributionDiagnosis(
            ok=False,
            classification=AttributionClass.SAMPLE_INCOMPLETE,
            reasons=reasons,
            r_ratio=None,
            delta_pre=delta_pre,
            delta_odom=delta_odom,
            v_expected=None,
            v_measured=None,
            recover_recommended=False,
            stamp_spread_s=spread,
            pre_dispatch_drift=pre_drift,
            missing_fields=missing,
        )

    if spread is None:
        reasons.append("fail-closed: no timestamp on sample; cannot prove Δt ≤ 10 ms")
        return AttributionDiagnosis(
            ok=False,
            classification=AttributionClass.TEMPORAL_MISALIGNED,
            reasons=reasons,
            r_ratio=None,
            delta_pre=delta_pre,
            delta_odom=delta_odom,
            recover_recommended=False,
            stamp_spread_s=spread,
            pre_dispatch_drift=pre_drift,
        )

    if spread > ALIGNMENT_DT_MAX_S:
        reasons.append(
            f"fail-closed: stamp spread {spread:.4f} s exceeds "
            f"{ALIGNMENT_DT_MAX_S * 1000:.0f} ms alignment window"
        )
        return AttributionDiagnosis(
            ok=False,
            classification=AttributionClass.TEMPORAL_MISALIGNED,
            reasons=reasons,
            r_ratio=None,
            delta_pre=delta_pre,
            delta_odom=delta_odom,
            recover_recommended=False,
            stamp_spread_s=spread,
            pre_dispatch_drift=pre_drift,
        )

    # Completeness already guaranteed these are finite numbers.
    vx = sample.cmd_vel_linear_x
    yaw = sample.bridge_yaw
    dx_dt = sample.world_dx_dt
    dy_dt = sample.world_dy_dt
    assert vx is not None and yaw is not None
    assert dx_dt is not None and dy_dt is not None

    v_expected = expected_world_velocity(vx, yaw)
    v_measured = (dx_dt, dy_dt)
    residual, heading_err = _kinematics_residuals(v_measured, v_expected)
    exp_norm = _hypot(v_expected[0], v_expected[1])
    meas_norm = _hypot(v_measured[0], v_measured[1])

    if delta_odom is not None:
        reasons.append(f"odometry vs bridge δ_odom={delta_odom!r}")

    if exp_norm <= ZERO_VEL_EPS:
        if meas_norm <= ZERO_VEL_EPS:
            reasons.append("commanded and measured world velocity both ~0; R undefined")
            return AttributionDiagnosis(
                ok=True,
                classification=AttributionClass.CLEAN,
                reasons=reasons,
                r_ratio=None,
                delta_pre=delta_pre,
                delta_odom=delta_odom,
                v_expected=v_expected,
                v_measured=v_measured,
                recover_recommended=False,
                stamp_spread_s=spread,
                pre_dispatch_drift=pre_drift,
                vector_residual_m_s=residual,
                heading_error_rad=heading_err,
            )
        reasons.append(
            "commanded world velocity ~0 but measured motion is non-zero; "
            "R undefined (not invented)"
        )
        return AttributionDiagnosis(
            ok=False,
            classification=AttributionClass.ZERO_EXPECTED_VELOCITY,
            reasons=reasons,
            r_ratio=None,
            delta_pre=delta_pre,
            delta_odom=delta_odom,
            v_expected=v_expected,
            v_measured=v_measured,
            recover_recommended=False,
            stamp_spread_s=spread,
            pre_dispatch_drift=pre_drift,
            vector_residual_m_s=residual,
            heading_error_rad=heading_err,
        )

    r_ratio = meas_norm / exp_norm
    recover = r_ratio >= R_RECOVER_THRESHOLD
    vectors_match = _vectors_match(v_measured, v_expected)
    dx_matches_vx = math.isclose(dx_dt, vx, rel_tol=0.05, abs_tol=COMPONENT_MATCH_ABS)
    dy_matches_vx = math.isclose(dy_dt, vx, rel_tol=0.05, abs_tol=COMPONENT_MATCH_ABS)
    r_is_sqrt2 = math.isclose(
        r_ratio, DOUBLE_FRAME_R, rel_tol=0.0, abs_tol=DOUBLE_FRAME_R_ABS_TOL,
    )

    if r_is_sqrt2 and dx_matches_vx and dy_matches_vx:
        reasons.append(
            f"R={r_ratio:.4f} ≈ √2 and dx≈vx, dy≈vx -> double-frame / missing rotation"
        )
        return AttributionDiagnosis(
            ok=False,
            classification=AttributionClass.DOUBLE_FRAME,
            reasons=reasons,
            r_ratio=r_ratio,
            delta_pre=delta_pre,
            delta_odom=delta_odom,
            v_expected=v_expected,
            v_measured=v_measured,
            recover_recommended=recover,
            stamp_spread_s=spread,
            pre_dispatch_drift=pre_drift,
            vector_residual_m_s=residual,
            heading_error_rad=heading_err,
        )

    if R_CLEAN_LOW < r_ratio < R_CLEAN_HIGH and vectors_match:
        reasons.append(
            f"R={r_ratio:.4f} ≈ 1.0 and v_measured matches "
            f"[vx cos θ, vx sin θ] -> clean"
        )
        return AttributionDiagnosis(
            ok=True,
            classification=AttributionClass.CLEAN,
            reasons=reasons,
            r_ratio=r_ratio,
            delta_pre=delta_pre,
            delta_odom=delta_odom,
            v_expected=v_expected,
            v_measured=v_measured,
            recover_recommended=False,
            stamp_spread_s=spread,
            pre_dispatch_drift=pre_drift,
            vector_residual_m_s=residual,
            heading_error_rad=heading_err,
        )

    reasons.append(
        f"R={r_ratio:.4f} is not a clean match to [vx cos θ, vx sin θ] "
        f"(v_meas={v_measured!r} v_exp={v_expected!r}) -> integration/tick-rate"
    )
    return AttributionDiagnosis(
        ok=False,
        classification=AttributionClass.INTEGRATION_TICK_RATE,
        reasons=reasons,
        r_ratio=r_ratio,
        delta_pre=delta_pre,
        delta_odom=delta_odom,
        v_expected=v_expected,
        v_measured=v_measured,
        recover_recommended=recover,
        stamp_spread_s=spread,
        pre_dispatch_drift=pre_drift,
        vector_residual_m_s=residual,
        heading_error_rad=heading_err,
    )


def diagnose_trace(
    samples: Iterable[AttributionSample],
    recover_fail_streak_n: int = RECOVER_FAIL_STREAK_N,
) -> TraceDiagnosis:
    """Run diagnose_sample on each timestamped sample. Pure; no recover().

    Recovery is warranted only after ``recover_fail_streak_n`` consecutive
    complete, time-aligned failing samples (default 3). Incomplete and
    temporally misaligned ticks hold the streak; a clean complete sample
    resets it. Per-sample residual / heading error live on each sample
    diagnosis.
    """
    diagnosed = [diagnose_sample(sample) for sample in samples]
    n = _normalize_fail_streak_n(recover_fail_streak_n)
    if not diagnosed:
        return TraceDiagnosis(
            ok=False,
            classification=AttributionClass.EMPTY_TRACE,
            reasons=["fail-closed: empty attribution trace"],
            samples=[],
            r_ratio=None,
            recover_recommended=False,
            fail_streak=0,
            fail_streak_peak=0,
            recover_fail_streak_n=n,
        )

    streak = 0
    peak = 0
    for item in diagnosed:
        if _is_unusable_for_streak(item):
            continue
        if _is_failing_complete(item):
            streak += 1
            if streak > peak:
                peak = streak
            continue
        if item.classification is AttributionClass.CLEAN:
            streak = 0
            continue
        # Other complete classifications (e.g. zero_expected_velocity)
        # are neither failing nor clean: hold the streak.

    recover = peak >= n
    recover_ratios = [
        item.r_ratio
        for item in diagnosed
        if _is_failing_complete(item) and item.r_ratio is not None
    ]
    ratios = [item.r_ratio for item in diagnosed if item.r_ratio is not None]
    r_ratio = recover_ratios[-1] if recover_ratios else (ratios[-1] if ratios else None)

    reasons: List[str] = []
    for index, item in enumerate(diagnosed):
        reasons.extend(f"sample[{index}] {note}" for note in item.reasons)

    # A two-observe Adapter run always has an incomplete first sample
    # (no previous pose → no dx/dt). Do not let that structural None
    # hide a later kinematics classification. Misaligned stamps still
    # fail the whole trace — those are not structural.
    misaligned = [
        item for item in diagnosed
        if item.classification is AttributionClass.TEMPORAL_MISALIGNED
    ]
    scored = [
        item for item in diagnosed
        if item.classification
        not in (
            AttributionClass.SAMPLE_INCOMPLETE,
            AttributionClass.TEMPORAL_MISALIGNED,
        )
    ]
    priority = {
        AttributionClass.SAMPLE_INCOMPLETE: 0,
        AttributionClass.TEMPORAL_MISALIGNED: 1,
        AttributionClass.EMPTY_TRACE: 2,
        AttributionClass.DOUBLE_FRAME: 3,
        AttributionClass.INTEGRATION_TICK_RATE: 4,
        AttributionClass.ZERO_EXPECTED_VELOCITY: 5,
        AttributionClass.CLEAN: 6,
    }
    if misaligned:
        worst = min(misaligned, key=lambda item: priority[item.classification])
        ok = False
    elif scored:
        worst = min(scored, key=lambda item: priority[item.classification])
        ok = all(item.ok for item in scored) and not recover
    else:
        worst = min(diagnosed, key=lambda item: priority[item.classification])
        ok = False
    if recover:
        reasons.append(
            f"fail_streak_peak={peak} >= {n} consecutive complete failing "
            "samples; adapter recover_robot() is indicated for a live envelope"
        )
    return TraceDiagnosis(
        ok=ok,
        classification=worst.classification,
        reasons=reasons,
        samples=diagnosed,
        r_ratio=r_ratio,
        recover_recommended=recover,
        fail_streak=streak,
        fail_streak_peak=peak,
        recover_fail_streak_n=n,
    )
