"""
Minimal architectural-seam bridge: Governed-Optimus-Swarm -> OmniSim.

Scope (explicitly NOT Nav2 / MoveIt / GPU batching)
---------------------------------------------------
Exactly the five-item surface the OmniLink team asked for:

  1. Two mobile robots + two named waypoints.
  2. Governed-Optimus-Swarm emits ONE assignment object:
        robot_id, task_id, target waypoint, policy decision.
  3. A thin adapter turns that into one OmniSim mobile command.
  4. OmniSim returns ONE measured pose + a terminal outcome.
  5. An evidence stream records:
        assignment, request_id, dispatch, observation,
        outcome, and one rejected duplicate attempt.

It tests the seam, and says nothing about a full Nav2/MoveIt
integration having been verified.

HONESTY NOTE -- what the reference swarm ACTUALLY emits
------------------------------------------------------
Before designing a second vocabulary, read the real output of the
reference implementation. Governed-Optimus-Swarm does NOT currently
emit an "assignment" object with robot_id/task_id/target/policy. What
it emits per robot per tick is a *log entry*:

    # governed_swarm.py
    {"robot": id, "role": role,
     "from": old_pos, "to": new_pos,
     "task_before": old_task, "task_after": new_task,
     "state_hash": sha256}

    # swarm_core.py (mining baseline)
    {"drone": id, "from": old_pos, "to": new_pos,
     "carrying": bool, "state_hash": sha256}

There is no task_id, no request_id, no discrete waypoint -- movement
is stepwise grid-cell advancement toward a nearest-viable target.

So this file adds a THIN ENVELOPE (Assignment) on top of the observed
log entry -- it does not change the governance vocabulary, it only
promotes the per-tick decision into the discrete assignment shape the
OmniSim adapter needs. The envelope is the bridge, not a rewrite.

Frame mapping
-------------
  swarm grid (int, int)  ->  OmniSim world (metres, ENU, +X east +Y north)
  This adapter holds a WaypointTable: named waypoint -> (x, y) metres.
  Robot spawn/origin also come from the table. The HTTP wait for a drive
  is billed from the *observed* starting pose, not from spawn; spawn is
  fallback only when that pose is unusable. See omnisim_seam/ADAPTER.md.
"""

from __future__ import annotations

import json
import logging
import sys
import time
import urllib.request
import urllib.error
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    from omnisim_seam.route_geometry import (
        DEFAULT_ARRIVAL_TOLERANCE_M,
        DEFAULT_YAW_TOLERANCE_RAD,
        heading_error_rad,
        heading_to_goal_rad,
        plan_route_budget,
        pose_delta_m,
        remaining_to_goal_m,
        within_tolerance,
        wrap_heading_rad,
        xy_from_observation,
    )
    from omnisim_seam.geometry_attribution import (
        AttributionLogger,
        diagnose_trace,
    )
except ImportError:  # `python3 omnisim_seam/__init__.py` (script, not package)
    from route_geometry import (
        DEFAULT_ARRIVAL_TOLERANCE_M,
        DEFAULT_YAW_TOLERANCE_RAD,
        heading_error_rad,
        heading_to_goal_rad,
        plan_route_budget,
        pose_delta_m,
        remaining_to_goal_m,
        within_tolerance,
        wrap_heading_rad,
        xy_from_observation,
    )
    from geometry_attribution import (
        AttributionLogger,
        diagnose_trace,
    )

try:
    from control_plane import (
        AdmissionRecord,
        OperationLedger,
        RunManifest,
        admit_assignment,
        dispatch_call_kwargs,
        roles_from_robots,
        verify_rebind_authorization,
    )
except ImportError:
    _SEAM_ROOT = str(Path(__file__).resolve().parent.parent)
    if _SEAM_ROOT not in sys.path:
        sys.path.insert(0, _SEAM_ROOT)
    from control_plane import (
        AdmissionRecord,
        OperationLedger,
        RunManifest,
        admit_assignment,
        dispatch_call_kwargs,
        roles_from_robots,
        verify_rebind_authorization,
    )

log = logging.getLogger("omnisim_seam")
log.addHandler(logging.NullHandler())


# ======================================================================== #
# 1. Worlds / waypoints / robots                                          #
# ======================================================================== #
@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float          # metres
    y: float          # metres
    yaw: float = 0.0  # radians (facing; unused for pose-only)


# Two mobile robots, two named waypoints. Grid coords are illustrative; the
# metres here are what OmniSim actually commands.
SPAWN_LOCATIONS: Dict[str, Waypoint] = {
    "robot_a": Waypoint("spawn_a", x=-4.0, y=-2.0),
    "robot_b": Waypoint("spawn_b", x=-4.0, y=+2.0),
}
WAYPOINTS: Dict[str, Waypoint] = {
    "wp_x": Waypoint("wp_x", x=+4.0, y=-2.0),
    "wp_y": Waypoint("wp_y", x=+4.0, y=+2.0),
}

# The endpoints exposed by the shipped OmniSim Husky world.  These are
# example defaults only: deployments can replace them with --robot-endpoint.
DEFAULT_ROBOT_PORTS: Dict[str, int] = {
    "husky_ne": 8865,
    "husky_nw": 8866,
    "husky_se": 8867,
    "husky_sw": 8868,
}


def default_spawn_locations(robot_ids: List[str]) -> Dict[str, Waypoint]:
    """Give configured robots deterministic spawn metadata for the seam."""
    return {
        robot_id: Waypoint(f"spawn_{robot_id}", x=-4.0, y=-2.0 + 2.0 * index)
        for index, robot_id in enumerate(robot_ids)
    }


# ======================================================================== #
# 2. The ONE assignment object (envelope over the swarm's real log entry)  #
# ======================================================================== #
@dataclass
class Assignment:
    robot_id: str
    task_id: str            # e.g. "go_to_wp_x"
    target: str             # named waypoint ("wp_x")
    policy: str             # policy decision: "navigate" | "hold" | "abstain"
    request_id: str         # unique idempotency key for the whole attempt
    # provenance: the exact swarm line that produced this assignment
    source_log: Dict[str, Any] = field(default_factory=dict)
    _is_duplicate: bool = field(default=False, repr=False)

    def to_json(self) -> Dict[str, Any]:
        return asdict(self)


class AssignmentEnvelope:
    """Promotes a Governed-Optimus-Swarm log entry into an Assignment.

    Deterministic: given a swarm 'to' grid cell, resolves the nearest named
    waypoint and stamps a request_id. Idempotency: assigning the same
    (robot, target) twice in a row reuses the prior request_id iff the
    previous attempt has NOT completed -- so a duplicate is *rejected*, not
    double-dispatched (this is the "one rejected duplicate" in the evidence
    stream).
    """

    _seq: int = 0

    def __init__(self, waypoints: Dict[str, Waypoint] = WAYPOINTS,
                 enabled_policy: str = "navigate"):
        self.waypoints = waypoints
        self.enabled_policy = enabled_policy
        # in-flight open requests: robot_id -> {"request_id", "target"}
        self._open: Dict[str, Dict[str, Any]] = {}

    @classmethod
    def _next_request_id(cls, robot_id: str) -> str:
        cls._seq += 1
        return f"req-{robot_id}-{cls._seq:05d}"

    def from_swarm_log(self, log_entry: Dict[str, Any]) -> Optional[Assignment]:
        """Build ONE Assignment from one governed log line.

        Reads only fields the reference swarm provably emits: robot/drone id,
        and the 'to' grid cell. Everything else is derived here.
        """
        robot_id = log_entry.get("robot") or log_entry.get("drone")
        target_cell = log_entry.get("to")
        if robot_id is None or target_cell is None:
            return None

        target_name = self._nearest_waypoint(target_cell)
        task_id = "hold" if target_name is None else f"go_to_{target_name}"
        policy = "abstain" if target_name is None else self.enabled_policy

        open_req = self._open.get(str(robot_id))
        if open_req is not None and open_req["target"] == target_name:
            # Same robot + same unfinished target -> reuse id (duplicate).
            request_id = open_req["request_id"]
            # Stamp that this is a duplicate so the adapter can reject it.
            return Assignment(
                robot_id=str(robot_id),
                task_id=task_id,
                target=target_name or "",
                policy=policy,
                request_id=request_id,
                source_log=log_entry,
                _is_duplicate=True,
            )
        # Otherwise a fresh, distinct request.
        request_id = self._next_request_id(str(robot_id))
        self._open[str(robot_id)] = {"request_id": request_id, "target": target_name}
        return Assignment(
            robot_id=str(robot_id),
            task_id=task_id,
            target=target_name or "",
            policy=policy,
            request_id=request_id,
            source_log=log_entry,
        )

    def mark_terminal(self, robot_id: str, request_id: str) -> None:
        """Free the in-flight slot after a TERMINAL outcome (completed OR
        transport_error). A terminal attempt may not be re-issued as a
        duplicate; a later genuine re-issue gets a fresh id."""
        open_req = self._open.get(robot_id)
        if open_req is not None and open_req["request_id"] == request_id:
            del self._open[robot_id]

    def is_duplicate(self, assignment: Assignment) -> bool:
        return assignment._is_duplicate

    def _nearest_waypoint(self, grid_cell) -> Optional[str]:
        """grid (x,y) -> nearest named waypoint by Euclidean metres.

        The reference swarm emits INT grid cells; we map to the nearest of
        the (few) named waypoints. With only 2 waypoints this is cheap and
        deterministic. Pass through known waypoint names untouched.
        """
        if isinstance(grid_cell, str) and grid_cell in self.waypoints:
            return grid_cell
        try:
            gx, gy = float(grid_cell[0]), float(grid_cell[1])
        except (TypeError, IndexError, ValueError):
            return None
        return min(
            self.waypoints,
            key=lambda n: (self.waypoints[n].x - gx) ** 2 + (self.waypoints[n].y - gy) ** 2,
        )


# ======================================================================== #
# 3. OmniSim robot bridge (thin adapter, mobile class)                     #
# ======================================================================== #
class OmniSimMobile:
    """One OmniSim mobile-robot bridge. Exposes only what the seam needs."""

    def __init__(self, robot_id: str, base_url: str,
                 spawn: Waypoint, waypoints: Dict[str, Waypoint] = WAYPOINTS,
                 timeout_s: float = 45.0, cruise_speed_mps: float = 0.20,
                 settle_timeout_s: float = 10.0):
        self.robot_id = robot_id
        self.base_url = base_url.rstrip("/")
        self.spawn = spawn
        self.waypoints = waypoints
        self.timeout_s = timeout_s
        self.cruise_speed_mps = cruise_speed_mps
        self.settle_timeout_s = settle_timeout_s
        self._used_operation_ids: set[str] = set()

    def _post(self, path: str, body: Dict[str, Any], timeout_s: Optional[float] = None) -> Dict[str, Any]:
        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout_s or self.timeout_s) as r:
                return json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            return {"ok": False, "http": e.code, "body": e.read().decode()}
        except Exception as e:
            return {"ok": False, "error": "transport", "message": str(e)}

    # -- dispatch: ONE command per assignment ------------------------------
    def dispatch(self, assignment: Assignment,
                 start_pose: Optional[Dict[str, Any]] = None,
                 operation_id: Optional[str] = None) -> Dict[str, Any]:
        """Turn ONE assignment into ONE OmniSim mobile command.

        Waypoint -> /drive_to_waypoint {x, y, wait}
        hold      -> /stop_robot
        abstain   -> /stop_robot (fail closed; no motion on stale/absent target)

        The HTTP wait is billed from the *observed* starting pose (OmniLink-
        validated). Configured spawn is a fallback only when that pose is
        unusable (NaN/Inf/missing). An implausible distance aborts before
        any `/drive_to_waypoint` POST. Heading error is logged, not used
        to veto: turn-control is OmniSim physics, not this adapter.

        A one-use operation_id binds this physical attempt. Replay of the
        same id is rejected with no HTTP.

        Returns the raw bridge reply (measured outcome) plus a `route`
        budget object for the evidence stream.
        """
        if operation_id:
            if operation_id in self._used_operation_ids:
                return {
                    "ok": False,
                    "error": "replayed_operation",
                    "operation_id": operation_id,
                    "arrived": False,
                    "settled": False,
                    "timed_out": False,
                }
            self._used_operation_ids.add(operation_id)

        if assignment.policy == "hold" or not assignment.target:
            return self._post("/stop_robot", {"robot_id": self.robot_id})

        wp = self.waypoints.get(assignment.target)
        if wp is None:
            return {"ok": False, "error": "unknown_waypoint", "target": assignment.target}
        if assignment.policy == "abstain":
            # Evidence insufficient -> fail closed, do not move.
            return self._post("/stop_robot", {"robot_id": self.robot_id})

        budget = plan_route_budget(
            start_obs=start_pose,
            goal_xy=(wp.x, wp.y),
            fallback_xy=(self.spawn.x, self.spawn.y),
            cruise_speed_mps=self.cruise_speed_mps,
            settle_timeout_s=self.settle_timeout_s,
            min_timeout_s=self.timeout_s,
        )
        log.info(
            "route robot=%s request_id=%s source=%s distance_m=%.3f "
            "timeout_s=%.1f spawn_drift_m=%s heading_error_rad=%s notes=%s",
            self.robot_id, assignment.request_id, budget.source,
            budget.distance_m, budget.timeout_s, budget.spawn_drift_m,
            budget.heading_error_rad, list(budget.notes),
        )
        if budget.abort_reason:
            log.warning(
                "early_abort robot=%s request_id=%s reason=%s",
                self.robot_id, assignment.request_id, budget.abort_reason,
            )
            return {
                "ok": False,
                "error": "aborted",
                "reason": budget.abort_reason,
                "arrived": False,
                "settled": False,
                "timed_out": False,
                "route": budget.to_json(),
            }

        reply = self._post("/drive_to_waypoint", {
            "robot_id": self.robot_id,
            "x": wp.x, "y": wp.y, "wait": True,
        }, timeout_s=budget.timeout_s)
        if isinstance(reply, dict):
            reply = dict(reply)
            reply["route"] = budget.to_json()
        return reply

    # -- observation: measured pose ----------------------------------------
    def observe(self) -> Dict[str, Any]:
        st = self._post("/get_robot_state", {"robot_id": self.robot_id})
        if "x" in st:
            out: Dict[str, Any] = {
                "pose": [st["x"], st.get("y", 0.0)],
                "yaw": st.get("yaw", 0.0),
                "sim_time": st.get("sim_time"),
                "mode": st.get("mode"),
            }
            # Pass through optional telemetry only when the bridge actually
            # sent it. Do not invent odom / cmd_vel / world-root zeros.
            for key in (
                "raw_world_root", "world_root", "world_x", "world_y",
                "odometry", "odom", "odometry_pose",
                "cmd_vel", "v_linear", "v_angular", "linear", "angular",
                "odom_time", "cmd_vel_time", "world_root_time",
            ):
                if key in st and st[key] is not None:
                    out[key] = st[key]
            return out
        return {"pose": None, "yaw": None, "note": st.get("error")}


# ======================================================================== #
# 4. Terminal outcome + 5. evidence stream                                 #
# ======================================================================== #
def pose_snapshot(label: str, obs: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Timestamped pose record for the evidence stream.

    Captures wall-clock and sim time so a later reader can line this
    snapshot up with OmniSim logs without guessing.
    """
    obs = obs or {}
    raw_yaw = obs.get("yaw")
    try:
        yaw_f = float(raw_yaw) if raw_yaw is not None else None
    except (TypeError, ValueError):
        yaw_f = None
    return {
        "label": label,
        "wall_time": time.time(),
        "wall_time_iso": datetime.now(timezone.utc).isoformat(),
        "pose": obs.get("pose"),
        "yaw": raw_yaw,
        "yaw_wrapped_rad": wrap_heading_rad(yaw_f) if yaw_f is not None else None,
        "sim_time": obs.get("sim_time"),
        "mode": obs.get("mode"),
        "note": obs.get("note"),
    }


def evaluate_completion_gate(dispatch: Dict[str, Any]) -> Dict[str, Any]:
    """Explain the arrived / settled / timed_out contract in writing.

    Completion is recorded only when OmniSim reports arrived=true,
    settled=true, and timed_out=false. Pose tolerances sit next to this
    decision as diagnostics; they do not flip it. Turn-control failures
    are OmniSim physics, not an adapter veto.
    """
    arrived = dispatch.get("arrived")
    settled = dispatch.get("settled")
    timed_out = dispatch.get("timed_out")
    ok_flag = dispatch.get("ok")
    error = dispatch.get("error")

    reasons: List[str] = []
    if error == "transport":
        decision = "transport_error"
        reasons.append("dispatch.error == transport")
    elif error == "aborted":
        decision = "aborted"
        reasons.append(f"adapter early abort: {dispatch.get('reason')}")
    else:
        reasons.append(f"arrived is {arrived!r} (need True)")
        reasons.append(f"settled is {settled!r} (need True)")
        reasons.append(f"timed_out is {timed_out!r} (need False)")
        reasons.append(f"ok is {ok_flag!r} (must not be False)")
        gate_ok = (
            ok_flag is not False
            and arrived is True
            and settled is True
            and timed_out is False
        )
        decision = "completed" if gate_ok else "rejected"
        if gate_ok:
            reasons.append("all three OmniSim flags satisfied -> completed")
        else:
            reasons.append("OmniSim completion contract not met -> rejected")

    return {
        "arrived": arrived,
        "settled": settled,
        "timed_out": timed_out,
        "ok": ok_flag,
        "error": error,
        "decision": decision,
        "reasons": reasons,
        "note": (
            "Pose tolerances are diagnostic; they do not override this gate. "
            "Turn-control failures belong to OmniSim physics, not the adapter."
        ),
    }


def annotate_completion_geometry(
    gate: Dict[str, Any],
    dispatch: Dict[str, Any],
    remaining_after_m: Optional[float],
    heading_err_rad: Optional[float] = None,
) -> Dict[str, Any]:
    """Stamp geometry diagnostics without overriding OmniSim flags.

    OmniLink recommendation: if the bridge reports arrived/settled while
    the measured remaining distance is outside the arrival window, keep
    the transport flags as-is and emit explicit
    ``completion_conflict=true`` / ``geometry_consistent=false`` so a
    contradictory completion cannot be read as a clean one.
    """
    pose_ok = within_tolerance(remaining_after_m, DEFAULT_ARRIVAL_TOLERANCE_M)
    heading_ok = within_tolerance(heading_err_rad, DEFAULT_YAW_TOLERANCE_RAD)
    upstream_arrived_settled = (
        dispatch.get("arrived") is True and dispatch.get("settled") is True
    )
    # Missing remaining distance is not a conflict — we cannot judge.
    completion_conflict = bool(upstream_arrived_settled and pose_ok is False)

    gate["pose_within_arrival_tolerance"] = pose_ok
    gate["heading_within_yaw_tolerance"] = heading_ok
    gate["arrival_tolerance_m"] = DEFAULT_ARRIVAL_TOLERANCE_M
    gate["yaw_tolerance_rad"] = DEFAULT_YAW_TOLERANCE_RAD
    gate["geometry_consistent"] = pose_ok
    gate["completion_conflict"] = completion_conflict
    if completion_conflict:
        gate["reasons"].append(
            f"completion_conflict: OmniSim arrived={dispatch.get('arrived')!r} "
            f"settled={dispatch.get('settled')!r} but remaining "
            f"{remaining_after_m} m exceeds {DEFAULT_ARRIVAL_TOLERANCE_M} m "
            "(upstream flags preserved; geometry_consistent=false)"
        )
    return gate


def _route_evidence(
    assignment: Assignment,
    waypoints: Dict[str, Waypoint],
    before: Dict[str, Any],
    after: Dict[str, Any],
    dispatch: Dict[str, Any],
) -> tuple:
    """Route-distance deltas + heading diagnostics for one attempt."""
    route = dict(dispatch.get("route") or {})
    wp = waypoints.get(assignment.target)
    goal_xy = (wp.x, wp.y) if wp is not None else None
    remaining_before = remaining_to_goal_m(before, goal_xy) if goal_xy else None
    remaining_after = remaining_to_goal_m(after, goal_xy) if goal_xy else None
    travelled = pose_delta_m(before, after)
    route["remaining_before_m"] = remaining_before
    route["remaining_after_m"] = remaining_after
    route["travelled_m"] = travelled
    if remaining_before is not None and remaining_after is not None:
        # Negative delta means the robot closed distance to the goal.
        route["route_distance_delta_m"] = remaining_after - remaining_before
    start_xy = xy_from_observation(before)
    if start_xy is not None and goal_xy is not None:
        desired = heading_to_goal_rad(start_xy, goal_xy)
        current = before.get("yaw")
        try:
            current_f = float(current) if current is not None else None
        except (TypeError, ValueError):
            current_f = None
        route["heading_to_goal_rad"] = desired
        route["heading_error_rad"] = heading_error_rad(current_f, desired)
    return route, goal_xy


@dataclass
class EvidenceRecord:
    assignment: Dict[str, Any]
    request_id: str
    accepted: bool
    dispatch: Dict[str, Any]
    observation_before: Dict[str, Any]
    observation_after: Dict[str, Any]
    outcome: str                      # "completed" | "rejected_duplicate" | "rejected" | "transport_error" | "aborted" | "rejected_admission" | "unverified" | "replayed_operation"
    wall_time: float = field(default_factory=time.time)
    pose_snapshots: List[Dict[str, Any]] = field(default_factory=list)
    route: Dict[str, Any] = field(default_factory=dict)
    completion_gate: Dict[str, Any] = field(default_factory=dict)
    # Top-level copies so a reader scanning outcome=completed cannot miss
    # a contradictory arrival (OmniLink evidence-model recommendation).
    completion_conflict: bool = False
    geometry_consistent: Optional[bool] = None
    # Per-tick synchronized geometry-attribution samples (OmniLink 1 Sep 2026).
    # Empty when observe()/dispatch did not yield a trace; fields inside a
    # sample stay None rather than invented zeros.
    attribution_trace: List[Dict[str, Any]] = field(default_factory=list)
    attribution_diagnosis: Dict[str, Any] = field(default_factory=dict)
    admission: Dict[str, Any] = field(default_factory=dict)
    operation_id: Optional[str] = None

    def to_json(self) -> Dict[str, Any]:
        return asdict(self)


class Adapter:
    """Wires ONE governed log line through to one OmniSim command and an
    evidence record -- the whole seam in one object."""

    def __init__(
        self,
        robots: Dict[str, OmniSimMobile],
        waypoints: Dict[str, Waypoint] = WAYPOINTS,
        enabled_policy: str = "navigate",
        roles: Optional[Dict[str, str]] = None,
        verification_secret: Optional[str] = None,
    ):
        self.robots = robots
        self.envelope = AssignmentEnvelope(waypoints, enabled_policy)
        # The adapter owns the waypoint table.  This makes a custom table
        # authoritative for both envelope resolution and mobile dispatch.
        for robot in self.robots.values():
            if isinstance(robot, OmniSimMobile):
                robot.waypoints = waypoints
        self.evidence: List[EvidenceRecord] = []
        self.manifest = RunManifest.issue(
            roles_from_robots(robots, roles),
            verification_secret=verification_secret,
        )
        self.operations = OperationLedger()
        self._last_attribution: Dict[str, tuple] = {}

    def recover_robot(self, robot_id: str) -> EvidenceRecord:
        """Clear this robot's in-flight envelope slot and stamp an abort.

        Used when attribution R >= 1.1 on a real Adapter run: the kinematics
        envelope should not stay open. Safe if the slot is already empty —
        we still record recover-{id} so the attempt is auditable. This does
        not talk to OmniSim and does not claim to fix turn-control.
        """
        robot_id = str(robot_id)
        open_req = self.envelope._open.pop(robot_id, None)
        recover_id = f"recover-{robot_id}"
        prior_id = None if open_req is None else open_req.get("request_id")
        prior_target = "" if open_req is None else (open_req.get("target") or "")
        already_clear = open_req is None
        prior_trace, prior_diagnosis = self._last_attribution.get(robot_id, ([], {}))
        rec = EvidenceRecord(
            assignment={
                "robot_id": robot_id,
                "task_id": "recover",
                "target": prior_target,
                "policy": "hold",
                "request_id": recover_id,
                "recovered_request_id": prior_id,
            },
            request_id=recover_id,
            accepted=False,
            dispatch={
                "ok": False,
                "error": "aborted",
                "reason": recover_id,
                "arrived": False,
                "settled": False,
                "timed_out": False,
            },
            observation_before={},
            observation_after={},
            outcome="aborted",
            completion_gate={
                "decision": "aborted",
                "arrived": False,
                "settled": False,
                "timed_out": False,
                "reasons": [
                    f"recover_robot({robot_id}): envelope in-flight slot "
                    + (
                        f"cleared (was {prior_id})"
                        if not already_clear
                        else "already clear"
                    ),
                ],
                "completion_conflict": False,
                "geometry_consistent": None,
            },
            completion_conflict=False,
            geometry_consistent=None,
            attribution_trace=list(prior_trace),
            attribution_diagnosis=dict(prior_diagnosis),
        )
        self.evidence.append(rec)
        log.info(
            "recover_robot robot=%s request_id=%s prior=%s already_clear=%s",
            robot_id, recover_id, prior_id, already_clear,
        )
        return rec

    def _record_attribution(
        self,
        robot_id: str,
        before: Dict[str, Any],
        after: Dict[str, Any],
        dispatch: Dict[str, Any],
        before_snap: Dict[str, Any],
        after_snap: Dict[str, Any],
    ) -> tuple:
        """Build the synchronized trace OmniLink asked for from this attempt.

        Two observe() snapshots is what a blocking /drive_to_waypoint wait
        actually gives us. Mid-drive ticks are not invented. cmd_vel / odom /
        world-root stay None unless observe() or dispatch carried them.
        """
        logger = AttributionLogger(robot_id=robot_id)
        logger.record(
            before,
            wall_time=before_snap.get("wall_time"),
        )
        logger.record(
            after,
            dispatch=dispatch,
            wall_time=after_snap.get("wall_time"),
        )
        diagnosis = diagnose_trace(logger.samples)
        trace = logger.to_json()
        diagnosis_json = diagnosis.to_json()
        self._last_attribution[robot_id] = (trace, diagnosis_json)
        return trace, diagnosis_json, diagnosis.recover_recommended

    def admit(self, assignment: Assignment):
        """Bind the envelope assignment into a verified AdmissionRecord."""
        return admit_assignment(assignment, self.manifest)

    def rebind_role(
        self,
        robot_id: str,
        new_role: str,
        authorization: Any = None,
    ) -> EvidenceRecord:
        """Authorized role change: hashed ticket, new immutable manifest, receipt."""
        ok = verify_rebind_authorization(
            self.manifest, str(robot_id), new_role, authorization
        )
        if not ok or str(robot_id) not in self.manifest.roles:
            rec = EvidenceRecord(
                assignment={
                    "robot_id": robot_id,
                    "task_id": "rebind_role",
                    "target": "",
                    "policy": "hold",
                    "request_id": f"rebind-{robot_id}",
                    "new_role": new_role,
                },
                request_id=f"rebind-{robot_id}",
                accepted=False,
                dispatch={},
                observation_before={},
                observation_after={},
                outcome="rebind_rejected",
                completion_gate={
                    "decision": "rebind_rejected",
                    "reasons": ["unauthorized or unknown robot; manifest unchanged"],
                    "completion_conflict": False,
                    "geometry_consistent": None,
                },
                completion_conflict=False,
                geometry_consistent=None,
                attribution_trace=[],
            )
            self.evidence.append(rec)
            log.info("rebind_rejected robot=%s new_role=%s", robot_id, new_role)
            return rec
        self.manifest = self.manifest.with_role(str(robot_id), new_role)
        robot = self.robots.get(str(robot_id))
        if robot is not None and hasattr(robot, "role"):
            robot.role = new_role
        rec = EvidenceRecord(
            assignment={
                "robot_id": robot_id,
                "task_id": "rebind_role",
                "target": "",
                "policy": "hold",
                "request_id": f"rebind-{robot_id}",
                "new_role": new_role,
                "generation": self.manifest.generation,
            },
            request_id=f"rebind-{robot_id}",
            accepted=True,
            dispatch={},
            observation_before={},
            observation_after={},
            outcome="rebind_role",
            completion_gate={
                "decision": "rebind_role",
                "reasons": [
                    f"authorized rebind of {robot_id} -> {new_role} "
                    f"(generation {self.manifest.generation})"
                ],
                "completion_conflict": False,
                "geometry_consistent": None,
            },
            completion_conflict=False,
            geometry_consistent=None,
            attribution_trace=[],
            admission={"role": new_role, "generation": self.manifest.generation},
        )
        self.evidence.append(rec)
        log.info(
            "rebind_role robot=%s new_role=%s generation=%s",
            robot_id, new_role, self.manifest.generation,
        )
        return rec

    def dispatch_with_operation_id(
        self,
        robot: Any,
        assignment: Assignment,
        start_pose: Optional[Dict[str, Any]],
        operation_id: str,
    ) -> Dict[str, Any]:
        """Single-effect dispatch. Replay of operation_id never calls the robot."""
        if self.operations.already_used(operation_id) or not self.operations.consume(operation_id):
            return {
                "ok": False,
                "error": "replayed_operation",
                "operation_id": operation_id,
                "arrived": False,
                "settled": False,
                "timed_out": False,
            }
        kwargs = dispatch_call_kwargs(robot.dispatch, start_pose, operation_id)
        try:
            reply = robot.dispatch(assignment, **kwargs)
        except TypeError as exc:
            # Do not retry. Signature was resolved before this call.
            return {
                "ok": False,
                "error": "transport",
                "message": f"TypeError (no retry): {exc}",
                "operation_id": operation_id,
            }
        if isinstance(reply, dict):
            reply = dict(reply)
            reply.setdefault("operation_id", operation_id)
            return reply
        return {
            "ok": False,
            "error": "transport",
            "message": f"non-dict dispatch: {type(reply).__name__}",
            "operation_id": operation_id,
        }

    def _reject_closed(
        self,
        assignment: Assignment,
        reason: str,
        outcome: str,
        admission: Optional[AdmissionRecord] = None,
        free_slot: bool = True,
    ) -> EvidenceRecord:
        if free_slot:
            self.envelope.mark_terminal(assignment.robot_id, assignment.request_id)
        rec = EvidenceRecord(
            assignment=assignment.to_json(),
            request_id=assignment.request_id,
            accepted=False,
            dispatch={},
            observation_before={},
            observation_after={},
            outcome=outcome,
            completion_gate={
                "decision": outcome,
                "reasons": [reason],
                "completion_conflict": False,
                "geometry_consistent": None,
            },
            completion_conflict=False,
            geometry_consistent=None,
            attribution_trace=[],
            admission=admission.to_json() if admission is not None else {},
        )
        self.evidence.append(rec)
        log.info(
            "%s request_id=%s robot=%s reason=%s",
            outcome, assignment.request_id, assignment.robot_id, reason,
        )
        return rec

    def run(self, swarm_log_entry: Dict[str, Any]) -> Optional[EvidenceRecord]:
        """Process ONE swarm log line end to end through the seam."""
        assignment = self.envelope.from_swarm_log(swarm_log_entry)
        if assignment is None:
            return None
        robot = self.robots.get(assignment.robot_id)
        if robot is None:
            return None

        key = {
            "request_id": assignment.request_id,
            "robot_id": assignment.robot_id,
            "target": assignment.target,
            "task_id": assignment.task_id,
        }
        json.dumps(key)  # canonical/stable serializable for dedup auditing

        # Bound admission BEFORE any OmniSim HTTP or motion command.
        admission, admit_reason = self.admit(assignment)
        if admission is None:
            return self._reject_closed(
                assignment,
                admit_reason or "rejected_admission",
                "rejected_admission",
            )
        if not admission.verify(self.manifest.verification_secret):
            return self._reject_closed(
                assignment, "admission digest mismatch", "unverified",
                admission=admission,
            )

        # 5b. Reject a duplicate: same robot, same in-flight request id.
        if self.envelope.is_duplicate(assignment):
            rec = EvidenceRecord(
                assignment=assignment.to_json(),
                request_id=assignment.request_id,
                accepted=False,
                dispatch={},
                observation_before={},
                observation_after={},
                outcome="rejected_duplicate",
                completion_gate={
                    "decision": "rejected_duplicate",
                    "reasons": [
                        "same robot + same in-flight request_id; "
                        "no OmniSim command issued",
                    ],
                    "completion_conflict": False,
                    "geometry_consistent": None,
                },
                completion_conflict=False,
                geometry_consistent=None,
                admission=admission.to_json(),
            )
            self.evidence.append(rec)
            log.info(
                "rejected_duplicate request_id=%s robot=%s target=%s",
                assignment.request_id, assignment.robot_id, assignment.target,
            )
            return rec

        # Re-verify immediately before the first side effect.
        if not admission.verify(self.manifest.verification_secret):
            return self._reject_closed(
                assignment, "admission digest mismatch before dispatch",
                "unverified", admission=admission,
            )

        # 3/4. observe start -> dispatch (billed from that pose) -> observe after
        before = robot.observe()
        before_snap = pose_snapshot("before", before)
        log.info(
            "pose_snapshot request_id=%s label=before pose=%s yaw=%s "
            "yaw_wrapped_rad=%s sim_time=%s",
            assignment.request_id, before_snap.get("pose"),
            before_snap.get("yaw"), before_snap.get("yaw_wrapped_rad"),
            before_snap.get("sim_time"),
        )

        operation_id = self.operations.mint()
        dispatch = self.dispatch_with_operation_id(
            robot, assignment, before, operation_id,
        )
        if not isinstance(dispatch, dict):
            dispatch = {
                "ok": False,
                "error": "transport",
                "message": f"non-dict dispatch: {type(dispatch).__name__}",
                "operation_id": operation_id,
            }

        after = robot.observe()
        after_snap = pose_snapshot("after", after)
        log.info(
            "pose_snapshot request_id=%s label=after pose=%s yaw=%s "
            "yaw_wrapped_rad=%s sim_time=%s",
            assignment.request_id, after_snap.get("pose"),
            after_snap.get("yaw"), after_snap.get("yaw_wrapped_rad"),
            after_snap.get("sim_time"),
        )

        route, _goal_xy = _route_evidence(
            assignment, self.envelope.waypoints, before, after, dispatch,
        )
        gate = evaluate_completion_gate(dispatch)
        remaining_after = route.get("remaining_after_m")
        heading_err = route.get("heading_error_rad")
        annotate_completion_geometry(
            gate, dispatch, remaining_after, heading_err,
        )

        outcome = gate["decision"]
        ok = outcome == "completed"
        log.info(
            "completion_gate request_id=%s decision=%s "
            "completion_conflict=%s geometry_consistent=%s "
            "remaining_before_m=%s remaining_after_m=%s "
            "route_distance_delta_m=%s travelled_m=%s reasons=%s",
            assignment.request_id, outcome,
            gate.get("completion_conflict"), gate.get("geometry_consistent"),
            route.get("remaining_before_m"), remaining_after,
            route.get("route_distance_delta_m"), route.get("travelled_m"),
            gate["reasons"],
        )
        if route.get("spawn_drift_m"):
            log.info(
                "pose_drift request_id=%s spawn_drift_m=%s travelled_m=%s",
                assignment.request_id, route.get("spawn_drift_m"),
                route.get("travelled_m"),
            )

        trace, attribution_diagnosis, recover = self._record_attribution(
            assignment.robot_id, before, after, dispatch, before_snap, after_snap,
        )
        rec = EvidenceRecord(
            assignment=assignment.to_json(),
            request_id=assignment.request_id,
            accepted=ok,
            dispatch=dispatch,
            observation_before=before,
            observation_after=after,
            outcome=outcome,
            pose_snapshots=[before_snap, after_snap],
            route=route,
            completion_gate=gate,
            completion_conflict=bool(gate.get("completion_conflict")),
            geometry_consistent=gate.get("geometry_consistent"),
            attribution_trace=trace,
            attribution_diagnosis=attribution_diagnosis,
            admission=admission.to_json(),
            operation_id=operation_id,
        )
        self.evidence.append(rec)

        # Any terminal outcome frees the request id, so a later genuine
        # identical assignment is NOT a duplicate. Adapter-side aborts are
        # terminal: the pose was unusable, and retrying the same id would
        # hide the next real attempt.
        if outcome in ("completed", "transport_error", "aborted"):
            self.envelope.mark_terminal(assignment.robot_id, assignment.request_id)

        # R >= 1.1 on a real Adapter run: clear the envelope. Pure math
        # tests never construct an Adapter, so they cannot auto-recover.
        if recover:
            log.warning(
                "attribution_recover robot=%s request_id=%s R=%s class=%s",
                assignment.robot_id, assignment.request_id,
                attribution_diagnosis.get("r_ratio"),
                attribution_diagnosis.get("classification"),
            )
            self.recover_robot(assignment.robot_id)
        return rec

    def export_evidence(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump([r.to_json() for r in self.evidence], f, indent=2)


# ======================================================================== #
# Demo: two robots, two waypoints, one duplicate to be rejected            #
# ======================================================================== #
def main() -> None:
    import argparse
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    p = argparse.ArgumentParser(description="OmniSim <-> governed-swarm seam")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument(
        "--robot-endpoint", action="append", metavar="ID:PORT",
        help="Override example Husky endpoint(s); repeat as needed (e.g. robot_a:8765).",
    )
    p.add_argument("--timeout-s", type=float, default=45.0,
                   help="Minimum HTTP timeout; distance and settling can extend it.")
    p.add_argument("--cruise-speed-mps", type=float, default=0.20)
    p.add_argument("--settle-timeout-s", type=float, default=10.0)
    p.add_argument("--out", default="omnisim_seam_evidence.json")
    p.add_argument("--headless", action="store_true",
                   help="don't require live OmniSim; emit transport_error not dispatch")
    args = p.parse_args()

    robot_ports = dict(DEFAULT_ROBOT_PORTS)
    if args.robot_endpoint:
        robot_ports = {}
        for raw in args.robot_endpoint:
            robot_id, separator, raw_port = raw.partition(":")
            if not robot_id or not separator or not raw_port.isdigit():
                p.error(f"invalid --robot-endpoint {raw!r}; expected ID:PORT")
            robot_ports[robot_id] = int(raw_port)
    spawns = default_spawn_locations(list(robot_ports))
    robots = {
        robot_id: OmniSimMobile(
            robot_id, f"http://{args.host}:{port}", spawns[robot_id],
            WAYPOINTS, args.timeout_s, args.cruise_speed_mps, args.settle_timeout_s,
        )
        for robot_id, port in robot_ports.items()
    }
    adapter = Adapter(robots)

    # Serving two real swarm log lines (the honest shape Governed-Optimus-Swarm
    # emits in governed_swarm.py / swarm_core.py).
    swarm_lines = [
        {"robot": "husky_ne", "role": "carrier", "from": [0, 0], "to": [5, 0],
         "task_before": "idle", "task_after": "moving", "state_hash": "h1"},
        {"robot": "husky_nw", "role": "carrier", "from": [0, 1], "to": [5, 1],
         "task_before": "idle", "task_after": "moving", "state_hash": "h2"},
    ]

    # --- Dispatch each fresh assignment through the seam (against OmniSim). ---
    for line in swarm_lines:
        adapter.run(line)

    # --- Prove the duplicate-rejection invariant (one rejected duplicate). ---
    # Independent of the live sim's timing: model the case where robot_a's
    # request is IN FLIGHT (accepted, not yet terminal) and an identical
    # assignment arrives. The envelope's in-flight state is pure logic, so we
    # construct it deterministically here and record the rejection.
    dup_env = AssignmentEnvelope()
    first = dup_env.from_swarm_log(swarm_lines[0])          # opens req-...-00001
    assert first is not None and not dup_env.is_duplicate(first)
    duplicate = dup_env.from_swarm_log(swarm_lines[0])      # same robot+target
    assert duplicate is not None and dup_env.is_duplicate(duplicate)
    adapter.evidence.append(EvidenceRecord(
        assignment=duplicate.to_json(),
        request_id=duplicate.request_id,
        accepted=False,
        dispatch={},
        observation_before=robots["husky_ne"].observe(),
        observation_after=robots["husky_ne"].observe(),
        outcome="rejected_duplicate",
    ))

    adapter.export_evidence(args.out)

    print(json.dumps([r.to_json() for r in adapter.evidence], indent=2))

    outcomes = [r.outcome for r in adapter.evidence]
    print("\n=== seam summary ===")
    print("records:", len(outcomes))
    print("rejected_duplicate:", outcomes.count("rejected_duplicate"))
    print("transport_error:", outcomes.count("transport_error"))
    print("evidence file:", args.out)


if __name__ == "__main__":
    main()
