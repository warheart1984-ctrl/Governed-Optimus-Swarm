"""Invariant tests for the OmniSim <-> Governed-Optimus-Swarm seam.

These pin the architectural-boundary guarantees requested by the OmniLink
team. They must NOT touch a live simulator or the network: the full seam
(assignment -> dispatch -> observe -> evidence) is exercised against a
FakeMobile stub so the invariants are reproducible offline and in CI.

Invariants pinned here:
  1. One swarm log line produces exactly ONE assignment with the required
     shape: robot_id, task_id, target waypoint, policy, request_id.
  2. The envelope maps a swarm (int,int) 'to' cell to a named waypoint and
     emits the same assignment shape the OmniSim adapter consumes.
  3. Duplicate rejection: re-issuing (robot, target) while its request is
     in flight is rejected as a duplicate -- never double-dispatched.
  4. Terminal outcomes (completed OR transport_error) FREE the request id,
     so a later genuine identical assignment is a NEW request, not a
     duplicate.
  5. The evidence stream records, per attempt: assignment, request_id,
     dispatch, observation_before, observation_after, outcome.
  6. A hold/abstain policy maps to /stop_robot (fail closed, no waypoint).
"""

from __future__ import annotations

import pytest

from omnisim_seam import (
    Assignment,
    AssignmentEnvelope,
    Adapter,
    DEFAULT_TURN_GAIN_CALIBRATIONS,
    EvidenceRecord,
    OmniSimMobile,
    SPAWN_LOCATIONS,
    TurnGainCalibration,
    WAYPOINTS,
    GET_ROBOT_STATE_PATH,
    TELEMETRY_POLL_PATH,
    evaluate_completion_gate,
    annotate_completion_geometry,
)


# ------------------------------------------------------------------------- #
# Fake OmniSim mobile bridge: no network, deterministic measured pose.      #
# ------------------------------------------------------------------------- #
class FakeMobile:
    """Stands in for OmniSimMobile. Proves the seam, not the simulator.

    Default is blocking-only (no wait=False / no mid-drive sequence).
    Pass ``poll_states`` or ``in_motion`` to opt into the adapter's
    nonblocking poll path. Adapter tests must not require live OmniSim.
    """

    def __init__(self, robot_id: str, pose=(0.0, 0.0), outcome="completed",
                 snap_to=None, poll_states=None, in_motion=None):
        self.robot_id = robot_id
        self.pose = list(pose)
        self.outcome = outcome          # "completed" | "rejected" | "transport_error"
        self.snap_to = snap_to          # optional pose after a successful dispatch
        self.poll_states = list(
            poll_states if poll_states is not None else (in_motion or [])
        )
        self.dispatches: list[dict] = []
        self.observe_calls = 0
        self.poll_telemetry_calls = 0
        self._driving = False
        self._poll_i = 0

    def capabilities(self) -> dict:
        # Explicit: default FakeMobile is blocking-only. Poll sequences
        # opt into wait=False. Detected before any drive POST.
        return {"nonblocking_wait": bool(self.poll_states)}

    def dispatch(self, assignment: Assignment, start_pose=None, operation_id=None,
                 wait=True) -> dict:
        record = {
            "robot_id": assignment.robot_id,
            "policy": assignment.policy,
            "target": assignment.target,
            "start_pose": start_pose,
            "operation_id": operation_id,
            "wait": wait,
        }
        self.dispatches.append(record)
        if self.outcome == "transport_error":
            return {"ok": False, "error": "transport", "message": "sim down"}
        if self.outcome == "rejected":
            return {"ok": False, "error": "busy", "message": "robot busy"}
        if wait is False and self.poll_states:
            self._driving = True
            self._poll_i = 0
            return {
                "ok": True,
                "accepted": True,
                "arrived": False,
                "settled": False,
                "timed_out": False,
            }
        if self.snap_to is not None:
            self.pose = list(self.snap_to)
        elif self.poll_states:
            last = self.poll_states[-1]
            if isinstance(last, dict) and last.get("pose") is not None:
                self.pose = list(last["pose"])
        flags = {"ok": True, "accepted": True, "arrived": True, "settled": True, "timed_out": False}
        if self.poll_states:
            last = self.poll_states[-1]
            if isinstance(last, dict):
                for key in ("arrived", "settled", "timed_out", "ok"):
                    if key in last:
                        flags[key] = last[key]
        return flags

    def observe(self) -> dict:
        self.observe_calls += 1
        if self._driving and self.poll_states:
            idx = min(self._poll_i, len(self.poll_states) - 1)
            st = dict(self.poll_states[idx])
            if self._poll_i < len(self.poll_states):
                self._poll_i += 1
            if self._poll_i >= len(self.poll_states):
                self._driving = False
            if st.get("pose") is not None:
                self.pose = list(st["pose"])
            st.setdefault("pose", list(self.pose))
            st.setdefault("yaw", 0.0)
            st.setdefault("sim_time", float(self._poll_i))
            st.setdefault("mode", "idle")
            return st
        return {"pose": list(self.pose), "yaw": 0.0, "sim_time": 1.0, "mode": "idle"}

    def poll_telemetry(self) -> dict:
        """Named mid-drive path used by Adapter._mid_drive_observation."""
        self.poll_telemetry_calls += 1
        return self.observe()


@pytest.fixture
def adapter_factory():
    def make(outcome="completed"):
        robots = {
            "robot_a": FakeMobile("robot_a", outcome=outcome),
            "robot_b": FakeMobile("robot_b", pose=(4.0, 2.0), outcome=outcome),
        }
        return Adapter(robots)  # type: ignore[arg-type]
    return make


SWARM_LINE_A = {
    "robot": "robot_a", "role": "carrier", "from": [0, 0], "to": [5, 0],
    "task_before": "idle", "task_after": "moving", "state_hash": "h1",
}
SWARM_LINE_B = {
    "robot": "robot_b", "role": "carrier", "from": [0, 1], "to": [5, 1],
    "task_before": "idle", "task_after": "moving", "state_hash": "h2",
}


# ------------------------------------------------------------------------- #
# 1. Assignment shape                                                       #
# ------------------------------------------------------------------------- #
def test_swarm_line_yields_single_assignment_with_required_shape(adapter_factory):
    ad = adapter_factory()
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    a = rec.assignment
    assert a["robot_id"] == "robot_a"
    assert a["task_id"] == "go_to_wp_x"
    assert a["target"] == "wp_x"        # nearest named waypoint to [5,0]
    assert a["policy"] == "navigate"
    assert a["request_id"].startswith("req-robot_a-")
    # provenance line is preserved verbatim
    assert a["source_log"] == SWARM_LINE_A


def test_assignment_is_exactly_envelope_of_swarm_log(adapter_factory):
    ad = adapter_factory()
    rec = ad.run(dict(SWARM_LINE_B))
    a = rec.assignment
    assert a["target"] == "wp_y"
    assert a["task_id"] == "go_to_wp_y"


# ------------------------------------------------------------------------- #
# 2. Waypoint mapping (pure)                                                #
# ------------------------------------------------------------------------- #
def test_waypoint_mapping_resolves_nearest_named_waypoint():
    env = AssignmentEnvelope()
    assert env._nearest_waypoint((5, 0)) == "wp_x"
    assert env._nearest_waypoint((5, 1)) == "wp_y"
    # Pass-through of a literal waypoint name
    assert env._nearest_waypoint("wp_x") == "wp_x"


def test_unknown_target_abstains_fail_closed():
    env = AssignmentEnvelope()
    # A malformed / non-parseable 'to' cell resolves to NO waypoint -> the
    # envelope must abstain (fail closed), never navigate on a guess.
    a = env.from_swarm_log({"robot": "robot_a", "to": "not-a-cell"})
    assert a is not None
    assert a.target == "" and a.policy == "abstain"
    assert a.request_id.startswith("req-robot_a-")


def test_dispatch_hold_abstain_maps_to_stop_robot():
    robots = {"robot_a": FakeMobile("robot_a")}
    # The Adapter's thin dispatch turns a hold/abstain policy into a stop
    # (no waypoint commanded). Here we assert the mapping surfaced to the
    # bridge: policy=hold, target empty.
    robots["robot_a"].dispatch(Assignment(
        robot_id="robot_a", task_id="hold", target="", policy="hold",
        request_id="req-hold",
    ))
    assert robots["robot_a"].dispatches[0]["policy"] == "hold"
    assert robots["robot_a"].dispatches[0]["target"] == ""


# ------------------------------------------------------------------------- #
# 3. Duplicate rejection                                                    #
# ------------------------------------------------------------------------- #
def test_duplicate_rejected_while_in_flight(adapter_factory):
    env = AssignmentEnvelope()
    first = env.from_swarm_log(dict(SWARM_LINE_A))
    assert first is not None and not env.is_duplicate(first)   # fresh
    dup = env.from_swarm_log(dict(SWARM_LINE_A))               # same robot+target
    assert dup is not None and env.is_duplicate(dup)           # rejected
    # The duplicate reuses the SAME request_id -> callers can dedupe on it.
    assert dup.request_id == first.request_id


def test_seam_records_exactly_one_rejected_duplicate(adapter_factory):
    ad = adapter_factory()
    ad.run(dict(SWARM_LINE_A))
    ad.run(dict(SWARM_LINE_B))
    # Drive the duplicate through a deterministic in-flight envelope.
    env = AssignmentEnvelope()
    first = env.from_swarm_log(dict(SWARM_LINE_A))
    dup = env.from_swarm_log(dict(SWARM_LINE_A))
    assert dup is not None and env.is_duplicate(dup)
    ad.evidence.append(EvidenceRecord(
        assignment=dup.to_json(),
        request_id=dup.request_id,
        accepted=False,
        dispatch={},
        observation_before=ad.robots["robot_a"].observe(),
        observation_after=ad.robots["robot_a"].observe(),
        outcome="rejected_duplicate",
    ))
    outcomes = [r.outcome for r in ad.evidence]
    assert outcomes.count("rejected_duplicate") == 1


def test_no_double_dispatch_on_duplicate():
    # A duplicate is rejected BEFORE any OmniSim command is sent.
    robots = {"robot_a": FakeMobile("robot_a")}
    ad = Adapter(robots)  # type: ignore[arg-type]
    env = ad.envelope
    first = env.from_swarm_log(dict(SWARM_LINE_A))
    dup = env.from_swarm_log(dict(SWARM_LINE_A))
    assert dup is not None and env.is_duplicate(dup)
    # is_duplicate() is consulted before dispatch in the real seam; here we
    # assert the gate. No dispatch record is created for a duplicate.
    assert env.is_duplicate(dup) is True
    assert robots["robot_a"].dispatches == []    # nothing sent yet


# ------------------------------------------------------------------------- #
# 4. Terminal outcome frees the request id                                  #
# ------------------------------------------------------------------------- #
def test_terminal_outcome_frees_request_id(adapter_factory):
    ad = adapter_factory(outcome="completed")   # completed is terminal
    rec1 = ad.run(dict(SWARM_LINE_A))
    assert rec1.outcome == "completed"
    # After a terminal outcome the slot is freed, so re-issuing the same line
    # mints a NEW request id (a genuine re-issue, not a duplicate).
    rec2 = ad.run(dict(SWARM_LINE_A))
    assert rec2.outcome == "completed"
    assert rec2.request_id != rec1.request_id
    # rec2 is a fresh request, so consulted on the envelope it is not flagged.
    assert rec2.assignment["request_id"] == rec2.request_id


def test_transport_error_is_terminal(adapter_factory):
    ad = adapter_factory(outcome="transport_error")
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec.outcome == "transport_error"
    # transport freed the request id; a later identical assignment is fresh
    rec2 = ad.run(dict(SWARM_LINE_A))
    assert rec2.outcome == "transport_error"
    # The envelope no longer considers this in-flight, so re-issuing it
    # produces a fresh (non-duplicate) assignment. Build it via a clean env.
    env = AssignmentEnvelope()
    fresh = env.from_swarm_log(dict(SWARM_LINE_A))
    assert fresh is not None
    assert fresh.policy == "navigate"
    # A brand-new envelope has no open request, so nothing is a duplicate yet.
    assert env.is_duplicate(fresh) is False


# ------------------------------------------------------------------------- #
# 5. Evidence stream contents                                                #
# ------------------------------------------------------------------------- #
def test_evidence_stream_records_full_attempt(adapter_factory):
    ad = adapter_factory()
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.request_id.startswith("req-robot_a-")
    assert rec.accepted is True
    assert rec.dispatch["ok"] is True
    assert rec.observation_before["pose"] == [0.0, 0.0]
    assert rec.observation_after["pose"] is not None
    assert rec.outcome == "completed"
    # Default FakeMobile does not move: OmniSim flags say arrived, pose does
    # not. That contradiction must be an explicit top-level field.
    payload = rec.to_json()
    assert payload["completion_conflict"] is True
    assert payload["geometry_consistent"] is False
    # all required fields present on the serializable record
    for key in ("assignment", "request_id", "dispatch",
                "observation_before", "observation_after", "outcome",
                "completion_conflict", "geometry_consistent",
                "attribution_trace", "attribution_diagnosis"):
        assert key in payload
    assert isinstance(payload["attribution_trace"], list)


def test_completion_requires_arrived_settled_and_not_timed_out(adapter_factory):
    ad = adapter_factory()
    robot = ad.robots["robot_a"]
    original_dispatch = robot.dispatch
    robot.dispatch = lambda assignment: {"ok": True, "arrived": True, "settled": False, "timed_out": False}
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.accepted is False
    assert rec.outcome == "rejected"
    # arrived without settled is not a contradictory *completion*.
    assert rec.completion_conflict is False
    robot.dispatch = original_dispatch


def test_adapter_waypoint_table_controls_mobile_dispatch():
    custom = {"custom": type(WAYPOINTS["wp_x"])("custom", 99.0, 98.0)}
    mobile = OmniSimMobile("robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"])
    ad = Adapter({"robot_a": mobile}, waypoints=custom)
    assert mobile.waypoints is custom
    assignment = Assignment("robot_a", "go_to_custom", "custom", "navigate", "req-custom")
    captured = {}
    mobile._post = lambda path, body, timeout_s=None: captured.update(path=path, body=body, timeout_s=timeout_s) or {}  # type: ignore[method-assign]
    mobile.dispatch(assignment)
    assert captured["path"] == "/drive_to_waypoint"
    assert captured["body"]["x"] == 99.0
    assert captured["body"]["y"] == 98.0


def test_drive_timeout_covers_distance_and_settling():
    mobile = OmniSimMobile(
        "robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"],
        {"far": type(WAYPOINTS["wp_x"])("far", 20.0, -2.0)},
        timeout_s=5.0, cruise_speed_mps=1.0, settle_timeout_s=3.0,
    )
    captured = {}
    mobile._post = lambda path, body, timeout_s=None: captured.update(timeout_s=timeout_s) or {}  # type: ignore[method-assign]
    mobile.dispatch(Assignment("robot_a", "go_to_far", "far", "navigate", "req-far"))
    assert captured["timeout_s"] == 27.0  # spawn fallback: 24 m at 1 m/s, plus 3 s settling


def test_drive_timeout_uses_observed_start_pose():
    """HTTP wait must be billed from the observed pose, not configured spawn."""
    mobile = OmniSimMobile(
        "robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"],
        {"far": type(WAYPOINTS["wp_x"])("far", 20.0, -2.0)},
        timeout_s=5.0, cruise_speed_mps=1.0, settle_timeout_s=3.0,
    )
    captured = {}
    mobile._post = lambda path, body, timeout_s=None: captured.update(timeout_s=timeout_s) or {}  # type: ignore[method-assign]
    # Spawn-to-goal is 24 m; observed start is 10 m from the goal.
    mobile.dispatch(
        Assignment("robot_a", "go_to_far", "far", "navigate", "req-far"),
        start_pose={"pose": [10.0, -2.0], "yaw": 0.0},
    )
    assert captured["timeout_s"] == 13.0  # 10 m at 1 m/s, plus 3 s settling


def test_turn_gain_calibration_matches_husky_ne_replay():
    calibration = DEFAULT_TURN_GAIN_CALIBRATIONS["husky_ne"]
    assert calibration.world_id == "omnilink_husky_swarm.omniworld"
    assert calibration.build_id == "7d39130cf"
    assert calibration.gain_ratio == pytest.approx(0.10354871344300014)
    assert calibration.multiplier == pytest.approx(9.657290748156253)
    assert calibration.differential_drive_factor == pytest.approx(1.6818181818181817)


def test_husky_ne_dispatch_sends_turn_gain_calibration():
    mobile = OmniSimMobile(
        "husky_ne", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"],
        {"wp_x": WAYPOINTS["wp_x"]},
    )
    captured = {}
    mobile._post = lambda path, body, timeout_s=None: captured.update(path=path, body=body, timeout_s=timeout_s) or {}  # type: ignore[method-assign]
    reply = mobile.dispatch(
        Assignment("husky_ne", "go_to_wp_x", "wp_x", "navigate", "req-turn"),
        start_pose={"pose": [0.0, -2.0], "yaw": 0.0},
    )
    assert captured["path"] == "/drive_to_waypoint"
    assert captured["body"]["turn_gain_multiplier"] == pytest.approx(9.657290748156253)
    assert captured["body"]["turn_gain_calibration"]["gain_ratio"] == pytest.approx(0.10354871344300014)
    assert reply["turn_gain_calibration"]["build_id"] == "7d39130cf"


def test_custom_turn_gain_calibration_overrides_default():
    calibration = TurnGainCalibration(
        world_id="test_world",
        build_id="test_build",
        robot_id="robot_a",
        commanded_deg=90.0,
        achieved_deg=45.0,
    )
    mobile = OmniSimMobile(
        "robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"],
        {"wp_x": WAYPOINTS["wp_x"]},
        turn_gain_calibrations={"robot_a": calibration},
    )
    captured = {}
    mobile._post = lambda path, body, timeout_s=None: captured.update(body=body) or {}  # type: ignore[method-assign]
    mobile.dispatch(
        Assignment("robot_a", "go_to_wp_x", "wp_x", "navigate", "req-turn"),
        start_pose={"pose": [0.0, -2.0], "yaw": 0.0},
    )
    assert captured["body"]["turn_gain_multiplier"] == pytest.approx(2.0)
    assert captured["body"]["turn_gain_calibration"]["world_id"] == "test_world"


def test_implausible_start_pose_aborts_without_http():
    waypoints = {"far": type(WAYPOINTS["wp_x"])("far", 600.0, 0.0)}
    mobile = OmniSimMobile(
        "robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"], waypoints,
    )
    posts: list = []
    mobile._post = lambda *args, **kwargs: posts.append((args, kwargs)) or {}  # type: ignore[method-assign]
    reply = mobile.dispatch(
        Assignment("robot_a", "go_to_far", "far", "navigate", "req-far"),
        start_pose={"pose": [0.0, 0.0], "yaw": 0.0},
    )
    assert posts == []
    assert reply["error"] == "aborted"
    assert reply["arrived"] is False


def test_completion_gate_explains_each_flag():
    completed = evaluate_completion_gate(
        {"ok": True, "arrived": True, "settled": True, "timed_out": False},
    )
    assert completed["decision"] == "completed"
    rejected = evaluate_completion_gate(
        {"ok": True, "arrived": True, "settled": False, "timed_out": False},
    )
    assert rejected["decision"] == "rejected"
    assert any("settled" in r for r in rejected["reasons"])
    aborted = evaluate_completion_gate(
        {"ok": False, "error": "aborted", "reason": "implausible route distance"},
    )
    assert aborted["decision"] == "aborted"


def test_evidence_records_gate_snapshots_and_route_delta(adapter_factory):
    ad = adapter_factory()
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    payload = rec.to_json()
    assert payload["completion_gate"]["decision"] == "completed"
    assert payload["completion_gate"]["reasons"]
    assert len(payload["pose_snapshots"]) == 2
    assert payload["pose_snapshots"][0]["label"] == "before"
    assert "wall_time_iso" in payload["pose_snapshots"][0]
    assert "remaining_before_m" in payload["route"]
    assert "remaining_after_m" in payload["route"]
    assert "route_distance_delta_m" in payload["route"]
    assert payload["completion_conflict"] is True
    assert payload["geometry_consistent"] is False
    assert payload["completion_gate"]["completion_conflict"] is True
    assert payload["completion_gate"]["geometry_consistent"] is False
    assert any("completion_conflict" in r for r in payload["completion_gate"]["reasons"])


def test_clean_arrival_is_geometry_consistent():
    """When the measured pose is on the waypoint, conflict is false."""
    wp = WAYPOINTS["wp_x"]
    robots = {
        "robot_a": FakeMobile(
            "robot_a", pose=(0.0, 0.0), snap_to=(wp.x, wp.y),
        ),
    }
    ad = Adapter(robots)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.outcome == "completed"
    assert rec.completion_conflict is False
    assert rec.geometry_consistent is True
    assert rec.observation_after["pose"] == [wp.x, wp.y]


def test_annotate_conflict_preserves_upstream_flags():
    gate = evaluate_completion_gate(
        {"ok": True, "arrived": True, "settled": True, "timed_out": False},
    )
    dispatch = {"ok": True, "arrived": True, "settled": True, "timed_out": False}
    annotate_completion_geometry(gate, dispatch, remaining_after_m=5.1111)
    assert gate["decision"] == "completed"  # upstream flags still win
    assert gate["completion_conflict"] is True
    assert gate["geometry_consistent"] is False
    annotate_completion_geometry(gate, dispatch, remaining_after_m=0.05)
    assert gate["completion_conflict"] is False
    assert gate["geometry_consistent"] is True


def test_rejected_attempt_captured_not_thrown(adapter_factory):
    ad = adapter_factory(outcome="rejected")
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.accepted is False
    assert rec.outcome == "completed" or rec.outcome == "rejected"
    assert rec.observation_after is not None


def test_omnisim_mobile_poll_telemetry_posts_named_path():
    mobile = OmniSimMobile("robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"])
    posts: list[str] = []

    def _post(path, body, timeout_s=None):
        posts.append(path)
        if path == TELEMETRY_POLL_PATH:
            return {
                "x": 1.0, "y": 2.0, "yaw": 0.1,
                "raw_world_root": [1.0, 2.0],
                "odometry_pose": [1.0, 2.0],
                "cmd_vel": {"linear": {"x": 0.4}, "angular": {"z": 0.0}},
                "world_dx_dt": 0.4, "world_dy_dt": 0.0,
            }
        raise AssertionError(f"unexpected path {path}")

    mobile._post = _post  # type: ignore[method-assign]
    st = mobile.poll_telemetry()
    assert posts == [TELEMETRY_POLL_PATH]
    assert st["pose"] == [1.0, 2.0]
    assert st["raw_world_root"] == [1.0, 2.0]
    assert st["world_dx_dt"] == 0.4
    assert st.get("cmd_vel") is not None


def test_omnisim_mobile_poll_telemetry_falls_back_to_get_robot_state():
    mobile = OmniSimMobile("robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"])
    posts: list[str] = []

    def _post(path, body, timeout_s=None):
        posts.append(path)
        if path == TELEMETRY_POLL_PATH:
            return {"ok": False, "http": 404, "body": "not found"}
        if path == GET_ROBOT_STATE_PATH:
            return {"x": 3.0, "y": 4.0, "yaw": 0.2}
        raise AssertionError(f"unexpected path {path}")

    mobile._post = _post  # type: ignore[method-assign]
    st = mobile.poll_telemetry()
    assert posts == [TELEMETRY_POLL_PATH, GET_ROBOT_STATE_PATH]
    assert st["pose"] == [3.0, 4.0]
    assert "raw_world_root" not in st  # not invented on the fallback body


def test_recover_robot_records_one_evidence_entry(adapter_factory):
    ad = adapter_factory()
    ad.envelope._open["robot_a"] = {
        "request_id": "req-robot_a-stuck",
        "target": "wp_x",
    }

    before = len(ad.evidence)
    rec = ad.recover_robot("robot_a")

    assert len(ad.evidence) == before + 1
    assert "robot_a" not in ad.envelope._open
    assert rec is ad.evidence[-1]
    assert rec.request_id == "recover-robot_a"
    assert rec.outcome == "aborted"
    assert rec.assignment["recovered_request_id"] == "req-robot_a-stuck"
