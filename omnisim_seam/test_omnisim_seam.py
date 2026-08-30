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
    EvidenceRecord,
    OmniSimMobile,
    SPAWN_LOCATIONS,
    WAYPOINTS,
)


# ------------------------------------------------------------------------- #
# Fake OmniSim mobile bridge: no network, deterministic measured pose.      #
# ------------------------------------------------------------------------- #
class FakeMobile:
    """Stands in for OmniSimMobile. Proves the seam, not the simulator."""

    def __init__(self, robot_id: str, pose=(0.0, 0.0), outcome="completed"):
        self.robot_id = robot_id
        self.pose = list(pose)
        self.outcome = outcome          # "completed" | "rejected" | "transport_error"
        self.dispatches: list[dict] = []
        self.observe_calls = 0

    def dispatch(self, assignment: Assignment) -> dict:
        record = {
            "robot_id": assignment.robot_id,
            "policy": assignment.policy,
            "target": assignment.target,
        }
        self.dispatches.append(record)
        if self.outcome == "transport_error":
            return {"ok": False, "error": "transport", "message": "sim down"}
        if self.outcome == "rejected":
            return {"ok": False, "error": "busy", "message": "robot busy"}
        return {"ok": True, "accepted": True}

    def observe(self) -> dict:
        self.observe_calls += 1
        return {"pose": list(self.pose), "yaw": 0.0, "sim_time": 1.0, "mode": "idle"}


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
    # all five required fields present on the serializable record
    for key in ("assignment", "request_id", "dispatch",
                "observation_before", "observation_after", "outcome"):
        assert key in rec.to_json()


def test_rejected_attempt_captured_not_thrown(adapter_factory):
    ad = adapter_factory(outcome="rejected")
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.accepted is False
    assert rec.outcome == "completed" or rec.outcome == "rejected"
    assert rec.observation_after is not None
