"""Fail-closed control-plane tests for the OmniSim adapter (offline).

Covers AdmissionRecord verification, one-use operation_id dispatch,
immutable RunManifest roles, and recover_robot schema. Prototype HMAC
only — not PKI.
"""

from __future__ import annotations

from control_plane import make_rebind_ticket
from omnisim_seam import (
    Adapter,
    Assignment,
    OmniSimMobile,
    SPAWN_LOCATIONS,
    WAYPOINTS,
)
from omnisim_seam.test_omnisim_seam import FakeMobile, SWARM_LINE_A


def test_missing_state_hash_rejects_without_dispatch():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    line = dict(SWARM_LINE_A)
    del line["state_hash"]
    rec = ad.run(line)
    assert rec is not None
    assert rec.outcome == "rejected_admission"
    assert rec.accepted is False
    assert fake.dispatches == []
    assert rec.completion_gate["decision"] == "rejected_admission"


def test_empty_state_hash_rejects_without_dispatch():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    line = dict(SWARM_LINE_A)
    line["state_hash"] = "  "
    rec = ad.run(line)
    assert rec is not None
    assert rec.outcome == "rejected_admission"
    assert fake.dispatches == []


def test_tampered_source_log_rejects_without_dispatch():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    original_admit = ad.admit

    def tamper(assignment):
        record, err = original_admit(assignment)
        record.source_log["state_hash"] = "tampered"
        return record, err

    ad.admit = tamper  # type: ignore[method-assign]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.outcome == "unverified"
    assert fake.dispatches == []


def test_tampered_state_hash_field_fails_verify():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    assignment = ad.envelope.from_swarm_log(dict(SWARM_LINE_A))
    record, err = ad.admit(assignment)
    assert err is None and record is not None
    assert record.verify(ad.manifest.verification_secret) is True
    record.source_log["to"] = [99, 99]
    assert record.verify(ad.manifest.verification_secret) is False


def test_valid_admission_dispatches_once():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.outcome == "completed"
    assert len(fake.dispatches) == 1
    assert rec.admission["digest"]
    assert rec.admission["role"] == "carrier"
    assert rec.admission["state_hash"] == "h1"
    assert rec.operation_id
    assert fake.dispatches[0]["operation_id"] == rec.operation_id


def test_typeerror_after_recording_dispatch_is_not_retried():
    class TypeErrorAfterRecordFake:
        def __init__(self, robot_id: str):
            self.robot_id = robot_id
            self.dispatches: list = []

        def observe(self):
            return {"pose": [0.0, 0.0], "yaw": 0.0, "sim_time": 1.0, "mode": "idle"}

        def dispatch(self, assignment, start_pose=None, operation_id=None):
            self.dispatches.append({
                "assignment": assignment,
                "start_pose": start_pose,
                "operation_id": operation_id,
            })
            raise TypeError("simulated signature mismatch after side effect")

    fake = TypeErrorAfterRecordFake("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.outcome == "transport_error"
    assert len(fake.dispatches) == 1


def test_operation_id_replay_rejected_no_second_dispatch():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    op = rec.operation_id
    assert op
    n = len(fake.dispatches)
    assignment = Assignment(
        robot_id="robot_a", task_id="go_to_wp_x", target="wp_x",
        policy="navigate", request_id="replay",
        source_log=dict(SWARM_LINE_A),
    )
    reply = ad.dispatch_with_operation_id(fake, assignment, None, op)
    assert reply["error"] == "replayed_operation"
    assert len(fake.dispatches) == n


def test_omnisim_mobile_operation_id_replay_issues_no_http():
    mobile = OmniSimMobile(
        "robot_a", "http://127.0.0.1:1", SPAWN_LOCATIONS["robot_a"], WAYPOINTS,
    )
    posts: list = []
    mobile._post = lambda *args, **kwargs: posts.append((args, kwargs)) or {  # type: ignore[method-assign]
        "ok": True, "arrived": True, "settled": True, "timed_out": False,
    }
    assignment = Assignment(
        "robot_a", "go_to_wp_x", "wp_x", "navigate", "req-op",
    )
    first = mobile.dispatch(assignment, operation_id="op-fixed")
    assert first.get("error") != "replayed_operation"
    assert len(posts) == 1
    replay = mobile.dispatch(assignment, operation_id="op-fixed")
    assert replay["error"] == "replayed_operation"
    assert len(posts) == 1


def test_mutating_robot_role_does_not_change_admitted_role():
    fake = FakeMobile("robot_a")
    fake.role = "carrier"
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    fake.role = "assembler"
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.admission["role"] == "carrier"
    assert ad.manifest.roles["robot_a"] == "carrier"


def test_unauthorized_adapter_rebind_fails():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    rec = ad.rebind_role("robot_a", "assembler", authorization={"digest": "nope"})
    assert rec.outcome == "rebind_rejected"
    assert rec.accepted is False
    assert ad.manifest.roles["robot_a"] == "carrier"


def test_authorized_adapter_rebind_logs_receipt_and_new_role():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    ticket = make_rebind_ticket(ad.manifest, "robot_a", "assembler")
    rec = ad.rebind_role("robot_a", "assembler", authorization=ticket)
    assert rec.outcome == "rebind_role"
    assert rec.accepted is True
    assert ad.manifest.roles["robot_a"] == "assembler"
    assert ad.manifest.generation == 1
    line = dict(SWARM_LINE_A)
    line["role"] = "assembler"
    admitted = ad.run(line)
    assert admitted is not None
    assert admitted.admission["role"] == "assembler"


def test_recover_twice_is_safe_and_includes_attribution_trace():
    fake = FakeMobile("robot_a", outcome="rejected")
    ad = Adapter({"robot_a": fake})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    first = ad.recover_robot("robot_a")
    second = ad.recover_robot("robot_a")
    assert first.outcome == "aborted"
    assert second.outcome == "aborted"
    assert "attribution_trace" in first.to_json()
    assert "attribution_trace" in second.to_json()
    assert isinstance(first.to_json()["attribution_trace"], list)
