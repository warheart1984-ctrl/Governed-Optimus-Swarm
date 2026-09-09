"""Negative-path tests for SwarmLaw R1–R7 and FloorModel fail-closed checks.

These prove each rule rejects (fail closed). No skipped tests existed for
R1–R7; this file is the executable suite.
"""

from __future__ import annotations

import pytest

from control_plane import make_rebind_ticket
from governed_swarm import GovernedSwarm
from spatial_model import DuplicateRobotIdError, FloorModel, Robot, TaskNode, Zone
from specialist_registry import SpecialistRegistry, build_default_registry
from swarm_law import LawViolation, SwarmLaw


def _registry() -> SpecialistRegistry:
    reg = SpecialistRegistry()
    reg.register(
        "assembler",
        {"assemble", "idle", "moving", "working", "returning"},
    )
    reg.register("carrier", {"carry", "idle", "moving", "working", "returning"})
    reg.register("charger", {"charge", "idle", "moving"})
    reg.lock()
    return reg


def _model(*robots: Robot, zones=None, tasks=None, width=20, height=20) -> FloorModel:
    return FloorModel(
        robots=list(robots),
        zones=list(zones or []),
        tasks=list(tasks or []),
        width=width,
        height=height,
    )


def test_r1_identity_anchor_drift_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="assembler", pos=(1, 1), identity_anchor="orig")
    robot.identity_anchor = "tampered"
    model = _model(robot)
    with pytest.raises(LawViolation, match="R1"):
        law.law_gate(robot, (1, 2), "moving", model, original_anchor="orig")


def test_r2_unknown_role_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="pirate", pos=(1, 1), identity_anchor="a")
    model = _model(robot)
    with pytest.raises(LawViolation, match="R2"):
        law.law_gate(robot, (1, 1), "idle", model, original_anchor="a")


def test_r3_task_not_permitted_for_role_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="assembler", pos=(1, 1), identity_anchor="a")
    model = _model(robot)
    with pytest.raises(LawViolation, match="R3"):
        law.law_gate(robot, (1, 1), "carry", model, original_anchor="a")


def test_r4_out_of_bounds_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="assembler", pos=(1, 1), identity_anchor="a")
    model = _model(robot, width=5, height=5)
    with pytest.raises(LawViolation, match="R4"):
        law.law_gate(robot, (-1, 0), "moving", model, original_anchor="a")


def test_r5_blocked_zone_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="assembler", pos=(1, 1), identity_anchor="a")
    model = _model(robot, zones=[Zone(pos=(2, 2), zone_type="blocked")])
    with pytest.raises(LawViolation, match="R5"):
        law.law_gate(robot, (2, 2), "moving", model, original_anchor="a")


def test_r6_locked_robot_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(
        id="r1", role="assembler", pos=(1, 1), task="locked", identity_anchor="a",
    )
    model = _model(robot)
    with pytest.raises(LawViolation, match="R6"):
        law.law_gate(robot, (1, 1), "idle", model, original_anchor="a")


def test_r7_collision_with_another_robot_rejects():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="assembler", pos=(1, 1), identity_anchor="a")
    other = Robot(id="r2", role="carrier", pos=(3, 3), identity_anchor="b")
    model = _model(robot, other)
    with pytest.raises(LawViolation, match="R7"):
        law.law_gate(robot, (3, 3), "moving", model, original_anchor="a")


def test_charger_has_no_global_swarlaw_exemption():
    """Charger still fails R5 on a blocked cell and R3 on a foreign task."""
    law = SwarmLaw(_registry())
    charger = Robot(id="c1", role="charger", pos=(1, 1), identity_anchor="c")
    model = _model(charger, zones=[Zone(pos=(2, 2), zone_type="blocked")])
    with pytest.raises(LawViolation, match="R5"):
        law.law_gate(charger, (2, 2), "moving", model, original_anchor="c")
    with pytest.raises(LawViolation, match="R3"):
        law.law_gate(charger, (1, 1), "assemble", model, original_anchor="c")


def test_duplicate_robot_ids_fail_closed_at_floor_model():
    with pytest.raises(DuplicateRobotIdError, match="duplicate robot ids"):
        FloorModel(
            robots=[
                Robot(id="r1", role="assembler", pos=(0, 0)),
                Robot(id="r1", role="carrier", pos=(1, 1)),
            ],
            zones=[],
            tasks=[],
        )


def test_zone_update_bumps_cache_generation_and_invalidates_clear():
    model = FloorModel(
        robots=[Robot(id="r1", role="assembler", pos=(0, 0))],
        zones=[],
        tasks=[],
    )
    g0 = model.cache_generation
    assert model.is_blocked((5, 5)) is False
    g1 = model.update_zones([Zone(pos=(5, 5), zone_type="blocked")])
    assert g1 == g0 + 1
    assert model.cache_generation == g1
    assert model.is_blocked((5, 5)) is True
    snapshot = model.snapshot()
    assert snapshot["cache_generation"] == g1


def test_task_update_also_bumps_cache_generation():
    model = FloorModel(
        robots=[Robot(id="r1", role="assembler", pos=(0, 0))],
        zones=[],
        tasks=[],
    )
    g0 = model.cache_generation
    g1 = model.update_tasks([
        TaskNode(id="t1", pos=(3, 3), task_type="assemble", remaining=1),
    ])
    assert g1 == g0 + 1


def test_mutating_robot_role_does_not_change_admitted_swarm_role():
    registry = build_default_registry()
    robot = Robot(id="r1", role="assembler", pos=(1, 1), identity_anchor="a")
    model = FloorModel(
        robots=[robot],
        zones=[],
        tasks=[
            TaskNode(id="assemble-near", pos=(2, 1), task_type="assemble", remaining=3),
            TaskNode(id="carry-near", pos=(1, 2), task_type="carry", remaining=3),
        ],
    )
    swarm = GovernedSwarm(model, registry)
    robot.role = "carrier"
    swarm.step()
    assert swarm.manifest.roles["r1"] == "assembler"
    # Frozen assembler still walks toward the assemble node, not carry.
    assert robot.pos == (2, 1)
    roles_logged = [e.get("role") for e in swarm.log if e.get("robot") == "r1"]
    assert "assembler" in roles_logged
    assert "carrier" not in roles_logged


def test_unauthorized_swarm_rebind_fails():
    registry = build_default_registry()
    robot = Robot(id="r1", role="assembler", pos=(1, 1))
    model = FloorModel(robots=[robot], zones=[], tasks=[])
    swarm = GovernedSwarm(model, registry)
    receipt = swarm.rebind_role("r1", "carrier", authorization={"digest": "nope"})
    assert receipt["event"] == "rebind_rejected"
    assert swarm.manifest.roles["r1"] == "assembler"
    assert any(e.get("event") == "rebind_rejected" for e in swarm.log)


def test_authorized_swarm_rebind_logs_receipt_and_new_role():
    registry = build_default_registry()
    robot = Robot(id="r1", role="assembler", pos=(1, 1))
    model = FloorModel(robots=[robot], zones=[], tasks=[])
    swarm = GovernedSwarm(model, registry)
    ticket = make_rebind_ticket(swarm.manifest, "r1", "carrier")
    receipt = swarm.rebind_role("r1", "carrier", authorization=ticket)
    assert receipt["event"] == "rebind_role"
    assert receipt["new_role"] == "carrier"
    assert swarm.manifest.roles["r1"] == "carrier"
    assert robot.role == "carrier"
    assert any(e.get("event") == "rebind_role" for e in swarm.log)


def test_bound_role_overrides_mutated_robot_role_in_law_gate():
    law = SwarmLaw(_registry())
    robot = Robot(id="r1", role="carrier", pos=(1, 1), identity_anchor="a")
    model = _model(robot)
    # Mutated live field says carrier (carry permitted); frozen role is assembler.
    with pytest.raises(LawViolation, match="R3"):
        law.law_gate(
            robot, (1, 1), "carry", model,
            original_anchor="a", bound_role="assembler",
        )
def test_quarantine_recovery_path() -> None:
    """Quarantine -> idle recovery (supervisor with warrant).

    This test currently FAILS against the base implementation because:
      - SwarmLaw is fail-closed: permanent lock, no auto-recovery
      - No unlock(), no timeout, no supervisor override in base design
      - Recovery requires a separate supervisor with written warrant

    The test verifies that a quarantine->idle path exists when a
    RecoveryPolicy is wired in, but the base SwarmLaw itself remains
    fail-closed (R6 still blocks locked actors).

    RECOVERABLE = {"R4", "R5", "R7"}          # kinematics / occupancy
    TERMINAL    = {"R1", "R2", "R3", "R6"}    # identity / role / already locked

    A RecoveryPolicy pattern is suggested in the analysis:

        class RecoveryPolicy:
            max_attempts = 3
            cooldown_ticks = 2

            def __init__(self):
                self.attempts = {}          # robot_id -> int
                self.cooldown = {}          # robot_id -> ticks remaining
                self.reason = {}            # robot_id -> rule id

            def on_violation(self, robot_id, detail: str) -> str:
                rule = detail.split(":", 1)[0]  # "R7"
                self.reason[robot_id] = rule
                if rule in TERMINAL:
                    return "locked"         # no auto-recovery
                n = self.attempts.get(robot_id, 0) + 1
                self.attempts[robot_id] = n
                if n > self.max_attempts:
                    return "locked"
                self.cooldown[robot_id] = self.cooldown_ticks
                return "quarantined"

            def tick(self, robot) -> None:
                if robot.task != "quarantined":
                    return
                left = self.cooldown.get(robot.id, 0) - 1
                self.cooldown[robot.id] = left
                if left <= 0:
                    robot.task = "idle"     # re-enter assignment; still passes law_gate

    # Wire it in the except LawViolation path: set quarantined or locked
    # from the policy, log {event, rule, attempts, state_hash}.
    # Quarantined robots skip actuation like locked ones, but the
    # supervisor may return them to idle.
    # For R7, also reject the move without locking if you want production
    # kinematics: stay put, keep moving, retry next tick.
    # Law can return a "deny move, keep state" result instead of only
    # raise. That is the TPNN-vs-infra split: SwarmLaw should decide;
    # the orchestrator should recover.
    """
    a = Robot("a", "assembler", (0, 0), identity_anchor="A1")
    m = FloorModel(
        robots=[a],
        zones=[Zone((1, 0), "blocked")],
        tasks=[TaskNode("t1", (2, 0), "assemble", 3)],
    )
    s = GovernedSwarm(m, build_default_registry())

    s.step()
    assert a.task == "quarantined"
    assert any(
        e.get("event") == "law_violation"
        and e.get("recovery", {}).get("action") == "quarantine"
        for e in s.log
    )

    s.step()
    assert a.task == "quarantined"

    m.zones.clear()
    m._blocked_cache = None
    s.step()
    assert a.task == "moving"
    assert a.pos == (1, 0)


def test_terminal_violation_still_locks() -> None:
    a = Robot("a", "pirate", (0, 0), identity_anchor="A1")
    m = FloorModel(
        robots=[a],
        zones=[],
        tasks=[TaskNode("t1", (2, 0), "assemble", 3)],
    )
    s = GovernedSwarm(m, build_default_registry())

    s.step()
    assert a.task == "locked"
    assert any(
        e.get("event") == "law_violation"
        and e.get("recovery", {}).get("action") == "lock"
        for e in s.log
    )
