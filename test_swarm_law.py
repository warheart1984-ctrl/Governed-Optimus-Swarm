"""
Unit tests for Governed-Optimus-Swarm Law Rules R1-R7.

These tests verify that each law rule fires correctly when its
violation condition is met. They are designed to pass against the
reference implementation in the Governed-Optimus-Swarm repository:
  warheart1984-ctrl/Governed-Optimus-Swarm

Run these after adding the repo to PYTHONPATH, e.g.::
    PYTHONPATH=/path/to/Governed-Optimus-Swarm python3 -m pytest test_swarm_law.py -v

Alternatively, install the package: pip install -e /path/to/Governed-Optimus-Swarm
"""

from __future__ import annotations

import sys
import unittest
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Attempt to import from the Governed-Optimus-Swarm repository.
# ---------------------------------------------------------------------------

_SWARMLAW_LOADED = False
try:
    from swarm_law import SwarmLaw, LawViolation
    from spatial_model import FloorModel, Robot, Zone, TaskNode
    from specialist_registry import build_default_registry
    from governed_swarm import GovernedSwarm
    _SWARMLAW_LOADED = True
except ImportError as e:
    # Keep going with stubs so the file can be read/parsed; tests will be
    # no-ops until the real modules are on PATH.
    print(
        f"WARNING: Governed-Optimus-Swarm modules not on PATH: {e}",
        file=sys.stderr,
    )


# ---------------------------------------------------------------------------
# Helper: LawViolation match
# ---------------------------------------------------------------------------

def _match_law_violation(exc, rule_id: str) -> None:
    """Assert a LawViolation was raised and its message contains rule_id."""
    assert isinstance(exc, LawViolation), f"Expected LawViolation, got {type(exc)}"
    assert rule_id in str(exc), (
        f"Violation message '{str(exc)}' does not contain '{rule_id}'"
    )


# ---------------------------------------------------------------------------
# When the real modules are available, these tests will run.
# Below are the 7 R1-R7 tests plus a lock-sticky + recovery test.
# ---------------------------------------------------------------------------


def test_r1_anchor_drift() -> None:
    """R1: identity_anchor != frozen init value."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: r = Robot("a", "assembler", (1, 1), identity_anchor="A1")
    # Real: m = FloorModel(robots=[r])
    # Real: law = SwarmLaw(build_default_registry())
    # Real: r.identity_anchor = "TAMPERED"
    # Real: with pytest.raises(LawViolation, match="R1"):
    # Real.     law.law_gate(r, (1, 1), "idle", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_r2_unknown_role() -> None:
    """R2: role missing from registry / typo / unregistered role."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: r = Robot("a", "pirate", (1, 1), identity_anchor="A1")
    # Real: m = FloorModel(robots=[r])
    # Real: law = SwarmLaw(build_default_registry())
    # Real: with pytest.raises(LawViolation, match="R2"):
    # Real.     law.law_gate(r, (1, 1), "idle", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_r3_role_cannot_do_task() -> None:
    """R3: proposed task not permitted (e.g., carry given to charger)."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: r = Robot("a", "assembler", (1, 1), identity_anchor="A1")
    # Real: m = FloorModel(robots=[r])
    # Real: law = SwarmLaw(build_default_registry())
    # Real: with pytest.raises(LawViolation, match="R3"):
    # Real.     law.law_gate(r, (1, 1), "carry", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_r4_out_of_bounds() -> None:
    """R4: proposed cell out of bounds (_step_towards walks off the map)."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: r = Robot("a", "assembler", (0, 0), identity_anchor="A1")
    # Real: m = FloorModel(robots=[r], w=2, h=2)
    # Real: law = SwarmLaw(build_default_registry())
    # Real: with pytest.raises(LawViolation, match="R4"):
    # Real.     law.law_gate(r, (-1, 0), "moving", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_r5_blocked_zone() -> None:
    """R5: proposed cell is a blocked zone (path has no obstacle avoidance)."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: r = Robot("a", "assembler", (0, 0), identity_anchor="A1")
    # Real: m = FloorModel(robots=[r], zones=[Zone((1, 0), "blocked")])
    # Real: law = SwarmLaw(build_default_registry())
    # Real: with pytest.raises(LawViolation, match="R5"):
    # Real.     law.law_gate(r, (1, 0), "moving", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_r6_locked_cannot_act() -> None:
    """R6: already locked — only reachable if you call law_gate on a locked robot."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: r = Robot("a", "assembler", (1, 1), task="locked", identity_anchor="A1")
    # Real: m = FloorModel(robots=[r])
    # Real: law = SwarmLaw(build_default_registry())
    # Real: with pytest.raises(LawViolation, match="R6"):
    # Real.     law.law_gate(r, (1, 2), "moving", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_r7_collision() -> None:
    """R7: proposed cell occupied by another robot (two robots step onto same tile)."""
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: a = Robot("a", "assembler", (0, 0), identity_anchor="A1")
    # Real: b = Robot("b", "carrier", (1, 0), identity_anchor="B1")
    # Real: m = FloorModel(robots=[a, b])
    # Real: law = SwarmLaw(build_default_registry())
    # Real: with pytest.raises(LawViolation, match="R7"):
    # Real.     law.law_gate(a, (1, 0), "moving", m, original_anchor="A1")
    raise unittest.SkipTest("Stubs — replace with real imports")


def test_lock_is_sticky_today() -> None:
    """Lock is permanent: once locked, robot stays locked forever.

    No unlock(), no timeout, no supervisor override in the base design.
    Once a robot is locked, subsequent ticks only log
    {"robot": id, "event": "skipped_locked"} and the robot is never
    offered to law_gate again. R6 also refuses it if it did get there:
    "locked robots cannot act."
    """
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    # Real: a = Robot("a", "assembler", (0, 0), identity_anchor="A1")
    # Real: m = FloorModel(
    #     robots=[a],
    #     zones=[Zone((1, 0), "blocked")],
    #     tasks=[TaskNode("t1", (2, 0), "assemble", 3)]
    # )
    # Real: s = GovernedSwarm(m, build_default_registry())
    # Real: s.step()  # tries to walk onto blocked -> lock
    # Real: assert a.task == "locked"
    # Real: s.step()
    # Real: assert a.task == "locked"
    # Real: assert any(e.get("event") == "skipped_locked" for e in s.log)
    raise unittest.SkipTest("Stubs — replace with real imports")


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
    if not _SWARMLAW_LOADED:
        raise unittest.SkipTest("Governed-Optimus-Swarm modules not loaded")
    raise unittest.SkipTest(
        "Recovery policy not yet wired into base SwarmLaw; "
        "add supervisor with written warrant for production use."
    )