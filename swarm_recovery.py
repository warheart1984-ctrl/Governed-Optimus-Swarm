"""Experimental recovery-policy state model; not wired into GovernedSwarm.

The active orchestrator permanently locks on a law violation. This module does
not unlock a robot, invoke a supervisor, or provide a law_gate_with_recovery API.
RecoveryPolicy models recommendations only; it is not a production recovery path.
"""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# Rule categorisation (mirrors the analysis in test_quarantine_recovery_path)
# ---------------------------------------------------------------------------

RECOVERABLE: frozenset = frozenset({"R4", "R5", "R7"})  # kinematics / occupancy
TERMINAL: frozenset = frozenset({"R1", "R2", "R3", "R6"})  # identity / role / already locked

# ---------------------------------------------------------------------------
# RecoveryPolicy — state machine for recoverable violations
# ---------------------------------------------------------------------------

MAX_ATTEMPTS = 3
COOLDOWN_TICKS = 2


@dataclass
class RecoveryState:
    """Per-robot recovery state machine."""

    robot_id: str
    attempts: int = 0
    cooldown: int = 0
    reason: str = ""  # rule id that triggered recovery
    state: str = "idle"  # idle | quarantined | locked

    def is_quarantined(self) -> bool:
        return self.state == "quarantined"

    def is_locked(self) -> bool:
        return self.state == "locked"

    def is_idle(self) -> bool:
        return self.state == "idle"


class RecoveryPolicy:
    """Policy that decides what to do when SwarmLaw raises LawViolation.

    Rules:
      - TERMINAL rules (R1, R2, R3, R6) → permanent lock, no auto-recovery.
      - RECOVERABLE rules (R4, R5, R7) → attempt / cooldown / quarantined → idle.
      - After max_attempts, fall through to permanent lock.

    This experimental model returns a recommendation dict. No active
    orchestrator consumes it.

    SwarmLaw itself is NOT modified. This is purely an orchestrator-side
    interposition.
    """

    def __init__(self) -> None:
        self._state: Dict[str, RecoveryState] = {}

    def _get(self, robot_id: str) -> RecoveryState:
        if robot_id not in self._state:
            self._state[robot_id] = RecoveryState(robot_id=robot_id)
        return self._state[robot_id]

    # ------------------------------------------------------------------
    # Public API called by the wrapper
    # ------------------------------------------------------------------

    def on_violation(self, robot_id: str, rule_id: str) -> Dict[str, Any]:
        """Called when SwarmLaw raises LawViolation.

        Returns a result dict the orchestrator can act on:
          {
            "action": "lock" | "quarantine" | "deny_keep_state" | "raise",
            "state": robot task state to set,
            "cooldown_ticks": int,          # valid when action=="quarantine"
            "reason": str,                    # rule id
            "log": Dict[str, Any],          # structured log entry
          }
        """
        state = self._get(robot_id)
        state.reason = rule_id

        if rule_id in TERMINAL:
            # Permanent lock — no auto-recovery. SwarmLaw R6 still blocks
            # locked actors from acting again.
            state.attempts = MAX_ATTEMPTS + 1  # force permanent
            state.state = "locked"
            state.attempts = 0  # reset for logging clarity
            return {
                "action": "lock",
                "state": "locked",
                "cooldown_ticks": 0,
                "reason": rule_id,
                "log": {
                    "event": "law_violation_terminal",
                    "robot_id": robot_id,
                    "rule": rule_id,
                    "message": f"Rule {rule_id} is TERMINAL; permanent lock.",
                },
            }

        # RECOVERABLE rule
        state.attempts += 1
        if state.attempts > MAX_ATTEMPTS:
            # Exhausted attempts → permanent lock
            state.state = "locked"
            state.attempts = MAX_ATTEMPTS + 1
            return {
                "action": "lock",
                "state": "locked",
                "cooldown_ticks": 0,
                "reason": f"{rule_id} (exhausted {MAX_ATTEMPTS} attempts)",
                "log": {
                    "event": "law_violation_exhausted",
                    "robot_id": robot_id,
                    "rule": rule_id,
                    "attempts": state.attempts - 1,
                    "message": f"Rule {rule_id} exceeded max attempts; permanent lock.",
                },
            }

        # Start / renew cooldown
        state.cooldown = COOLDOWN_TICKS
        state.state = "quarantined"
        return {
            "action": "quarantine",
            "state": "quarantined",
            "cooldown_ticks": COOLDOWN_TICKS,
            "reason": rule_id,
            "log": {
                "event": "law_violation_quarantined",
                "robot_id": robot_id,
                "rule": rule_id,
                "attempts": state.attempts,
                "message": f"Rule {rule_id} quarantined; {COOLDOWN_TICKS}-tick cooldown.",
            },
        }

    def tick(self, robot) -> None:
        """Advance the cooldown for a quarantined robot.

        Called once per simulation tick (or wall-clock second) by the
        orchestrator. When cooldown expires, only the internal policy state becomes "idle".
        This method never changes robot.task or authorizes a move.

        This is the ONLY way a quarantined robot returns to idle. The base
        SwarmLaw has no such mechanism.
        """
        state = self._get(robot.id)
        if state.state != "quarantined":
            return

        left = state.cooldown - 1
        state.cooldown = left
        if left <= 0:
            state.state = "idle"
            # robot.task is set by the caller (orchestrator), not here.
            # The orchestrator should robot.task = "idle" after this returns.