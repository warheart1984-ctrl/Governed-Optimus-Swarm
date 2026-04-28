from typing import Tuple
from spatial_model import Robot, FloorModel, Vec2
from specialist_registry import SpecialistRegistry

"""
SwarmLaw — ARIS-style authority gate for the Governed Optimus Swarm.

Every proposed action passes through law_gate() before it is committed.
If the action violates any rule, it is rejected and the robot is locked.
No exception, no fallback behavior — fail closed.

Rules enforced:
  R1 — Identity anchor must not have changed.
  R2 — Robot role must exist in the specialist registry.
  R3 — Proposed task must be permitted for the robot's role
         (universal locomotion/system states are exempt).
  R4 — Proposed position must be in-bounds.
  R5 — Proposed position must not be a blocked zone.
  R6 — A locked robot may not take any action.
  R7 — Proposed position must not collide with another robot.
"""

UNIVERSAL_STATES = {"moving", "idle", "returning", "working", "locked"}


class LawViolation(Exception):
    """Raised when a proposed action fails the law gate."""
    pass


class SwarmLaw:
    def __init__(self, registry: SpecialistRegistry) -> None:
        self.registry = registry
        self.violation_log: list = []

    def law_gate(
        self,
        robot: Robot,
        proposed_pos: Vec2,
        proposed_task: str,
        model: FloorModel,
        original_anchor: str,
    ) -> Tuple[Vec2, str]:

        violations = []

        # R6 — locked robots cannot act
        if robot.task == "locked":
            violations.append(f"R6: Robot {robot.id} is locked and cannot act.")

        # R1 — identity anchor drift
        if robot.identity_anchor != original_anchor:
            violations.append(
                f"R1: Identity anchor drift on robot {robot.id}. "
                f"Expected '{original_anchor}', got '{robot.identity_anchor}'."
            )

        # R2 — role must exist
        if self.registry.get(robot.role) is None:
            violations.append(f"R2: Role '{robot.role}' not in registry.")

        # R3 — task must be permitted (unless universal)
        if proposed_task not in UNIVERSAL_STATES and not self.registry.is_permitted(robot.role, proposed_task):
            violations.append(
                f"R3: Role '{robot.role}' not permitted to perform task '{proposed_task}'."
            )

        # R4 — in bounds
        if not model.in_bounds(proposed_pos):
            violations.append(
                f"R4: Proposed position {proposed_pos} is out of bounds."
            )

        # R5 — not blocked
        if model.is_blocked(proposed_pos):
            violations.append(
                f"R5: Proposed position {proposed_pos} is a blocked zone."
            )

        # R7 — no collision with other robots
        if proposed_pos in [r.pos for r in model.robots if r.id != robot.id]:
            violations.append(
                f"R7: Proposed position {proposed_pos} collides with another robot."
            )

        if violations:
            for v in violations:
                self.violation_log.append({"robot": robot.id, "violation": v})
            raise LawViolation("; ".join(violations))

        return proposed_pos, proposed_task
