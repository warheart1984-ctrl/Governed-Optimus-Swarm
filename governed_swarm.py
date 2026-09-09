from typing import List, Dict, Any, Optional
import hashlib
import json

from control_plane import (
    RunManifest,
    verify_rebind_authorization,
)
from spatial_model import DuplicateRobotIdError, FloorModel, Robot, TaskNode, Vec2
from specialist_registry import SpecialistRegistry
from swarm_law import SwarmLaw, LawViolation
from swarm_recovery import RecoveryPolicy


class GovernedSwarm:
    """
    Governed multi-robot swarm.
    Every robot action is passed through SwarmLaw before being committed.
    Recoverable law violations enter quarantine and cool down before retry.
    """

    def __init__(
        self,
        model: FloorModel,
        registry: SpecialistRegistry,
        recovery_policy: Optional[RecoveryPolicy] = None,
        re_evaluate_interval_ticks: int = 1,
    ) -> None:
        self.model = model
        self.registry = registry
        self.law = SwarmLaw(registry)
        self.recovery = recovery_policy or RecoveryPolicy()
        self.re_evaluate_interval_ticks = max(1, re_evaluate_interval_ticks)
        self._tick_count = 0
        self.log: List[Dict[str, Any]] = []

        ids = [r.id for r in model.robots]
        duplicates = sorted({i for i in ids if ids.count(i) > 1})
        if duplicates:
            raise DuplicateRobotIdError(
                f"duplicate robot ids are forbidden at swarm init: {duplicates}"
            )

        # Freeze identity anchors at init — any drift is a law violation.
        self._anchors: Dict[str, str] = {r.id: r.identity_anchor for r in model.robots}
        # Freeze roles into an immutable RunManifest. Mutating Robot.role
        # after this point does not change admitted authority.
        self.manifest = RunManifest.issue({r.id: r.role for r in model.robots})

    # ------------------------------------------------------------------ #
    # Movement                                                           #
    # ------------------------------------------------------------------ #

    def _step_towards(self, src: Vec2, dst: Vec2) -> Vec2:
        x, y = src
        tx, ty = dst

        dx = 1 if tx > x else -1 if tx < x else 0
        dy = 1 if ty > y else -1 if ty < y else 0

        return (x + dx, y + dy)

    # ------------------------------------------------------------------ #
    # Task assignment — deterministic, role-gated                        #
    # ------------------------------------------------------------------ #

    def _nearest_viable_task(self, robot: Robot, role: str) -> Optional[TaskNode]:
        viable = [
            t for t in self.model.tasks
            if t.remaining > 0
            and self.registry.is_permitted(role, t.task_type)
        ]
        if not viable:
            return None
        return min(
            viable,
            key=lambda t: (
                abs(t.pos[0] - robot.pos[0]) + abs(t.pos[1] - robot.pos[1]),
                t.pos[0],
                t.pos[1],
            ),
        )

    # ------------------------------------------------------------------ #
    # Hashing / snapshot                                                 #
    # ------------------------------------------------------------------ #

    def _hash_snapshot(self) -> str:
        snap = self.model.snapshot()
        return hashlib.sha256(
            json.dumps(snap, sort_keys=True).encode()
        ).hexdigest()

    def snapshot(self) -> Dict[str, Any]:
        return self.model.snapshot()

    # ------------------------------------------------------------------ #
    # Per-robot update                                                   #
    # ------------------------------------------------------------------ #

    def _update_robot(self, robot: Robot) -> None:
        if robot.task == "locked":
            self.log.append({"robot": robot.id, "event": "skipped_locked"})
            return
        if robot.task == "quarantined":
            self.log.append({"robot": robot.id, "event": "skipped_quarantined"})
            return

        old_pos = robot.pos
        old_task = robot.task
        bound_role = self.manifest.roles.get(robot.id, robot.role)

        task_node = self._nearest_viable_task(robot, bound_role)

        if task_node is None:
            proposed_pos = robot.pos
            proposed_task = "idle"
        elif robot.pos == task_node.pos:
            proposed_pos = robot.pos
            proposed_task = task_node.task_type
        else:
            proposed_pos = self._step_towards(robot.pos, task_node.pos)
            proposed_task = "moving"

        try:
            approved_pos, approved_task = self.law.law_gate(
                robot=robot,
                proposed_pos=proposed_pos,
                proposed_task=proposed_task,
                model=self.model,
                original_anchor=self._anchors[robot.id],
                bound_role=bound_role,
            )
        except LawViolation as e:
            detail = str(e)
            rule_id = detail.split(":", 1)[0]
            decision = self.recovery.on_violation(robot.id, rule_id)
            robot.task = decision["state"]
            self.log.append({
                "robot": robot.id,
                "event": "law_violation",
                "detail": detail,
                "recovery": decision,
                "state_hash": self._hash_snapshot(),
            })
            return

        robot.pos = approved_pos
        robot.task = approved_task

        # Execute work if at task node — single-claim per tick
        if task_node and robot.pos == task_node.pos and task_node.remaining > 0 and not task_node._claimed:
            task_node.remaining -= 1
            task_node._claimed = True
            if task_node.remaining == 0:
                robot.task = "returning"

        self.log.append({
            "robot": robot.id,
            "role": bound_role,
            "from": old_pos,
            "to": robot.pos,
            "task_before": old_task,
            "task_after": robot.task,
            "state_hash": self._hash_snapshot(),
        })

    # ------------------------------------------------------------------ #
    # Public step                                                        #
    # ------------------------------------------------------------------ #

    def step(self) -> None:
        self._tick_count += 1
        if self._tick_count % self.re_evaluate_interval_ticks == 0:
            self.recovery.re_evaluate_all(self.model.robots)

        # reset per-tick claims
        self.model.reset_claims()
        for robot in self.model.robots:
            self._update_robot(robot)

    # ------------------------------------------------------------------ #
    # Status helpers                                                     #
    # ------------------------------------------------------------------ #

    def rebind_role(
        self,
        robot_id: str,
        new_role: str,
        authorization: Any = None,
    ) -> Dict[str, Any]:
        """Replace the frozen role via a hashed ticket. Unauthorized = no-op."""
        if not verify_rebind_authorization(
            self.manifest, robot_id, new_role, authorization
        ):
            receipt = {
                "event": "rebind_rejected",
                "robot": robot_id,
                "new_role": new_role,
                "reason": "unauthorized",
            }
            self.log.append(receipt)
            return receipt
        if self.registry.get(new_role) is None:
            receipt = {
                "event": "rebind_rejected",
                "robot": robot_id,
                "new_role": new_role,
                "reason": "unknown_role",
            }
            self.log.append(receipt)
            return receipt
        if robot_id not in self.manifest.roles:
            receipt = {
                "event": "rebind_rejected",
                "robot": robot_id,
                "new_role": new_role,
                "reason": "unknown_robot",
            }
            self.log.append(receipt)
            return receipt
        self.manifest = self.manifest.with_role(robot_id, new_role)
        for robot in self.model.robots:
            if robot.id == robot_id:
                robot.role = new_role
                break
        receipt = {
            "event": "rebind_role",
            "robot": robot_id,
            "new_role": new_role,
            "run_id": self.manifest.run_id,
            "generation": self.manifest.generation,
        }
        self.log.append(receipt)
        return receipt

    def locked_robots(self) -> List[str]:
        return [r.id for r in self.model.robots if r.task == "locked"]

    def active_robots(self) -> int:
        return sum(1 for r in self.model.robots if r.task not in {"locked", "quarantined"})
