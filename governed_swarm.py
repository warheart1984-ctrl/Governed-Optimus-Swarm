from typing import List, Dict, Any, Optional
import hashlib
import json

from spatial_model import FloorModel, Robot, TaskNode, Vec2
from specialist_registry import SpecialistRegistry
from swarm_law import SwarmLaw, LawViolation


class GovernedSwarm:
    """
    Governed multi-robot swarm.
    Every robot action is passed through SwarmLaw before being committed.
    A robot that violates law is locked immediately — no recovery in this step.
    """

    def __init__(self, model: FloorModel, registry: SpecialistRegistry) -> None:
        self.model = model
        self.registry = registry
        self.law = SwarmLaw(registry)
        self.log: List[Dict[str, Any]] = []

        # Freeze identity anchors at init — any drift is a law violation.
        self._anchors: Dict[str, str] = {r.id: r.identity_anchor for r in model.robots}

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

    def _nearest_viable_task(self, robot: Robot) -> Optional[TaskNode]:
        viable = [
            t for t in self.model.tasks
            if t.remaining > 0
            and self.registry.is_permitted(robot.role, t.task_type)
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

        old_pos = robot.pos
        old_task = robot.task

        task_node = self._nearest_viable_task(robot)

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
            )
        except LawViolation as e:
            robot.task = "locked"
            self.log.append({
                "robot": robot.id,
                "event": "law_violation",
                "detail": str(e),
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
            "role": robot.role,
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
        # reset per-tick claims
        self.model.reset_claims()
        for robot in self.model.robots:
            self._update_robot(robot)

    # ------------------------------------------------------------------ #
    # Status helpers                                                     #
    # ------------------------------------------------------------------ #

    def locked_robots(self) -> List[str]:
        return [r.id for r in self.model.robots if r.task == "locked"]

    def active_robots(self) -> int:
        return sum(1 for r in self.model.robots if r.task != "locked")
