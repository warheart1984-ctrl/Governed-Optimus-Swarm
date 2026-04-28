from typing import List, Dict, Any, Optional, Tuple
import hashlib
import json

Vec2 = Tuple[int, int]


class Drone:
    def __init__(self, id: int, pos: Vec2, velocity: int = 1) -> None:
        self.id = id
        self.pos = pos
        self.velocity = velocity
        self.carrying: bool = False


class ResourceNode:
    def __init__(self, pos: Vec2, remaining: int) -> None:
        self.pos = pos
        self.remaining = remaining
        self._claimed: bool = False


class Base:
    def __init__(self, pos: Vec2) -> None:
        self.pos = pos


class Obstacle:
    def __init__(self, pos: Vec2) -> None:
        self.pos = pos


class SpatialModel:
    def __init__(
        self,
        drones: List[Drone],
        base: Base,
        resources: List[ResourceNode],
        obstacles: List[Obstacle],
        width: int = 20,
        height: int = 20,
    ) -> None:
        self.drones = drones
        self.base = base
        self.resources = resources
        self.obstacles = obstacles
        self.width = width
        self.height = height

    def is_blocked(self, pos: Vec2) -> bool:
        return pos in [o.pos for o in self.obstacles]

    def in_bounds(self, pos: Vec2) -> bool:
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def snapshot(self) -> Dict[str, Any]:
        return {
            "drones": [
                {"id": d.id, "pos": d.pos, "carrying": d.carrying}
                for d in self.drones
            ],
            "resources": [
                {"pos": r.pos, "remaining": r.remaining}
                for r in self.resources
            ],
            "base": {"pos": self.base.pos},
            "obstacles": [{"pos": o.pos} for o in self.obstacles],
            "bounds": {"width": self.width, "height": self.height},
        }

    def reset_claims(self) -> None:
        for r in self.resources:
            r._claimed = False


class Swarm:
    def __init__(self, model: SpatialModel):
        self.model = model
        self.log: List[Dict[str, Any]] = []

    # Movement
    def _step_towards(self, src: Vec2, dst: Vec2, speed: int = 1) -> Vec2:
        x, y = src
        tx, ty = dst

        dx = 1 if tx > x else -1 if tx < x else 0
        dy = 1 if ty > y else -1 if ty < y else 0

        return (x + dx * speed, y + dy * speed)

    # Resource selection — deterministic, ignores depleted nodes
    def _nearest_resource(self, pos: Vec2) -> Optional[ResourceNode]:
        viable = [r for r in self.model.resources if r.remaining > 0]
        if not viable:
            return None
        return min(
            viable,
            key=lambda r: (
                abs(r.pos[0] - pos[0]) + abs(r.pos[1] - pos[1]),
                r.pos[0],
                r.pos[1],
            ),
        )

    # Law gate — validates proposed position before committing
    def _law_gate(self, drone: Drone, proposed: Vec2) -> Vec2:
        # bounds
        if not self.model.in_bounds(proposed):
            return drone.pos
        # obstacles
        if self.model.is_blocked(proposed):
            return drone.pos
        # collisions
        if proposed in [d.pos for d in self.model.drones if d.id != drone.id]:
            return drone.pos
        return proposed

    def _hash_snapshot(self) -> str:
        snap = self.model.snapshot()
        return hashlib.sha256(
            json.dumps(snap, sort_keys=True).encode()
        ).hexdigest()

    def snapshot(self) -> Dict[str, Any]:
        return self.model.snapshot()

    # Per-drone update
    def _update_drone(self, drone: Drone) -> None:
        old_pos = drone.pos

        if drone.carrying:
            target = self.model.base.pos
        else:
            node = self._nearest_resource(drone.pos)
            target = node.pos if node else self.model.base.pos

        proposed = self._step_towards(drone.pos, target, drone.velocity)
        drone.pos = self._law_gate(drone, proposed)

        # Mining (single-claim per tick)
        if not drone.carrying:
            node = self._nearest_resource(drone.pos)
            if node and drone.pos == node.pos and node.remaining > 0 and not node._claimed:
                node.remaining -= 1
                node._claimed = True
                drone.carrying = True

        # Deposit
        elif drone.pos == self.model.base.pos:
            drone.carrying = False

        self.log.append({
            "drone": drone.id,
            "from": old_pos,
            "to": drone.pos,
            "carrying": drone.carrying,
            "state_hash": self._hash_snapshot(),
        })

    # Public step
    def step(self) -> None:
        self.model.reset_claims()
        for drone in self.model.drones:
            self._update_drone(drone)
