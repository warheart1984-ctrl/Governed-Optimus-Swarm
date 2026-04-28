from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional

Vec2 = Tuple[int, int]

ZONE_TYPES = {"floor", "storage", "charging", "blocked", "entry"}


@dataclass
class Robot:
    id: str
    role: str                  # matches a registered specialist role
    pos: Vec2
    task: str = "idle"         # idle | moving | working | returning | locked
    identity_anchor: str = ""  # set at registration; must not change


@dataclass
class Zone:
    pos: Vec2
    zone_type: str             # must be in ZONE_TYPES

    def __post_init__(self) -> None:
        if self.zone_type not in ZONE_TYPES:
            raise ValueError(f"Unknown zone type: {self.zone_type}")


@dataclass
class TaskNode:
    id: str
    pos: Vec2
    task_type: str             # e.g. "assemble", "carry", "inspect", "charge"
    remaining: int             # units of work left
    _claimed: bool = False     # reset each tick by the orchestrator


@dataclass
class FloorModel:
    robots: List[Robot]
    zones: List[Zone]
    tasks: List[TaskNode]
    width: int = 20
    height: int = 20

    def __post_init__(self) -> None:
        self._blocked_cache: Optional[List[Vec2]] = None
        self._zone_map: Dict[Vec2, Zone] = {z.pos: z for z in self.zones}

    @property
    def blocked_positions(self) -> List[Vec2]:
        if self._blocked_cache is None:
            self._blocked_cache = [z.pos for z in self.zones if z.zone_type == "blocked"]
        return self._blocked_cache

    def is_blocked(self, pos: Vec2) -> bool:
        return pos in self.blocked_positions

    def in_bounds(self, pos: Vec2) -> bool:
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def zone_at(self, pos: Vec2) -> Optional[Zone]:
        return self._zone_map.get(pos)

    def snapshot(self) -> Dict[str, Any]:
        return {
            "robots": [
                {
                    "id": r.id,
                    "role": r.role,
                    "pos": r.pos,
                    "task": r.task,
                    "identity_anchor": r.identity_anchor,
                }
                for r in self.robots
            ],
            "tasks": [
                {
                    "id": t.id,
                    "pos": t.pos,
                    "task_type": t.task_type,
                    "remaining": t.remaining,
                }
                for t in self.tasks
            ],
            "zones": [
                {
                    "pos": z.pos,
                    "zone_type": z.zone_type,
                }
                for z in self.zones
            ],
            "bounds": {
                "width": self.width,
                "height": self.height,
            },
        }

    def reset_claims(self) -> None:
        for t in self.tasks:
            t._claimed = False
