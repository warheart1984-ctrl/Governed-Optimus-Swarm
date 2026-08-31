"""Example: Governed Optimus Swarm wired to the Jarvis Memoryboard + RAG.

Runs the governed swarm and, via memoryboard_adapter, optionally routes a
bounded digest of the tick log toward the Jarvis Memoryboard (offline by
default). Uses the default specialist registry so the law gate permits the
robots' tasks.
"""

from __future__ import annotations

import pprint

from governed_swarm import GovernedSwarm
from memoryboard_adapter import MemoryboardAdapter
from spatial_model import FloorModel, Robot, TaskNode, Zone
from specialist_registry import build_default_registry


def demo_governed_with_memory(steps: int = 15) -> dict[str, object]:
    registry = build_default_registry()

    robots = [
        Robot(id="r1", role="assembler", pos=(1, 1)),
        Robot(id="r2", role="carrier", pos=(4, 2)),
    ]
    tasks = [
        TaskNode(id="t1", pos=(6, 6), task_type="assemble", remaining=4),
        TaskNode(id="t2", pos=(9, 3), task_type="carry", remaining=3),
    ]
    zones = [
        Zone(pos=(0, 0), zone_type="storage"),
        Zone(pos=(2, 2), zone_type="charging"),
    ]
    model = FloorModel(robots=robots, zones=zones, tasks=tasks)
    swarm = GovernedSwarm(model, registry)

    adapter = MemoryboardAdapter(session_id="optimus-demo", source_agent="demo")

    for _ in range(steps):
        swarm.step()

    # user_requested=False => governed gateway refuses (fail-closed) by design
    # when no operator opts in; offline still degrades gracefully.
    ingest = adapter.ingest_swarm_log(swarm.log, user_requested=False)

    # Hot-swap demo: undock (swarm would keep ticking) then re-seat the same
    # instrument. A different URL would mint a new session_id.
    undock = adapter.detach(reason="demo_undock")
    redock = adapter.attach(adapter.base_url, reason="demo_redock")

    return {
        "steps": steps,
        "robots": [(r.id, r.task) for r in swarm.model.robots],
        "locked": swarm.locked_robots(),
        "log_len": len(swarm.log),
        "offline_summary": ingest[-1] if ingest else {},
        "instrument": adapter.instrument_state(),
        "last_undock": undock["event"],
        "last_redock": redock["continuity"],
    }


if __name__ == "__main__":
    pprint.pprint(demo_governed_with_memory())
