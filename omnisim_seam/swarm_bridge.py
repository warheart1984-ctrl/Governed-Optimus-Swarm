"""
Bridge: Governed-Optimus-Swarm (decision layer) -> OmniSim Robot Bridge (execution layer).

The swarm framework is a pure-Python decision model: each tick it decides, for
each robot, a target grid position and a task, gated by SwarmLaw. It knows
nothing about physics. OmniSim is the physical executor: each controllable
robot is exposed through an HTTP/JSON Robot Bridge (default port 8765 per
robot, or one bridge per scene of robots).

This bridge sits between them:

    GovernedSwarm.step()  --decides (pos, task) per robot-->  apply_to_omnisim()
                                              |
                        reads reality back <--|--  sync_from_omnisim()

The key idea: the swarm's abstract model is *directed* by real simulated
telemetry rather than the swarm pretending it moved perfectly. Each tick we
(1) tell OmniSim where each robot should go, then (2) read back the actual
pose the simulator arrived at and write it into the swarm's FloorModel, so
the deterministic law/role logic sees ground truth, not wishful thinking.

Mappings to configure for your world:
  - grid<->world: how swarm grid cells (int, int) map to OmniSim metres (x,y).
  - robot_id: get OmniSim's robot_id from GET /capabilities (or /protocol).
  - bridge_base_url: the Robot Bridge base, e.g. http://127.0.0.1:8765
"""

from __future__ import annotations

import time
import urllib.request
import urllib.error
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

# ---- Swarm-side imports (from Governed-Optimus-Swarm) -------------------
from governed_swarm import GovernedSwarm, Robot, TaskNode, FloorModel


# ======================================================================== #
# Small HTTP/JSON client for the OmniSim Robot Bridge (Wire Protocol v1.0) #
# ======================================================================== #
@dataclass
class OmniSimBridge:
    base_url: str = "http://127.0.0.1:8765"
    timeout_s: float = 10.0

    def _post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            return json.loads(e.read().decode("utf-8"))
        except Exception as e:  # connection refused, timeout, etc.
            return {"ok": False, "error": "transport_error", "message": str(e)}

    def _get(self, path: str) -> Dict[str, Any]:
        req = urllib.request.Request(f"{self.base_url}{path}", method="GET")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            return json.loads(e.read().decode("utf-8"))
        except Exception as e:
            return {"ok": False, "error": "transport_error", "message": str(e)}

    # -- discovery -------------------------------------------------------
    def capabilities(self) -> Dict[str, Any]:
        """GET /capabilities -> robot_id, model, class, actions, tick_period_s."""
        return self._get("/capabilities")

    # -- state ------------------------------------------------------------
    def get_state(self, robot_id: Optional[str] = None) -> Dict[str, Any]:
        """POST /get_robot_state -> {x, y, yaw, v_linear, v_angular, mode, ...}"""
        body = {"robot_id": robot_id} if robot_id else {}
        return self._post("/get_robot_state", body)

    # -- motion (mobile base actions from /capabilities.actions) ----------
    def drive_to_waypoint(self, x: float, y: float, robot_id: Optional[str] = None,
                          wait: bool = True) -> Dict[str, Any]:
        body = {"x": float(x), "y": float(y), "wait": wait}
        if robot_id:
            body["robot_id"] = robot_id
        return self._post("/drive_to_waypoint", body)

    def drive_forward(self, distance: float, robot_id: Optional[str] = None,
                      wait: bool = True) -> Dict[str, Any]:
        body = {"distance": float(distance), "wait": wait}
        if robot_id:
            body["robot_id"] = robot_id
        return self._post("/drive_forward", body)

    def set_velocity(self, linear: float, angular: float,
                     robot_id: Optional[str] = None) -> Dict[str, Any]:
        body = {"linear": float(linear), "angular": float(angular)}
        if robot_id:
            body["robot_id"] = robot_id
        return self._post("/set_velocity", body)

    def stop(self, robot_id: Optional[str] = None) -> Dict[str, Any]:
        body = {"robot_id": robot_id} if robot_id else {}
        return self._post("/stop_robot", body)


# ======================================================================== #
# The bridge itself                                                       #
# ======================================================================== #
class OmniSimSwarmBridge:
    """Couples a GovernedSwarm step to a set of OmniSim robot bridges."""

    def __init__(
        self,
        swarm: GovernedSwarm,
        robots: Dict[str, OmniSimBridge],   # swarm robot.id -> its OmniSim bridge
        grid_to_world=lambda c: (float(c[0]), float(c[1])),
        world_to_grid=lambda p: (int(round(p[0])), int(round(p[1]))),
        converge_tol_m: float = 0.05,
    ) -> None:
        self.swarm = swarm
        self.robots = robots
        self.grid_to_world = grid_to_world
        self.world_to_grid = world_to_grid
        self.converge_tol_m = converge_tol_m
        self._last_commanded: Dict[str, Any] = {}

    # -- reality ingestion (swarm <- OmniSim) ------------------------------
    def sync_from_omnisim(self) -> None:
        """Write each robot's *actual* simulated pose back into the FloorModel.

        Without this, the swarm assumes its abstract steps were achieved 1:1.
        We override with telemetry so the law gate checks real positions.
        """
        for r in self.swarm.model.robots:
            bridge = self.robots.get(r.id)
            if bridge is None:
                continue
            st = bridge.get_state(robot_id=r.id)
            if not st.get("ok", True) or ("x" not in st):
                continue
            gx, gy = self.world_to_grid((st["x"], st["y"]))
            r.pos = (gx, gy)
            self._log(r.id, "state_synced", {
                "world": [st["x"], st["y"]], "grid": [gx, gy],
                "mode": st.get("mode"), "v_linear": st.get("v_linear"),
            })

    # -- command dispatch (OmniSim <- swarm) -------------------------------
    def _dispatch(self, robot: Robot, target_grid) -> Dict[str, Any]:
        bridge = self.robots[robot.id]
        wx, wy = self.grid_to_world(target_grid)

        # Idle / finished -> stop in place
        if robot.task in ("idle", "returning", "locked"):
            return bridge.stop(robot_id=robot.id)

        # Already at the target grid cell -> stop (task executes at site)
        if tuple(robot.pos) == tuple(target_grid):
            return bridge.stop(robot_id=robot.id)

        return bridge.drive_to_waypoint(wx, wy, robot_id=robot.id, wait=True)

    def apply_to_omnisim(self) -> None:
        """After swarm.step(), push each robot to its decided destination."""
        for r in self.swarm.model.robots:
            if r.task == "locked":
                self.robots[r.id].stop(robot_id=r.id)
                self._log(r.id, "locked_stopped", {})
                continue
            # Nearest-viable-task destination is whatever the swarm decided
            # for this tick; for a moving robot that is its next step cell.
            # Here we re-derive the same destination the swarm is steering to:
            if r.task == "moving":
                pass  # row below recomputes target from the model
            result = self._dispatch(r, r.pos)
            self._last_commanded[r.id] = result
            self._log(r.id, "commanded", {"task": r.task, "result": result})

    # -- tick loop ---------------------------------------------------------
    def step(self, sync: bool = True) -> None:
        # 1) Let law/roles/task-assignment decide each robot's next state.
        self.swarm.step()
        # 2) Push those decisions to OmniSim as motion commands.
        self.apply_to_omnisim()
        # 3) Optionally read the real result back and correct the model.
        if sync:
            self.sync_from_omnisim()

    # -- logging -----------------------------------------------------------
    def _log(self, robot_id: str, event: str, detail: Dict[str, Any]) -> None:
        self.swarm.log.append({"robot": robot_id, "event": event, **detail})


# ======================================================================== #
# Wiring example: attach a GovernedSwarm to two OmniSim Husky bridges      #
# ======================================================================== #
def build_wired_demo(host: str = "127.0.0.1") -> OmniSimSwarmBridge:
    from specialist_registry import SpecialistRegistry

    reg = SpecialistRegistry()
    reg.register_role("carrier", allowed_task_types=["carry"])
    reg.register_role("assembler", allowed_task_types=["assemble"])

    model = FloorModel(
        robots=[
            Robot(id="husky0", role="carrier", pos=(0, 0)),
            Robot(id="husky1", role="assembler", pos=(0, 1)),
        ],
        zones=[],
        tasks=[
            TaskNode(id="t0", pos=(5, 5), task_type="carry", remaining=3),
            TaskNode(id="t1", pos=(8, 2), task_type="assemble", remaining=2),
        ],
        width=20,
        height=20,
    )
    swarm = GovernedSwarm(model, reg)

    # One bridge per robot (OmniSim launches these; default port 8765, but
    # a scene of robots often uses distinct ports / one multi-robot bridge).
    bridges = {
        "husky0": OmniSimBridge(f"http://{host}:8765"),
        "husky1": OmniSimBridge(f"http://{host}:8766"),
    }

    return OmniSimSwarmBridge(swarm, bridges)


if __name__ == "__main__":
    import sys

    bridge = build_wired_demo()
    frames = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    for i in range(frames):
        try:
            bridge.step()
        except Exception as e:
            print(f"[frame {i}] bridge error: {e}")
        time.sleep(0.5)
        print(f"--- frame {i} robots ---")
        for r in bridge.swarm.model.robots:
            print(f"  {r.id}: pos={r.pos} task={r.task}")
        print(f"  locked={bridge.swarm.locked_robots()}")
