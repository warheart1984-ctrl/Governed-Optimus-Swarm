"""Offline adapter abort receipt. No simulator, transport, or hardware calls."""
import json
from omnisim_seam import Adapter


def main():
    adapter = Adapter({})
    adapter.envelope._open["demo_robot"] = {
        "request_id": "demo_request", "target": "demo_waypoint"
    }
    record = adapter.recover_robot("demo_robot")
    assert record.outcome == "aborted"
    assert "demo_robot" not in adapter.envelope._open
    assert len(adapter.evidence) == 1
    print(json.dumps({
        "mode": "offline prototype", "abort_recorded": record.outcome == "aborted",
        "request_slot_released": "demo_robot" not in adapter.envelope._open,
        "evidence_rows": len(adapter.evidence), "robot_unlocked": False,
        "compensation_executed": False, "hardware_contacted": False,
    }, indent=2))


if __name__ == "__main__":
    main()
