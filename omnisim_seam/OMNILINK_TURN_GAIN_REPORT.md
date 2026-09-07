# OmniLink Turn-Gain Compensation Report

Date: 2026-09-07
Repository: `warheart1984-ctrl/Governed-Optimus-Swarm`
Commit: `b7d995d Add Husky turn gain compensation`
Subject: Husky NE under-turn in `omnilink_husky_swarm.omniworld`

## Executive Summary

The current OmniSim v8.3 replay for `omnilink_husky_swarm.omniworld`
does not reproduce the earlier published 0.44 deg mean-error result.
The supplied replay evidence reports a +90 deg requested turn on
`husky_ne` settling at +9.319384209870012 deg after ten pulse/settle
corrections, leaving an error of -80.68061579012999 deg.

That result implies an end-to-end gain ratio of:

```text
r = achieved_deg / commanded_deg
r = 9.319384209870012 / 90.0
r = 0.10354871344300014
```

The reciprocal temporary compensation is:

```text
g = 1 / r
g = 9.657290436065768
```

This report documents the adapter-side mitigation now pushed to the
Governed Optimus Swarm repository. The mitigation is intentionally
scoped, visible in evidence, and temporary. It does not claim that
OmniSim physics or the upstream mobile bridge PID has been fixed.

## Observed Failure

The evidence package from the current build reports:

| Metric | Value |
| --- | --- |
| World | `omnilink_husky_swarm.omniworld` |
| OmniSim build/source | `7d39130cf` |
| Robot | `husky_ne` |
| Commanded turn | `90.0 deg` |
| Achieved turn | `9.319384209870012 deg` |
| Error | `-80.68061579012999 deg` |
| Correction pulses | `10` |
| Completion after reporting fix | `settled=false`, `completion_reason=correction_limit` |
| Effective gain ratio | `0.10354871344300014` |
| Reciprocal multiplier | `9.657290436065768` |

The failure mode is a severe mechanical/control under-turn: the robot
achieves about 10.35% of the requested yaw change. The prior 0.44 deg
mean-error result should therefore be treated as historical evidence,
not current evidence for this build and world.

## Root-Cause Interpretation

The current local Governed Optimus Swarm seam does not contain a literal
`omnilink_mobile_bridge.py` file and does not compute:

```text
wheel_vel_cmd = Kp * yaw_error
```

The active local seam sends `/drive_to_waypoint` requests and records
pose, completion, and telemetry attribution evidence. The actual
yaw-error-to-wheel-velocity PID path therefore appears to live upstream
in the OmniLink/OmniSim mobile bridge rather than in this repository.

The measured scalar is nevertheless consistent with a bridge-control
gain problem. A differential-drive yaw controller must account for the
wheel radius, track width, and the left/right wheel relationship when
converting body yaw rate or yaw error into wheel angular velocity. The
physical constants currently tracked in the mitigation are:

| Constant | Value |
| --- | --- |
| `wheel_radius_m` | `0.165` |
| `track_width_m` | `0.555` |
| `track_width / (2 * wheel_radius)` | `1.6818181818181819` |

The reported 9.66x end-to-end correction is larger than that single
differential-drive factor, which suggests that the upstream bridge gain
may also include pulse timing, proportional gain normalization, damping,
or another conversion layer. The recommended upstream fix is therefore
to recompute the yaw-error-to-wheel-velocity path from first principles
inside the mobile bridge, then rerun the same four-Husky replay.

## Adapter-Side Mitigation

Commit `b7d995d` adds `TurnGainCalibration` in
`omnisim_seam/__init__.py`.

The default calibration is scoped to:

```text
world_id = omnilink_husky_swarm.omniworld
build_id = 7d39130cf
robot_id = husky_ne
commanded_deg = 90.0
achieved_deg = 9.319384209870012
```

For `husky_ne`, `OmniSimMobile.dispatch()` now includes these fields in
the `/drive_to_waypoint` request body:

```json
{
  "turn_gain_multiplier": 9.657290436065768,
  "turn_gain_calibration": {
    "enabled": true,
    "world_id": "omnilink_husky_swarm.omniworld",
    "build_id": "7d39130cf",
    "robot_id": "husky_ne",
    "commanded_deg": 90.0,
    "achieved_deg": 9.319384209870012,
    "gain_ratio": 0.10354871344300014,
    "multiplier": 9.657290436065768,
    "wheel_radius_m": 0.165,
    "track_width_m": 0.555,
    "differential_drive_factor": 1.6818181818181819,
    "reason": "temporary bridge turn-gain compensation"
  }
}
```

The bridge response is also copied into the evidence record as
`dispatch.turn_gain_calibration` when present. This makes the
compensation auditable in replay artifacts and prevents downstream
readers from mistaking the result for an unmodified physics pass.

## What This Does Not Claim

This mitigation does not:

- Modify OmniSim physics.
- Modify the upstream OmniLink mobile bridge PID implementation.
- Relabel the Husky under-turn as fixed.
- Prove the robot now achieves approximately 90 deg in live simulation.
- Replace the need for a per-tick diagnostic trace.

The local sandboxed live replay could not establish a physical result
because localhost bridge access was blocked. The validated result here is
that the adapter emits the correct scoped compensation and preserves the
existing evidence and completion contracts.

## Local Validation

The following checks passed after rebasing on the current `origin/main`:

```bash
python3 -m py_compile \
  omnisim_seam/__init__.py \
  omnisim_seam/route_geometry.py \
  control_plane.py \
  omnisim_seam/geometry_attribution.py

python3 -m pytest \
  omnisim_seam/test_route_geometry.py \
  omnisim_seam/test_omnisim_seam.py \
  omnisim_seam/test_omnisim_narrow.py \
  omnisim_seam/test_control_plane.py \
  omnisim_seam/test_geometry_attribution.py \
  -q
```

Result:

```text
91 passed
```

The tests added or updated for this mitigation assert:

- The calibration matches the supplied Husky NE replay.
- The derived gain ratio is approximately `0.10354871344300014`.
- The emitted multiplier is approximately `9.657290436065768`.
- The dispatch request includes `turn_gain_multiplier`.
- The dispatch request includes the full `turn_gain_calibration`.
- Custom per-robot calibration can override the default map.

During rebase, the newer upstream branch also exposed an old duplicate
`recover_robot()` implementation that returned `None` and appended the
same recovery evidence twice. The final pushed commit removes the stale
shadowing implementation, preserving the newer idempotent evidence-return
behavior.

## Requested OmniLink Review

Please review the following:

1. Confirm whether the mobile bridge accepts or should accept
   `turn_gain_multiplier` and `turn_gain_calibration` on
   `/drive_to_waypoint`.
2. Confirm the canonical upstream location for the yaw PID path,
   especially the line equivalent to `wheel_vel_cmd = Kp * yaw_error`.
3. Verify whether the bridge Kp includes wheel radius, track width, and
   the left/right wheel factor required for differential-drive yaw.
4. Share any internal calibration data for `husky_ne` in
   `omnilink_husky_swarm.omniworld` on build `7d39130cf` or later.
5. Confirm whether the joint-order warning and 1 mm placeholder colliders
   are expected in this world or need correction.
6. Provide a candidate upstream fix commit so the same replay can be run
   without the temporary adapter-side multiplier.

## Recommended Next Replay

Once OmniLink confirms the upstream bridge behavior or provides a fix
commit, rerun the same four-Husky replay with per-tick evidence enabled:

- `raw_world_root`
- `bridge_x`
- `bridge_y`
- `bridge_yaw`
- `odometry_pose`
- `cmd_vel_linear_x`
- `cmd_vel_angular_z`
- `world_dx_dt`
- `world_dy_dt`

The target acceptance condition is not merely `settled=true`. The replay
should show the requested +90 deg turn reaching approximately +90 deg,
with the correction limit not exhausted and the evidence trace explaining
which layer produced the final yaw.

## Removal Criteria

Remove or disable the adapter-side compensation when:

1. OmniLink fixes the upstream yaw PID conversion.
2. The same world and robot reproduce the 90 deg turn without the
   adapter multiplier.
3. The evidence record shows no hidden gain compensation was applied.
4. Completion is based on normal bridge behavior, not a blind scaling
   patch.

Until then, this mitigation should remain labeled as temporary
control-side compensation for a known current-build under-turn.
