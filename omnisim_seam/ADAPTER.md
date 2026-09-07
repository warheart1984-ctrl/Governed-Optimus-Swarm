# OmniSim seam adapter — pose-billed routes

This note covers the adapter-side polish on `omnisim_seam`. It does **not**
change OmniSim physics or the completion contract OmniLink owns. The
physical Husky miss is treated as a bridge/control gain failure on their
path. This adapter now carries a temporary, evidence-bound turn-gain
compensation for the current failing Husky NE replay so future reruns can
be audited without relabeling the upstream mechanical/control issue as
fixed.

This is a **Governed Runtime Control Plane Prototype**, not a certified
production system. `--live` stays off. This work does not fix OmniSim
Husky turn-control.

## Governed admission (prototype, not PKI)

The adapter no longer dispatches from an unbound raw swarm dict as the
sole authority. `Adapter.run` is: envelope assignment → `admit()` →
dispatch only if admitted.

`AdmissionRecord` is a bound dataclass (robot_id, task_id, target, policy,
request_id, source_log, state_hash, role, admission_id, issued_at, digest).
The digest is HMAC-SHA256 over canonical JSON plus a run-scoped secret
from `RunManifest`. Missing or mismatched `state_hash` / digest rejects
on the negative path (`rejected_admission` / `unverified`) with **no**
motion command. FakeMobile tests compute this locally. That is prototype
independent verification, not a public-key infrastructure.

Each physical attempt is bound to a one-use `operation_id` (UUID4). The
dispatch signature is resolved with `inspect.signature` *before* the first
call. There is no `except TypeError: dispatch(...)` retry. Replay of the
same id is rejected with no HTTP.

Robot roles are frozen in an immutable `RunManifest` at adapter/swarm
startup. `robot.role = ...` after init does not change admitted authority.
An authorized `rebind_role(..., authorization=ticket)` verifies a hashed
ticket, installs a new immutable manifest copy, and logs a receipt.

## What changed

1. **Route distance is billed from the observed starting pose.**
   `plan_route_budget()` in `route_geometry.py` takes the `/get_robot_state`
   observation captured immediately before `/drive_to_waypoint`. Euclidean
   metres from that pose to the named waypoint set the HTTP wait:

   `timeout = max(min_timeout, distance / cruise + settle)`

   Configured spawn is used only when the observation is missing, truncated,
   NaN, or Inf.

2. **Defensive checks, fail-closed, no physics rewrite.**
   - Heading is wrapped to `(-π, π]`. Non-finite yaw is logged as absent,
     not invented.
   - A large initial heading error is recorded and still dispatched —
     turn-control is OmniSim's job.
   - Implausible distances (default cap 500 m, far beyond the Husky
     example world) abort *before* any drive POST.
   - Arrival and yaw tolerance windows are diagnostic. They never flip
     the OmniSim completion gate.

3. **Evidence stream reads like a lab notebook.**
   Each attempt now records:
   - timestamped pose snapshots (`before` / `after`, wall clock + sim time)
   - route-distance source (`observed` vs `fallback_spawn`)
   - remaining-to-goal before/after and the signed delta
   - spawn-vs-observed drift
   - completion-gate reasoning (which flags passed, which did not)

4. **Rerun harness.** `omnisim_seam/rerun.sh` resets local artifacts and
   runs the offline geometry + seam tests in one command. Live OmniSim
   is opt-in (`--live`) and should stay unused until OmniLink signs off
   on the physics fix.

5. **Geometry-attribution logger (waiting on OmniLink).** Each attempt
   records a synchronized `attribution_trace` of timestamped samples
   (raw world root, bridge x/y/yaw, odometry pose, commanded body
   `linear.x` / `angular.z`, pose-derived world `dx/dt` `dy/dt`) plus a
   pure diagnostic (`omnisim_seam/geometry_attribution.py`). Missing
   OmniSim fields stay `None`; they are not filled with zeros.

6. **Mid-drive polling vs blocking fallback.** For navigate/drive the
   adapter prefers `wait=False` (or equivalent) and polls
   `/telemetry/poll` on a short interval, appending an
   `AttributionSample` on each tick. `/get_robot_state` remains the
   observe / route-billing path and the fallback when `/telemetry/poll`
   is absent (HTTP 404) or returns no pose. Support for nonblocking wait
   is detected *before* any drive POST via `capabilities()` (explicit
   `nonblocking_wait`) and/or `inspect.signature(dispatch)` for a `wait`
   parameter. A blocking-only stub (no `wait`, or `nonblocking_wait:
   false`) falls back to the existing before/after snapshots — one
   dispatch, no TypeError retry, no second drive. This adapter does
   **not** claim OmniSim already streams the nine fields; absent keys
   stay `None`. Polling does not weaken the completion gate: completed
   iff `arrived=true`, `settled=true`, `timed_out=false`.

7. **Recovery hysteresis.** Do not call `recover_robot()` on a single
   `R ≥ 1.1` sample. `diagnose_trace` requires **N=3** consecutive
   complete, time-aligned failing samples (configurable
   `recover_fail_streak_n`). Incomplete or temporally misaligned ticks
   **hold** the streak (neither increment nor reset). A clean complete
   sample **resets** it. A failing sample is complete + time-aligned +
   `recover_recommended` / `R ≥ 1.1` (`double_frame` or
   `integration_tick_rate`). Alongside R, each complete diagnosis
   records `vector_residual_m_s` = ||v_measured − v_expected|| and
   `heading_error_rad` = wrapped atan2(v_measured) − atan2(v_expected)
   (OmniLink comparison of measured world velocity vs
   `[vx cos yaw, vx sin yaw]`, **not** yaw vs commanded heading).
   Incomplete samples leave both `None`. Those two fields are stamped
   on each complete sample diagnosis **and** on the trace-level
   `attribution_diagnosis` alongside `r_ratio`, and logged on the
   evidence path. `Adapter.run` recovers only after this
   consecutive-complete rule.

   This does **not** fix Husky turn-control. The last live four-Husky
   result (5.1111 m and 7.0711 m misses) remains governing. `--live`
   stays off until OmniLink sends a physics fix commit.

8. **Temporary Husky NE turn-gain compensation.** The v8.3 replay for
   `omnilink_husky_swarm.omniworld` on build `7d39130cf` reported a
   commanded +90 deg turn settling at +9.319384209870012 deg. That gives
   an end-to-end bridge gain ratio of about `0.1035` and a reciprocal
   multiplier of about `9.66`. For `husky_ne`, the adapter sends both:

   - `turn_gain_multiplier`
   - `turn_gain_calibration`

   The calibration record includes the commanded/achieved angles,
   `wheel_radius_m=0.165`, `track_width_m=0.555`, and the differential
   drive factor `track_width / (2 * wheel_radius)`. This is an interim
   control-side compensation only; the real bridge PID should compute the
   yaw-error-to-wheel-velocity gain from the physical constants upstream.

## Why

The previous wait was `hypot(waypoint − spawn)`. Spawn is a table entry,
not where the robot stood. If the Husky had already moved, or spawned
with an offset, the adapter under-billed the HTTP budget and could time
out a drive that was still in progress. OmniLink confirmed the correct
origin is the observed start pose.

Weird poses (NaN, Inf, 1-element lists, already-at-goal, 10⁹ rad yaw)
used to be implicit. They are now explicit fallback / wrap / abort
paths, covered by `test_route_geometry.py`.

## How this improves accuracy

Accuracy here means **timeout accuracy and evidence accuracy**, not
path-following accuracy.

- Timeouts scale with the metres the robot actually has to cover, so a
  short remaining hop no longer waits out an 8 m spawn-to-goal budget,
  and a long remaining hop no longer dies at a spawn-sized wait.
- Evidence shows whether a rejection was `arrived=false`, `settled=false`,
  `timed_out=true`, an adapter abort, or a duplicate — instead of a
  single opaque `rejected`.
- Pose drift and remaining-distance delta make a later live run
  comparable to this one without reconstructing the geometry by hand.
- When OmniSim (or a future logger tick) supplies odom / cmd_vel /
  world-root on each sample, `diagnose_trace()` reports R, residual,
  heading error, and a classification (`clean`, `double_frame`,
  `integration_tick_rate`) without flipping the completion gate.
  Recovery stays hysteresis-gated (N=3 consecutive complete fails).
  The adapter still does not invent those nine fields.

The adapter still does not estimate slip, curvature, or turn rate. Those
stay on the OmniSim side of the seam. This attribution logger does not
change that, and it is not a green light to rerun `--live`. The last live
four-Husky miss (5.1111 m / 7.0711 m) remains the governing result. The
turn-gain field is scoped to the measured Husky NE bridge regression and
is meant to be removed once the OmniLink bridge computes the physical gain
correctly.

## How it interacts with the completion gate

The gate is unchanged:

> completed iff `arrived is True` and `settled is True` and `timed_out is False`
> (and `ok` is not `False`). Transport errors and adapter aborts are
> separate terminal outcomes.

Pose tolerances sit **beside** that decision:

- `pose_within_arrival_tolerance` / `heading_within_yaw_tolerance` are
  recorded on `completion_gate`.
- If OmniSim reports `arrived` and `settled` while remaining distance
  exceeds the window, the adapter **still records `completed`** (upstream
  flags stay authoritative) and emits explicit
  `completion_conflict=true` / `geometry_consistent=false` on both the
  gate and the top-level evidence record. Downstream readers cannot
  mistake a contradictory completion for a clean one.
- Those two fields are the OmniLink evidence-model recommendation
  answering the 5.1111 m / 7.0711 m class of live route miss: the last
  live four-Husky result remains the governing result until OmniLink
  sends a fix commit. `--live` stays off.
- Adapter aborts (`error: aborted`) happen only *before* dispatch, when
  the start pose cannot form a sane route. They free the request id so a
  later genuine retry is not classified as a duplicate.

## How to verify (offline)

```bash
./omnisim_seam/rerun.sh
```

That is the whole loop until OmniLink provides a fixed commit to rerun
against a live Husky world.
