# OmniSim seam adapter — pose-billed routes

This note covers the adapter-side polish on `omnisim_seam`. It does **not**
change OmniSim physics, turn-control, or the completion contract OmniLink
owns. The physical Husky miss was a turn-control failure on their path;
we are not rerunning against that known-failing path, and we are not
patching their physics layer.

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

The adapter still does not estimate slip, curvature, or turn rate. Those
stay on the OmniSim side of the seam.

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
