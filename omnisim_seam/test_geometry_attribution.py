"""Offline tests for OmniLink geometry-attribution (no network, no --live).

Pins the 6-step protocol on a 9-field AttributionSample:

  1. Temporal alignment (Δt ≤ 10 ms) and fail-closed nulls
  2. Pre-dispatch world-to-bridge drift
  3. v_world_expected = [vx cos θ, vx sin θ]
  4. R = ||v_meas|| / ||v_expected||  (clean / double-frame / other)
  5. δ_odom = bridge pose − odometry_pose
  6. recover_robot() on Adapter after N=3 consecutive complete failing
     samples (not inside pure math; not on a single R ≥ 1.1 tick)

This does not claim a Husky turn-control fix.
"""

from __future__ import annotations

import math

from omnisim_seam import Adapter
from omnisim_seam.geometry_attribution import (
    ALIGNMENT_DT_MAX_S,
    AttributionClass,
    AttributionLogger,
    AttributionSample,
    RECOVER_FAIL_STREAK_N,
    diagnose_sample,
    diagnose_trace,
    extract_cmd_vel,
    world_velocity_from_poses,
)
from omnisim_seam.test_omnisim_seam import FakeMobile, SWARM_LINE_A


def _complete(**overrides) -> AttributionSample:
    """A fully populated, time-aligned sample. Overrides replace fields."""
    fields = dict(
        raw_world_root=(0.0, 0.0),
        bridge_x=0.0,
        bridge_y=0.0,
        bridge_yaw=0.0,
        odometry_pose=(0.0, 0.0),
        cmd_vel_linear_x=1.0,
        cmd_vel_angular_z=0.0,
        world_dx_dt=1.0,
        world_dy_dt=0.0,
        wall_time=1.0,
        source_stamps=(1.0,),
    )
    fields.update(overrides)
    return AttributionSample(**fields)


def test_clean_r_approx_one():
    """vx=1, yaw=0, measured (1, 0) -> R≈1, vectors match, clean."""
    result = diagnose_sample(_complete())
    assert result.ok is True
    assert result.classification is AttributionClass.CLEAN
    assert result.r_ratio is not None
    assert math.isclose(result.r_ratio, 1.0, abs_tol=1e-9)
    assert result.v_expected == (1.0, 0.0)
    assert result.v_measured == (1.0, 0.0)
    assert result.recover_recommended is False
    assert result.delta_pre == (0.0, 0.0)


def test_double_frame_r_sqrt2_yaw_zero():
    """vx=1, yaw=0, measured (1, 1) -> R≈√2 and dx≈vx, dy≈vx -> double-frame."""
    result = diagnose_sample(_complete(world_dx_dt=1.0, world_dy_dt=1.0))
    assert result.ok is False
    assert result.classification is AttributionClass.DOUBLE_FRAME
    assert result.r_ratio is not None
    assert math.isclose(result.r_ratio, math.sqrt(2.0), rel_tol=0.0, abs_tol=1e-9)
    assert result.v_expected == (1.0, 0.0)
    assert result.v_measured == (1.0, 1.0)
    assert result.recover_recommended is True  # √2 ≥ 1.1


def test_double_frame_r_sqrt2_yaw_pi_over_two():
    """Same double-frame signature with yaw=π/2 (v_expected = [0, vx])."""
    result = diagnose_sample(_complete(
        bridge_yaw=math.pi / 2.0,
        world_dx_dt=1.0,
        world_dy_dt=1.0,
    ))
    assert result.classification is AttributionClass.DOUBLE_FRAME
    assert result.r_ratio is not None
    assert math.isclose(result.r_ratio, math.sqrt(2.0), rel_tol=0.0, abs_tol=1e-9)
    assert result.recover_recommended is True


def test_incomplete_sample_fail_closed_no_r():
    """None fields -> sample_incomplete. R is not invented."""
    result = diagnose_sample(AttributionSample(wall_time=1.0, source_stamps=(1.0,)))
    assert result.ok is False
    assert result.classification is AttributionClass.SAMPLE_INCOMPLETE
    assert result.r_ratio is None
    assert result.recover_recommended is False
    assert "raw_world_root" in result.missing_fields
    assert "cmd_vel_linear_x" in result.missing_fields
    assert "world_dx_dt" in result.missing_fields
    assert result.vector_residual_m_s is None
    assert result.heading_error_rad is None


def test_nan_is_incomplete_not_zero():
    result = diagnose_sample(_complete(bridge_x=float("nan")))
    assert result.classification is AttributionClass.SAMPLE_INCOMPLETE
    assert result.r_ratio is None
    assert "bridge_x" in result.missing_fields


def test_temporal_alignment_dt_within_10ms():
    aligned = diagnose_sample(_complete(source_stamps=(1.000, 1.009)))
    assert aligned.classification is AttributionClass.CLEAN
    assert aligned.stamp_spread_s is not None
    assert aligned.stamp_spread_s <= ALIGNMENT_DT_MAX_S

    skewed = diagnose_sample(_complete(source_stamps=(1.000, 1.011)))
    assert skewed.ok is False
    assert skewed.classification is AttributionClass.TEMPORAL_MISALIGNED
    assert skewed.r_ratio is None
    assert skewed.recover_recommended is False
    assert skewed.stamp_spread_s is not None
    assert skewed.stamp_spread_s > ALIGNMENT_DT_MAX_S


def test_pre_dispatch_drift_and_odom_delta():
    result = diagnose_sample(_complete(
        raw_world_root=(0.0, 0.0),
        bridge_x=0.5,
        bridge_y=-0.25,
        odometry_pose=(0.4, -0.20, 0.0),
        bridge_yaw=0.0,
        # Keep R clean: measured world vel still matches vx=1, yaw=0.
        world_dx_dt=1.0,
        world_dy_dt=0.0,
    ))
    assert result.delta_pre == (0.5, -0.25)
    assert result.pre_dispatch_drift is True
    assert result.delta_odom is not None
    assert math.isclose(result.delta_odom[0], 0.1, abs_tol=1e-9)
    assert math.isclose(result.delta_odom[1], -0.05, abs_tol=1e-9)
    assert result.classification is AttributionClass.CLEAN


def test_other_r_is_integration_tick_rate():
    result = diagnose_sample(_complete(world_dx_dt=2.0, world_dy_dt=0.0))
    assert result.classification is AttributionClass.INTEGRATION_TICK_RATE
    assert result.r_ratio is not None
    assert math.isclose(result.r_ratio, 2.0, abs_tol=1e-9)
    assert result.recover_recommended is True


def test_extract_cmd_vel_honest_none():
    assert extract_cmd_vel({}, {}) == (None, None)
    assert extract_cmd_vel(None, None) == (None, None)
    vx, wz = extract_cmd_vel({"cmd_vel": {"linear": {"x": 0.4}, "angular": {"z": -0.1}}})
    assert vx == 0.4 and wz == -0.1


def test_world_velocity_from_poses_requires_positive_dt():
    assert world_velocity_from_poses((0.0, 0.0), (1.0, 0.0), 0.0) == (None, None)
    assert world_velocity_from_poses((0.0, 0.0), (1.0, 2.0), 0.5) == (2.0, 4.0)


def test_logger_derives_velocity_from_successive_poses():
    logger = AttributionLogger(robot_id="robot_a")
    logger.record(
        {
            "pose": [0.0, 0.0], "yaw": 0.0, "sim_time": 0.0,
            "raw_world_root": [0.0, 0.0], "odometry_pose": [0.0, 0.0],
            "cmd_vel": {"linear": 1.0, "angular": 0.0},
        },
        wall_time=10.0,
    )
    logger.record(
        {
            "pose": [1.0, 0.0], "yaw": 0.0, "sim_time": 1.0,
            "raw_world_root": [1.0, 0.0], "odometry_pose": [1.0, 0.0],
            "cmd_vel": {"linear": 1.0, "angular": 0.0},
        },
        wall_time=11.0,
    )
    assert logger.samples[0].world_dx_dt is None  # no previous pose
    assert logger.samples[1].world_dx_dt == 1.0
    assert logger.samples[1].world_dy_dt == 0.0
    trace = diagnose_trace(logger.samples)
    assert trace.samples[0].classification is AttributionClass.SAMPLE_INCOMPLETE
    assert trace.samples[1].classification is AttributionClass.CLEAN
    assert trace.classification is AttributionClass.CLEAN
    assert trace.ok is True
    assert trace.recover_recommended is False
    assert trace.vector_residual_m_s is not None
    assert math.isclose(trace.vector_residual_m_s, 0.0, abs_tol=1e-9)
    assert trace.heading_error_rad is not None
    assert math.isclose(trace.heading_error_rad, 0.0, abs_tol=1e-9)


def test_logger_uses_explicit_world_velocity_from_poll_tick():
    """A /telemetry/poll tick that already includes dx/dt is complete without a prior pose."""
    logger = AttributionLogger(robot_id="robot_a")
    logger.record(
        {
            "pose": [1.0, 0.0], "yaw": 0.0, "sim_time": 1.0,
            "raw_world_root": [1.0, 0.0], "odometry_pose": [1.0, 0.0],
            "cmd_vel": {"linear": 1.0, "angular": 0.0},
            "world_dx_dt": 1.0, "world_dy_dt": 0.0,
        },
        wall_time=10.0,
    )
    assert logger.samples[0].world_dx_dt == 1.0
    assert logger.samples[0].world_dy_dt == 0.0
    result = diagnose_sample(logger.samples[0])
    assert result.classification is AttributionClass.CLEAN
    assert result.vector_residual_m_s is not None
    assert result.heading_error_rad is not None


def test_diagnose_trace_empty_fail_closed():
    result = diagnose_trace([])
    assert result.ok is False
    assert result.classification is AttributionClass.EMPTY_TRACE
    assert result.r_ratio is None
    assert result.recover_recommended is False


def test_recover_robot_frees_in_flight_request():
    robots = {"robot_a": FakeMobile("robot_a", outcome="rejected")}
    ad = Adapter(robots)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.outcome == "rejected"
    assert "robot_a" in ad.envelope._open
    recovered = ad.recover_robot("robot_a")
    assert "robot_a" not in ad.envelope._open
    assert recovered.outcome == "aborted"
    assert recovered.request_id == "recover-robot_a"
    assert ad.envelope._open.get("robot_a") is None
    assert "attribution_trace" in recovered.to_json()
    # Already-clear must not crash; stamps another recover row.
    again = ad.recover_robot("robot_a")
    assert again.outcome == "aborted"
    assert again.request_id == "recover-robot_a"
    assert "attribution_trace" in again.to_json()


class _DoubleFrameFake:
    """Observe/dispatch stub with honest 9-field telemetry and R=√2."""

    def __init__(self, robot_id: str = "robot_a"):
        self.robot_id = robot_id
        self.dispatches: list = []
        self._observe_n = 0
        self.pose = [0.0, 0.0]

    def dispatch(self, assignment, start_pose=None):
        self.dispatches.append({
            "robot_id": assignment.robot_id,
            "target": assignment.target,
            "start_pose": start_pose,
        })
        self.pose = [1.0, 1.0]
        return {
            "ok": True, "arrived": True, "settled": True, "timed_out": False,
            "cmd_vel": {"linear": {"x": 1.0}, "angular": {"z": 0.0}},
        }

    def observe(self):
        self._observe_n += 1
        if self._observe_n == 1:
            pose = [0.0, 0.0]
            sim_time = 0.0
        else:
            pose = list(self.pose)
            sim_time = 1.0
        return {
            "pose": pose,
            "yaw": 0.0,
            "sim_time": sim_time,
            "raw_world_root": list(pose),
            "odometry_pose": list(pose),
            "cmd_vel": {"linear": {"x": 1.0}, "angular": {"z": 0.0}},
            "mode": "idle",
        }


def test_adapter_does_not_recover_on_single_r_at_least_1_1():
    ad = Adapter({"robot_a": _DoubleFrameFake()})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.attribution_trace
    assert rec.attribution_diagnosis["classification"] == "double_frame"
    # Sample-level R ≥ 1.1 is still flagged; hysteresis withholds recover.
    assert rec.attribution_diagnosis["fail_streak_peak"] == 1
    assert rec.attribution_diagnosis["recover_recommended"] is False
    assert rec.attribution_diagnosis["recover_fail_streak_n"] == RECOVER_FAIL_STREAK_N
    assert not any(e.request_id == "recover-robot_a" for e in ad.evidence)
    # Drive completed (arrived+settled), so the in-flight slot is already
    # freed by mark_terminal — recover() is not what cleared it.


def test_adapter_does_not_auto_recover_on_incomplete_trace():
    """Default FakeMobile has no odom/cmd_vel: fail-closed, envelope follows gate."""
    ad = Adapter({"robot_a": FakeMobile("robot_a", outcome="rejected")})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert rec.attribution_diagnosis.get("classification") == "sample_incomplete"
    assert rec.attribution_diagnosis.get("recover_recommended") is False
    assert not any(e.request_id == "recover-robot_a" for e in ad.evidence)
    assert "robot_a" in ad.envelope._open


def test_evidence_json_includes_attribution_trace():
    ad = Adapter({"robot_a": FakeMobile("robot_a")})  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    payload = rec.to_json()
    assert isinstance(payload["attribution_trace"], list)
    assert len(payload["attribution_trace"]) == 2
    assert "attribution_diagnosis" in payload
    # Honesty: missing OmniSim fields stay None, not 0.0.
    sample = payload["attribution_trace"][0]
    assert sample["cmd_vel_linear_x"] is None
    assert sample["odometry_pose"] is None
    assert sample["raw_world_root"] is None


def test_residual_and_heading_error_on_complete_absent_on_incomplete():
    clean = diagnose_sample(_complete())
    assert clean.vector_residual_m_s is not None
    assert math.isclose(clean.vector_residual_m_s, 0.0, abs_tol=1e-9)
    assert clean.heading_error_rad is not None
    assert math.isclose(clean.heading_error_rad, 0.0, abs_tol=1e-9)

    double = diagnose_sample(_complete(world_dx_dt=1.0, world_dy_dt=1.0))
    assert double.vector_residual_m_s is not None
    assert math.isclose(double.vector_residual_m_s, 1.0, abs_tol=1e-9)
    assert double.heading_error_rad is not None
    assert math.isclose(double.heading_error_rad, math.pi / 4.0, abs_tol=1e-9)
    payload = double.to_json()
    assert payload["vector_residual_m_s"] == double.vector_residual_m_s
    assert payload["heading_error_rad"] == double.heading_error_rad

    incomplete = diagnose_sample(AttributionSample(wall_time=1.0, source_stamps=(1.0,)))
    assert incomplete.classification is AttributionClass.SAMPLE_INCOMPLETE
    assert incomplete.vector_residual_m_s is None
    assert incomplete.heading_error_rad is None


def test_diagnose_trace_stamps_residual_and_heading_alongside_r():
    """Trace-level diagnosis copies ||Δv|| and Δθ next to R, not only per-sample."""
    failing = _complete(world_dx_dt=1.0, world_dy_dt=1.0)
    trace = diagnose_trace([failing])
    payload = trace.to_json()
    assert payload["r_ratio"] is not None
    assert math.isclose(payload["vector_residual_m_s"], 1.0, abs_tol=1e-9)
    assert math.isclose(payload["heading_error_rad"], math.pi / 4.0, abs_tol=1e-9)
    empty = diagnose_trace([])
    assert empty.vector_residual_m_s is None
    assert empty.heading_error_rad is None
    incomplete = diagnose_trace([AttributionSample(wall_time=1.0, source_stamps=(1.0,))])
    assert incomplete.vector_residual_m_s is None
    assert incomplete.heading_error_rad is None


def test_diagnose_trace_single_failing_does_not_recover():
    incomplete = AttributionSample(wall_time=1.0, source_stamps=(1.0,))
    failing = _complete(world_dx_dt=1.0, world_dy_dt=1.0)
    trace = diagnose_trace([incomplete, failing])
    assert trace.fail_streak_peak == 1
    assert trace.fail_streak == 1
    assert trace.recover_recommended is False
    assert trace.recover_fail_streak_n == RECOVER_FAIL_STREAK_N
    assert trace.samples[1].recover_recommended is True  # sample-level still flags R


def test_diagnose_trace_three_consecutive_complete_failing_recovers():
    samples = [
        AttributionSample(wall_time=0.0, source_stamps=(0.0,)),
        _complete(world_dx_dt=1.0, world_dy_dt=1.0, wall_time=1.0, source_stamps=(1.0,)),
        _complete(world_dx_dt=1.0, world_dy_dt=1.0, wall_time=2.0, source_stamps=(2.0,)),
        _complete(world_dx_dt=1.0, world_dy_dt=1.0, wall_time=3.0, source_stamps=(3.0,)),
    ]
    trace = diagnose_trace(samples)
    assert trace.fail_streak_peak == 3
    assert trace.fail_streak == 3
    assert trace.recover_recommended is True
    assert trace.samples[0].vector_residual_m_s is None
    assert trace.samples[1].vector_residual_m_s is not None
    assert trace.samples[1].heading_error_rad is not None


def test_incomplete_holds_fail_streak_clean_resets_it():
    fail = _complete(world_dx_dt=1.0, world_dy_dt=1.0)
    incomplete = AttributionSample(wall_time=1.0, source_stamps=(1.0,))
    clean = _complete()
    held = diagnose_trace([fail, incomplete, fail])
    assert held.fail_streak_peak == 2
    assert held.recover_recommended is False
    reset = diagnose_trace([fail, clean, fail, fail])
    assert reset.fail_streak_peak == 2
    assert reset.recover_recommended is False
    three_with_gap = diagnose_trace([fail, incomplete, fail, fail])
    assert three_with_gap.fail_streak_peak == 3
    assert three_with_gap.recover_recommended is True


def test_polling_path_records_mid_drive_samples():
    states = [
        {"pose": [1.0, 0.0], "yaw": 0.0, "sim_time": 1.0,
         "arrived": False, "settled": False, "timed_out": False, "mode": "moving"},
        {"pose": [2.0, 0.0], "yaw": 0.0, "sim_time": 2.0,
         "arrived": False, "settled": False, "timed_out": False, "mode": "moving"},
        {"pose": [4.0, -2.0], "yaw": 0.0, "sim_time": 3.0,
         "arrived": True, "settled": True, "timed_out": False, "mode": "idle"},
    ]
    fake = FakeMobile("robot_a", poll_states=states)
    ad = Adapter({"robot_a": fake}, poll_interval_s=0.0)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert len(rec.attribution_trace) > 2
    assert fake.dispatches[0]["wait"] is False
    assert rec.dispatch.get("arrived") is True
    assert rec.dispatch.get("settled") is True
    assert rec.dispatch.get("timed_out") is False
    assert rec.outcome == "completed"
    assert len(fake.dispatches) == 1
    assert fake.poll_telemetry_calls > 0


def test_blocking_only_stub_before_after_only():
    class BlockingOnlyStub:
        def __init__(self, robot_id="robot_a"):
            self.robot_id = robot_id
            self.pose = [0.0, 0.0]
            self.dispatches: list = []

        def dispatch(self, assignment, start_pose=None, operation_id=None):
            self.dispatches.append({
                "target": assignment.target,
                "start_pose": start_pose,
                "operation_id": operation_id,
            })
            self.pose = [4.0, -2.0]
            return {
                "ok": True, "arrived": True, "settled": True, "timed_out": False,
            }

        def observe(self):
            return {"pose": list(self.pose), "yaw": 0.0, "sim_time": 1.0, "mode": "idle"}

    stub = BlockingOnlyStub()
    ad = Adapter({"robot_a": stub}, poll_interval_s=0.0)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert len(rec.attribution_trace) == 2
    assert len(stub.dispatches) == 1
    assert "wait" not in stub.dispatches[0]
    assert rec.outcome == "completed"


def test_default_fakemobile_stays_blocking_before_after():
    fake = FakeMobile("robot_a")
    ad = Adapter({"robot_a": fake}, poll_interval_s=0.0)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert len(rec.attribution_trace) == 2
    assert fake.dispatches[0]["wait"] is True
    assert len(fake.dispatches) == 1


class _NineFieldPollingFake:
    """wait=False stub with honest 9-field mid-drive states (R=√2)."""

    def __init__(self, robot_id: str = "robot_a"):
        self.robot_id = robot_id
        self.dispatches: list = []
        self._n = 0
        self._frames = [
            ([0.0, 0.0], 0.0, False, False),
            ([1.0, 1.0], 1.0, False, False),
            ([2.0, 2.0], 2.0, False, False),
            ([3.0, 3.0], 3.0, True, True),
        ]

    def capabilities(self):
        return {"nonblocking_wait": True}

    def dispatch(self, assignment, start_pose=None, operation_id=None, wait=True):
        self.dispatches.append({"wait": wait, "operation_id": operation_id})
        if wait is False:
            return {
                "ok": True, "arrived": False, "settled": False, "timed_out": False,
                "cmd_vel": {"linear": {"x": 1.0}, "angular": {"z": 0.0}},
            }
        return {
            "ok": True, "arrived": True, "settled": True, "timed_out": False,
            "cmd_vel": {"linear": {"x": 1.0}, "angular": {"z": 0.0}},
        }

    def observe(self):
        pose, sim_time, arrived, settled = self._frames[min(self._n, len(self._frames) - 1)]
        self._n += 1
        return {
            "pose": list(pose),
            "yaw": 0.0,
            "sim_time": sim_time,
            "raw_world_root": list(pose),
            "odometry_pose": list(pose),
            "cmd_vel": {"linear": {"x": 1.0}, "angular": {"z": 0.0}},
            "arrived": arrived,
            "settled": settled,
            "timed_out": False,
            "mode": "idle" if arrived else "moving",
        }


def test_adapter_recovers_after_three_consecutive_complete_failing_samples():
    fake = _NineFieldPollingFake()
    ad = Adapter({"robot_a": fake}, poll_interval_s=0.0)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert len(rec.attribution_trace) > 2
    assert rec.attribution_diagnosis["fail_streak_peak"] >= 3
    assert rec.attribution_diagnosis["recover_recommended"] is True
    recover_rows = [e for e in ad.evidence if e.request_id == "recover-robot_a"]
    assert len(recover_rows) == 1
    assert recover_rows[0].outcome == "aborted"
    assert "robot_a" not in ad.envelope._open
    assert fake.dispatches[0]["wait"] is False
    assert len(fake.dispatches) == 1
    assert rec.attribution_diagnosis["vector_residual_m_s"] is not None
    assert rec.attribution_diagnosis["heading_error_rad"] is not None


class _TelemetryPollNineFieldFake:
    """wait=False stub: poll_telemetry supplies 9 fields; observe() does not."""

    def __init__(self, robot_id: str = "robot_a"):
        self.robot_id = robot_id
        self.dispatches: list = []
        self.poll_calls = 0
        self.observe_calls = 0
        self._n = 0
        self._frames = [
            ([1.0, 0.0], 1.0, False, False),
            ([2.0, 0.0], 2.0, False, False),
            ([4.0, -2.0], 3.0, True, True),
        ]

    def capabilities(self):
        return {"nonblocking_wait": True}

    def dispatch(self, assignment, start_pose=None, operation_id=None, wait=True):
        self.dispatches.append({"wait": wait, "operation_id": operation_id})
        if wait is False:
            return {
                "ok": True, "arrived": False, "settled": False, "timed_out": False,
            }
        return {
            "ok": True, "arrived": True, "settled": True, "timed_out": False,
        }

    def observe(self):
        self.observe_calls += 1
        if self.observe_calls == 1:
            return {"pose": [0.0, 0.0], "yaw": 0.0, "sim_time": 0.0, "mode": "idle"}
        pose, sim_time, arrived, settled = self._frames[
            min(max(self._n - 1, 0), len(self._frames) - 1)
        ]
        return {
            "pose": list(pose), "yaw": 0.0, "sim_time": sim_time,
            "arrived": arrived, "settled": settled, "timed_out": False,
            "mode": "idle" if arrived else "moving",
        }

    def poll_telemetry(self):
        self.poll_calls += 1
        pose, sim_time, arrived, settled = self._frames[
            min(self._n, len(self._frames) - 1)
        ]
        if self._n < len(self._frames):
            self._n += 1
        return {
            "pose": list(pose),
            "yaw": 0.0,
            "sim_time": sim_time,
            "raw_world_root": list(pose),
            "odometry_pose": list(pose),
            "cmd_vel": {"linear": {"x": 1.0}, "angular": {"z": 0.0}},
            "world_dx_dt": 1.0,
            "world_dy_dt": 0.0,
            "arrived": arrived,
            "settled": settled,
            "timed_out": False,
            "mode": "idle" if arrived else "moving",
        }


def test_telemetry_poll_mid_drive_supplies_nine_fields_not_incomplete():
    fake = _TelemetryPollNineFieldFake()
    ad = Adapter({"robot_a": fake}, poll_interval_s=0.0)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert fake.poll_calls >= 1
    assert fake.dispatches[0]["wait"] is False
    assert len(fake.dispatches) == 1
    classes = [s["classification"] for s in rec.attribution_diagnosis["samples"]]
    assert "sample_incomplete" in classes  # before observe has no 9 fields
    assert "clean" in classes  # mid-drive /telemetry/poll ticks do
    assert rec.attribution_diagnosis["vector_residual_m_s"] is not None
    assert rec.attribution_diagnosis["heading_error_rad"] is not None
    assert rec.dispatch.get("arrived") is True
    assert rec.dispatch.get("settled") is True
    assert rec.dispatch.get("timed_out") is False
    assert rec.outcome == "completed"


class _Poll404ThenObserveFake:
    """poll_telemetry is unusable; Adapter must fall back to observe()."""

    def __init__(self, robot_id: str = "robot_a"):
        self.robot_id = robot_id
        self.dispatches: list = []
        self.poll_calls = 0
        self._n = 0
        self._frames = [
            {"pose": [0.0, 0.0], "yaw": 0.0, "sim_time": 0.0,
             "arrived": False, "settled": False, "timed_out": False, "mode": "idle"},
            {"pose": [1.0, 0.0], "yaw": 0.0, "sim_time": 1.0,
             "arrived": False, "settled": False, "timed_out": False, "mode": "moving"},
            {"pose": [4.0, -2.0], "yaw": 0.0, "sim_time": 2.0,
             "arrived": True, "settled": True, "timed_out": False, "mode": "idle"},
        ]

    def capabilities(self):
        return {"nonblocking_wait": True}

    def dispatch(self, assignment, start_pose=None, operation_id=None, wait=True):
        self.dispatches.append({"wait": wait})
        if wait is False:
            return {
                "ok": True, "arrived": False, "settled": False, "timed_out": False,
            }
        return {"ok": True, "arrived": True, "settled": True, "timed_out": False}

    def poll_telemetry(self):
        self.poll_calls += 1
        return {"ok": False, "http": 404, "body": "no such route"}

    def observe(self):
        st = dict(self._frames[min(self._n, len(self._frames) - 1)])
        if self._n < len(self._frames):
            self._n += 1
        return st


def test_telemetry_poll_404_falls_back_to_observe():
    fake = _Poll404ThenObserveFake()
    ad = Adapter({"robot_a": fake}, poll_interval_s=0.0)  # type: ignore[arg-type]
    rec = ad.run(dict(SWARM_LINE_A))
    assert rec is not None
    assert fake.poll_calls >= 1
    assert rec.dispatch.get("arrived") is True
    assert rec.dispatch.get("settled") is True
    assert rec.outcome == "completed"
    assert len(rec.attribution_trace) > 2
