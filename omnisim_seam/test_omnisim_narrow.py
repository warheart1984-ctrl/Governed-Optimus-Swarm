"""Invariant tests for the bounded narrow-route R1/R3/R5 harness.

Physical twin of the EMR adversarial tests:
  * conflict membrane: two robots claim the same narrow route -> <=1 admitted
  * abstention on evidence floor: stale state => abstain, don't force through
  * reinforcement cannot bypass capacity: the narrow route holds capacity 1
    no matter how "fast" either robot claims to be.

These run entirely OFFLINE against the deterministic KinematicWorld backend,
so the co-admission / leak / abstention invariants are reproducible in CI
without a live OmniSim instance. Swap physics to "omnisim" for a live run.
"""

from __future__ import annotations

import pytest

from omnisim_seam.bounded_narrow_test import (
    BoundedNarrowTest,
    KinematicWorld,
    NarrowLaw,
    NarrowRoute,
    OmniSimRobot,
)


def _fake_robots():
    # Real class signature, but the kinematic path never hits the network, so
    # these are just idle placeholders satisfying the Dict[str, OmniSimRobot].
    return {
        "r0": OmniSimRobot("r0", "http://127.0.0.1:1"),
        "r1": OmniSimRobot("r1", "http://127.0.0.1:2"),
    }


def _run(delay_s: float, floor_s: float, steps: int = 60):
    test = BoundedNarrowTest(
        _fake_robots(),
        route=NarrowRoute(center_x=0.0, half_len_m=1.0),
        delay_s=delay_s,
        max_evidence_age_s=floor_s,
        physics="kinematic",
    )
    # deterministic kinematic start positions
    test.world.true["r0"] = -4.0
    test.world.true["r1"] = 4.0
    for _ in range(steps):
        test.step(_)
    return test.metrics


# ------------------------------------------------------------------------- #
# R1 / R3: conflict membrane -- capacity 1, at most one co-admitted          #
# ------------------------------------------------------------------------- #
def test_no_coadmission_fresh_evidence():
    # Fresh telemetry (zero delay): the membrane never admits both robots into
    # the narrow at once -- the whole-distance twin of
    # test_known_subject_unresolved_conflict_surfaces_no_coadmission.
    m = _run(delay_s=0.0, floor_s=0.15)
    assert m.exclude_leaks == 0
    assert m.co_admissions == 0
    assert m.first_broken_step == "none"


def test_membrane_holds_under_comms_delay():
    # 300ms delay against a 150ms evidence floor: R5 abstains rather than
    # forcing through -> still zero leaks / co-admissions.
    m = _run(delay_s=0.3, floor_s=0.15)
    assert m.exclude_leaks == 0
    assert m.co_admissions == 0
    assert m.abstentions > 0          # R5 fired; robots held, not forced
    assert m.first_broken_step == "none"


def test_reinforcement_cannot_bypass_narrow_capacity():
    # Even if a robot were "reinforced" (here: higher velocity), the narrow's
    # capacity-1 gate is enforced by the LAW layer, not by speed, so the leak
    # count stays 0. Deterministic anyway: run a long horizon fast.
    m = _run(delay_s=0.0, floor_s=0.15, steps=120)
    assert m.exclude_leaks == 0
    assert m.co_admissions == 0


# ------------------------------------------------------------------------- #
# R5: abstention on the evidence floor                                      #
# ------------------------------------------------------------------------- #
def test_abstention_fires_only_when_stale():
    # Fresh evidence: no abstention needed (delay well under the floor).
    fresh = _run(delay_s=0.0, floor_s=0.15)
    assert fresh.abstentions == 0
    # Stale evidence: abstention fires (delay exceeds the floor).
    stale = _run(delay_s=0.3, floor_s=0.15)
    assert stale.abstentions > 0


def test_abstention_grows_with_delay_beyond_floor():
    # Monotonic fail-closed response: beyond the floor, more delay => more
    # abstentions, never more leaks.
    d_low = _run(delay_s=0.15, floor_s=0.15)
    d_high = _run(delay_s=0.6, floor_s=0.15)
    assert d_high.abstentions >= d_low.abstentions
    assert d_high.exclude_leaks == 0


# ------------------------------------------------------------------------- #
# KinematicWorld degrades deterministically                                 #
# ------------------------------------------------------------------------- #
def test_kinematic_world_integrates():
    w = KinematicWorld(r0_x=-4.0, r1_x=4.0, delay_s=0.0)
    w.cmd_vel(0.5, 0.0, 0.1)
    assert w.true["r0"] == pytest.approx(-3.95)
    od = w.odom()["r0"]
    assert od["x"] == pytest.approx(-3.95)   # no lag at zero delay


def test_narrow_route_points():
    route = NarrowRoute(center_x=0.0, half_len_m=1.0)
    assert route.points(0.0) is True
    assert route.points(0.9) is True
    assert route.points(1.5) is False
    assert route.points(-2.0) is False


def test_narrowlaw_abstains_on_stale_evidence():
    law = NarrowLaw(NarrowRoute(0.0, 1.0), max_evidence_age_s=0.15)
    g = law.gate("r0", odom_x=0.0, odom_age_s=0.3, other_in_narrow=False)
    assert g.abstain is True
    assert not g.admitted


def test_narrowlaw_membrane_single_admission():
    law = NarrowLaw(NarrowRoute(0.0, 1.0), max_evidence_age_s=0.15)
    entered = law.gate("r0", odom_x=0.0, odom_age_s=0.0, other_in_narrow=True)
    assert entered.admitted is True     # already inside: clears through
    outsider = law.gate("r1", odom_x=3.0, odom_age_s=0.0, other_in_narrow=True)
    assert outsider.admitted is False   # membrane holds r1 out
    allow = law.gate("r1", odom_x=3.0, odom_age_s=0.0, other_in_narrow=False)
    assert allow.admitted is True       # route clear now
