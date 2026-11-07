import pytest
from eco_cps.models import Scenario
from eco_cps.simulation import simulate


@pytest.mark.parametrize('policy', ['fixed', 'sensing', 'computing', 'control', 'joint'])
def test_nominal_mission_completes(policy):
    r = simulate(policy, Scenario(), seed=0)
    assert r.completed
    assert 0 < r.duration_s < 52
    assert r.energy.total_j > 0
    assert r.deadline_misses == 0
    assert r.constraint_violations == 0
    assert r.final_distance_m < 0.2


def test_seeded_run_reproducible():
    a = simulate('joint', Scenario(), seed=42, include_trace=True)
    b = simulate('joint', Scenario(), seed=42, include_trace=True)
    assert a.summary() == b.summary()
    assert a.trace == b.trace
    assert len(a.trace) > 5


def test_adaptation_is_not_guaranteed_to_save_energy_in_every_case():
    fixed = simulate('fixed', Scenario(), seed=0)
    joint = simulate('joint', Scenario(), seed=0)
    assert joint.energy.total_j < fixed.energy.total_j
    assert joint.duration_s > fixed.duration_s


def test_energy_depletion_is_recorded_not_hidden():
    scenario = Scenario(initial_energy_j=20, reserve_energy_j=18, deadline_s=52)
    r = simulate('joint', scenario, seed=0)
    assert r.fallback_count > 0
    assert r.constraint_violations > 0 or not r.completed


def test_invalid_scenario_rejected():
    with pytest.raises(ValueError):
        Scenario(initial_energy_j=10, reserve_energy_j=11)
    with pytest.raises(ValueError):
        Scenario(waypoints=((0.0, 0.0),))
    with pytest.raises(ValueError):
        Scenario(cpu_utilization_limit=1.1)
