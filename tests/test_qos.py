"""v0.2 tests for service-time targets and matched-seed research reporting."""
from dataclasses import replace
import json
import subprocess
import sys

import pytest

from eco_cps.benchmark import benchmark, paired_analysis, save_results, summarize
from eco_cps.config import load_scenario
from eco_cps.models import FIXED_MODE, OperatingMode, RobotState, Scenario
from eco_cps.policies import choose_mode, feasibility_reasons
from eco_cps.simulation import simulate


def test_service_target_is_independent_of_hard_mission_deadline():
    scenario = Scenario(service_time_limit_s=18.0)
    slow = OperatingMode(2, 80, 0.4, 0.4)
    assert 'mission_deadline' not in feasibility_reasons(slow, scenario, RobotState())
    assert 'service_time_target' not in feasibility_reasons(slow, scenario, RobotState())
    assert 'service_time_target' in feasibility_reasons(
        slow, scenario, RobotState(), enforce_service_target=True)
    chosen, fallback = choose_mode('joint_qos', scenario, RobotState(), FIXED_MODE)
    assert not fallback
    assert not feasibility_reasons(chosen, scenario, RobotState(), enforce_service_target=True)


def test_qos_aware_policy_exposes_energy_time_tradeoff():
    scenario = Scenario(service_time_limit_s=18.0)
    unrestricted = simulate('joint', scenario, seed=1)
    qos = simulate('joint_qos', scenario, seed=1)
    fixed = simulate('fixed', scenario, seed=1)
    assert unrestricted.completed and qos.completed and fixed.completed
    assert unrestricted.service_target_met is False
    assert qos.service_target_met is True
    assert qos.duration_s <= 18
    assert qos.energy.total_j < fixed.energy.total_j
    assert qos.energy.total_j > unrestricted.energy.total_j
    assert qos.deadline_misses == 0
    assert qos.constraint_violations == 0


@pytest.mark.parametrize('bad', [0.0, -1, 100, float('nan'), float('inf')])
def test_invalid_service_targets_rejected(bad):
    with pytest.raises(ValueError):
        Scenario(service_time_limit_s=bad)


def test_qos_reporting_is_matched_and_reproducible(tmp_path):
    scenario = load_scenario('configs/qos-18s.json')
    results = benchmark([scenario], seeds=(0, 1, 2))
    pairs = paired_analysis(results)
    assert len(results) == 18
    assert len(pairs) == 5
    assert paired_analysis(list(reversed(results))) == pairs
    qos = next(row for row in pairs if row['policy'] == 'joint_qos')
    joint = next(row for row in pairs if row['policy'] == 'joint')
    assert qos['paired_trials'] == 3
    assert qos['complete_pairs'] == 3
    assert qos['candidate_service_met'] == 3
    assert joint['candidate_service_met'] == 0
    assert qos['mean_energy_delta_j'] < 0
    assert qos['mean_duration_delta_s'] > 0
    assert qos['energy_delta_ci95_low_j'] <= qos['energy_delta_ci95_high_j']
    folder = save_results(results, tmp_path / 'output')
    assert json.loads((folder / 'paired.json').read_text()) == pairs
    summary = summarize(results)
    assert next(row for row in summary if row['policy'] == 'joint_qos')['service_target_met'] == 3


def test_mission_failure_not_omitted_from_paired_reporting():
    scenario = Scenario(service_time_limit_s=18)
    base = simulate('fixed', scenario, seed=0)
    candidate = replace(simulate('joint_qos', scenario, seed=0), completed=False)
    row, = paired_analysis([base, candidate])
    assert row['paired_trials'] == 1
    assert row['complete_pairs'] == 0
    assert row['candidate_completions'] == 0
    assert row['candidate_service_met'] == 0
    assert row['mean_energy_delta_j'] is None
    assert row['energy_delta_ci95_low_j'] is None
    with pytest.raises(ValueError, match='Duplicate'):
        paired_analysis([base, base, candidate])
    with pytest.raises(ValueError, match='same service-time target'):
        paired_analysis([base, replace(candidate, service_time_limit_s=None)])


def test_cli_qos_and_paired_report(tmp_path):
    dest = tmp_path / 'benchmark'
    run = subprocess.run([sys.executable, '-m', 'eco_cps', 'benchmark',
                          '--config', 'configs/qos-18s.json', '--seeds', '0', '1',
                          '--out', str(dest)], capture_output=True, text=True, check=True)
    assert len(json.loads(run.stdout)) == 6
    report = subprocess.run([sys.executable, '-m', 'eco_cps', 'report',
                             '--input', str(dest), '--paired'],
                            capture_output=True, text=True, check=True)
    parsed = json.loads(report.stdout)
    assert len(parsed) == 5
    assert any(row['policy'] == 'joint_qos' for row in parsed)


def test_without_service_target_qos_matches_joint():
    scenario = Scenario()
    for seed in (0, 1, 2):
        energy = simulate('joint', scenario, seed=seed)
        qos = simulate('joint_qos', scenario, seed=seed)
        assert qos.energy.as_dict() == energy.energy.as_dict()
        assert qos.duration_s == energy.duration_s
        assert qos.service_target_met is None


def test_service_target_infeasibility_is_reported_not_guaranteed():
    scenario = Scenario(service_time_limit_s=1.0)
    run = simulate('joint_qos', scenario, seed=0)
    assert run.fallback_count > 0
    assert run.service_target_met is False
    assert run.duration_s > 1.0
