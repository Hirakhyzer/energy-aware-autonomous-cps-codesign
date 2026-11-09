"""v0.3 tests for bounded uncertainty, disturbances, and robust co-design."""
from dataclasses import replace
import json
import subprocess
import sys

import pytest

from eco_cps.benchmark import POLICIES, benchmark, paired_analysis, summarize
from eco_cps.config import load_scenario
from eco_cps.models import FIXED_MODE, OperatingMode, RobotState, Scenario
from eco_cps.policies import candidate_modes, choose_mode, feasibility_reasons
from eco_cps.simulation import simulate


def test_uncertainty_parameters_are_bounded():
    with pytest.raises(ValueError, match="uncertainty_fraction"):
        Scenario(uncertainty_fraction=0.51)
    with pytest.raises(ValueError, match="workload_variation_fraction"):
        Scenario(workload_variation_fraction=0.51)
    with pytest.raises(ValueError, match="robust_risk_weight"):
        Scenario(robust_risk_weight=1.01)
    with pytest.raises(ValueError, match="disturbance_mps"):
        Scenario(disturbance_mps=-0.01)


def test_robust_policy_searches_same_joint_configuration_space():
    assert len(candidate_modes("joint_robust")) == 81
    assert "joint_robust" in POLICIES


def test_zero_uncertainty_robust_policy_reduces_to_joint():
    scenario = Scenario(uncertainty_fraction=0.0, disturbance_mps=0.0)
    for seed in (0, 1, 2):
        ordinary = simulate("joint", scenario, seed=seed)
        robust = simulate("joint_robust", scenario, seed=seed)
        assert robust.energy.as_dict() == ordinary.energy.as_dict()
        assert robust.duration_s == ordinary.duration_s
        assert robust.completed == ordinary.completed


def test_robust_feasibility_accounts_for_worst_case_sensor_noise():
    scenario = Scenario(sensor_noise_m=0.08, process_noise_m_per_s=0.03,
                        max_position_uncertainty_m=0.12, uncertainty_fraction=0.5)
    mode = OperatingMode(2.0, 240, 0.1, 0.8)
    ordinary = feasibility_reasons(mode, scenario, RobotState(), robust=False)
    robust = feasibility_reasons(mode, scenario, RobotState(), robust=True)
    assert "estimation_uncertainty" not in ordinary
    assert "estimation_uncertainty" in robust


def test_robust_feasibility_rejects_insufficient_disturbance_speed_margin():
    scenario = Scenario(disturbance_mps=0.30, uncertainty_fraction=0.1)
    slow = OperatingMode(10.0, 240, 0.1, 0.4)
    assert "disturbance_speed_margin" in feasibility_reasons(
        slow, scenario, RobotState(), robust=True)


def test_uncertainty_realization_is_reproducible_and_paired():
    scenario = Scenario(uncertainty_fraction=0.25, disturbance_mps=0.02)
    a = simulate("fixed", scenario, seed=42)
    b = simulate("joint_robust", scenario, seed=42)
    c = simulate("fixed", scenario, seed=42)
    assert a.uncertainty_factors == b.uncertainty_factors == c.uncertainty_factors
    assert a.summary() == c.summary()
    assert all(0.75 <= value <= 1.25 for value in a.uncertainty_factors.values())


def test_uncertainty_changes_episode_energy_and_state_noise():
    nominal = simulate("fixed", Scenario(), seed=9)
    uncertain = simulate("fixed", Scenario(uncertainty_fraction=0.3), seed=9)
    assert nominal.uncertainty_factors != uncertain.uncertainty_factors
    assert nominal.energy.total_j != uncertain.energy.total_j


def test_robust_policy_uses_optional_service_target():
    scenario = Scenario(service_time_limit_s=18.0, uncertainty_fraction=0.1)
    chosen, fallback = choose_mode("joint_robust", scenario, RobotState(), FIXED_MODE)
    assert not fallback
    assert not feasibility_reasons(chosen, scenario, RobotState(),
                                   enforce_service_target=True, robust=True)


def test_benchmark_and_paired_analysis_include_robust_policy():
    scenario = load_scenario("configs/robust-moderate.json")
    runs = benchmark([scenario], seeds=(0, 1))
    assert len(runs) == 14
    assert {run.policy for run in runs} == set(POLICIES)
    pairs = paired_analysis(runs)
    assert len(pairs) == 6
    robust = next(row for row in pairs if row["policy"] == "joint_robust")
    assert robust["paired_trials"] == 2
    assert next(row for row in summarize(runs) if row["policy"] == "joint_robust")["trials"] == 2


def test_robust_cli_smoke(tmp_path):
    out = tmp_path / "robust"
    run = subprocess.run([sys.executable, "-m", "eco_cps", "benchmark",
                          "--config", "configs/robust-moderate.json",
                          "--seeds", "0", "1", "--out", str(out)],
                         capture_output=True, text=True, check=True)
    assert len(json.loads(run.stdout)) == 7
    report = subprocess.run([sys.executable, "-m", "eco_cps", "report",
                             "--input", str(out), "--paired"],
                            capture_output=True, text=True, check=True)
    assert len(json.loads(report.stdout)) == 6


def test_per_tick_workload_variation_is_seeded_and_reported():
    scenario = Scenario(uncertainty_fraction=0.15, workload_variation_fraction=0.2)
    a = simulate("joint_robust", scenario, seed=31, include_trace=True)
    b = simulate("fixed", scenario, seed=31, include_trace=True)
    c = simulate("joint_robust", scenario, seed=31, include_trace=True)
    assert a.workload_scale_mean == pytest.approx(c.workload_scale_mean)
    assert a.workload_scale_max == pytest.approx(c.workload_scale_max)
    assert [row["workload_scale"] for row in a.trace[:100]] == [
        row["workload_scale"] for row in b.trace[:100]
    ]
    assert 0.8 <= a.workload_scale_mean <= 1.2
    assert a.workload_scale_max <= 1.2


def test_risk_weight_changes_the_selected_tradeoff():
    from dataclasses import replace
    scenario = load_scenario("configs/robust-moderate.json")
    state = RobotState(energy_remaining_j=scenario.initial_energy_j)
    mean_only, _ = choose_mode("joint_robust", replace(scenario, robust_risk_weight=0.0), state, FIXED_MODE)
    risk_averse, _ = choose_mode("joint_robust", replace(scenario, robust_risk_weight=1.0), state, FIXED_MODE)
    assert mean_only != risk_averse
    assert risk_averse.cruise_speed_mps < mean_only.cruise_speed_mps
