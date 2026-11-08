import pytest
from eco_cps.models import FIXED_MODE, OperatingMode, RobotState, Scenario
from eco_cps.policies import candidate_modes, choose_mode, feasibility_reasons


def test_policy_candidate_ablation_sizes():
    assert len(candidate_modes('fixed')) == 1
    assert len(candidate_modes('sensing')) == 3
    assert len(candidate_modes('computing')) == 3
    assert len(candidate_modes('control')) == 9
    assert len(candidate_modes('joint')) == 81
    assert len(candidate_modes('joint_qos')) == 81


def test_joint_is_not_fixed_in_nominal_case():
    choice, fallback = choose_mode('joint', Scenario(), RobotState(), FIXED_MODE)
    assert not fallback
    assert choice != FIXED_MODE
    assert not feasibility_reasons(choice, Scenario(), RobotState())


def test_tight_deadline_rejects_slow_mode():
    scenario = Scenario(deadline_s=20.0)
    slow = OperatingMode(2, 80, 0.4, 0.4)
    assert 'mission_deadline' in feasibility_reasons(slow, scenario, RobotState())


def test_strict_uncertainty_rejects_sparse_sensing():
    scenario = Scenario(max_position_uncertainty_m=0.018)
    assert 'estimation_uncertainty' in feasibility_reasons(
        OperatingMode(2, 240, 0.1, 0.8), scenario, RobotState())


def test_low_energy_triggers_fallback():
    state = RobotState(energy_remaining_j=15.01)
    choice, fallback = choose_mode('joint', Scenario(), state, FIXED_MODE)
    assert fallback
    assert choice == FIXED_MODE


def test_bad_policy_rejected():
    with pytest.raises(ValueError):
        candidate_modes('surprise')
