from math import isclose
import pytest
from eco_cps.energy import (compute_power_w, cpu_execution_s, expected_energy_rate_w,
                            motion_power_w, remaining_route_m, sensing_power_w)
from eco_cps.models import FIXED_MODE, OperatingMode, Scenario, EnergyLedger


def test_component_powers_are_nonnegative():
    scenario = Scenario()
    assert expected_energy_rate_w(FIXED_MODE, scenario) > 0
    assert sensing_power_w(FIXED_MODE) > 0
    assert compute_power_w(FIXED_MODE) > 0
    assert motion_power_w(0.0, 0.0) > 0


def test_cpu_latency_improves_with_frequency():
    slow = OperatingMode(10, 80, 0.1, 0.8)
    assert cpu_execution_s(slow, sampled=True, controlled=True) > cpu_execution_s(
        FIXED_MODE, sampled=True, controlled=True)
    assert compute_power_w(slow) < compute_power_w(FIXED_MODE)


def test_energy_ledger_sums_components():
    ledger = EnergyLedger(1, 2, 3, 4, 5)
    assert ledger.total_j == 15
    assert sum(v for k, v in ledger.as_dict().items() if k != 'total_j') == ledger.total_j


def test_route_length_and_end_point():
    waypoints = Scenario().waypoints
    expected = sum(((b[0]-a[0])**2 + (b[1]-a[1])**2)**0.5
                   for a, b in zip(waypoints, waypoints[1:]))
    assert isclose(remaining_route_m(*waypoints[0], 1, waypoints), expected)
    assert remaining_route_m(*waypoints[-1], len(waypoints), waypoints) == 0


@pytest.mark.parametrize('value', [0, -1, float('inf')])
def test_invalid_speed_rejected(value):
    with pytest.raises(ValueError):
        OperatingMode(5, 80, 0.2, value)
