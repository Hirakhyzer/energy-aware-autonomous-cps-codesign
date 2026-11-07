"""Finite-search co-design policies with explicit analytical feasibility screening."""
from __future__ import annotations
from itertools import product
from .models import (CPU_MHZ, CONTROL_PERIODS, FIXED_MODE, SENSOR_HZ, SPEEDS,
                     OperatingMode, PolicyName, Scenario, RobotState)
from .energy import expected_energy_rate_w, remaining_route_m, cpu_execution_s


def candidate_modes(policy: PolicyName) -> list[OperatingMode]:
    if policy not in ("fixed", "sensing", "computing", "control", "joint"):
        raise ValueError(f"Unknown policy: {policy}")
    sensors = SENSOR_HZ if policy in ("sensing", "joint") else (FIXED_MODE.sensor_hz,)
    cpus = CPU_MHZ if policy in ("computing", "joint") else (FIXED_MODE.cpu_mhz,)
    controls = CONTROL_PERIODS if policy in ("control", "joint") else (FIXED_MODE.control_period_s,)
    speeds = SPEEDS if policy in ("control", "joint") else (FIXED_MODE.cruise_speed_mps,)
    return [OperatingMode(*parameters) for parameters in product(sensors, cpus, controls, speeds)]


def feasibility_reasons(mode: OperatingMode, scenario: Scenario, state: RobotState) -> list[str]:
    """Predictive constraints are model-based; do not interpret as a safety proof."""
    reasons: list[str] = []
    uncertainty_bound = scenario.sensor_noise_m + scenario.process_noise_m_per_s / mode.sensor_hz
    if uncertainty_bound > scenario.max_position_uncertainty_m + 1e-12:
        reasons.append("estimation_uncertainty")
    if mode.control_period_s > scenario.max_control_period_s + 1e-12:
        reasons.append("control_period")
    # Worst step: sample, control, and optimizer simultaneously.
    runtime = cpu_execution_s(mode, sampled=True, controlled=True, optimized=True)
    if runtime > scenario.dt_s * scenario.cpu_utilization_limit + 1e-12:
        reasons.append("cpu_deadline")
    distance = remaining_route_m(state.x_m, state.y_m, state.target_index, scenario.waypoints)
    # Conservative cornering allowance: one second per remaining corner.
    corners = max(0, len(scenario.waypoints) - state.target_index - 1)
    required_time = distance / mode.cruise_speed_mps + 0.7 * corners
    if state.time_s + required_time > scenario.deadline_s + 1e-12:
        reasons.append("mission_deadline")
    predicted_energy = required_time * expected_energy_rate_w(mode, scenario)
    if state.energy_remaining_j - predicted_energy < scenario.reserve_energy_j:
        reasons.append("battery_reserve")
    return reasons


def choose_mode(policy: PolicyName, scenario: Scenario, state: RobotState,
                previous: OperatingMode) -> tuple[OperatingMode, bool]:
    if policy == "fixed":
        return FIXED_MODE, False
    ranked: list[tuple[float, OperatingMode]] = []
    for mode in candidate_modes(policy):
        if feasibility_reasons(mode, scenario, state):
            continue
        distance = remaining_route_m(state.x_m, state.y_m, state.target_index, scenario.waypoints)
        corners = max(0, len(scenario.waypoints) - state.target_index - 1)
        time_s = distance / mode.cruise_speed_mps + 0.7 * corners
        predicted = time_s * expected_energy_rate_w(mode, scenario)
        # Model switching cost and a small guard against mode chattering.
        score = predicted + (scenario.switch_energy_j + 0.12 if mode != previous else 0.0)
        ranked.append((score, mode))
    if not ranked:
        # Conservative fallback; report each occurrence, never declare feasibility.
        return FIXED_MODE, True
    ranked.sort(key=lambda item: (item[0], -item[1].cpu_mhz,
                                  -item[1].sensor_hz, -item[1].cruise_speed_mps))
    return ranked[0][1], False
