"""Finite-search co-design policies with explicit analytical feasibility screening."""
from __future__ import annotations
from itertools import product
from functools import lru_cache
from .models import (CPU_MHZ, CONTROL_PERIODS, FIXED_MODE, SENSOR_HZ, SPEEDS,
                     OperatingMode, PolicyName, Scenario, RobotState)
from .energy import (expected_energy_rate_w, robust_energy_upper_rate_w,
                     robust_risk_adjusted_rate_w, remaining_route_m, cpu_execution_s)


@lru_cache(maxsize=7)
def candidate_modes(policy: PolicyName) -> tuple[OperatingMode, ...]:
    if policy not in ("fixed", "sensing", "computing", "control", "joint", "joint_qos", "joint_robust"):
        raise ValueError(f"Unknown policy: {policy}")
    sensors = SENSOR_HZ if policy in ("sensing", "joint", "joint_qos", "joint_robust") else (FIXED_MODE.sensor_hz,)
    cpus = CPU_MHZ if policy in ("computing", "joint", "joint_qos", "joint_robust") else (FIXED_MODE.cpu_mhz,)
    controls = CONTROL_PERIODS if policy in ("control", "joint", "joint_qos", "joint_robust") else (FIXED_MODE.control_period_s,)
    speeds = SPEEDS if policy in ("control", "joint", "joint_qos", "joint_robust") else (FIXED_MODE.cruise_speed_mps,)
    return tuple(OperatingMode(*parameters) for parameters in product(sensors, cpus, controls, speeds))


def feasibility_reasons(mode: OperatingMode, scenario: Scenario, state: RobotState,
                        *, enforce_service_target: bool = False,
                        robust: bool = False) -> list[str]:
    """Predictive constraints are model-based; do not interpret as a safety proof."""
    reasons: list[str] = []
    uncertainty_scale = 1.0 + scenario.uncertainty_fraction if robust else 1.0
    uncertainty_bound = (scenario.sensor_noise_m +
                         scenario.process_noise_m_per_s / mode.sensor_hz) * uncertainty_scale
    if uncertainty_bound > scenario.max_position_uncertainty_m + 1e-12:
        reasons.append("estimation_uncertainty")
    if mode.control_period_s > scenario.max_control_period_s + 1e-12:
        reasons.append("control_period")
    # Worst step: sample, control, and optimizer simultaneously.
    workload_scale = 1.0 + scenario.workload_variation_fraction if robust else 1.0
    runtime = (cpu_execution_s(mode, sampled=True, controlled=True, optimized=True)
               * uncertainty_scale * workload_scale)
    if runtime > scenario.dt_s * scenario.cpu_utilization_limit + 1e-12:
        reasons.append("cpu_deadline")
    distance = remaining_route_m(state.x_m, state.y_m, state.target_index, scenario.waypoints)
    # Conservative cornering allowance: one second per remaining corner.
    corners = max(0, len(scenario.waypoints) - state.target_index - 1)
    effective_speed = mode.cruise_speed_mps
    if robust and scenario.disturbance_mps > 0:
        # Worst-case projection of independent x/y velocity disturbances onto the route.
        effective_speed -= 2 ** 0.5 * scenario.disturbance_mps
        if effective_speed <= 0:
            reasons.append("disturbance_speed_margin")
            return reasons
    required_time = distance / effective_speed + 0.7 * corners
    if robust:
        required_time *= uncertainty_scale
    if state.time_s + required_time > scenario.deadline_s + 1e-12:
        reasons.append("mission_deadline")
    if (enforce_service_target and scenario.service_time_limit_s is not None and
        state.time_s + required_time > scenario.service_time_limit_s + 1e-12):
        reasons.append("service_time_target")
    if robust:
        predicted_rate = robust_energy_upper_rate_w(mode, scenario)
    else:
        predicted_rate = expected_energy_rate_w(mode, scenario)
    predicted_energy = required_time * predicted_rate
    if state.energy_remaining_j - predicted_energy < scenario.reserve_energy_j:
        reasons.append("battery_reserve")
    return reasons


def choose_mode(policy: PolicyName, scenario: Scenario, state: RobotState,
                previous: OperatingMode) -> tuple[OperatingMode, bool]:
    if policy == "fixed":
        return FIXED_MODE, False
    ranked: list[tuple[float, OperatingMode]] = []
    for mode in candidate_modes(policy):
        robust = policy == "joint_robust"
        enforce_target = policy in ("joint_qos", "joint_robust")
        if feasibility_reasons(mode, scenario, state,
                               enforce_service_target=enforce_target, robust=robust):
            continue
        distance = remaining_route_m(state.x_m, state.y_m, state.target_index, scenario.waypoints)
        corners = max(0, len(scenario.waypoints) - state.target_index - 1)
        speed = mode.cruise_speed_mps
        if robust:
            speed -= 2 ** 0.5 * scenario.disturbance_mps
        time_s = distance / speed + 0.7 * corners
        if robust:
            time_s *= 1.0 + scenario.uncertainty_fraction
            predicted = time_s * robust_risk_adjusted_rate_w(mode, scenario)
        else:
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
