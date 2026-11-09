"""Deterministic seeded closed-loop simulation of an autonomous mobile robot."""
from __future__ import annotations
from dataclasses import replace
from math import hypot, cos, sin
import random

from .energy import compute_power_w, cpu_execution_s, motion_power_w, sensing_power_w
from .models import EnergyLedger, FIXED_MODE, PolicyName, RobotState, RunResult, Scenario
from .plant import cross_track_distance, integrate_robot, track_control
from .policies import choose_mode


def simulate(policy: PolicyName = "joint", scenario: Scenario | None = None,
             *, seed: int = 0, include_trace: bool = False) -> RunResult:
    """Run a synthetic experiment. Same seed means paired localization noise."""
    scenario = scenario if scenario is not None else Scenario()
    if policy not in ("fixed", "sensing", "computing", "control", "joint", "joint_qos", "joint_robust"):
        raise ValueError(f"Invalid policy: {policy}")
    rng = random.Random(seed)
    # Independent streams keep episode uncertainty and disturbances paired across policies,
    # regardless of how often a policy samples its sensors.
    uncertainty_rng = random.Random(seed + 104729)
    disturbance_rng = random.Random(seed + 130363)
    workload_rng = random.Random(seed + 155921)
    workload_scales: list[float] = []
    spread = scenario.uncertainty_fraction
    def factor() -> float:
        return uncertainty_rng.uniform(1.0 - spread, 1.0 + spread)
    uncertainty_factors = {
        "motion_load": factor(), "compute_power": factor(),
        "sensing_power": factor(), "communication": factor(),
        "sensor_noise": factor(), "process_noise": factor(),
        "cpu_runtime": factor(),
    }
    realized_scenario = replace(
        scenario,
        motion_load=scenario.motion_load * uncertainty_factors["motion_load"],
        sensor_noise_m=scenario.sensor_noise_m * uncertainty_factors["sensor_noise"],
        process_noise_m_per_s=scenario.process_noise_m_per_s * uncertainty_factors["process_noise"],
    )
    state = RobotState(x_m=scenario.waypoints[0][0], y_m=scenario.waypoints[0][1],
                       estimate_x_m=scenario.waypoints[0][0],
                       estimate_y_m=scenario.waypoints[0][1],
                       estimate_uncertainty_m=realized_scenario.sensor_noise_m,
                       energy_remaining_j=scenario.initial_energy_j)
    energy = EnergyLedger()
    mode = FIXED_MODE
    fallback_count = 0
    mode_changes = 0
    constraint_violations = 0
    trace: list[dict[str, float | int | str | bool]] = []
    completed = False
    last_distance = 0.0
    # Integer ticks avoid accumulation of floating-point clock drift.
    max_steps = int(scenario.deadline_s / scenario.dt_s + 1e-9)
    for tick in range(max_steps):
        state.time_s = tick * scenario.dt_s
        optimize = tick == 0 or state.time_s + 1e-10 >= state.next_optimize_s
        if optimize:
            next_mode, fallback = choose_mode(policy, scenario, state, mode)
            fallback_count += int(fallback)
            if next_mode != mode:
                mode_changes += 1
                energy.switching_j += scenario.switch_energy_j
            mode = next_mode
            state.next_optimize_s = state.time_s + scenario.optimize_period_s
        sampled = state.time_s + 1e-10 >= state.next_sample_s
        controlled = state.time_s + 1e-10 >= state.next_control_s
        if sampled:
            state.next_sample_s = state.time_s + 1.0 / mode.sensor_hz
        if controlled:
            state.speed_command_mps, state.turn_command_radps = track_control(state, mode, scenario)
            state.next_control_s = state.time_s + mode.control_period_s
        workload_scale = workload_rng.uniform(
            1.0 - scenario.workload_variation_fraction,
            1.0 + scenario.workload_variation_fraction,
        )
        workload_scales.append(workload_scale)
        runtime = (cpu_execution_s(mode, sampled=sampled, controlled=controlled, optimized=optimize)
                   * uncertainty_factors["cpu_runtime"] * workload_scale)
        if runtime > scenario.cpu_utilization_limit * scenario.dt_s + 1e-10:
            state.deadline_misses += 1
        # Bounded environmental disturbances are independent of sensor sampling decisions.
        disturbance_x = disturbance_rng.uniform(-scenario.disturbance_mps, scenario.disturbance_mps)
        disturbance_y = disturbance_rng.uniform(-scenario.disturbance_mps, scenario.disturbance_mps)
        actual_speed = hypot(state.speed_command_mps * cos(state.heading_rad) + disturbance_x,
                             state.speed_command_mps * sin(state.heading_rad) + disturbance_y)
        # Bookkeeping accounts for uncertain subsystem power separately.
        energy.sensing_j += sensing_power_w(mode) * uncertainty_factors["sensing_power"] * scenario.dt_s
        energy.computing_j += (0.065 * scenario.dt_s + compute_power_w(mode) * runtime) * uncertainty_factors["compute_power"]
        energy.communication_j += 0.008 * uncertainty_factors["communication"] if sampled else 0.0
        energy.actuation_j += motion_power_w(actual_speed, state.turn_command_radps,
                                             realized_scenario.motion_load) * scenario.dt_s
        last_distance = integrate_robot(state, realized_scenario, rng, mode, sample=sampled,
                                        disturbance_x_mps=disturbance_x,
                                        disturbance_y_mps=disturbance_y)
        state.time_s = (tick + 1) * scenario.dt_s
        state.energy_remaining_j = scenario.initial_energy_j - energy.total_j
        tracking = cross_track_distance(state, realized_scenario)
        state.max_cross_track_m = max(state.max_cross_track_m, tracking)
        if (tracking > scenario.max_cross_track_error_m + 1e-12 or
            state.estimate_uncertainty_m > scenario.max_position_uncertainty_m + 1e-12 or
            state.energy_remaining_j < scenario.reserve_energy_j - 1e-12):
            constraint_violations += 1
        if include_trace:
            trace.append({"t_s": round(state.time_s, 5), "x_m": round(state.x_m, 5),
                          "y_m": round(state.y_m, 5), "heading_rad": round(state.heading_rad, 5),
                          "remaining_energy_j": round(state.energy_remaining_j, 5),
                          "cumulative_energy_j": round(energy.total_j, 5),
                          "sensor_hz": mode.sensor_hz, "cpu_mhz": mode.cpu_mhz,
                          "control_period_s": mode.control_period_s,
                          "cruise_speed_mps": mode.cruise_speed_mps,
                          "target_index": state.target_index, "cross_track_error_m": round(tracking, 5),
                          "cpu_execution_s": round(runtime, 6), "workload_scale": round(workload_scale, 6),
                          "sampled": sampled})
        if state.energy_remaining_j <= 0:
            break
        if state.target_index >= len(scenario.waypoints):
            completed = True
            last_distance = hypot(state.x_m - scenario.waypoints[-1][0],
                                  state.y_m - scenario.waypoints[-1][1])
            break
    return RunResult(policy=policy, scenario=scenario.name, seed=seed, completed=completed,
                     duration_s=round(state.time_s, 6), energy=energy,
                     deadline_misses=state.deadline_misses,
                     constraint_violations=constraint_violations,
                     max_tracking_error_m=state.max_cross_track_m,
                     final_distance_m=last_distance, fallback_count=fallback_count,
                     mode_changes=mode_changes, trace=trace,
                     service_time_limit_s=scenario.service_time_limit_s,
                     uncertainty_factors=uncertainty_factors,
                     workload_scale_mean=(sum(workload_scales) / len(workload_scales) if workload_scales else 1.0),
                     workload_scale_max=(max(workload_scales) if workload_scales else 1.0))
