"""Reduced-order unicycle robot and noisy, periodic localization model."""
from __future__ import annotations
import math
import random
from .models import RobotState, OperatingMode, Scenario


def wrapped_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def track_control(state: RobotState, mode: OperatingMode, scenario: Scenario) -> tuple[float, float]:
    if state.target_index >= len(scenario.waypoints):
        return 0.0, 0.0
    target = scenario.waypoints[state.target_index]
    dx = target[0] - state.estimate_x_m
    dy = target[1] - state.estimate_y_m
    angle_error = wrapped_angle(math.atan2(dy, dx) - state.heading_rad)
    turn = max(-1.4, min(1.4, 2.3 * angle_error))
    speed = mode.cruise_speed_mps * max(0.22, 1.0 - 0.48 * abs(angle_error))
    if math.hypot(dx, dy) < 0.28:
        speed *= 0.58
    return speed, turn


def integrate_robot(state: RobotState, scenario: Scenario, rng: random.Random, mode: OperatingMode, *, sample: bool) -> float:
    """Advance a single step; return current waypoint tracking distance."""
    dt = scenario.dt_s
    dx = state.speed_command_mps * math.cos(state.heading_rad) * dt
    dy = state.speed_command_mps * math.sin(state.heading_rad) * dt
    state.x_m += dx
    state.y_m += dy
    state.heading_rad = wrapped_angle(state.heading_rad + state.turn_command_radps * dt)
    state.estimate_x_m += dx
    state.estimate_y_m += dy
    state.estimate_uncertainty_m += scenario.process_noise_m_per_s * dt
    if sample:
        noisy_x = state.x_m + rng.gauss(0.0, scenario.sensor_noise_m)
        noisy_y = state.y_m + rng.gauss(0.0, scenario.sensor_noise_m)
        # First-order complementary correction; not a calibrated Kalman filter.
        state.estimate_x_m = 0.75 * noisy_x + 0.25 * state.estimate_x_m
        state.estimate_y_m = 0.75 * noisy_y + 0.25 * state.estimate_y_m
        state.estimate_uncertainty_m = scenario.sensor_noise_m
    if state.target_index < len(scenario.waypoints):
        tx, ty = scenario.waypoints[state.target_index]
        distance = math.hypot(state.x_m - tx, state.y_m - ty)
        if distance < 0.20:
            state.target_index += 1
            if state.target_index < len(scenario.waypoints):
                tx, ty = scenario.waypoints[state.target_index]
                distance = math.hypot(state.x_m - tx, state.y_m - ty)
        return distance
    return 0.0


def cross_track_distance(state: RobotState, scenario: Scenario) -> float:
    """Distance to current route segment, bounded by its endpoints."""
    if state.target_index >= len(scenario.waypoints):
        return 0.0
    index = state.target_index
    ax, ay = scenario.waypoints[index - 1]
    bx, by = scenario.waypoints[index]
    length_sq = (bx - ax) ** 2 + (by - ay) ** 2
    if length_sq == 0:
        return math.hypot(state.x_m - ax, state.y_m - ay)
    t = max(0.0, min(1.0, ((state.x_m - ax) * (bx - ax) + (state.y_m - ay) * (by - ay)) / length_sq))
    return math.hypot(state.x_m - ax - t * (bx - ax), state.y_m - ay - t * (by - ay))
