"""Transparent, intentionally synthetic component power and timing model."""
from __future__ import annotations
from math import hypot
from .models import OperatingMode, Scenario


def cpu_execution_s(mode: OperatingMode, *, sampled: bool, controlled: bool, optimized: bool = False) -> float:
    """Execution latency = modeled processor cycles / configured clock frequency."""
    cycles = 200_000 + (500_000 if sampled else 0) + (1_000_000 if controlled else 0) + (350_000 if optimized else 0)
    return cycles / (mode.cpu_mhz * 1_000_000)


def sensing_power_w(mode: OperatingMode) -> float:
    """Average sensing power (duty-cycling approximation)."""
    return 0.025 + 0.020 * mode.sensor_hz


def compute_power_w(mode: OperatingMode) -> float:
    return 0.05 + 0.000020 * mode.cpu_mhz ** 2


def motion_power_w(speed_mps: float, turn_radps: float, motion_load: float = 1.0) -> float:
    return motion_load * (0.50 + 2.7 * speed_mps ** 2 + 0.11 * abs(turn_radps))


def expected_energy_rate_w(mode: OperatingMode, scenario: Scenario) -> float:
    """Steady-state expected rate; feasibility and switching costs are checked separately."""
    samples_s = mode.sensor_hz
    controls_s = 1.0 / mode.control_period_s
    cycles_s = 200_000 / scenario.dt_s + 500_000 * samples_s + 1_000_000 * controls_s
    utilization = min(1.0, cycles_s / (mode.cpu_mhz * 1_000_000))
    computing = 0.065 + compute_power_w(mode) * utilization
    communication = 0.008 * samples_s
    sensing = sensing_power_w(mode)
    # A turn-energy allowance approximates turns at corners of the route.
    actuation = motion_power_w(mode.cruise_speed_mps, 0.20, scenario.motion_load)
    return computing + communication + sensing + actuation


def remaining_route_m(x: float, y: float, index: int, waypoints: tuple[tuple[float, float], ...]) -> float:
    if index >= len(waypoints):
        return 0.0
    remaining = hypot(x - waypoints[index][0], y - waypoints[index][1])
    for a, b in zip(waypoints[index:], waypoints[index + 1:]):
        remaining += hypot(a[0] - b[0], a[1] - b[1])
    return remaining
