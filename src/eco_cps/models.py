"""Typed research models. All energy values are joules; power values are watts."""
from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Literal

PolicyName = Literal["fixed", "sensing", "computing", "control", "joint", "joint_qos"]


@dataclass(frozen=True)
class OperatingMode:
    sensor_hz: float
    cpu_mhz: int
    control_period_s: float
    cruise_speed_mps: float

    def __post_init__(self) -> None:
        if self.sensor_hz <= 0 or self.cpu_mhz <= 0 or self.control_period_s <= 0 or self.cruise_speed_mps <= 0:
            raise ValueError("Operating mode parameters must be positive")
        if not all(isfinite(x) for x in (self.sensor_hz, self.control_period_s, self.cruise_speed_mps)):
            raise ValueError("Operating mode parameters must be finite")


FIXED_MODE = OperatingMode(10.0, 240, 0.1, 0.8)
SENSOR_HZ = (2.0, 5.0, 10.0)
CPU_MHZ = (80, 160, 240)
CONTROL_PERIODS = (0.1, 0.2, 0.4)
SPEEDS = (0.4, 0.6, 0.8)


@dataclass(frozen=True)
class Scenario:
    """Illustrative parameters, not calibrated properties of hardware."""

    name: str = "nominal"
    dt_s: float = 0.1
    deadline_s: float = 52.0
    initial_energy_j: float = 240.0
    reserve_energy_j: float = 15.0
    max_position_uncertainty_m: float = 0.19
    max_control_period_s: float = 0.4
    cpu_utilization_limit: float = 0.8
    max_cross_track_error_m: float = 1.5
    motion_load: float = 1.0
    sensor_noise_m: float = 0.012
    process_noise_m_per_s: float = 0.022
    switch_energy_j: float = 0.018
    optimize_period_s: float = 0.5
    service_time_limit_s: float | None = None
    waypoints: tuple[tuple[float, float], ...] = (
        (0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (5.0, 2.0), (5.0, 5.0)
    )

    def __post_init__(self) -> None:
        numeric = (self.dt_s, self.deadline_s, self.initial_energy_j, self.reserve_energy_j,
                   self.max_position_uncertainty_m, self.max_control_period_s,
                   self.cpu_utilization_limit, self.max_cross_track_error_m,
                   self.motion_load, self.sensor_noise_m, self.process_noise_m_per_s,
                   self.optimize_period_s)
        if self.service_time_limit_s is not None:
            if (not isfinite(self.service_time_limit_s) or
                not 0 < self.service_time_limit_s <= self.deadline_s):
                raise ValueError("Service-time target must be positive, finite, and at most the mission deadline")
        if any(not isfinite(v) for v in numeric):
            raise ValueError("Scenario parameters must be finite")
        if any(x <= 0 for x in (self.dt_s, self.deadline_s, self.initial_energy_j,
                                self.max_position_uncertainty_m, self.max_control_period_s,
                                self.motion_load, self.optimize_period_s)):
            raise ValueError("Time, energy, uncertainty, and load parameters must be positive")
        if not 0 < self.cpu_utilization_limit <= 1:
            raise ValueError("CPU utilization limit must be in (0, 1]")
        if not 0 <= self.reserve_energy_j < self.initial_energy_j:
            raise ValueError("Reserve energy must be nonnegative and below initial energy")
        if self.sensor_noise_m < 0 or self.process_noise_m_per_s < 0 or self.switch_energy_j < 0:
            raise ValueError("Noise and switching cost must be nonnegative")
        if len(self.waypoints) < 2 or any(len(w) != 2 or not all(isfinite(v) for v in w) for w in self.waypoints):
            raise ValueError("At least two finite 2D waypoints are required")
        if self.dt_s > min(CONTROL_PERIODS) + 1e-9:
            raise ValueError("dt_s may not exceed the shortest control period")


@dataclass
class RobotState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    target_index: int = 1
    estimate_x_m: float = 0.0
    estimate_y_m: float = 0.0
    estimate_uncertainty_m: float = 0.012
    speed_command_mps: float = 0.0
    turn_command_radps: float = 0.0
    time_s: float = 0.0
    energy_remaining_j: float = 240.0
    next_sample_s: float = 0.0
    next_control_s: float = 0.0
    next_optimize_s: float = 0.0
    deadline_misses: int = 0
    max_cross_track_m: float = 0.0


@dataclass
class EnergyLedger:
    sensing_j: float = 0.0
    computing_j: float = 0.0
    communication_j: float = 0.0
    actuation_j: float = 0.0
    switching_j: float = 0.0

    @property
    def total_j(self) -> float:
        return sum((self.sensing_j, self.computing_j, self.communication_j,
                    self.actuation_j, self.switching_j))

    def as_dict(self) -> dict[str, float]:
        return {"sensing_j": self.sensing_j, "computing_j": self.computing_j,
                "communication_j": self.communication_j, "actuation_j": self.actuation_j,
                "switching_j": self.switching_j, "total_j": self.total_j}


@dataclass
class RunResult:
    policy: PolicyName
    scenario: str
    seed: int
    completed: bool
    duration_s: float
    energy: EnergyLedger
    deadline_misses: int
    constraint_violations: int
    max_tracking_error_m: float
    final_distance_m: float
    fallback_count: int
    mode_changes: int
    trace: list[dict[str, float | int | str | bool]] = field(default_factory=list)
    service_time_limit_s: float | None = None

    @property
    def service_target_met(self) -> bool | None:
        if self.service_time_limit_s is None:
            return None
        return self.completed and self.duration_s <= self.service_time_limit_s + 1e-9

    def summary(self) -> dict[str, object]:
        return {"policy": self.policy, "scenario": self.scenario, "seed": self.seed,
                "completed": self.completed, "duration_s": round(self.duration_s, 4),
                **{k: round(v, 6) for k, v in self.energy.as_dict().items()},
                "deadline_misses": self.deadline_misses,
                "constraint_violations": self.constraint_violations,
                "max_tracking_error_m": round(self.max_tracking_error_m, 6),
                "final_distance_m": round(self.final_distance_m, 6),
                "fallback_count": self.fallback_count, "mode_changes": self.mode_changes,
                "service_time_limit_s": self.service_time_limit_s,
                "service_target_met": self.service_target_met}
