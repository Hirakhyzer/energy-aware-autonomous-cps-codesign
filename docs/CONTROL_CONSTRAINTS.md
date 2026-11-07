# Control, compute, and battery constraints

## Candidate feasibility guard

A candidate must pass each of the following **model-based** checks:

1. `sensor_noise_m + process_noise_m_per_s / sensor_hz <= max_position_uncertainty_m` (an approximation, not a probabilistic confidence bound).
2. `control_period_s <= max_control_period_s`.
3. Worst coincident modeled CPU work must finish within `cpu_utilization_limit * dt_s`.
4. Current time plus predicted remaining travel time and cornering allowance must not exceed `deadline_s`.
5. Remaining battery energy minus predicted mission energy must preserve `reserve_energy_j`.

## Monitored execution constraints

Every step records actual modeled CPU utilization deadline misses, route-segment cross-track error, estimated uncertainty, and remaining battery energy. Violations are counted. Mission completion is checked at each waypoint using a 0.20 m arrival radius.

## Critical limitation

There is **no formally proven closed-loop safety invariant**. Candidate checks do not directly bound all future trajectories. The `max_cross_track_error_m` requirement is monitored, not included in an exact reachability calculation. A conservative fixed-mode fallback is logged when no candidate remains feasible; it does not guarantee the fallback satisfies the constraints.

An actual embedded autonomous platform would require separately verified actuator limits, collision avoidance, sensor fault handling, safe-stop logic, real-time runtime measurement, and domain-specific certification.
