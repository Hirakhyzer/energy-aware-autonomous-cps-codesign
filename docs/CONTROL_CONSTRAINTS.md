# Control, compute, and battery constraints

## Candidate feasibility guard

A candidate must pass each of the following **model-based** checks:

1. `sensor_noise_m + process_noise_m_per_s / sensor_hz <= max_position_uncertainty_m` (an approximation, not a probabilistic confidence bound).
2. `control_period_s <= max_control_period_s`.
3. Worst coincident modeled CPU work must finish within `cpu_utilization_limit * dt_s`.
4. Current time plus predicted remaining travel time and cornering allowance must not exceed `deadline_s`.
5. Remaining battery energy minus predicted mission energy must preserve `reserve_energy_j`.
6. In v0.2, `joint_qos` optionally adds `now + predicted_remaining_time <= service_time_limit_s`; `joint` and single-layer baselines do **not** use this additional selection constraint.

## Service-time target versus mission deadline

The hard modeled mission deadline (`deadline_s`) bounds the simulation horizon for every policy. The optional performance target (`service_time_limit_s`) is a **stricter quality-of-service metric**, with `0 < service_time_limit_s <= deadline_s`. The v0.2 `joint_qos` policy uses a simple route-length / cruise-speed + corner penalty forecast to reject slow configurations; this estimate is not a validated closed-loop arrival-time guarantee. Actual target attainment is computed after simulation, including the effect of early failures. When all configurations are predicted infeasible the fixed fallback is chosen and counted, without asserting that the fallback meets the target.

## Monitored execution constraints

Every step records actual modeled CPU utilization deadline misses, route-segment cross-track error, estimated uncertainty, and remaining battery energy. Violations are counted. Mission completion is checked at each waypoint using a 0.20 m arrival radius.

## Critical limitation

There is **no formally proven closed-loop safety invariant**. Candidate checks do not directly bound all future trajectories. The `max_cross_track_error_m` requirement is monitored, not included in an exact reachability calculation. A conservative fixed-mode fallback is logged when no candidate remains feasible; it does not guarantee the fallback satisfies the constraints.

An actual embedded autonomous platform would require separately verified actuator limits, collision avoidance, sensor fault handling, safe-stop logic, real-time runtime measurement, and domain-specific certification.
