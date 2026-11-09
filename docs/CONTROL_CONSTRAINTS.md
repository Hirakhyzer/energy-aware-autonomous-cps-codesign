# Control, compute, and battery constraints

## Nominal candidate feasibility

A candidate must pass each of the following model-based checks:

1. `sensor_noise_m + process_noise_m_per_s / sensor_hz <= max_position_uncertainty_m`.
2. `control_period_s <= max_control_period_s`.
3. Worst coincident modeled CPU work must finish within `cpu_utilization_limit * dt_s`.
4. Current time plus predicted remaining travel time and cornering allowance must not exceed `deadline_s`.
5. Remaining battery energy minus predicted mission energy must preserve `reserve_energy_j`.
6. `joint_qos` optionally adds `now + predicted_remaining_time <= service_time_limit_s`.

These checks are approximate model-based screens, not formal safety invariants.

## v0.3 robust screening

For `joint_robust`, the screening bounds are inflated using the declared uncertainty set:

- sensor and process-noise bounds are multiplied by `1 + uncertainty_fraction`;
- coincident CPU execution time is multiplied by both `1 + uncertainty_fraction` and `1 + workload_variation_fraction`;
- predicted remaining mission time uses a conservative progress speed `cruise_speed_mps - sqrt(2) * disturbance_mps`, then applies the episode-level time uncertainty factor;
- predicted energy uses the maximum modeled rate from the five stress cases;
- an optional `service_time_limit_s` is enforced in addition to the hard mission deadline.

If disturbance-adjusted speed is nonpositive, the candidate is rejected. If no candidate remains, the fixed fallback is selected and logged. A fallback count is not a success indicator.

## Service-time target versus mission deadline

The hard modeled mission deadline (`deadline_s`) bounds the simulation horizon for every policy. The optional performance target (`service_time_limit_s`) is a stricter quality-of-service metric with `0 < service_time_limit_s <= deadline_s`. `joint_qos` and `joint_robust` use the forecast to reject candidate modes that are predicted to miss the target. Actual target attainment is computed after simulation, including early failures. The forecast is not a validated closed-loop arrival-time guarantee.

## Monitored constraints

Every step records modeled CPU deadline misses, route-segment cross-track error, estimated uncertainty, and remaining battery energy. Violations are counted. Mission completion is checked at each waypoint using a 0.20 m arrival radius. Bounded x/y velocity disturbances and per-tick CPU workload variation are applied in the v0.3 scenarios.

## Critical limitation

There is no formally proven closed-loop safety invariant. Candidate checks do not bound all future trajectories. The `max_cross_track_error_m` requirement is monitored, not included in an exact reachability calculation. The fixed-mode fallback does not guarantee that constraints are met. Results are limited to the declared simulator, uncertainty intervals, disturbance bounds, and seeds.
