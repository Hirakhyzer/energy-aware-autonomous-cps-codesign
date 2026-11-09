# Energy and execution-time models

All coefficients in this document are illustrative simulation assumptions. They have not been calibrated to real hardware, and no hardware is required for v0.3.

## Nominal component model

For each simulation step of duration `dt`, the energy ledger contains:

- `E_sensing += (0.025 + 0.020 * sensor_hz) * dt` joules.
- `E_computing += 0.065 * dt + (0.05 + 0.000020 * cpu_mhz²) * task_runtime_s` joules.
- `E_communication += 0.008` joules for every synthetic observation.
- `E_actuation += motion_load * (0.50 + 2.7 * speed_mps² + 0.11 * abs(turn_radps)) * dt` joules.
- `E_switching += switch_energy_j` each time an operating mode changes.

A task's modeled execution time is `cycles / (cpu_mhz * 1e6)`, with a 200,000-cycle monitoring overhead per step, 500,000 cycles per observation, 1,000,000 cycles per control update, and 350,000 cycles per optimization. The feasibility check assumes all task types coincide in a single simulation tick.

## v0.3 uncertainty and workload

`uncertainty_fraction = u` defines independent episode-level multipliers sampled uniformly from `[1-u, 1+u]` for motion load, compute power, sensing power, communication cost, sensor measurement noise, process-noise growth, and CPU execution time. `workload_variation_fraction = w` samples a new CPU task-demand multiplier from `[1-w, 1+w]` at every tick. `disturbance_mps = d` samples independent additive x/y velocity disturbances from `[-d, d]` at every tick.

The `joint_robust` optimizer evaluates five energy stress cases: nominal; upper-bound motion load; upper-bound compute power/runtime/workload; upper-bound sensing and communication cost; and all modeled energy costs elevated together. It uses a risk-adjusted cost as a premium on the stress-case standard deviation. These stress cases are intentionally transparent and limited; they do not cover every possible joint distribution or guarantee worst-case performance outside the declared bounds.

## Predicted mission cost

The optimizer approximates `predicted_energy = predicted_remaining_time * predicted_total_power`, with a cornering allowance and a switching surcharge. Robust feasibility uses a disturbance-adjusted speed lower bound. Predictions can differ from integrated simulated energy because heading correction, per-step disturbances, noisy observations, workload variation, waypoint tolerance, and mode changes affect the trajectory.

## Interpretation

Every joule in the repository is a synthetic model output. The v0.3 results demonstrate behavior under declared simulated assumptions only; they are not measured energy consumption, device-level DVFS results, or physical robot performance.
