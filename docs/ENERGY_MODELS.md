# Energy and execution-time models

All coefficients in this document are **illustrative simulation assumptions**. They have not been calibrated to real hardware.

For each simulation step of duration `dt`, the energy ledger contains:

- `E_sensing += (0.025 + 0.020 * sensor_hz) * dt` joules.
- `E_computing += 0.065 * dt + (0.05 + 0.000020 * cpu_mhz²) * task_runtime_s` joules.
- `E_communication += 0.008` joules for every synthetic observation.
- `E_actuation += motion_load * (0.50 + 2.7 * speed_mps² + 0.11 * abs(turn_radps)) * dt` joules.
- `E_switching += switch_energy_j` each time an operating mode changes.

A task's modeled execution time is `cycles / (cpu_mhz * 1e6)`, with a 200,000-cycle monitoring overhead per step, 500,000 cycles per observation, 1,000,000 cycles per control update, and 350,000 cycles per optimization. The benchmark's feasibility check assumes all task types coincide in a single simulation tick. Actual runtime execution is recorded separately.

This simple CPU model is *deliberately* nonlinear in clock rate so that the relative energy cost of computing modes can be studied. It is **not** a physically fitted voltage-frequency model. For any proposed real processor, measure actual power, wake-up overhead, latency, temperature, and feasible clocks.

## Predicted mission cost

The optimizer uses an approximation: `predicted_energy = remaining_route_distance / cruise_speed * predicted_total_power`, with an additional 0.7 s allowance per remaining corner. Candidate switching incurs a joule surcharge. These projections can differ from actual integrated energy because heading correction, speed reduction in turns, noisy observations, and waypoint tolerances affect motion.

## Accounting checklist for hardware validation

Measure idle current and active CPU current separately, sensor sample/wake energy, sensor leakage, wireless transmission energy, motor power vs speed/turn/load, battery voltage drop, and thermal effects. Validate energy by `E = integral(V(t)*I(t) dt)` with synchronized timestamps and correct instrument sampling bandwidth. Report measurement accuracy and repeatability. Avoid double-counting idle energy or sensor energy in system-level battery measurements.
