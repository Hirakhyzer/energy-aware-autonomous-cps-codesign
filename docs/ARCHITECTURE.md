# ECO-CPS architecture (v0.3)

## Components and data flow

1. `models.py` defines operating modes, mission constraints, uncertainty bounds, per-tick workload variation, disturbance bounds, robot state, and energy/result ledgers.
2. `plant.py` integrates a sampled planar unicycle model, generates noisy position observations, and applies independently seeded bounded x/y velocity disturbances. Estimated position is corrected by a simple complementary filter.
3. `energy.py` defines component-level power and CPU execution-time proxies. It also evaluates nominal and upper-bound subsystem-cost combinations for robust candidate ranking.
4. `policies.py` implements fixed and single-layer ablations, nominal joint energy search (`joint`), service-time-aware search (`joint_qos`), and the v0.3 risk-aware `joint_robust` search over 81 sensing/CPU/control/speed configurations.
5. `simulation.py` executes the closed-loop mission. Separate random streams generate localization observations, episode-level parameter uncertainty, per-step disturbances, and per-step CPU workload variation. These streams are reproducible and paired across policies for the same seed.
6. `benchmark.py` evaluates seven policies on matched seeds and writes per-run, aggregate, and paired JSON/CSV evidence, including completion rates, completed-run metrics, and p95 energy.
7. `cli.py` exposes `simulate`, `benchmark`, and `report`; `experiments/run_robust.py` runs the 100-seed-per-scenario v0.3 evaluation.

## v0.3 robust decision cycle

At each optimization period, `joint_robust`:

1. Enumerates the 81 candidate operating modes.
2. Rejects modes that fail upper-bound sensor/process-noise, CPU execution, disturbance-adjusted progress, deadline, optional service-time, or battery-reserve checks.
3. Evaluates five synthetic energy stress cases: nominal; high motion load; high computing cost; high sensing/communication cost; and all modeled energy costs elevated together.
4. Scores each candidate with `mean(stress_energy) + rho * std(stress_energy)`, plus the existing switching penalty.
5. Selects the lowest-scoring feasible mode, or uses and records the fixed high-fidelity fallback if no candidate passes.

The risk weight `rho` ranges from zero (stress-set mean only) to one (full stress-set standard-deviation premium). This is a transparent finite-scenario heuristic, not a formal robust-control theorem or proof of safety.

## Model boundaries

The robot is a synthetic, differential-drive-compatible unicycle, not a motor or embedded-controller emulator. Processor clock and execution cycles are proxies, not actual DVFS tuning. Energy coefficients are illustrative, not measured. The v0.3 uncertainty factors and disturbances are synthetic. No physical robot, hardware, firmware, device driver, or robotic middleware is required or controlled.

## Fallback semantics

The fallback is a high-fidelity fixed mode, not an emergency stop or safety shield. When no operating mode is predicted feasible, experiments may legitimately use the fallback and log it; the fallback is not guaranteed to meet energy, deadline, or safety constraints.
