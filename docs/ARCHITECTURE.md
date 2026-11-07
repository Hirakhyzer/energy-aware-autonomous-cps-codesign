# ECO-CPS architecture

## Components and data flow

1. `models.py` defines typed operating modes, scenario limits, robot state, and energy ledger.
2. `plant.py` integrates a sampled planar unicycle model and generates seeded, noisy position observations. Estimated position is corrected by a simple complementary filter. Each update uses bounded proportional heading correction.
3. `energy.py` defines component-level power and CPU execution-time proxies. Units are seconds, watts, joules, meters, radians, and hertz.
4. `policies.py` enumerates operating modes and evaluates model-based feasibility checks for position uncertainty, control period, one-step CPU execution, route completion time, and energy reserve. The `joint` policy searches all 81 combinations; ablation policies only vary the named dimension.
5. `simulation.py` executes closed-loop robot motion on a fixed-step clock and records energy, completion, deadline misses, tracking errors, and fallback events.
6. `benchmark.py` evaluates each policy with matched seeds, aggregates by (scenario, policy), and writes evidence artifacts.
7. `cli.py` exposes the `simulate`, `benchmark`, and `report` commands.

## Model boundaries

The robot is a synthetic, differential-drive-compatible unicycle, **not** a dynamic motor or embedded controller emulator. Processor clock and execution cycles are proxies, not actual DVFS tuning. Sensor duty-cycle is average-power modeled and observations occur at configured intervals. Communication is approximated as a per-sample joule expense. No network stack, driver, firmware, or robotic middleware is controlled.

## Decision cycle

Every optimization period, the supervisory policy ranks configurations by predicted total mission energy. Feasibility rejection occurs before energy ranking. Chosen modes determine sample intervals and controller update intervals. Hardware-like CPU work is accounted for in the energy ledger. Each physical step monitors explicit limits and records evidence.

The fallback is a high-fidelity fixed mode, not an emergency stop or safety shield. When no operating mode is feasible, experiments may legitimately fail: they are **not** reported as guaranteed safe runs.
