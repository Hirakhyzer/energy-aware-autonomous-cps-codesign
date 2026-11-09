# v0.3 uncertainty and disturbance model

## Scope

This milestone is entirely software-based. It uses bounded mathematical uncertainty and synthetic disturbances in the reduced-order robot simulator. No hardware, physical robot, measured battery, or hardware-in-the-loop data are required or included.

## Episode-level parameter uncertainty

For a scenario uncertainty fraction `u` in `[0, 0.5]`, each run draws independent factors uniformly from `[1-u, 1+u]` for:

- motion load;
- computing power;
- sensing power;
- communication energy per observation;
- sensor measurement noise;
- process-noise growth;
- CPU execution time.

In addition, `workload_variation_fraction` samples a new multiplicative CPU workload factor at every simulation tick. It models changing embedded task demand within a mission, rather than only changing the workload between episodes.

The uncertainty stream is seeded independently of the localization-noise and disturbance streams. Consequently, the same scenario and seed use the same episode-level factors across policy comparisons, even when policies sample at different rates. Factors are written into each run's JSON/CSV record for auditability.

## Per-step disturbances

At each simulation tick, independent x/y velocity disturbances are drawn uniformly from `[-disturbance_mps, +disturbance_mps]`. The plant integrates commanded velocity plus disturbance. The robust feasibility check uses a conservative projection bound of `sqrt(2) * disturbance_mps` against cruise speed.

## Robust candidate screening

The `joint_robust` policy evaluates the joint 81-mode grid. It rejects candidates that violate upper-bound estimation uncertainty, CPU execution budget, disturbance-adjusted progress, mission deadline, optional service-time target, or predicted battery reserve. Energy ranking uses five stress cases and a mean-plus-standard-deviation risk premium. The risk weight `rho=0` uses the stress-set mean only; `rho=1` applies the full population-standard-deviation premium.

This method is a transparent finite-scenario heuristic. It does not establish worst-case guarantees outside the specified uncertainty set, nor does it prove closed-loop stability or safety.

## Experimental design

The committed experiment runner evaluates seven policies on 100 matched seeds for each of three declared scenarios: moderate uncertainty, high uncertainty, and a low-battery stress case. Results are synthetic and reproducible under the same code/configuration/runtime. Statistical intervals from paired bootstrap analysis describe this simulation sample; they are not evidence about physical hardware or a wider real-world population.
