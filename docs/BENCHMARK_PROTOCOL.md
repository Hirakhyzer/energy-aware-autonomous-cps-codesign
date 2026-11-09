# Benchmark protocol

## Policies

- `fixed`: sensor 10 Hz, CPU 240 MHz, control 0.1 s, cruise speed 0.8 m/s.
- `sensing`: vary only sensor rates 2/5/10 Hz.
- `computing`: vary only CPU rates 80/160/240 MHz.
- `control`: vary only control periods 0.1/0.2/0.4 s and speeds 0.4/0.6/0.8 m/s.
- `joint`: vary all 81 sensor/CPU/control/speed combinations, minimizing predicted energy.
- `joint_qos` (v0.2): enumerate the same 81 configurations but reject modes whose predicted remaining travel would miss the optional `service_time_limit_s`. This is a prediction, not a guarantee.
- `joint_robust` (v0.3): evaluate all 81 configurations against a five-case uncertainty stress set, screen candidates using upper-bound sensor noise, CPU execution time, disturbance-adjusted speed, deadline, and battery reserve, then minimize a risk-adjusted energy score.

All adaptive policies use the same joint finite configuration grid and switching accounting, but their objective and robustness screening differ by policy. For paired comparisons, every policy receives the same seed and scenario. Episode-level uncertainty factors, per-tick workload variation, and per-tick x/y disturbances use independent seeded streams and are paired across policies. Sensor measurement noise is sampled only when a policy takes an observation, so observation-noise sequences are not identical tick-for-tick across policies with different sampling rates.

## v0.2 service-time quality-of-service benchmark

Run `eco-cps benchmark --config configs/qos-18s.json --seeds 0 1 2 --out results/qos-18s`, then `eco-cps report --input results/qos-18s --paired`.

The `service_time_limit_s` value is a performance target separate from `deadline_s`. It is monitored across **all seven** policies but used during mode selection by `joint_qos` and `joint_robust`. Report target attainment and mission completion for **all** policies rather than excluding slower policies after the fact. Absent a target, `joint_qos` is deliberately equivalent to `joint` under the same conditions.

`paired.json` and `paired.csv` compare each candidate policy with the fixed baseline for seeds that are present in both. Each record includes number of matched pairs, paired successful completions, total completions, service-target counts, signed energy and duration differences, and mean percent savings among successful pairs. Negative energy differences (`candidate − fixed`) indicate less candidate energy, while positive time differences indicate slower candidate missions. Incomplete trials contribute to success counts but are **excluded from conditional energy/time effects**; missing seeds are never imputed. Always report both denominators.

The deterministic seeded 2000-draw percentile bootstrap of paired differences is a descriptive interval for the simulated matched-seed sample, not a guarantee about a wider real-world population. v0.2 examples with three seeds are especially exploratory; v0.3 runs 100 seeds per scenario. Broader scenario families and alternative model structures are still needed before making general claims.

## Core metrics

- Total joules and individual subsystem joules.
- Mission completion (numerator and denominator, never omitted).
- Mission duration; energy per successfully completed mission.
- CPU deadline misses and state-constraint violation counts.
- Maximum cross-track error and final waypoint distance.
- Conservative fallback activations and operating-mode changes.

## Recommended analysis

Use at least 30 randomized seeds for exploratory comparisons and a separately declared, preregistered evaluation suite for final claims. For experimental comparisons, present paired differences with confidence intervals using a documented resampling procedure. Report all incomplete missions and failures, not just successful energy means. Examine sensitivity to load, localization noise, route complexity, deadline, and battery reserve. Distinguish the estimated-energy model from any measured ground-truth energy.

## Run full example

```bash
python experiments/run_all.py
```

Artifacts are saved in `results/paper-benchmark`. They are generated, not shipped as precomputed experimental evidence. `save_results` also writes a deterministic `runs.json.gz` archive for compact per-run sharing.

## v0.3 bounded uncertainty and disturbance experiments

Use `configs/robust-moderate.json` and `configs/robust-high.json`. The `uncertainty_fraction` parameter defines a symmetric multiplicative interval `[1-u, 1+u]` for episode-level motion load, compute power, sensing power, communication cost, sensor noise, process-noise rate, and CPU execution time. Each seed draws one factor per quantity from a local, independently seeded random stream. This creates matched latent episode conditions across policies. `disturbance_mps` defines a bounded per-step additive velocity disturbance independently sampled on the x and y axes from `[-d, d]`. `workload_variation_fraction` samples a fresh per-tick CPU task-demand multiplier from `[1-w, 1+w]`.

`joint_robust` uses a five-case stress set: nominal, high motion load, high compute cost (including runtime uncertainty), high sensing/communication cost, and all modeled energy costs high together. Its energy objective is `mean(stress_energy) + rho * population_std(stress_energy)`, where `rho = robust_risk_weight` lies in `[0,1]`. Increasing `rho` makes the optimizer more sensitive to dispersion across the declared stress cases. It also screens against upper-bound sensor/process noise, CPU execution time, and a conservative disturbance-adjusted speed. These are finite-model robustness heuristics, not formal robust-control guarantees.

Run `python experiments/run_robust.py` for 100 matched seeds per scenario across moderate, high, and low-battery conditions (2,100 policy-mission runs total). Report completion rate, all-run mean energy, completed-run mean energy, 95th-percentile energy, mission duration, deadline misses, constraint violations, and paired differences. Do not discard failed missions.
