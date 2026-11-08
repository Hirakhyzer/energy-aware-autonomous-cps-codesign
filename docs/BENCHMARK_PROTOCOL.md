# Benchmark protocol

## Policies

- `fixed`: sensor 10 Hz, CPU 240 MHz, control 0.1 s, cruise speed 0.8 m/s.
- `sensing`: vary only sensor rates 2/5/10 Hz.
- `computing`: vary only CPU rates 80/160/240 MHz.
- `control`: vary only control periods 0.1/0.2/0.4 s and speeds 0.4/0.6/0.8 m/s.
- `joint`: vary all 81 sensor/CPU/control/speed combinations, minimizing predicted energy.
- `joint_qos` (v0.2): enumerate the same 81 configurations but reject modes whose predicted remaining travel would miss the optional `service_time_limit_s`. This is a prediction, not a guarantee.

Every adaptive policy uses the same finite-search objective, guard thresholds, and switching accounting. For paired comparisons, every policy receives the same seed and scenario. Different control decisions change when observations occur, so shared seeds do not guarantee identical noise sequences per simulation tick.

## v0.2 service-time quality-of-service benchmark

Run `eco-cps benchmark --config configs/qos-18s.json --seeds 0 1 2 --out results/qos-18s`, then `eco-cps report --input results/qos-18s --paired`.

The `service_time_limit_s` value is a performance target separate from `deadline_s`. It is monitored across **all six** policies but used during mode selection only by `joint_qos`. Report target attainment and mission completion for **all** policies rather than excluding slower policies after the fact. Absent a target, `joint_qos` is deliberately equivalent to `joint` under the same conditions.

`paired.json` and `paired.csv` compare each candidate policy with the fixed baseline for seeds that are present in both. Each record includes number of matched pairs, paired successful completions, total completions, service-target counts, signed energy and duration differences, and mean percent savings among successful pairs. Negative energy differences (`candidate − fixed`) indicate less candidate energy, while positive time differences indicate slower candidate missions. Incomplete trials contribute to success counts but are **excluded from conditional energy/time effects**; missing seeds are never imputed. Always report both denominators.

The deterministic seeded 2000-draw percentile bootstrap of paired differences is intended to demonstrate reproducible uncertainty reporting, **not validated population-level uncertainty**, particularly at 3 seeds. Use more independent scenarios and independently calibrated hardware data for research claims.

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

Artifacts are saved in `results/paper-benchmark`. They are generated, not shipped as precomputed experimental evidence.
