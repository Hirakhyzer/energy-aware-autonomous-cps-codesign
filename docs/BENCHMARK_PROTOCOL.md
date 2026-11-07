# Benchmark protocol

## Policies

- `fixed`: sensor 10 Hz, CPU 240 MHz, control 0.1 s, cruise speed 0.8 m/s.
- `sensing`: vary only sensor rates 2/5/10 Hz.
- `computing`: vary only CPU rates 80/160/240 MHz.
- `control`: vary only control periods 0.1/0.2/0.4 s and speeds 0.4/0.6/0.8 m/s.
- `joint`: vary all 81 sensor/CPU/control/speed combinations.

Every adaptive policy uses the same finite-search objective, guard thresholds, and switching accounting. For paired comparisons, every policy receives the same seed and scenario. Different control decisions change when observations occur, so shared seeds do not guarantee identical noise sequences per simulation tick.

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
