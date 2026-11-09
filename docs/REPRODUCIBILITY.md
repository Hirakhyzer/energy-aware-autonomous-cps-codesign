# Reproducibility and code quality

## Environment

- Python 3.10, 3.11, 3.12, or 3.13.
- v0.3 uses only synthetic bounded uncertainty and disturbances; hardware is not required.
- Standard-library-only runtime; `pytest` is used for development tests.
- Install with `python -m pip install -e '.[dev]'`.
- Run `python -m pytest -q` before generating evidence.

## Seed and configuration discipline

The simulator uses independent local `random.Random` streams for localization observations, episode-level parameter uncertainty, per-tick disturbances, and per-tick workload variation; it does not modify global random state. The discrete clock advances by integer tick; the same seed and scenario are expected to produce bit-for-bit identical JSON within the same interpreter/runtime. Scenario JSON is checked against known dataclass fields; all numeric model parameters should be preserved when making comparisons.

## Experiments

Run `eco-cps benchmark --config configs/baseline.json --seeds 0 1 2 --out results/benchmark --traces`. For v0.2, use `eco-cps benchmark --config configs/qos-18s.json --seeds 0 1 2 --out results/qos-18s`, followed by `eco-cps report --input results/qos-18s --paired` to inspect matched-seed outcomes and service-target attainment. For v0.3, run `python experiments/run_robust.py` to evaluate 100 matched seeds on both bounded-uncertainty configurations. The output includes per-run, aggregate, and paired analysis JSON/CSV; the trace option writes per-step positioning, energy, and selected modes. Keep external backups of the exact code revision, configuration, generated data, and experiment environment when publishing. Hardware calibration is not part of this simulation-only release.

## Limitations

Scenarios and uncertainty bounds are illustrative; CI verifies code behavior and contract-level properties, not validation against external CPS hardware or a proof of energy optimality. The v0.3 milestone is intentionally simulation-only. Numerical results should include processor, sampling, platform, and software-version provenance when transferred to hardware.
