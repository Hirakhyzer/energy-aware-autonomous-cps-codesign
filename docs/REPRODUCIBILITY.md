# Reproducibility and code quality

## Environment

- Python 3.10, 3.11, 3.12, or 3.13.
- Standard-library-only runtime; `pytest` is used for development tests.
- Install with `python -m pip install -e '.[dev]'`.
- Run `python -m pytest -q` before generating evidence.

## Seed and configuration discipline

The simulator uses `random.Random(seed)` locally; it does not modify global random state. The discrete clock advances by integer tick; the same seed and scenario are expected to produce bit-for-bit identical JSON within the same interpreter/runtime. Scenario JSON is checked against known dataclass fields; all numeric model parameters should be preserved when making comparisons.

## Experiments

Run `eco-cps benchmark --config configs/baseline.json --seeds 0 1 2 --out results/benchmark --traces`. The output includes per-run and aggregated JSON/CSV; the trace option writes per-step positioning, energy, and selected modes. Keep external backups of the exact code revision, configuration, generated data, hardware calibration version, and experiment environment when publishing.

## Limitations

Scenarios are illustrative; CI verifies code behavior and contract-level properties, not validation against external CPS hardware or a proof of energy optimality. Numerical results should include processor, sampling, platform, and software-version provenance when transferred to hardware.
