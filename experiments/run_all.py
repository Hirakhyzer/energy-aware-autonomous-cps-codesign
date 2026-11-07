"""Run paired benchmark scenarios with a stable set of seeds."""
from pathlib import Path
from eco_cps.benchmark import benchmark, save_results, summarize
from eco_cps.config import load_scenario

if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    scenarios = [load_scenario(root / "configs" / name) for name in ("baseline.json", "heavy-load.json")]
    results = benchmark(scenarios, seeds=(0, 1, 2, 3, 4), include_traces=False)
    output = save_results(results, root / "results" / "paper-benchmark")
    for row in summarize(results):
        print(row)
    print(f"Artifacts: {output}")
