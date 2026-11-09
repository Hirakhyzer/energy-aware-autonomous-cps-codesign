"""Run the v0.3 uncertainty benchmark with 100 matched seeds per scenario.

All results are generated from synthetic models; this script does not require hardware.
"""
from pathlib import Path
from eco_cps.benchmark import benchmark, save_results, summarize
from eco_cps.config import load_scenario

if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    scenarios = [load_scenario(root / "configs" / name)
                 for name in ("robust-moderate.json", "robust-high.json", "robust-low-battery.json")]
    seeds = tuple(range(100))
    for scenario in scenarios:
        results = benchmark([scenario], seeds=seeds, include_traces=False)
        output = save_results(results, root / "results" / "v0.3" / scenario.name)
        print(f"\nScenario: {scenario.name}; seeds: {len(seeds)}; runs: {len(results)}")
        for row in summarize(results):
            print(row)
        print(f"Artifacts: {output}")
