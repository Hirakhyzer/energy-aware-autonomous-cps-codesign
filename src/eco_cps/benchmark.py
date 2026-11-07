"""Paired reproducible benchmarks and auditable JSON/CSV artifacts."""
from __future__ import annotations
import csv
import json
from pathlib import Path
from statistics import mean, stdev
from .models import PolicyName, RunResult, Scenario
from .simulation import simulate

POLICIES: tuple[PolicyName, ...] = ("fixed", "sensing", "computing", "control", "joint")


def benchmark(scenarios: list[Scenario] | None = None, *, seeds: tuple[int, ...] = (0, 1, 2),
              include_traces: bool = False) -> list[RunResult]:
    if not seeds:
        raise ValueError("At least one seed is required")
    if len(set(seeds)) != len(seeds):
        raise ValueError("Seeds must be unique")
    scenarios = scenarios if scenarios is not None else [Scenario()]
    if not scenarios:
        raise ValueError("At least one scenario is required")
    return [simulate(policy, scenario, seed=seed, include_trace=include_traces)
            for scenario in scenarios for seed in seeds for policy in POLICIES]


def summarize(results: list[RunResult]) -> list[dict[str, object]]:
    if not results:
        raise ValueError("Cannot summarize an empty benchmark")
    groups: dict[tuple[str, str], list[RunResult]] = {}
    for r in results:
        groups.setdefault((r.scenario, r.policy), []).append(r)
    output: list[dict[str, object]] = []
    for (scenario, policy), group in sorted(groups.items()):
        energies = [r.energy.total_j for r in group]
        output.append({"scenario": scenario, "policy": policy, "trials": len(group),
                       "completed": sum(r.completed for r in group),
                       "mean_energy_j": round(mean(energies), 6),
                       "sd_energy_j": round(stdev(energies), 6) if len(group) > 1 else 0.0,
                       "mean_duration_s": round(mean(r.duration_s for r in group), 6),
                       "deadline_misses": sum(r.deadline_misses for r in group),
                       "constraint_violations": sum(r.constraint_violations for r in group),
                       "mean_max_tracking_error_m": round(mean(r.max_tracking_error_m for r in group), 6),
                       "fallback_count": sum(r.fallback_count for r in group)})
    return output


def save_results(results: list[RunResult], directory: str | Path, *, traces: bool = False) -> Path:
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    records = [r.summary() for r in results]
    (out / "runs.json").write_text(json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (out / "runs.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    summary = summarize(results)
    (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (out / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    if traces:
        trace_dir = out / "traces"
        trace_dir.mkdir(exist_ok=True)
        for r in results:
            (trace_dir / f"{r.scenario}-{r.policy}-{r.seed}.json").write_text(
                json.dumps(r.trace, indent=2) + "\n", encoding="utf-8")
    return out


def percent_energy_difference(fixed_energy: float, candidate_energy: float) -> float:
    if fixed_energy <= 0:
        raise ValueError("Baseline energy must be positive")
    return 100.0 * (fixed_energy - candidate_energy) / fixed_energy
