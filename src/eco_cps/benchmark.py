"""Paired reproducible benchmarks and auditable JSON/CSV artifacts."""
from __future__ import annotations
import csv
import gzip
import json
from pathlib import Path
from statistics import mean, stdev
from math import ceil
from .models import PolicyName, RunResult, Scenario
from .simulation import simulate

POLICIES: tuple[PolicyName, ...] = ("fixed", "sensing", "computing", "control", "joint", "joint_qos", "joint_robust")


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
        completed_runs = [r for r in group if r.completed]
        sorted_energies = sorted(energies)
        p95_energy = sorted_energies[min(len(sorted_energies) - 1, ceil(0.95 * len(sorted_energies)) - 1)]
        output.append({"scenario": scenario, "policy": policy, "trials": len(group),
                       "completed": len(completed_runs),
                       "completion_rate": round(len(completed_runs) / len(group), 6),
                       "mean_energy_j": round(mean(energies), 6),
                       "mean_completed_energy_j": round(mean(r.energy.total_j for r in completed_runs), 6) if completed_runs else None,
                       "p95_energy_j": round(p95_energy, 6),
                       "sd_energy_j": round(stdev(energies), 6) if len(group) > 1 else 0.0,
                       "mean_duration_s": round(mean(r.duration_s for r in group), 6),
                       "mean_completed_duration_s": round(mean(r.duration_s for r in completed_runs), 6) if completed_runs else None,
                       "deadline_misses": sum(r.deadline_misses for r in group),
                       "constraint_violations": sum(r.constraint_violations for r in group),
                       "mean_max_tracking_error_m": round(mean(r.max_tracking_error_m for r in group), 6),
                       "fallback_count": sum(r.fallback_count for r in group),
                       "service_time_limit_s": group[0].service_time_limit_s,
                       "service_target_met": (sum(r.service_target_met is True for r in group)
                                              if group[0].service_time_limit_s is not None else None)})
    return output


def save_results(results: list[RunResult], directory: str | Path, *, traces: bool = False) -> Path:
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    records = [r.summary() for r in results]
    run_json = json.dumps(records, indent=2, sort_keys=True) + "\n"
    (out / "runs.json").write_text(run_json, encoding="utf-8")
    # Stable gzip metadata keeps the archived raw experiment reproducible.
    with (out / "runs.json.gz").open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            compressed.write(run_json.encode("utf-8"))
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
    paired = paired_analysis(results)
    (out / "paired.json").write_text(json.dumps(paired, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if paired:
        with (out / "paired.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(paired[0]))
            writer.writeheader()
            writer.writerows(paired)
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


def _bootstrap_mean_ci(values: list[float], *, draws: int = 2000) -> tuple[float | None, float | None]:
    """Exploratory percentile bootstrap on *paired differences*, with deterministic RNG.

    This is descriptive uncertainty, not a generalization guarantee or a formal
    confidence interval for the underlying physical CPS process.
    """
    if len(values) < 2:
        return None, None
    import random

    rng = random.Random(2026)
    n = len(values)
    boot = sorted(mean(values[rng.randrange(n)] for _ in range(n)) for _ in range(draws))
    # Nearest observed order statistics avoid interpolation artifacts for small n.
    return round(boot[int(0.025 * draws)], 6), round(boot[int(0.975 * draws) - 1], 6)


def paired_analysis(results: list[RunResult]) -> list[dict[str, object]]:
    """Matched-seed comparisons against fixed control, preserving failure counts.

    Energy and duration effects use only pairs where *both* policies completed.
    All paired missions, including failures, contribute to completion and QoS
    counts. Missing fixed/candidate pairs are not manufactured or imputed.
    """
    seen: dict[tuple[str, int, str], RunResult] = {}
    for run in results:
        key = run.scenario, run.seed, run.policy
        if key in seen:
            raise ValueError(f"Duplicate scenario/seed/policy: {key}")
        seen[key] = run
    scenario_policies: dict[tuple[str, str], list[int]] = {}
    for scenario, seed, policy in seen:
        if policy == "fixed":
            continue
        if (scenario, seed, "fixed") in seen:
            scenario_policies.setdefault((scenario, policy), []).append(seed)
    report: list[dict[str, object]] = []
    for (scenario, policy), seeds in sorted(scenario_policies.items()):
        matched = [(seen[(scenario, s, "fixed")], seen[(scenario, s, policy)])
                   for s in sorted(seeds)]
        if any(base.service_time_limit_s != other.service_time_limit_s
               for base, other in matched):
            raise ValueError("Paired policies must use the same service-time target")
        complete = [(base, other) for base, other in matched if base.completed and other.completed]
        delta_j = [other.energy.total_j - base.energy.total_j for base, other in complete]
        delta_s = [other.duration_s - base.duration_s for base, other in complete]
        energy_ci = _bootstrap_mean_ci(delta_j)
        duration_ci = _bootstrap_mean_ci(delta_s)
        # Positive savings mean the candidate uses less energy.
        report.append({
            "scenario": scenario,
            "policy": policy,
            "paired_trials": len(matched),
            "complete_pairs": len(complete),
            "fixed_completions": sum(base.completed for base, _ in matched),
            "candidate_completions": sum(other.completed for _, other in matched),
            "service_time_limit_s": matched[0][0].service_time_limit_s,
            "fixed_service_met": (sum(base.service_target_met is True for base, _ in matched)
                                  if matched[0][0].service_time_limit_s is not None else None),
            "candidate_service_met": (sum(other.service_target_met is True for _, other in matched)
                                      if matched[0][0].service_time_limit_s is not None else None),
            "mean_energy_delta_j": round(mean(delta_j), 6) if delta_j else None,
            "energy_delta_ci95_low_j": energy_ci[0],
            "energy_delta_ci95_high_j": energy_ci[1],
            "mean_duration_delta_s": round(mean(delta_s), 6) if delta_s else None,
            "duration_delta_ci95_low_s": duration_ci[0],
            "duration_delta_ci95_high_s": duration_ci[1],
            "mean_energy_saving_pct": (round(mean(
                100.0 * (base.energy.total_j - other.energy.total_j) / base.energy.total_j
                for base, other in complete), 6) if complete else None),
        })
    return report
