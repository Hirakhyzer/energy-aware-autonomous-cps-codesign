"""Empirical energy-duration Pareto analysis of completed, constraint-clean policies.

This is descriptive analysis of simulated aggregate results, not a safety proof.
"""
from __future__ import annotations


def pareto_frontier(summary: list[dict[str, object]]) -> list[dict[str, object]]:
    """Return scenario-wise nondominated policies with fully successful trials.

    Minimize mean energy and mean duration simultaneously. Incomplete or
    constraint-violating groups are excluded rather than being rewarded for
    early termination. Equal objective pairs remain on the frontier.
    """
    eligible: list[dict[str, object]] = []
    for row in summary:
        trials = row["trials"]
        if (not isinstance(trials, int) or isinstance(trials, bool) or trials < 1):
            raise ValueError("trials must be a positive integer")
        if (row["completed"] == trials and row["deadline_misses"] == 0
                and row["constraint_violations"] == 0):
            eligible.append(row)
    frontier: list[dict[str, object]] = []
    for candidate in eligible:
        dominated = any(
            other["scenario"] == candidate["scenario"]
            and other is not candidate
            and other["mean_energy_j"] <= candidate["mean_energy_j"]
            and other["mean_duration_s"] <= candidate["mean_duration_s"]
            and (other["mean_energy_j"] < candidate["mean_energy_j"]
                 or other["mean_duration_s"] < candidate["mean_duration_s"])
            for other in eligible
        )
        if not dominated:
            frontier.append({
                "scenario": candidate["scenario"],
                "policy": candidate["policy"],
                "trials": candidate["trials"],
                "mean_energy_j": candidate["mean_energy_j"],
                "mean_duration_s": candidate["mean_duration_s"],
            })
    return sorted(frontier, key=lambda r: (r["scenario"], r["mean_energy_j"], r["mean_duration_s"], r["policy"]))
