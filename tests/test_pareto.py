"""Regression tests for v0.4 empirical Pareto screening."""
import pytest
from eco_cps.pareto import pareto_frontier


def row(policy, energy, duration, *, scenario="nominal", completed=3, misses=0, violations=0):
    return {
        "scenario": scenario, "policy": policy, "trials": 3,
        "completed": completed, "deadline_misses": misses,
        "constraint_violations": violations,
        "mean_energy_j": energy, "mean_duration_s": duration,
    }


def test_energy_time_tradeoff_retains_nondominated_policies():
    rows = [
        row("fast", 40, 10), row("efficient", 20, 20),
        row("dominated", 45, 25), row("middle", 30, 15),
    ]
    assert [r["policy"] for r in pareto_frontier(rows)] == [
        "efficient", "middle", "fast"
    ]


def test_failures_and_violations_are_not_rewarded():
    rows = [
        row("valid", 30, 15),
        row("incomplete", 1, 1, completed=2),
        row("missed", 1, 1, misses=1),
        row("violated", 1, 1, violations=1),
    ]
    assert [r["policy"] for r in pareto_frontier(rows)] == ["valid"]


def test_scenarios_are_separate_and_equal_points_preserved():
    rows = [row("a", 10, 10), row("b", 10, 10), row("c", 5, 5, scenario="other")]
    assert {(r["scenario"], r["policy"]) for r in pareto_frontier(rows)} == {
        ("nominal", "a"), ("nominal", "b"), ("other", "c")
    }


def test_no_feasible_policy_returns_empty_frontier():
    assert pareto_frontier([row("failed", 1, 1, completed=0)]) == []


@pytest.mark.parametrize("trials", [0, -1, True, 1.5])
def test_invalid_trial_counts_rejected(trials):
    candidate = row("a", 1, 1)
    candidate["trials"] = trials
    with pytest.raises(ValueError, match="trials"):
        pareto_frontier([candidate])


@pytest.mark.parametrize("field,bad_value", [
    ("mean_energy_j", float("nan")),
    ("mean_energy_j", float("inf")),
    ("mean_energy_j", -1.0),
    ("mean_duration_s", float("-inf")),
    ("mean_duration_s", True),
])
def test_invalid_pareto_metrics_rejected(field, bad_value):
    candidate = row("a", 1, 1)
    candidate[field] = bad_value
    with pytest.raises(ValueError, match=field):
        pareto_frontier([candidate])


def test_duplicate_policy_summaries_rejected():
    with pytest.raises(ValueError, match="Duplicate"):
        pareto_frontier([row("a", 1, 2), row("a", 2, 1)])
