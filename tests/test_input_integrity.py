"""Regression coverage for benchmark identity and input validation."""
import json

import pytest

from eco_cps.benchmark import benchmark, save_results
from eco_cps.config import load_scenario
from eco_cps.models import Scenario


@pytest.mark.parametrize("name", ["", "  ", 42, None])
def test_scenario_name_must_be_nonempty_text(name):
    with pytest.raises(ValueError, match="Scenario name"):
        Scenario(name=name)


@pytest.mark.parametrize("waypoints", [
    None, ((0, 0),), ((0, 0), (float("inf"), 2)),
    ((0, 0), (True, 2)), ((0, 0), ("1", 2)),
    ((0, 0), 2),
])
def test_waypoints_must_be_finite_numeric_points(waypoints):
    with pytest.raises(ValueError, match="waypoints"):
        Scenario(waypoints=waypoints)


def test_config_rejects_non_array_waypoint_entries(tmp_path):
    config = tmp_path / "invalid.json"
    config.write_text(json.dumps({"waypoints": [[0, 0], 42]}), encoding="utf-8")
    with pytest.raises(ValueError, match="Waypoints"):
        load_scenario(config)


def test_benchmark_rejects_ambiguous_scenario_names():
    a = Scenario(name="same")
    b = Scenario(name="same", motion_load=1.4)
    with pytest.raises(ValueError, match="names must be unique"):
        benchmark([a, b], seeds=(0,))


def test_saving_empty_results_fails_before_creating_directory(tmp_path):
    destination = tmp_path / "empty"
    with pytest.raises(ValueError, match="empty benchmark"):
        save_results([], destination)
    assert not destination.exists()

def test_trace_filenames_cannot_escape_output_directory(tmp_path):
    from eco_cps.simulation import simulate

    unsafe = "../../outside/example"
    run = simulate("fixed", Scenario(name=unsafe), seed=0, include_trace=True)
    folder = save_results([run], tmp_path / "out", traces=True)
    traces = list((folder / "traces").glob("*.json"))
    assert len(traces) == 1
    assert traces[0].parent == folder / "traces"
    assert json.loads((folder / "runs.json").read_text())[-1]["scenario"] == unsafe
    assert not (tmp_path / "outside").exists()


def test_trace_names_remain_distinct_after_sanitization(tmp_path):
    from eco_cps.simulation import simulate

    left = simulate("fixed", Scenario(name="route/a"), seed=0, include_trace=True)
    right = simulate("fixed", Scenario(name="route?a"), seed=0, include_trace=True)
    folder = save_results([left, right], tmp_path / "out", traces=True)
    assert len(list((folder / "traces").glob("*.json"))) == 2


def test_safe_trace_names_keep_existing_format(tmp_path):
    from eco_cps.simulation import simulate

    run = simulate("fixed", Scenario(name="nominal"), seed=0, include_trace=True)
    folder = save_results([run], tmp_path / "out", traces=True)
    assert (folder / "traces" / "nominal-fixed-0.json").is_file()


def test_duplicate_run_keys_rejected_before_saving(tmp_path):
    from eco_cps.simulation import simulate

    run = simulate("fixed", Scenario(), seed=0)
    destination = tmp_path / "out"
    with pytest.raises(ValueError, match="Duplicate scenario/seed/policy"):
        save_results([run, run], destination)
    assert not destination.exists()
