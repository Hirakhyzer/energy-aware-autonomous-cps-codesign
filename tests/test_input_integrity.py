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
