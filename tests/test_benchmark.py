import gzip
import json
import subprocess
import sys
from pathlib import Path
import pytest
from eco_cps.benchmark import benchmark, paired_analysis, percent_energy_difference, save_results, summarize
from eco_cps.config import load_scenario
from eco_cps.models import Scenario


def test_paired_benchmark_cross_product():
    rows = benchmark([Scenario()], seeds=(0, 1))
    assert len(rows) == 14
    assert {r.policy for r in rows} == {'fixed', 'sensing', 'computing', 'control', 'joint', 'joint_qos', 'joint_robust'}
    assert {r.seed for r in rows} == {0, 1}
    assert len(summarize(rows)) == 7


def test_benchmark_outputs_roundtrip(tmp_path):
    rows = benchmark(seeds=(1,))
    folder = save_results(rows, tmp_path / 'out', traces=True)
    assert len(json.loads((folder / 'runs.json').read_text())) == 7
    with gzip.open(folder / 'runs.json.gz', 'rt', encoding='utf-8') as archive:
        assert json.load(archive) == json.loads((folder / 'runs.json').read_text())
    assert len(json.loads((folder / 'summary.json').read_text())) == 7
    assert len(list((folder / 'traces').glob('*.json'))) == 7
    assert len((folder / 'runs.csv').read_text().splitlines()) == 8
    assert len(json.loads((folder / 'paired.json').read_text())) == 6
    assert len((folder / 'paired.csv').read_text().splitlines()) == 7


def test_config_loads_and_rejects_unknown_fields(tmp_path):
    base = Path(__file__).resolve().parents[1] / 'configs' / 'baseline.json'
    loaded = load_scenario(base)
    assert loaded.name == 'nominal'
    assert len(loaded.waypoints) == 5
    bad = tmp_path / 'bad.json'
    bad.write_text('{"unknown_field": 1}')
    with pytest.raises(ValueError):
        load_scenario(bad)


def test_cli_simulate_and_report(tmp_path):
    run = subprocess.run([sys.executable, '-m', 'eco_cps', 'simulate', '--seed', '2'],
                         capture_output=True, text=True, check=True)
    assert json.loads(run.stdout)['completed'] is True
    path = tmp_path / 'benchmark'
    subprocess.run([sys.executable, '-m', 'eco_cps', 'benchmark', '--seeds', '1',
                    '--out', str(path)], capture_output=True, text=True, check=True)
    report = subprocess.run([sys.executable, '-m', 'eco_cps', 'report', '--input', str(path)],
                            capture_output=True, text=True, check=True)
    assert len(json.loads(report.stdout)) == 7


def test_bad_benchmark_requests_rejected():
    with pytest.raises(ValueError):
        benchmark(seeds=())
    with pytest.raises(ValueError):
        benchmark(seeds=(1, 1))
    with pytest.raises(ValueError):
        summarize([])
    with pytest.raises(ValueError):
        percent_energy_difference(0, 1)
