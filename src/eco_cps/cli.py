"""Command-line entry point for simulation, benchmarks, and archived reports."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .benchmark import benchmark, save_results, summarize
from .config import load_scenario
from .models import Scenario
from .simulation import simulate


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="eco-cps", description="Energy-aware CPS simulation and benchmark toolkit")
    sub = p.add_subparsers(dest="command", required=True)
    s = sub.add_parser("simulate", help="Run one seeded autonomous mission")
    s.add_argument("--policy", choices=("fixed", "sensing", "computing", "control", "joint"), default="joint")
    s.add_argument("--config", type=Path, default=None)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--out", type=Path, default=None)
    b = sub.add_parser("benchmark", help="Run paired comparisons of all five policies")
    b.add_argument("--config", type=Path, default=None)
    b.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    b.add_argument("--out", type=Path, default=Path("results/benchmark"))
    b.add_argument("--traces", action="store_true")
    r = sub.add_parser("report", help="Read an existing benchmark summary")
    r.add_argument("--input", type=Path, default=Path("results/benchmark"))
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.command == "report":
        path = args.input / "summary.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, list):
            raise ValueError("Summary must be a JSON array")
        print(json.dumps(data, indent=2))
        return 0
    scenario = load_scenario(args.config) if args.config else Scenario()
    if args.command == "simulate":
        result = simulate(args.policy, scenario, seed=args.seed, include_trace=bool(args.out))
        print(json.dumps(result.summary(), indent=2))
        if args.out:
            save_results([result], args.out, traces=True)
        return 0 if result.completed else 2
    runs = benchmark([scenario], seeds=tuple(args.seeds), include_traces=args.traces)
    save_results(runs, args.out, traces=args.traces)
    print(json.dumps(summarize(runs), indent=2))
    # Infeasible scenarios are legitimate study inputs; preserve records and flag failures.
    return 0 if all(r.completed for r in runs) else 2


if __name__ == "__main__":
    raise SystemExit(main())
