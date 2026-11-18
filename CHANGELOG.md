# Changelog

## 0.3.1 — 2026-11-18

- Reject blank/non-text scenario names and malformed or non-finite waypoint coordinates.
- Reject duplicate scenario names in a benchmark to prevent mixing distinct experimental conditions.
- Reject empty result sets and duplicate run identifiers before writing benchmark artifacts.
- Keep trace artifact paths inside the output directory, retaining the previous filename format for ordinary scenario names.
- Add regression tests for malformed inputs, benchmark attribution, trace-path safety, and filename uniqueness.
- Retain the v0.3 simulation, research methods, and archived experiment results unchanged.
