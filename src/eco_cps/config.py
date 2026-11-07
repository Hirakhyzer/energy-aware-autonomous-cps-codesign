"""Safe JSON scenario loading without unsafe deserializers or third-party dependencies."""
from __future__ import annotations
import json
from dataclasses import fields
from pathlib import Path
from .models import Scenario


def load_scenario(path: str | Path) -> Scenario:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("Scenario configuration must be a JSON object")
    allowed = {field.name for field in fields(Scenario)}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"Unknown scenario fields: {', '.join(sorted(unknown))}")
    if "waypoints" in raw:
        if not isinstance(raw["waypoints"], list):
            raise ValueError("Waypoints must be an array of two-dimensional points")
        raw["waypoints"] = tuple(tuple(point) for point in raw["waypoints"])
    return Scenario(**raw)
