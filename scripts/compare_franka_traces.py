"""Compare retained Franka traces without simulator-specific dependencies."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from typing import Any


def _metrics(left: list[list[float]], right: list[list[float]], indices: range) -> dict[str, float]:
    errors = [abs(left[step][index] - right[step][index]) for step in range(len(left)) for index in indices]
    return {
        "max_abs_rad": max(errors),
        "rmse_rad": math.sqrt(sum(error * error for error in errors) / len(errors)),
    }


def compare(paths: list[Path]) -> dict[str, Any]:
    runs = {path.stem: json.loads(path.read_text(encoding="utf-8")) for path in paths}
    if len(runs) != len(paths):
        raise ValueError("input file stems must be unique")
    first = next(iter(runs.values()))
    for name, run in runs.items():
        for key in ("joint_names", "steps", "time_step_seconds", "targets"):
            if run[key] != first[key]:
                raise ValueError(f"{name} does not share identical {key}")

    groups = {"arm": range(0, 7), "gripper": range(7, 13)}
    pairwise: dict[str, dict[str, dict[str, float]]] = {}
    for left_name, right_name in itertools.combinations(runs, 2):
        pairwise[f"{left_name}__{right_name}"] = {
            group: _metrics(runs[left_name]["joint_positions"], runs[right_name]["joint_positions"], indices)
            for group, indices in groups.items()
        }
    tracking = {
        name: {group: _metrics(run["joint_positions"], run["targets"], indices) for group, indices in groups.items()}
        for name, run in runs.items()
    }
    thresholds = {
        "arm": {"max_abs_rad": 0.05, "rmse_rad": 0.02},
        "gripper": {"max_abs_rad": 0.10, "rmse_rad": 0.04},
    }
    checks = {
        pair: {
            group: all(values[metric] <= thresholds[group][metric] for metric in ("max_abs_rad", "rmse_rad"))
            for group, values in group_metrics.items()
        }
        for pair, group_metrics in pairwise.items()
    }
    return {
        "status": "passed" if all(all(groups_pass.values()) for groups_pass in checks.values()) else "failed",
        "inputs": {name: str(path.resolve()) for name, path in zip(runs, paths, strict=True)},
        "steps": first["steps"],
        "time_step_seconds": first["time_step_seconds"],
        "joint_names": first["joint_names"],
        "thresholds": thresholds,
        "pairwise": pairwise,
        "pairwise_checks": checks,
        "tracking_to_shared_target": tracking,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = compare(args.inputs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
