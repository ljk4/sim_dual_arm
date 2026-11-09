#!/usr/bin/env python3
"""Run one reserved teacher attempt against the already running MuJoCo backend."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
REFERENCES = (
    "robot_profile",
    "observation_schema",
    "action_schema",
    "control_constraints",
    "follower_config",
)


def main() -> int:
    attempt = Path(os.environ["STACK_ATTEMPT_DIR"]).resolve()
    seed = int(os.environ["STACK_SEED"])
    run_id = os.environ["STACK_RUN_ID"]
    episode_id = os.environ["STACK_EPISODE_ID"]
    template = (ROOT / os.environ["STACK_RUNTIME_CONFIG"]).resolve()
    backend_log = Path("/data/runs/backend/teacher-backend/log")
    if not backend_log.is_dir():
        raise FileNotFoundError(f"teacher backend log is missing: {backend_log}")

    stack = attempt / "stack" / run_id
    stack.mkdir(parents=True, exist_ok=False)
    recipe = yaml.safe_load(template.read_text(encoding="utf-8"))
    for name in REFERENCES:
        if name in recipe:
            recipe[name] = str((template.parent / recipe[name]).resolve())
    recipe["reset"]["seed"] = seed
    config = stack / "runtime.yaml"
    config.write_text(yaml.safe_dump(recipe, sort_keys=False), encoding="utf-8")

    raw = stack / "raw"
    result = subprocess.run(
        [
            str(ROOT / "runtime/ros2/scripts/run.sh"),
            "collect",
            "--config", str(config),
            "--run-id", run_id,
            "--episode-id", episode_id,
            "--output", str(raw),
            "--runs-output", str(stack / "runs"),
        ],
        check=False,
    )
    episode = raw / episode_id
    for name in ("manifest.json", "task_result.json"):
        source = episode / name
        if source.is_file():
            shutil.copy2(source, attempt / name)
    if not (attempt / "manifest.json").is_file() or not (attempt / "task_result.json").is_file():
        return result.returncode or 1

    export = subprocess.run(
        [
            str(ROOT / "runtime/ros2/scripts/run.sh"),
            "export",
            "--episode-dir", str(episode),
            "--out", str(stack / "export"),
            "--runtime-log", str(stack / "runs" / run_id / "logs" / "runtime"),
            "--backend-log", str(backend_log),
        ],
        check=False,
    )
    quality = stack / "export" / "quality.json"
    if quality.is_file():
        shutil.copy2(quality, attempt / "quality.json")
    (attempt / "launch_result.json").write_text(
        json.dumps({"runtime_exit_code": result.returncode, "export_exit_code": export.returncode})
        + "\n",
        encoding="utf-8",
    )
    return result.returncode or export.returncode or (0 if quality.is_file() else 1)


if __name__ == "__main__":
    raise SystemExit(main())
