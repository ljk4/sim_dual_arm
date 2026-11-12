#!/usr/bin/env python3
"""Run one reserved teacher attempt against the already running MuJoCo backend."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])
REFERENCES = (
    "robot_profile",
    "observation_schema",
    "action_schema",
    "control_constraints",
    "follower_config",
)


def sdk_check_command(
    project_root: Path = ROOT,
    *,
    scenario: str | None = None,
    shared_root: Path | None = None,
) -> list[str]:
    command = [
        sys.executable,
        str(project_root / "shared/common/tools/scripts/dev_workspace.py"),
        "check", "--project-root", str(project_root),
        "--build-root", os.environ.get("ROS_SDK_ROOT", "/build/ros-dev"),
    ]
    if scenario is not None:
        command += ["--scenario", scenario]
        if shared_root is not None:
            command += ["--shared-root", str(shared_root)]
    return command


def check_ros_sdk(
    project_root: Path = ROOT,
    *,
    scenario: str | None = None,
    shared_root: Path | None = None,
) -> int:
    command = sdk_check_command(
        project_root, scenario=scenario, shared_root=shared_root
    )
    return subprocess.run(command, check=False).returncode


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend-log", type=Path, default=Path("/data/runs/backend/teacher-backend/log")
    )
    parser.add_argument(
        "--campaign-config",
        type=Path,
        default=Path(
            "shared/scenarios/panda_stack/configs/runtime/experiments/stack_campaign.yaml"
        ),
        help="campaign recipe used for export quality rules",
    )
    parser.add_argument("--no-previews", action="store_true")
    parser.add_argument("--scenario")
    parser.add_argument("--shared-root", type=Path)
    args = parser.parse_args(argv)
    attempt = Path(os.environ["CAMPAIGN_ATTEMPT_DIR"]).resolve()
    seed = int(os.environ["CAMPAIGN_SEED"])
    run_id = os.environ["CAMPAIGN_RUN_ID"]
    episode_id = os.environ["CAMPAIGN_EPISODE_ID"]
    template = (ROOT / os.environ["CAMPAIGN_RUNTIME_CONFIG"]).resolve()
    backend_log = args.backend_log.resolve()
    if not backend_log.is_dir():
        raise FileNotFoundError(f"teacher backend log is missing: {backend_log}")
    scenario = args.scenario or os.environ.get("CAMPAIGN_SCENARIO_ID") or None
    shared_root = args.shared_root
    if shared_root is None and os.environ.get("SHARED_ROOT"):
        shared_root = Path(os.environ["SHARED_ROOT"])
    code = check_ros_sdk(ROOT, scenario=scenario, shared_root=shared_root)
    if code:
        return code

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
            sys.executable, "-m", "dual_arm_runtime.runtime_node",
            "--project-root", str(ROOT), "--once",
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
            sys.executable, "-m", "dual_arm_runtime.offline.export_episode",
            "--episode-dir", str(episode),
            "--out", str(stack / "export"),
            "--campaign-config", str(args.campaign_config),
            "--runtime-log", str(stack / "runs" / run_id / "logs" / "runtime"),
            "--backend-log", str(backend_log),
            *(["--no-previews"] if args.no_previews else []),
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
