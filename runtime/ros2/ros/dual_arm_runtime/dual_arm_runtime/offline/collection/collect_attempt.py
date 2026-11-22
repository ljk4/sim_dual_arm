#!/usr/bin/env python3
"""Run one reserved teacher attempt against the already running MuJoCo backend."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection.campaign_execution import (
    attempt_logs_root,
    default_backend_log,
)

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])
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
    robot: str | None = None,
    task: str | None = None,
    shared_root: Path | None = None,
) -> list[str]:
    command = [
        sys.executable,
        str(
            (shared_root or Path(os.environ.get("SHARED_ROOT", project_root / "shared")))
            / "common/tools/scripts/dev_workspace.py"
        ),
        "check",
        "--project-root",
        str(project_root),
        "--build-root",
        os.environ.get("ROS_SDK_ROOT", "/build/ros-dev"),
    ]
    if robot is not None:
        command += ["--robot", robot, "--task", task]
        if shared_root is not None:
            command += ["--shared-root", str(shared_root)]
    return command


def check_ros_sdk(
    project_root: Path = ROOT,
    *,
    robot: str | None = None,
    task: str | None = None,
    shared_root: Path | None = None,
) -> int:
    command = sdk_check_command(project_root, robot=robot, task=task, shared_root=shared_root)
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode:
        print(result.stdout.strip(), file=sys.stderr)
        print(result.stderr.strip(), file=sys.stderr)
    return result.returncode


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-log", type=Path, default=None)
    parser.add_argument(
        "--campaign-config",
        type=Path,
        required=True,
        help="campaign recipe used for export quality rules",
    )
    parser.add_argument("--robot")
    parser.add_argument("--task")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument("--runtime-config", type=Path)
    parser.add_argument("--robot-namespace", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.robot is None and args.runtime_config is None:
        parser.error("one of --robot/--task or --runtime-config is required")
    if args.robot is not None and args.task is None:
        parser.error("--robot requires --task")
    if args.task is not None and args.robot is None:
        parser.error("--task requires --robot")
    if args.robot is not None and args.runtime_config is not None:
        parser.error("--robot/--task and --runtime-config are mutually exclusive")
    attempt = Path(os.environ["CAMPAIGN_ATTEMPT_DIR"]).resolve()
    seed = int(os.environ["CAMPAIGN_SEED"])
    run_id = os.environ["CAMPAIGN_RUN_ID"]
    episode_id = os.environ["CAMPAIGN_EPISODE_ID"]
    robot = args.robot or os.environ.get("CAMPAIGN_ROBOT") or None
    task = args.task or os.environ.get("CAMPAIGN_TASK") or None
    backend_log = (
        args.backend_log if args.backend_log is not None else default_backend_log(robot, task)
    ).resolve()
    if not backend_log.is_dir():
        raise FileNotFoundError(f"teacher backend log is missing: {backend_log}")
    shared_root = args.shared_root
    if shared_root is None and os.environ.get("SHARED_ROOT"):
        shared_root = Path(os.environ["SHARED_ROOT"])
    code = check_ros_sdk(ROOT, robot=robot, task=task, shared_root=shared_root)
    if code:
        return code

    campaign_document = yaml.safe_load(args.campaign_config.read_text(encoding="utf-8"))
    logs = attempt_logs_root(campaign_document, ROOT) / attempt.name
    logs.mkdir(parents=True, exist_ok=True)
    raw = attempt / "raw"
    if robot is not None:
        runtime_arguments = ["--robot", robot, "--task", task, "--role", "teacher"]
        if shared_root is not None:
            runtime_arguments += ["--shared-root", str(shared_root)]
        if args.robot_namespace is not None:
            runtime_arguments += ["--robot-namespace", args.robot_namespace]
    else:
        template = project_path(args.runtime_config, ROOT).resolve()
        recipe = yaml.safe_load(template.read_text(encoding="utf-8"))
        for name in REFERENCES:
            if name in recipe:
                recipe[name] = str((template.parent / recipe[name]).resolve())
        if "reset" not in recipe or not isinstance(recipe["reset"], dict):
            raise ValueError("runtime recipe has no reset object")
        recipe["reset"]["seed"] = seed
        config = logs / "runtime.yaml"
        config.write_text(yaml.safe_dump(recipe, sort_keys=False), encoding="utf-8")
        runtime_arguments = ["--config", str(config)]
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "dual_arm_runtime.runtime_node",
            "--project-root",
            str(ROOT),
            "--once",
            *runtime_arguments,
            "--reset-seed",
            str(seed),
            "--run-id",
            run_id,
            "--episode-id",
            episode_id,
            "--output",
            str(raw),
            "--episode-dir",
            str(raw),
            "--runs-output",
            str(logs),
        ],
        check=False,
    )
    if not (raw / "manifest.json").is_file() or not (raw / "task_result.json").is_file():
        return result.returncode or 1

    export = subprocess.run(
        [
            sys.executable,
            "-m",
            "dual_arm_runtime.offline.export.export_episode",
            "--episode-dir",
            str(raw),
            "--out",
            str(attempt / "export"),
            "--campaign-config",
            str(args.campaign_config),
            "--runtime-log",
            str(logs / run_id / "logs" / "runtime"),
            "--backend-log",
            str(backend_log),
        ],
        check=False,
    )
    quality = attempt / "export" / "quality.json"
    return result.returncode or export.returncode or (0 if quality.is_file() else 1)


if __name__ == "__main__":
    raise SystemExit(main())
