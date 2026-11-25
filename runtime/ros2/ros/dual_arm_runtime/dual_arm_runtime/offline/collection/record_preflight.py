#!/usr/bin/env python3
"""Read-only collection preflight executed inside the runtime container.

The host parallel launcher calls this before it creates a pool directory,
reserves budget, allocates a seed or starts a simulation backend.  The command
prints one JSON plan to stdout and all diagnostics to stderr.  It never writes
a ledger or campaign attempt.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection.campaign_ledger import (
    CampaignError,
    check_environment,
    read_records,
    summarize,
)
from dual_arm_runtime.offline.collection.campaign_ledger import (
    load as load_campaign,
)
from dual_arm_runtime.offline.collection.collect_attempt import sdk_check_command
from dual_arm_runtime.offline.collection.scenario_selection import (
    _check_runtime_references,
    _check_teacher_templates,
    _load_yaml,
    direct_selection,
    select_collection,
)

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])


def _sdk_check(
    project_root: Path,
    *,
    robot: str | None = None,
    backend: str | None = None,
    task: str | None = None,
    shared_root: Path | None = None,
) -> tuple[int, str]:
    result = subprocess.run(
        sdk_check_command(
            project_root,
            robot=robot,
            backend=backend,
            task=task,
            shared_root=shared_root,
        ),
        check=False,
        capture_output=True,
        text=True,
    )
    return result.returncode, (result.stdout + result.stderr).strip()


def _campaign_status(project_root: Path, config: dict[str, Any]) -> dict[str, Any]:
    paths = config["paths"]
    output = (project_path(str(paths["attempts_root"]), project_root)).resolve()
    records = read_records(project_path(str(paths["ledger"]), project_root))
    return summarize(records, config, output)


def build_plan(args: argparse.Namespace) -> dict[str, Any]:
    project_root = Path(args.project_root).resolve()
    if any(value is not None for value in (args.robot, args.backend, args.task)):
        if not (args.robot and args.backend and args.task):
            raise CampaignError("--robot, --backend, and --task must be passed together")
        if args.config is not None or args.runtime_config is not None:
            raise CampaignError(
                "--robot/--backend/--task cannot be combined with --config/--runtime-config"
            )
        if args.backend_template is not None:
            raise CampaignError(
                "--robot/--backend/--task cannot be combined with --backend-template"
            )
        selection = select_collection(
            robot=args.robot,
            backend=args.backend,
            task=args.task,
            shared_root=args.shared_root,
            project_root=project_root,
            binding_sha256=args.binding_sha256,
            require_parallel=args.parallel,
        )
        campaign_path = Path(selection["recipes"]["campaign"]["path"])
        runtime_path = Path(selection["recipes"]["runtime_teacher"]["path"])
        backend_path = Path(selection["recipes"]["simulation_teacher"]["path"])
    elif args.shared_root is not None:
        raise CampaignError("--shared-root requires --robot/--backend/--task")
    else:
        if args.config is None or args.runtime_config is None:
            raise CampaignError("direct preflight requires --config and --runtime-config")
        if args.parallel and args.backend_template is None:
            raise CampaignError("parallel preflight requires --backend-template")
        selection = direct_selection(
            campaign=args.config,
            runtime_teacher=args.runtime_config,
            simulation_teacher=args.backend_template,
            project_root=project_root,
        )
        campaign_path = Path(selection["recipes"]["campaign"]["path"])
        runtime_path = Path(selection["recipes"]["runtime_teacher"]["path"])
        backend_path = (
            Path(selection["recipes"]["simulation_teacher"]["path"])
            if "simulation_teacher" in selection["recipes"]
            else None
        )
        runtime_document = _load_yaml(runtime_path, "runtime teacher recipe")
        _check_runtime_references(runtime_path, runtime_document)
        if backend_path is not None:
            backend_document = _load_yaml(backend_path, "simulation teacher recipe")
            _check_teacher_templates(
                runtime_path,
                runtime_document,
                backend_path,
                backend_document,
                require_parallel=args.parallel,
            )

    config = load_campaign(campaign_path)
    paths = config["paths"]
    attempts_root = (project_path(str(paths["attempts_root"]), project_root)).resolve()
    check_environment(project_root, config, attempts_root)
    status = _campaign_status(project_root, config)
    if args.target_total is not None and args.target_total < 1:
        raise CampaignError("--target-total must be positive")
    plan: dict[str, Any] = {
        "selection": selection,
        "campaign_config": str(campaign_path),
        "runtime_config": str(runtime_path),
        "simulation_template": None if backend_path is None else str(backend_path),
        "campaign_status": status,
        "stop_grace_s": float(config.get("launch", {}).get("stop_grace_s", 10.0)),
        "sdk": {"checked": not args.skip_sdk_check, "ok": None},
    }
    if not args.skip_sdk_check:
        code, output = _sdk_check(
            project_root,
            robot=args.robot,
            backend=args.backend,
            task=args.task,
            shared_root=args.shared_root,
        )
        plan["sdk"] = {"checked": True, "ok": code == 0, "output": output}
        if code:
            raise CampaignError(f"ROS SDK check failed ({code}): {output}")
    return plan


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--project-root", type=Path, default=ROOT)
    result.add_argument("--shared-root", type=Path)
    result.add_argument("--robot")
    result.add_argument("--backend")
    result.add_argument("--task")
    result.add_argument("--binding-sha256")
    result.add_argument("--config", type=Path)
    result.add_argument("--runtime-config", type=Path)
    result.add_argument("--backend-template", type=Path)
    result.add_argument("--parallel", action="store_true")
    result.add_argument("--purpose", choices=("formal", "pilot"), default="formal")
    result.add_argument("--target-total", type=int)
    result.add_argument("--skip-sdk-check", action="store_true", help=argparse.SUPPRESS)
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        plan = build_plan(args)
    except (CampaignError, OSError) as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2
    json.dump(plan, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
