"""Read-only preflight for the parallel inference launcher (runs in runtime)."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from robot_contracts.io import sha256_file
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection.collect_attempt import check_ros_sdk
from dual_arm_runtime.offline.collection.scenario_selection import select_inference

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])


def _record(field: str, value: Path, project_root: Path) -> dict[str, object]:
    path = project_path(value, project_root).resolve()
    if not path.is_file():
        raise ValueError(f"direct {field} recipe is missing: {path}")
    return {
        "field": f"direct.{field}",
        "declared": None,
        "path": str(path),
        "sha256": sha256_file(path),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--robot")
    parser.add_argument("--backend")
    parser.add_argument("--task")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument("--runtime-config", type=Path)
    parser.add_argument("--backend-template", type=Path)
    args = parser.parse_args(argv)
    if any(value is not None for value in (args.robot, args.backend, args.task)):
        if not (args.robot and args.backend and args.task):
            parser.error("--robot, --backend, and --task must be passed together")
        if args.runtime_config is not None or args.backend_template is not None:
            parser.error("--robot/--backend/--task cannot be combined with direct recipes")
    else:
        if args.shared_root is not None:
            parser.error("--shared-root requires --robot/--backend/--task")
        if args.runtime_config is None or args.backend_template is None:
            parser.error("direct mode requires --runtime-config and --backend-template")
    code = check_ros_sdk(
        args.project_root,
        robot=args.robot,
        backend=args.backend,
        task=args.task,
        shared_root=args.shared_root,
    )
    if code:
        return code
    try:
        if args.robot is not None:
            selection = select_inference(
                robot=args.robot,
                backend=args.backend,
                task=args.task,
                shared_root=args.shared_root,
                project_root=args.project_root,
            )
        else:
            selection = {
                "mode": "direct",
                "robot": None,
                "backend": None,
                "task": None,
                "scenario_id": None,
                "binding": None,
                "recipes": {
                    "runtime_policy": _record(
                        "runtime_policy", args.runtime_config, args.project_root
                    ),
                    "simulation_policy": _record(
                        "simulation_policy", args.backend_template, args.project_root
                    ),
                },
                "overrides": ["direct recipes supplied explicitly"],
            }
    except Exception as error:  # noqa: BLE001 - preflight reports one bounded reason
        print(json.dumps({"error": str(error)}), file=sys.stderr, flush=True)
        return 2
    print(json.dumps({"selection": selection}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
