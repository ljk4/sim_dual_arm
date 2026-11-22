"""Prepare isolated, headless simulation/runtime configurations for inference."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.io import sha256_file
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collection.collect_attempt import REFERENCES, check_ros_sdk
from dual_arm_runtime.offline.collection.scenario_selection import select_inference
from dual_arm_runtime.offline.parallel_common import (
    backend_config_for_simulation,
    simulation_path,
    validate_backend_references,
)

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])


def _direct_recipes(runtime_template: Path, backend_template: Path) -> dict[str, Any]:
    return {
        "runtime_policy": {
            "field": "direct.runtime_policy",
            "declared": None,
            "path": str(runtime_template),
            "sha256": sha256_file(runtime_template),
        },
        "simulation_policy": {
            "field": "direct.simulation_policy",
            "declared": None,
            "path": str(backend_template),
            "sha256": sha256_file(backend_template),
        },
    }


def prepare_workers(
    project: Path,
    directory: Path,
    workers: int,
    domain_base: int,
    runtime_template: Path,
    backend_template: Path,
    speed: float | None,
    selection: dict[str, Any] | None = None,
) -> Path:
    if not 1 <= workers <= 16 or not 0 <= domain_base <= 232 - workers + 1:
        raise CampaignError("workers must be 1..16 and ROS domain IDs must stay in 0..232")
    if speed is not None and (not math.isfinite(speed) or not 0.1 < speed <= 100):
        raise CampaignError("realtime speed must be finite and within (0.1, 100]")
    project, directory = project.resolve(), directory.resolve()
    if not directory.is_relative_to(project_path("var", project)):
        raise CampaignError("worker directory must be below the mounted project var directory")
    runtime_template, backend_template = (runtime_template.resolve(), backend_template.resolve())
    runtime = yaml.safe_load(runtime_template.read_text())
    backend = yaml.safe_load(backend_template.read_text())
    if speed is None:
        speed = float(backend.get("run", {}).get("realtime_speed", 1.0))
        if not math.isfinite(speed) or not 0.1 < speed <= 100:
            raise CampaignError("backend template realtime speed is not within (0.1, 100]")
    if selection is None and (
        runtime.get("action_source") != "policy"
        or backend["robot"].get("task_mode") != "inference"
    ):
        raise CampaignError("parallel inference requires policy runtime and inference backend templates")
    for name in REFERENCES:
        if name in runtime:
            runtime[name] = str((runtime_template.parent / runtime[name]).resolve())
    validate_backend_references(backend, backend_template, project)
    backend_config = backend_config_for_simulation(backend_template, project)
    recipes = (
        {
            "runtime_policy": selection["recipes"]["runtime_policy"],
            "simulation_policy": selection["recipes"]["simulation_policy"],
        }
        if selection is not None
        else _direct_recipes(runtime_template, backend_template)
    )
    directory.mkdir(parents=True, exist_ok=False)
    entries = []
    for index in range(workers):
        domain_id = domain_base + index
        worker_id = f"worker-{index + 1:02d}"
        worker_dir = directory / worker_id
        worker_dir.mkdir()
        namespace = f"/infer_{directory.name.replace('-', '_')[-40:]}_{index + 1}"
        runtime["robot_namespace"] = namespace
        runtime_path = worker_dir / "runtime.yaml"
        runtime_path.write_text(yaml.safe_dump(runtime, sort_keys=False))
        entries.append(
            {
                "id": worker_id,
                "domain_id": domain_id,
                "namespace": namespace,
                "run_id_prefix": f"{directory.name}-{worker_id}",
                "output_root": str(worker_dir),
                "runtime_config": str(runtime_path),
                "backend_config": backend_config,
                "backend_log": str(worker_dir / "backend-runs/backend/log"),
                "backend_ready": str(worker_dir / "backend-runs/backend/ready.json"),
                "run_root": simulation_path(worker_dir / "backend-runs", project),
                "binding": None
                if selection is None
                else {
                    **selection.get("binding", {}),
                    "robot": selection.get("robot"),
                    "task": selection.get("task"),
                    "scenario_id": selection.get("scenario_id"),
                },
                "recipes": recipes,
                "overrides": {
                    "domain_id": domain_id,
                    "namespace": namespace,
                    "realtime_speed": speed,
                    "visualization_enabled": False,
                    "task_mode": "inference",
                    "action_source": "policy",
                },
            }
        )
    manifest_selection = selection or {
        "mode": "direct",
        "robot": None,
        "task": None,
        "scenario_id": None,
        "binding": None,
        "profiles": None,
        "recipes": recipes,
        "overrides": ["direct recipes supplied explicitly"],
    }
    manifest = directory / "workers.json"
    manifest.write_text(
        json.dumps(
            {"selection": manifest_selection, "workers": entries},
            indent=2,
        )
        + "\n"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--domain-base", type=int, required=True)
    parser.add_argument("--realtime-speed", type=float, default=None)
    parser.add_argument("--robot")
    parser.add_argument("--task")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument("--binding-sha256")
    parser.add_argument("--runtime-config", type=Path)
    parser.add_argument("--backend-template", type=Path)
    args = parser.parse_args()
    if args.robot is not None:
        if args.task is None:
            parser.error("--robot requires --task")
        if args.runtime_config is not None or args.backend_template is not None:
            parser.error(
                "--robot/--task cannot be combined with --runtime-config or --backend-template"
            )
    else:
        if args.task is not None:
            parser.error("--task requires --robot")
        if args.shared_root is not None:
            parser.error("--shared-root requires --robot/--task")
        if args.binding_sha256 is not None:
            parser.error("--binding-sha256 requires --robot/--task")
        if args.runtime_config is None or args.backend_template is None:
            parser.error("direct mode requires --runtime-config and --backend-template")
    code = check_ros_sdk(
        ROOT,
        robot=args.robot,
        task=args.task,
        shared_root=args.shared_root,
    )
    if code:
        return code
    try:
        selection = None
        if args.robot is not None:
            selection = select_inference(
                robot=args.robot,
                task=args.task,
                shared_root=args.shared_root,
                project_root=ROOT,
                binding_sha256=args.binding_sha256,
            )
            runtime_config = Path(selection["recipes"]["runtime_policy"]["path"])
            backend_template = Path(selection["recipes"]["simulation_policy"]["path"])
        else:
            runtime_config = project_path(args.runtime_config, ROOT)
            backend_template = project_path(args.backend_template, ROOT)
        manifest = prepare_workers(
            ROOT,
            args.directory,
            args.workers,
            args.domain_base,
            runtime_config,
            backend_template,
            args.realtime_speed,
            selection,
        )
    except (CampaignError, OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
