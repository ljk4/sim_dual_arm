"""Prepare isolated, headless simulation/runtime configurations for inference."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collection.collect_attempt import REFERENCES, check_ros_sdk
from dual_arm_runtime.offline.collection.scenario_selection import select_inference
from dual_arm_runtime.offline.parallel_common import (
    backend_config_for_simulation,
    parse_worker_prep_args,
    recipe_records,
    validate_backend_references,
    validate_worker_bounds,
    write_worker_manifest,
)

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])


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
    validate_worker_bounds(workers, domain_base, speed)
    project, directory = project.resolve(), directory.resolve()
    if not directory.is_relative_to(project_path("output", project)):
        raise CampaignError(
            "worker directory must be below the mounted project output directory"
        )
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
    backend_impl = (
        str(selection["backend"])
        if selection is not None and selection.get("backend")
        else None
    )
    validate_backend_references(backend, backend_template, project, backend_impl)
    backend_config = backend_config_for_simulation(
        backend_template, project, backend_impl
    )
    recipes = recipe_records(
        ("runtime_policy", "simulation_policy"), runtime_template, backend_template, selection
    )
    return write_worker_manifest(
        directory=directory,
        project=project,
        workers=workers,
        domain_base=domain_base,
        namespace_prefix="infer",
        runtime_document=runtime,
        backend_config=backend_config,
        recipes=recipes,
        selection=selection,
        overrides=lambda domain_id, namespace: {
            "domain_id": domain_id,
            "namespace": namespace,
            "realtime_speed": speed,
            "visualization_enabled": False,
            "task_mode": "inference",
            "action_source": "policy",
        },
    )


def main() -> int:
    parser, args = parse_worker_prep_args(__doc__ or "", default_speed=None)
    code = check_ros_sdk(
        ROOT,
        robot=args.robot,
        backend=args.backend,
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
                backend=args.backend,
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
