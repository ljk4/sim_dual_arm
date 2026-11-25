"""Prepare isolated, headless simulation/runtime configurations for recording."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collection.collect_attempt import REFERENCES, check_ros_sdk
from dual_arm_runtime.offline.collection.scenario_selection import select_collection
from dual_arm_runtime.offline.parallel_common import (
    backend_config_for_simulation,
    parse_worker_prep_args,
    recipe_records,
    validate_backend_references,
    validate_worker_bounds,
    write_worker_manifest,
)

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])


def _check_selection_paths(
    selection: dict[str, Any],
    runtime_template: Path,
    backend_template: Path,
) -> None:
    for field, path in (
        ("runtime_teacher", runtime_template),
        ("simulation_teacher", backend_template),
    ):
        expected = Path(selection["recipes"][field]["path"]).resolve()
        actual = path.resolve()
        if actual != expected:
            raise CampaignError(f"{field} recipe {actual} differs from selected recipe {expected}")


def prepare_workers(
    project: Path,
    directory: Path,
    workers: int,
    domain_base: int,
    runtime_template: Path,
    backend_template: Path,
    speed: float,
    selection: dict[str, Any] | None = None,
) -> Path:
    validate_worker_bounds(workers, domain_base, speed)
    project, directory = project.resolve(), directory.resolve()
    if not directory.is_relative_to(project_path("output", project)):
        raise CampaignError(
            "worker directory must be below the mounted project output directory"
        )
    runtime_template, backend_template = (runtime_template.resolve(), backend_template.resolve())
    if selection is not None:
        _check_selection_paths(selection, runtime_template, backend_template)
    runtime = yaml.safe_load(runtime_template.read_text(encoding="utf-8"))
    backend = yaml.safe_load(backend_template.read_text(encoding="utf-8"))
    if selection is None:
        if (
            runtime.get("action_source") != "teacher"
            or backend["robot"].get("task_mode") != "teacher"
        ):
            raise CampaignError("parallel recording requires teacher runtime and backend templates")
        if not isinstance(runtime.get("teacher_api"), str) or not runtime["teacher_api"]:
            raise CampaignError("parallel recording runtime template must declare teacher_api")
        if backend["robot"].get("render_quality", {}).get("enabled") is not True:
            raise CampaignError("parallel recording requires backend render_quality.enabled")
    for name in REFERENCES:
        if name in runtime:
            runtime[name] = str((runtime_template.parent / runtime[name]).resolve())
    backend_impl = (
        str(selection["backend"])
        if selection is not None and selection.get("backend")
        else None
    )
    validate_backend_references(backend, backend_template, project, backend_impl)
    backend_config_sim = backend_config_for_simulation(
        backend_template, project, backend_impl
    )
    recipes = recipe_records(
        ("runtime_teacher", "simulation_teacher"), runtime_template, backend_template, selection
    )
    teacher_api = None if selection is None else selection["capabilities"]["teacher"]
    return write_worker_manifest(
        directory=directory,
        project=project,
        workers=workers,
        domain_base=domain_base,
        namespace_prefix="record",
        runtime_document=runtime,
        backend_config=backend_config_sim,
        recipes=recipes,
        selection=selection,
        extra_entry=lambda: {"teacher_api": teacher_api},
        overrides=lambda domain_id, namespace: {
            "domain_id": domain_id,
            "namespace": namespace,
            "realtime_speed": speed,
            "visualization_enabled": False,
            "render_quality_enabled": True,
            "task_mode": "teacher",
            "action_source": "teacher",
        },
    )


def main() -> int:
    parser, args = parse_worker_prep_args(__doc__ or "", default_speed=1.0)
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
            selection = select_collection(
                robot=args.robot,
                backend=args.backend,
                task=args.task,
                shared_root=args.shared_root,
                project_root=ROOT,
                binding_sha256=args.binding_sha256,
                require_parallel=True,
            )
            runtime_config = Path(selection["recipes"]["runtime_teacher"]["path"])
            backend_template = Path(selection["recipes"]["simulation_teacher"]["path"])
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
