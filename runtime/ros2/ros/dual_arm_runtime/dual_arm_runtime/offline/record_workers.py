"""Prepare isolated, headless simulation/runtime configurations for recording."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collect_attempt import REFERENCES, check_ros_sdk
from dual_arm_runtime.offline.scenario_selection import select_collection

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def simulation_path(path: Path, project: Path) -> str:
    # Paths already expressed in a simulation container mount stay unchanged.
    for container in ("/shared", "/data", "/app"):
        root = Path(container)
        if path == root or path.is_relative_to(root):
            return str(path)
    for source, destination in (
        (project / "simulation/mujoco", Path("/app")),
        (project / "shared", Path("/shared")),
        (project / "var", Path("/data")),
    ):
        if path.is_relative_to(source):
            return str(destination / path.relative_to(source))
    raise CampaignError(f"path is not mounted in simulation: {path}")


def _selection_recipes(
    runtime_template: Path, backend_template: Path, selection: dict[str, Any] | None
) -> dict[str, Any]:
    if selection is not None:
        return {
            "runtime_teacher": selection["recipes"]["runtime_teacher"],
            "simulation_teacher": selection["recipes"]["simulation_teacher"],
        }
    return {
        "runtime_teacher": {
            "field": "direct.runtime_teacher",
            "declared": None,
            "path": str(runtime_template),
            "sha256": _sha256(runtime_template),
        },
        "simulation_teacher": {
            "field": "direct.simulation_teacher",
            "declared": None,
            "path": str(backend_template),
            "sha256": _sha256(backend_template),
        },
    }


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
    if not 1 <= workers <= 16 or not 0 <= domain_base <= 232 - workers + 1:
        raise CampaignError("workers must be 1..16 and ROS domain IDs must stay in 0..232")
    if not math.isfinite(speed) or not 0.1 < speed <= 100:
        raise CampaignError("realtime speed must be finite and within (0.1, 100]")
    project, directory = project.resolve(), directory.resolve()
    if not directory.is_relative_to(project_path("var", project)):
        raise CampaignError("worker directory must be below the mounted project var directory")
    runtime_template, backend_template = (runtime_template.resolve(), backend_template.resolve())
    if selection is not None:
        _check_selection_paths(selection, runtime_template, backend_template)
    runtime = yaml.safe_load(runtime_template.read_text())
    backend = yaml.safe_load(backend_template.read_text())
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
    for name in ("profile", "schemas", "scene", "control", "stream", "follower", "task"):
        if backend["robot"].get(name):
            source = (backend_template.parent / backend["robot"][name]).resolve()
            backend["robot"][name] = simulation_path(source, project)
    recipes = _selection_recipes(runtime_template, backend_template, selection)
    teacher_api = None if selection is None else selection["capabilities"]["teacher"]
    directory.mkdir(parents=True, exist_ok=False)
    entries = []
    for index in range(workers):
        domain_id = domain_base + index
        worker_id = f"worker-{index + 1:02d}"
        worker_dir = directory / worker_id
        worker_dir.mkdir()
        namespace = f"/record_{directory.name.replace('-', '_')[-40:]}_{index + 1}"
        runtime["robot_namespace"] = backend["robot"]["namespace"] = namespace
        backend["robot"]["visualization_enabled"] = False
        backend["run"]["realtime_speed"] = speed
        runtime_path, backend_path = worker_dir / "runtime.yaml", worker_dir / "backend.yaml"
        runtime_path.write_text(yaml.safe_dump(runtime, sort_keys=False))
        backend_path.write_text(yaml.safe_dump(backend, sort_keys=False))
        entries.append(
            {
                "id": worker_id,
                "domain_id": domain_id,
                "namespace": namespace,
                "run_id_prefix": f"{directory.name}-{worker_id}",
                "output_root": str(worker_dir),
                "runtime_config": str(runtime_path),
                "backend_config": simulation_path(backend_path, project),
                "backend_log": str(worker_dir / "backend-runs/backend/log"),
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
                "teacher_api": teacher_api,
                "overrides": {
                    "domain_id": domain_id,
                    "namespace": namespace,
                    "realtime_speed": speed,
                    "visualization_enabled": False,
                    "render_quality_enabled": True,
                    "task_mode": "teacher",
                    "action_source": "teacher",
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
    parser.add_argument("--realtime-speed", type=float, default=1.0)
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
            selection = select_collection(
                robot=args.robot,
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
