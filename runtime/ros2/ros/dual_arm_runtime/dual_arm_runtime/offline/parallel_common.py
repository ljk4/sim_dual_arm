"""Shared worker preparation and container path mapping for parallel collection/inference."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Callable
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.io import sha256_file

from dual_arm_runtime.offline.collection.campaign_ledger import CampaignError

SIMULATION_REFERENCE_FIELDS = (
    "profile",
    "schemas",
    "scene",
    "control",
    "stream",
    "follower",
    "task",
)


def simulation_path(path: Path, project: Path) -> str:
    """Map a host/runtime path to the simulation container mount."""
    for container in ("/shared", "/data", "/app"):
        root = Path(container)
        if path == root or path.is_relative_to(root):
            return str(path)
    for source, destination in (
        (project / "simulation/mujoco", Path("/app")),
        (project / "shared", Path("/shared")),
        (project / "var", Path("/data")),
    ):
        if path == source or path.is_relative_to(source):
            return str(destination / path.relative_to(source))
    raise CampaignError(f"path is not mounted in simulation: {path}")


def backend_config_for_simulation(backend_template: Path, project: Path) -> str:
    """Return the simulation container path for the read-only backend template."""
    try:
        return simulation_path(backend_template, project)
    except CampaignError:
        if backend_template.is_relative_to(project):
            return str(backend_template)
        raise


def validate_backend_references(
    backend_document: dict, backend_template: Path, project: Path
) -> dict[str, str]:
    """Validate that all robot.* references inside backend_template map into simulation."""
    robot = backend_document.get("robot")
    if not isinstance(robot, dict):
        raise CampaignError(f"backend template has no robot mapping: {backend_template}")
    resolved: dict[str, str] = {}
    for name in SIMULATION_REFERENCE_FIELDS:
        value = robot.get(name)
        if value:
            source = (backend_template.parent / str(value)).resolve()
            resolved[name] = simulation_path(source, project)
    return resolved


def validate_worker_bounds(
    workers: int, domain_base: int, speed: float | None
) -> None:
    if not 1 <= workers <= 16 or not 0 <= domain_base <= 232 - workers + 1:
        raise CampaignError("workers must be 1..16 and ROS domain IDs must stay in 0..232")
    if speed is not None and (not math.isfinite(speed) or not 0.1 < speed <= 100):
        raise CampaignError("realtime speed must be finite and within (0.1, 100]")


def recipe_records(
    names: tuple[str, str],
    runtime_template: Path,
    backend_template: Path,
    selection: dict[str, Any] | None,
) -> dict[str, Any]:
    runtime_field, sim_field = names
    if selection is not None:
        return {
            runtime_field: selection["recipes"][runtime_field],
            sim_field: selection["recipes"][sim_field],
        }
    return {
        runtime_field: {
            "field": f"direct.{runtime_field}",
            "declared": None,
            "path": str(runtime_template),
            "sha256": sha256_file(runtime_template),
        },
        sim_field: {
            "field": f"direct.{sim_field}",
            "declared": None,
            "path": str(backend_template),
            "sha256": sha256_file(backend_template),
        },
    }


def write_worker_manifest(
    *,
    directory: Path,
    project: Path,
    workers: int,
    domain_base: int,
    namespace_prefix: str,
    runtime_document: dict[str, Any],
    backend_config: str,
    recipes: dict[str, Any],
    selection: dict[str, Any] | None,
    overrides: Callable[[int, str], dict[str, Any]],
    extra_entry: Callable[[], dict[str, Any]] | None = None,
) -> Path:
    directory.mkdir(parents=True, exist_ok=False)
    entries = []
    binding = (
        None
        if selection is None
        else {
            **selection.get("binding", {}),
            "robot": selection.get("robot"),
            "task": selection.get("task"),
            "scenario_id": selection.get("scenario_id"),
        }
    )
    for index in range(workers):
        domain_id = domain_base + index
        worker_id = f"worker-{index + 1:02d}"
        worker_dir = directory / worker_id
        worker_dir.mkdir()
        namespace = f"/{namespace_prefix}_{directory.name.replace('-', '_')[-40:]}_{index + 1}"
        runtime_document["robot_namespace"] = namespace
        runtime_path = worker_dir / "runtime.yaml"
        runtime_path.write_text(yaml.safe_dump(runtime_document, sort_keys=False), encoding="utf-8")
        entry: dict[str, Any] = {
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
            "binding": binding,
            "recipes": recipes,
            **(extra_entry() if extra_entry is not None else {}),
            "overrides": overrides(domain_id, namespace),
        }
        entries.append(entry)
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
        json.dumps({"selection": manifest_selection, "workers": entries}, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def parse_worker_prep_args(
    description: str, *, default_speed: float | None
) -> tuple[argparse.ArgumentParser, argparse.Namespace]:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--domain-base", type=int, required=True)
    parser.add_argument("--realtime-speed", type=float, default=default_speed)
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
    return parser, args


__all__ = [
    "SIMULATION_REFERENCE_FIELDS",
    "backend_config_for_simulation",
    "parse_worker_prep_args",
    "recipe_records",
    "simulation_path",
    "validate_backend_references",
    "validate_worker_bounds",
    "write_worker_manifest",
]
