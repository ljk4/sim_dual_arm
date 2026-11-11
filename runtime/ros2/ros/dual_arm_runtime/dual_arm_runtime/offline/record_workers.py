"""Prepare isolated, headless simulation/runtime configurations for recording."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

import yaml

from dual_arm_runtime.offline.campaign_ledger import CampaignError
from dual_arm_runtime.offline.collect_attempt import REFERENCES, check_ros_sdk

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])


def simulation_path(path: Path, project: Path) -> str:
    for source, destination in (
        (project / "simulation/mujoco", Path("/app")),
        (project / "contracts", Path("/contracts")),
        (project / "profiles", Path("/profiles")),
        (project / "var", Path("/data")),
    ):
        if path.is_relative_to(source):
            return str(destination / path.relative_to(source))
    raise CampaignError(f"path is not mounted in simulation: {path}")


def prepare_workers(
    project: Path, directory: Path, workers: int, domain_base: int,
    runtime_template: Path, backend_template: Path, speed: float,
) -> Path:
    if not 1 <= workers <= 16 or not 0 <= domain_base <= 232 - workers + 1:
        raise CampaignError("workers must be 1..16 and ROS domain IDs must stay in 0..232")
    if not math.isfinite(speed) or not 0.1 < speed <= 100:
        raise CampaignError("realtime speed must be finite and within (0.1, 100]")
    project, directory = project.resolve(), directory.resolve()
    if not directory.is_relative_to(project / "var"):
        raise CampaignError("worker directory must be below the mounted project var directory")
    runtime = yaml.safe_load(runtime_template.read_text())
    backend = yaml.safe_load(backend_template.read_text())
    if runtime.get("action_source") != "teacher" or backend["robot"].get("task_mode") != "teacher":
        raise CampaignError("parallel recording requires teacher runtime and backend templates")
    if backend["robot"].get("render_quality", {}).get("enabled") is not True:
        raise CampaignError("parallel recording requires backend render_quality.enabled")
    for name in REFERENCES:
        if name in runtime:
            runtime[name] = str((runtime_template.parent / runtime[name]).resolve())
    for name in ("profile", "schemas", "scene", "control", "stream", "follower", "task"):
        if backend["robot"].get(name):
            source = (backend_template.parent / backend["robot"][name]).resolve()
            backend["robot"][name] = simulation_path(source, project)
    directory.mkdir(parents=True, exist_ok=False)
    entries = []
    for index in range(workers):
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
                "id": worker_id, "domain_id": domain_base + index, "namespace": namespace,
                "runtime_config": str(runtime_path),
                "backend_config": simulation_path(backend_path, project),
                "backend_log": str(worker_dir / "backend-runs/backend/log"),
                "run_root": simulation_path(worker_dir / "backend-runs", project),
            }
        )
    manifest = directory / "workers.json"
    manifest.write_text(json.dumps({"schema_version": 1, "workers": entries}, indent=2) + "\n")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--workers", type=int, required=True)
    parser.add_argument("--domain-base", type=int, required=True)
    parser.add_argument("--realtime-speed", type=float, default=1.0)
    parser.add_argument("--runtime-config", type=Path, required=True)
    parser.add_argument("--backend-template", type=Path, required=True)
    args = parser.parse_args()
    code = check_ros_sdk(ROOT)
    if code:
        return code
    try:
        manifest = prepare_workers(
            ROOT, args.directory, args.workers, args.domain_base,
            args.runtime_config.resolve(), args.backend_template.resolve(), args.realtime_speed,
        )
    except (CampaignError, OSError, ValueError) as error:
        parser.error(str(error))
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
