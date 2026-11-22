"""Offline handover candidate-path diagnostic.

Loads the Marvin scene and the handover planning recipe, then calls the same
task planning module used by the P03 teacher.  It writes the immutable plan
with measured TCP errors and continuous clearance; it never steps the live
scene, starts ROS or executes a motion.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from robot_contracts.profiles import load_profile

from robot_backend_sim.marvin_sim.handover.scene_config import load_marvin_scene_config
from robot_backend_sim.marvin_sim.handover.scene import build_marvin_scene
from robot_backend_sim.common.runtime.diagnostics import code_provenance, sha256_file, utc_now, write_json_atomic
from robot_backend_sim.marvin_sim.handover.config import (
    HandoverPlanConfigError,
    load_handover_plan_config,
)
from robot_backend_sim.marvin_sim.handover.plan import plan_document, plan_handover

__all__ = ["main", "run"]

_SHARED_ROOT = Path(os.environ.get("SHARED_ROOT", "/shared"))
_SCENARIO = _SHARED_ROOT / "scenarios/marvin_sim/handover"
DEFAULT_SCENE_CONFIG = _SCENARIO / "configs/simulation/scene/marvin_handover.yaml"
DEFAULT_PLANNING_CONFIG = _SCENARIO / "configs/simulation/scene/handover_paths.yaml"
DEFAULT_PROFILE = _SCENARIO / "profiles/robot/profile.json"
DEFAULT_SCHEMAS = _SHARED_ROOT / "common/interfaces/schemas"
DEFAULT_REPORT_ROOT = (
    Path(os.environ.get("DATA_ROOT", "/data")) / "marvin_sim/handover/debug/handover"
)


@dataclass(frozen=True, slots=True)
class HandoverCheckArguments:
    scene_config: Path
    planning_config: Path
    profile: Path
    schemas: Path
    report_root: Path
    run_id: str


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=DEFAULT_SCENE_CONFIG)
    parser.add_argument("--planning-config", type=Path, default=DEFAULT_PLANNING_CONFIG)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--schemas", type=Path, default=DEFAULT_SCHEMAS)
    parser.add_argument("--report-root", type=Path, default=DEFAULT_REPORT_ROOT)
    parser.add_argument(
        "--run-id",
        default=None,
        help="fresh report run identity (default: handover-check-<UTC>-<pid>)",
    )
    return parser


def parse_arguments(argv: list[str] | None = None) -> HandoverCheckArguments:
    parsed = _parser().parse_args(argv)
    run_id = (
        f"handover-check-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
        if parsed.run_id is None
        else parsed.run_id
    )
    if not run_id or "/" in run_id or run_id in {".", ".."}:
        raise SystemExit("error: --run-id must be a simple path component")
    return HandoverCheckArguments(
        scene_config=parsed.scene_config.resolve(),
        planning_config=parsed.planning_config.resolve(),
        profile=parsed.profile.resolve(),
        schemas=parsed.schemas.resolve(),
        report_root=parsed.report_root.resolve(),
        run_id=run_id,
    )


def _velocities(profile: object) -> dict[str, tuple[float, ...]]:
    result: dict[str, tuple[float, ...]] = {}
    for side in ("left", "right"):
        names = [f"{side}_joint{index}" for index in range(1, 8)]
        by_name = {joint.name: joint for joint in profile.joints}
        try:
            result[side] = tuple(float(by_name[name].max_velocity) for name in names)
        except KeyError as error:
            raise SystemExit(f"error: profile lacks joint {error}") from error
    return result


def run(arguments: HandoverCheckArguments) -> int:
    run_dir = arguments.report_root / arguments.run_id
    if run_dir.exists():
        raise SystemExit(f"error: report run directory already exists: {run_dir}")
    started = time.monotonic()
    try:
        scene_config = load_marvin_scene_config(arguments.scene_config)
        planning = load_handover_plan_config(arguments.planning_config)
        profile = load_profile(arguments.profile, arguments.schemas)
    except HandoverPlanConfigError as error:
        print(f"error: planning_config_invalid: {error}", file=sys.stderr)
        return 2
    except Exception as error:  # noqa: BLE001 - configuration errors are user-facing
        print(f"error: config_invalid: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    scene = build_marvin_scene(scene_config)
    plan = plan_handover(
        scene,
        planning,
        max_velocities=_velocities(profile),
    )
    document = {
        "kind": "handover_paths",
        "run_id": arguments.run_id,
        "created_at_utc": utc_now(),
        "elapsed_s": round(time.monotonic() - started, 6),
        "scene_config": {
            "path": str(arguments.scene_config),
            "sha256": sha256_file(arguments.scene_config),
            "resolved": scene_config.source,
        },
        "planning_config": {
            "path": str(arguments.planning_config),
            "sha256": sha256_file(arguments.planning_config),
        },
        "profile": {
            "path": str(arguments.profile),
            "sha256": sha256_file(arguments.profile),
        },
        "source": code_provenance(),
        "plan": plan_document(plan, planning, scene),
    }
    run_dir.mkdir(parents=True, exist_ok=False)
    write_json_atomic(run_dir / "plan.json", document)
    status = "planned" if plan.passed else "failed"
    print(
        f"status: {status}; targets={len(plan.targets)} segments={len(plan.segments)} "
        f"duration={plan.planned_duration_s:.3f}s"
    )
    for failure in plan.failures:
        print(f"failure: {failure}", file=sys.stderr)
    return 0 if plan.passed else 1


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(argv)
    return run(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
