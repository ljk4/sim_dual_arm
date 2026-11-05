#!/usr/bin/env python3
"""Generate one explicit simulation backend configuration from seed and run id.

Pure YAML preparation: no MuJoCo import, no Docker call and no task start.  The
output directory must be new; previous backend runs are never overwritten.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

import yaml

RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
ROBOT_FIELDS = ("profile", "schemas", "scene", "control", "stream", "follower", "task")


class PrepareError(ValueError):
    """The requested backend preparation is unsafe or inconsistent."""


def _require_container() -> None:
    marker = os.environ.get("PROJECT_CONTAINER")
    if marker != "0" and (marker == "1" or Path("/.dockerenv").exists()):
        return
    print(
        "error: prepare_backend must run in the simulation environment; "
        "use scripts/run.sh prepare ...",
        file=sys.stderr,
    )
    raise SystemExit(2)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_template(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise PrepareError(f"cannot read backend template {path}: {error}") from error
    if not isinstance(document, dict):
        raise PrepareError("backend template must be one YAML object")
    return document


def prepare_backend(
    *,
    template: Path,
    seed: int,
    run_id: str,
    output: Path,
) -> dict[str, Any]:
    """Write backend.yaml and run_manifest.json into a new output directory."""
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise PrepareError("seed must be an integer")
    if RUN_ID.fullmatch(run_id) is None or ".." in run_id:
        raise PrepareError("run_id must be a safe 1..128 character path segment")
    template_path = Path(template).resolve()
    document = _load_template(template_path)
    robot = document.get("robot")
    if not isinstance(robot, dict):
        raise PrepareError("backend template has no robot object")
    for field in ROBOT_FIELDS:
        value = robot.get(field)
        if value is None:
            continue
        if not isinstance(value, str):
            raise PrepareError(f"robot.{field} must be a path string")
        source = (template_path.parent / value).resolve()
        if not source.exists():
            raise PrepareError(f"robot.{field} is missing: {source}")
        robot[field] = str(source)
    run = document.get("run")
    if not isinstance(run, dict):
        run = {}
        document["run"] = run
    run["seed"] = seed

    destination = Path(output).resolve()
    if destination.exists():
        raise PrepareError(f"output already exists; refusing to overwrite: {destination}")
    destination.mkdir(parents=True)
    backend_path = destination / "backend.yaml"
    backend_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "kind": "simulation_prepare",
        "run_id": run_id,
        "seed": seed,
        "component": "simulation",
        "inputs": {
            "template": str(template_path),
            "template_sha256": _sha256(template_path),
        },
        "outputs": {
            "backend": str(backend_path),
            "backend_sha256": _sha256(backend_path),
        },
    }
    (destination / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def main(argv: list[str] | None = None) -> int:
    _require_container()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        manifest = prepare_backend(
            template=arguments.template,
            seed=arguments.seed,
            run_id=arguments.run_id,
            output=arguments.output,
        )
    except (PrepareError, OSError) as error:
        print(f"prepare_backend: {error}", file=sys.stderr)
        return 1
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
