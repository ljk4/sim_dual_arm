#!/usr/bin/env python3
"""Generate one explicit simulation backend configuration from seed and run id.

Pure YAML preparation: no MuJoCo import, no Docker call and no task start.  The
output directory must be new; previous backend runs are never overwritten.

A recipe can be selected directly (``--template``) or through one scenario
binding (``--scenario`` + ``--role``).  Scenario selection resolves the recipe
relative to the binding directory, validates the binding stage and records the
scenario identity in the run manifest; it never copies numeric parameters into
the binding.
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
from robot_contracts.binding import BindingError, load_binding
from robot_contracts.models import ContractError

RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SCENARIO_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
ROBOT_FIELDS = ("profile", "schemas", "scene", "control", "stream", "follower", "task")
ROLE_RECIPES = {
    "teacher": ("simulation_teacher", "collection"),
    "policy": ("simulation_policy", "inference"),
}


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


def _shared_root(explicit: Path | None) -> Path:
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    here = Path(__file__).resolve()
    candidates = (here.parents[2] / "shared", here.parents[3] / "shared", Path("/shared"))
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    return Path("/shared")


def _scenario_binding(shared_root: Path, scenario: str) -> Path:
    if SCENARIO_ID.fullmatch(scenario) is None or ".." in scenario:
        raise PrepareError(
            "scenario id must contain 1..128 safe characters and no path traversal"
        )
    path = Path(shared_root) / "scenarios" / scenario / "binding.yaml"
    if not path.is_file():
        raise PrepareError(f"scenario {scenario!r} has no binding file: {path}")
    return path


def _select_scenario(
    *,
    scenario: str,
    role: str,
    shared_root: Path | None,
    bundle: Path | None,
) -> tuple[Path, dict[str, Any], Path]:
    if role not in ROLE_RECIPES:
        raise PrepareError("--role must be 'teacher' or 'policy' with --scenario")
    recipe_field, stage = ROLE_RECIPES[role]
    if bundle is not None and role != "policy":
        raise PrepareError("--bundle applies to policy preparation only")
    root = _shared_root(shared_root)
    binding_path = _scenario_binding(root, scenario)
    try:
        resolved = load_binding(binding_path, shared_root=root, bundle=bundle)
    except (BindingError, ContractError, OSError) as error:
        raise PrepareError(str(error)) from error
    declared = getattr(resolved.binding.recipes, recipe_field)
    if declared is None:
        raise PrepareError(
            f"scenario {resolved.binding.scenario_id!r} declares no recipes.{recipe_field}"
        )
    reference = resolved.references[f"recipes.{recipe_field}"]
    try:
        resolved.validate_stage(stage)
    except BindingError as error:
        hint = " (pass --bundle for inference)" if stage == "inference" else ""
        raise PrepareError(f"{error}{hint}") from error
    selection = {
        "mode": "scenario",
        "scenario_id": resolved.binding.scenario_id,
        "binding": {"path": str(resolved.path), "sha256": resolved.sha256},
        "recipe": reference.record(),
    }
    if role == "policy":
        selection["bundle"] = resolved.effective_bundle()
    profile = resolved.references["profiles.robot"].path
    return reference.path, selection, profile


def _direct_selection() -> dict[str, Any]:
    return {
        "mode": "direct",
        "scenario_id": None,
        "binding": None,
        "recipe": None,
    }


def _check_scenario_profile(
    document: dict[str, Any], template_path: Path, expected: Path
) -> None:
    """The selected recipe must use the profile the resolved binding declares."""
    robot = document.get("robot")
    declared = None if not isinstance(robot, dict) else robot.get("profile")
    if not isinstance(declared, str):
        return
    actual = (template_path.parent / declared).resolve()
    if actual != expected:
        raise PrepareError(
            f"recipe robot profile {actual} differs from scenario profile {expected}"
        )


def prepare_backend(
    *,
    template: Path | None = None,
    seed: int,
    run_id: str,
    output: Path,
    scenario: str | None = None,
    role: str | None = None,
    shared_root: Path | None = None,
    bundle: Path | None = None,
) -> dict[str, Any]:
    """Write backend.yaml and run_manifest.json into a new output directory."""
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise PrepareError("seed must be an integer")
    if RUN_ID.fullmatch(run_id) is None or ".." in run_id:
        raise PrepareError("run_id must be a safe 1..128 character path segment")
    if (template is None) == (scenario is None):
        raise PrepareError("exactly one of template or scenario is required")
    if template is not None:
        if role is not None:
            raise PrepareError("--role requires --scenario")
        if bundle is not None:
            raise PrepareError("--bundle requires --scenario")
        if shared_root is not None:
            raise PrepareError("--shared-root requires --scenario")
        template_path = Path(template).resolve()
        template_sha256 = _sha256(template_path)
        selection = _direct_selection()
        expected_profile = None
    else:
        assert scenario is not None
        template_path, selection, expected_profile = _select_scenario(
            scenario=scenario, role=str(role), shared_root=shared_root, bundle=bundle
        )
        template_sha256 = selection["recipe"]["sha256"]
    document = _load_template(template_path)
    if expected_profile is not None:
        _check_scenario_profile(document, template_path, expected_profile)
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
        "selection": selection,
        "overrides": {
            "seed": {"value": seed, "source": "argument", "target": "run.seed"},
            "run_id": {"value": run_id, "source": "argument"},
            "output": {"value": str(destination), "source": "argument"},
        },
        "inputs": {
            "template": str(template_path),
            "template_sha256": template_sha256,
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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--template", type=Path, help="direct backend recipe path")
    selection.add_argument(
        "--scenario", help="scenario id under <shared-root>/scenarios/<id>/binding.yaml"
    )
    parser.add_argument("--role", choices=sorted(ROLE_RECIPES), help="teacher or policy recipe")
    parser.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    parser.add_argument(
        "--bundle",
        type=Path,
        help="verified model bundle directory required for policy/inference preparation",
    )
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    _require_container()
    parser = _parser()
    arguments = parser.parse_args(argv)
    if arguments.scenario is not None and arguments.role is None:
        parser.error("--role is required with --scenario")
    if arguments.template is not None and arguments.role is not None:
        parser.error("--role requires --scenario")
    if arguments.template is not None and arguments.shared_root is not None:
        parser.error("--shared-root requires --scenario")
    if arguments.template is not None and arguments.bundle is not None:
        parser.error("--bundle requires --scenario")
    if (
        arguments.scenario is not None
        and arguments.role == "teacher"
        and arguments.bundle is not None
    ):
        parser.error("--bundle applies to policy preparation only")
    try:
        manifest = prepare_backend(
            template=arguments.template,
            seed=arguments.seed,
            run_id=arguments.run_id,
            output=arguments.output,
            scenario=arguments.scenario,
            role=arguments.role,
            shared_root=arguments.shared_root,
            bundle=arguments.bundle,
        )
    except (PrepareError, OSError) as error:
        print(f"prepare_backend: {error}", file=sys.stderr)
        return 1
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
