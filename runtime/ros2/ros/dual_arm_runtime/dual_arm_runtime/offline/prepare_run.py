#!/usr/bin/env python3
"""Bind a generic Policy JSON to one executable runtime configuration.

This is pure YAML/JSON preparation.  It never imports ``dual_arm_learning``,
Torch, MuJoCo or ROS, never calls Docker, and never starts a node.  The generic
Policy binding is produced by the learning-side prepare-policy command; all
model/processor/profile/schema identities are copied from that verified file.

A runtime recipe can be selected directly (``--template``) or through one
scenario binding (``--scenario`` + ``--role``).  Scenario selection only
chooses an existing recipe and validates the binding stage; the verified Policy
binding remains mandatory for policy runs and is never replaced by the
scenario document.
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
from robot_contracts.binding import BindingError, ResolvedBinding
from robot_contracts.binding import load_binding as load_scenario_binding
from robot_contracts.models import ContractError

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])
RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SCENARIO_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")
PLACEHOLDER = "FROM_VERIFIED_ACT_BUNDLE"
RUNTIME_FIELDS = (
    "robot_profile",
    "observation_schema",
    "action_schema",
    "control_constraints",
    "follower_config",
)
REQUIRED_BINDING = (
    "schema_version",
    "kind",
    "run_id",
    "endpoint",
    "model",
    "profile",
    "observation_schema",
    "action_schema",
    "service",
)
ROLE_RECIPES = {
    "teacher": ("runtime_teacher", "collection"),
    "policy": ("runtime_policy", "inference"),
}


class PrepareError(ValueError):
    """The Policy binding or runtime template is not deployable."""


def _require_container() -> None:
    marker = os.environ.get("PROJECT_CONTAINER")
    if marker != "0" and (marker == "1" or Path("/.dockerenv").exists()):
        return
    print(
        "error: prepare_run must run in the runtime environment; "
        "use runtime/ros2/scripts/run.sh prepare ...",
        file=sys.stderr,
    )
    raise SystemExit(2)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _digest(value: object, where: str) -> str:
    if not isinstance(value, str) or SHA256.fullmatch(value) is None:
        raise PrepareError(f"{where} must be a lowercase SHA-256 digest")
    return value


def _text(value: object, where: str) -> str:
    if not isinstance(value, str) or not value:
        raise PrepareError(f"{where} must be a non-empty string")
    return value


def _endpoint_port(value: object, where: str = "binding.endpoint") -> int:
    endpoint = _text(value, where)
    host, separator, port_text = endpoint.rpartition(":")
    if not separator or not host or not port_text.isdigit():
        raise PrepareError(f"{where} must be host:port")
    port = int(port_text)
    if not 1 <= port <= 65535:
        raise PrepareError(f"{where} port must be within 1..65535")
    return port


def _container_path(path: Path, project_root: Path) -> str:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(project_root.resolve())
    except ValueError:
        return str(resolved)
    return f"/workspace/{relative.as_posix()}"


def _load_yaml(path: Path, label: str) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise PrepareError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(document, dict):
        raise PrepareError(f"{label} must be one YAML object")
    return document


def _load_binding(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PrepareError(f"cannot read Policy binding {path}: {error}") from error
    if not isinstance(document, dict):
        raise PrepareError("Policy binding must be one JSON object")
    missing = [name for name in REQUIRED_BINDING if name not in document]
    if missing:
        raise PrepareError("Policy binding is missing: " + ", ".join(missing))
    if document.get("schema_version") != 1 or document.get("kind") != "policy-binding":
        raise PrepareError("Policy binding must have schema_version 1 and kind policy-binding")
    endpoint_port = _endpoint_port(document.get("endpoint"))
    model = document.get("model")
    if not isinstance(model, dict):
        raise PrepareError("binding.model must be an object")
    for field in ("bundle_digest", "processor_digest"):
        if field not in model:
            raise PrepareError(f"binding.model.{field} is required")
    for field in (
        "bundle_digest",
        "processor_digest",
        "model_sha256",
        "config_sha256",
        "stats_sha256",
    ):
        if field in model:
            _digest(model[field], f"binding.model.{field}")
    if "bundle" in model:
        _text(model["bundle"], "binding.model.bundle")
    for name in ("profile", "observation_schema", "action_schema"):
        record = document.get(name)
        if not isinstance(record, dict):
            raise PrepareError(f"binding.{name} must be an object")
        _text(record.get("path"), f"binding.{name}.path")
        _digest(record.get("sha256"), f"binding.{name}.sha256")
    service = document.get("service")
    if not isinstance(service, dict):
        raise PrepareError("binding.service must be an object")
    for name in (
        "port",
        "task_id",
        "task_text",
        "predict_deadline_s",
        "open_deadline_s",
        "close_deadline_s",
        "max_message_bytes",
    ):
        if name not in service:
            raise PrepareError(f"binding.service.{name} is required")
    service_port = service["port"]
    if isinstance(service_port, bool) or not isinstance(service_port, int):
        raise PrepareError("binding.service.port must be an integer")
    if service_port != endpoint_port:
        raise PrepareError(
            f"binding.service.port {service_port} must match endpoint port {endpoint_port}"
        )
    return document


def _shared_root(explicit: Path | None, project_root: Path) -> Path:
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    candidate = Path(project_root) / "shared"
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
    project_root: Path,
) -> tuple[Path, ResolvedBinding, dict[str, Any]]:
    if role not in ROLE_RECIPES:
        raise PrepareError("--role must be 'teacher' or 'policy' with --scenario")
    recipe_field, stage = ROLE_RECIPES[role]
    if bundle is not None and role != "policy":
        raise PrepareError("--bundle applies to policy preparation only")
    root = _shared_root(shared_root, project_root)
    binding_path = _scenario_binding(root, scenario)
    try:
        resolved = load_scenario_binding(binding_path, shared_root=root, bundle=bundle)
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
    if role == "teacher":
        teacher_api = resolved.binding.interfaces.teacher_api
        if not teacher_api:
            raise PrepareError(
                "scenario teacher preparation requires interfaces.teacher_api"
            )
        selection["teacher_api"] = teacher_api
    return reference.path, resolved, selection


def _direct_selection() -> dict[str, Any]:
    return {
        "mode": "direct",
        "scenario_id": None,
        "binding": None,
        "recipe": None,
    }


def _check_recipe_profiles(
    document: dict[str, Any], template_path: Path, resolved: ResolvedBinding
) -> None:
    for name, field in (
        ("robot", "robot_profile"),
        ("observation", "observation_schema"),
        ("action", "action_schema"),
    ):
        declared = document.get(field)
        if not isinstance(declared, str):
            continue
        actual = (template_path.parent / declared).resolve()
        expected = resolved.references[f"profiles.{name}"].path
        if actual != expected:
            raise PrepareError(
                f"runtime recipe {field} {actual} differs from scenario profile {expected}"
            )


def _check_policy_identity(
    resolved: ResolvedBinding, bound: dict[str, Any]
) -> None:
    """The verified Policy binding must describe the scenario's contracts."""
    for name, key in (
        ("robot", "profile"),
        ("observation", "observation_schema"),
        ("action", "action_schema"),
    ):
        reference = resolved.references[f"profiles.{name}"]
        record = bound.get(key)
        if not isinstance(record, dict):
            raise PrepareError(f"Policy binding has no {key} object")
        if record.get("sha256") != reference.sha256:
            raise PrepareError(
                f"Policy binding {key}.sha256 differs from scenario {name} profile"
            )
        if record.get("semantic_sha256") != reference.semantic_sha256:
            raise PrepareError(
                f"Policy binding {key}.semantic_sha256 differs from scenario {name} profile"
            )


def _existing_directory(path: Path, label: str) -> Path:
    try:
        resolved = path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise PrepareError(f"{label} is missing or unreadable: {path}: {error}") from error
    if not resolved.is_dir():
        raise PrepareError(f"{label} is not a directory: {resolved}")
    return resolved


def _check_model_source(
    resolved: ResolvedBinding, bound: dict[str, Any], binding_path: Path
) -> None:
    """The effective scenario bundle must be the directory named by the Policy binding.

    Only the real directory identity is compared.  Two different directories
    cannot be proven to hold the same model without re-verifying the model
    contents, so they are rejected rather than accepted on Profile identity.
    ``samefile`` also accepts symlinks, ``..`` forms and bind-mounted aliases of
    the same directory.
    """
    effective = resolved.bundle.path
    if effective is None:
        raise PrepareError("scenario inference has no effective model bundle")
    model = bound.get("model")
    if not isinstance(model, dict):
        raise PrepareError("Policy binding has no model object")
    declared = model.get("bundle")
    if not isinstance(declared, str) or not declared.strip():
        raise PrepareError(
            "Policy binding model.bundle is required for scenario preparation; "
            "the verified model source cannot be determined"
        )
    declared_path = Path(declared)
    if not declared_path.is_absolute():
        declared_path = binding_path.parent / declared_path
    scenario_dir = _existing_directory(effective, "effective scenario bundle")
    policy_dir = _existing_directory(declared_path, "Policy binding model.bundle")
    try:
        same = os.path.samefile(scenario_dir, policy_dir)
    except OSError as error:
        raise PrepareError(
            f"cannot compare model directories {scenario_dir} and {policy_dir}: {error}"
        ) from error
    if not same:
        raise PrepareError(
            f"Policy binding model.bundle {policy_dir} differs from effective "
            f"scenario bundle {scenario_dir}; different model directories cannot be "
            "proven identical without re-verifying model contents"
        )


def _bind_policy(document: dict[str, Any], bound: dict[str, Any], model: dict[str, Any]) -> None:
    policy = document.get("policy")
    if not isinstance(policy, dict):
        raise PrepareError("runtime template has no policy object")
    policy["endpoint"] = bound["endpoint"]
    policy["model_bundle_digest"] = model["bundle_digest"]
    policy["processor_digest"] = model["processor_digest"]
    policy["allow_test_policy"] = False
    policy["expected_test_only"] = False
    service = bound["service"]
    for field in (
        "task_id",
        "task_text",
        "predict_deadline_s",
        "open_deadline_s",
        "close_deadline_s",
        "max_message_bytes",
    ):
        policy[field] = service[field]
    for field in ("chunk_size", "action_period_ns", "action_lead_ns"):
        if field in service and field in policy and service[field] != policy[field]:
            raise PrepareError(
                f"binding.service.{field} differs from runtime template {policy[field]!r}"
            )
    policy["connect_timeout_s"] = max(float(policy.get("connect_timeout_s", 2.0)), 5.0)


def _resolve_bound_paths(
    document: dict[str, Any],
    bound: dict[str, Any] | None,
    template_path: Path,
    project_root: Path,
) -> None:
    bound_paths: dict[str, str] = {}
    if bound is not None:
        for name, record in (
            ("robot_profile", bound["profile"]),
            ("observation_schema", bound["observation_schema"]),
            ("action_schema", bound["action_schema"]),
        ):
            source = Path(record["path"]).resolve()
            if not source.is_file():
                raise PrepareError(f"binding.{name}.path is missing: {source}")
            if _sha256(source) != record["sha256"]:
                raise PrepareError(f"binding.{name} file hash differs from {source}")
            bound_paths[name] = _container_path(source, project_root)
    for field in RUNTIME_FIELDS:
        if field in bound_paths:
            document[field] = bound_paths[field]
            continue
        value = document.get(field)
        if value is None:
            continue
        if not isinstance(value, str):
            raise PrepareError(f"{field} must be a path string")
        source = (template_path.parent / value).resolve()
        if not source.exists():
            raise PrepareError(f"{field} is missing: {source}")
        document[field] = _container_path(source, project_root)


def _check_teacher_budget(
    document: dict[str, Any], resolved: ResolvedBinding
) -> None:
    """The runtime stream clock and budget must match the simulation task judge."""
    teacher = document.get("teacher")
    if not isinstance(teacher, dict) or "task_sim_budget_ns" not in teacher:
        # A minimal direct/synthetic recipe is validated by RuntimeRecipe.
        return
    simulation_reference = resolved.references.get("recipes.simulation_teacher")
    if simulation_reference is None:
        raise PrepareError("scenario declares no simulation_teacher recipe for teacher preparation")
    backend_path = simulation_reference.path
    backend_document = _load_yaml(backend_path, "simulation teacher recipe")
    robot = backend_document.get("robot")
    if not isinstance(robot, dict) or not isinstance(robot.get("task"), str):
        raise PrepareError(
            f"simulation teacher recipe has no robot.task ({backend_path})"
        )
    task_path = (backend_path.parent / robot["task"]).resolve()
    if not task_path.is_file():
        raise PrepareError(f"simulation task recipe is missing: {task_path}")
    task_document = _load_yaml(task_path, "simulation task recipe")
    success = task_document.get("success")
    task_teacher = task_document.get("teacher")
    if not isinstance(success, dict) or "max_task_sim_duration_s" not in success:
        return
    if not isinstance(task_teacher, dict):
        raise PrepareError(f"simulation task recipe is incomplete: {task_path}")
    try:
        task_budget_ns = round(float(success["max_task_sim_duration_s"]) * 1e9)
        task_chunk_size = int(task_teacher["chunk_size"])
        task_period_ns = int(task_teacher["action_period_ns"])
        runtime_budget_ns = int(teacher["task_sim_budget_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise PrepareError(f"teacher budget fields are not numeric: {error}") from error
    if runtime_budget_ns != task_budget_ns:
        raise PrepareError(
            f"runtime teacher.task_sim_budget_ns {runtime_budget_ns} differs from "
            f"simulation success.max_task_sim_duration_s ({task_budget_ns})"
        )
    if int(teacher.get("chunk_size", -1)) != task_chunk_size:
        raise PrepareError("runtime teacher.chunk_size differs from the simulation task")
    if int(teacher.get("action_period_ns", -1)) != task_period_ns:
        raise PrepareError(
            "runtime teacher.action_period_ns differs from the simulation task"
        )


def _run_manifest(
    *,
    run_id: str,
    seed: int,
    bound: dict[str, Any] | None,
    model: dict[str, Any] | None,
    selection: dict[str, Any],
    bundle: dict[str, Any] | None,
    template_path: Path,
    template_sha256: str,
    binding_path: Path | None,
    runtime_path: Path,
) -> dict[str, Any]:
    inputs: dict[str, Any] = {
        "template": str(template_path),
        "template_sha256": template_sha256,
    }
    if binding_path is not None:
        inputs["policy_binding"] = str(binding_path)
        inputs["policy_binding_sha256"] = _sha256(binding_path)
    if bundle is not None:
        inputs["bundle"] = bundle
    overrides = {
        "seed": {"value": seed, "source": "argument", "target": "reset.seed"},
        "run_id": {"value": run_id, "source": "argument"},
        "output": {"value": str(runtime_path.parent), "source": "argument"},
    }
    if bundle is not None:
        overrides["bundle"] = {"source": bundle["source"], "path": bundle["effective"]}
    return {
        "schema_version": 1,
        "kind": "runtime_prepare",
        "run_id": run_id,
        "seed": seed,
        "component": "runtime",
        "selection": selection,
        "overrides": overrides,
        "policy": None if bound is None or model is None else {
            "run_id": run_id,
            "endpoint": bound["endpoint"],
            "bundle_digest": model["bundle_digest"],
            "processor_digest": model["processor_digest"],
            "profile_sha256": bound["profile"]["sha256"],
            "observation_schema_sha256": bound["observation_schema"]["sha256"],
            "action_schema_sha256": bound["action_schema"]["sha256"],
        },
        "inputs": inputs,
        "outputs": {
            "runtime": str(runtime_path),
            "runtime_sha256": _sha256(runtime_path),
        },
    }


def prepare_run(
    *,
    template: Path | None = None,
    binding: Path | None = None,
    seed: int,
    run_id: str,
    output: Path,
    project_root: Path = ROOT,
    scenario: str | None = None,
    role: str | None = None,
    shared_root: Path | None = None,
    bundle: Path | None = None,
) -> dict[str, Any]:
    """Write runtime.yaml and run_manifest.json into a new output directory."""
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise PrepareError("seed must be an integer")
    if RUN_ID.fullmatch(run_id) is None or ".." in run_id:
        raise PrepareError("run_id must be a safe 1..128 character path segment")
    if (template is None) == (scenario is None):
        raise PrepareError("exactly one of template or scenario is required")
    resolved_scenario: ResolvedBinding | None = None
    if template is not None:
        if role is not None:
            raise PrepareError("--role requires --scenario")
        if shared_root is not None:
            raise PrepareError("--shared-root requires --scenario")
        if bundle is not None:
            raise PrepareError("--bundle requires --scenario")
        if binding is None:
            raise PrepareError("a verified Policy binding is required with --template")
        role = "policy"
        template_path = Path(template).resolve()
        selection = _direct_selection()
        template_sha256 = _sha256(template_path)
    else:
        assert scenario is not None
        if role is None:
            raise PrepareError("--role is required with --scenario")
        if role == "teacher" and binding is not None:
            raise PrepareError("a teacher runtime does not consume a Policy binding")
        if role == "policy" and binding is None:
            raise PrepareError("a policy runtime requires a verified Policy binding")
        template_path, resolved_scenario, selection = _select_scenario(
            scenario=scenario,
            role=role,
            shared_root=shared_root,
            bundle=bundle,
            project_root=project_root,
        )
        template_sha256 = selection["recipe"]["sha256"]
    document = _load_yaml(template_path, "runtime template")
    if role == "teacher":
        declared_api = document.get("teacher_api")
        if resolved_scenario is not None:
            selected_api = resolved_scenario.binding.interfaces.teacher_api
            if declared_api is not None and declared_api != selected_api:
                raise PrepareError(
                    f"runtime recipe teacher_api {declared_api!r} differs from "
                    f"binding {selected_api!r}"
                )
            document["teacher_api"] = selected_api
            _check_teacher_budget(document, resolved_scenario)
        elif not isinstance(declared_api, str) or not declared_api:
            raise PrepareError("direct teacher preparation requires recipe teacher_api")
    if resolved_scenario is not None:
        _check_recipe_profiles(document, template_path, resolved_scenario)
    bound: dict[str, Any] | None = None
    model: dict[str, Any] | None = None
    binding_path: Path | None = None
    if role == "policy":
        binding_path = Path(binding).resolve()  # type: ignore[arg-type]
        bound = _load_binding(binding_path)
        if bound["run_id"] != run_id:
            raise PrepareError(
                f"binding.run_id {bound['run_id']!r} differs from runtime run_id {run_id!r}"
            )
        if resolved_scenario is not None:
            _check_policy_identity(resolved_scenario, bound)
            _check_model_source(resolved_scenario, bound, binding_path)
        model = bound["model"]
        _bind_policy(document, bound, model)
    if PLACEHOLDER in yaml.safe_dump(document):
        raise PrepareError(f"runtime template still contains unresolved {PLACEHOLDER}")
    _resolve_bound_paths(document, bound, template_path, project_root)
    reset = document.get("reset")
    if not isinstance(reset, dict):
        reset = {}
    reset.update({"enabled": True, "seed": seed})
    document["reset"] = reset
    destination = Path(output).resolve()
    if destination.exists():
        raise PrepareError(f"output already exists; refusing to overwrite: {destination}")
    destination.mkdir(parents=True)
    runtime_path = destination / "runtime.yaml"
    runtime_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    bundle_summary = None if resolved_scenario is None else selection.get("bundle")
    manifest = _run_manifest(
        run_id=run_id,
        seed=seed,
        bound=bound,
        model=model,
        selection=selection,
        bundle=bundle_summary,
        template_path=template_path,
        template_sha256=template_sha256,
        binding_path=binding_path,
        runtime_path=runtime_path,
    )
    (destination / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def _parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    selection = result.add_mutually_exclusive_group(required=True)
    selection.add_argument("--template", type=Path, help="direct runtime recipe path")
    selection.add_argument(
        "--scenario", help="scenario id under <shared-root>/scenarios/<id>/binding.yaml"
    )
    result.add_argument("--role", choices=sorted(ROLE_RECIPES), help="teacher or policy recipe")
    result.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    result.add_argument(
        "--bundle",
        type=Path,
        help="verified model bundle required for policy/inference preparation",
    )
    result.add_argument("--policy-binding", type=Path, help="verified generic Policy binding")
    result.add_argument("--seed", type=int, required=True)
    result.add_argument("--run-id", required=True)
    result.add_argument("--output", type=Path, required=True)
    result.add_argument("--project-root", type=Path, default=ROOT)
    return result


def main(argv: list[str] | None = None) -> int:
    _require_container()
    parser = _parser()
    arguments = parser.parse_args(argv)
    if arguments.scenario is None:
        if arguments.role is not None:
            parser.error("--role requires --scenario")
        if arguments.shared_root is not None:
            parser.error("--shared-root requires --scenario")
        if arguments.bundle is not None:
            parser.error("--bundle requires --scenario")
        if arguments.policy_binding is None:
            parser.error("--policy-binding is required with --template")
    else:
        if arguments.role is None:
            parser.error("--role is required with --scenario")
        if arguments.role == "teacher" and arguments.policy_binding is not None:
            parser.error("a teacher runtime does not consume a Policy binding")
        if arguments.role == "policy" and arguments.policy_binding is None:
            parser.error("a policy runtime requires --policy-binding")
        if arguments.role == "teacher" and arguments.bundle is not None:
            parser.error("--bundle applies to policy preparation only")
    try:
        manifest = prepare_run(
            template=arguments.template,
            binding=arguments.policy_binding,
            seed=arguments.seed,
            run_id=arguments.run_id,
            output=arguments.output,
            project_root=arguments.project_root,
            scenario=arguments.scenario,
            role=arguments.role,
            shared_root=arguments.shared_root,
            bundle=arguments.bundle,
        )
    except (PrepareError, OSError) as error:
        print(f"prepare_run: {error}", file=sys.stderr)
        return 1
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
