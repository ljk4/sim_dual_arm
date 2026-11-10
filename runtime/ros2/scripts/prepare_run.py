#!/usr/bin/env python3
"""Bind a generic Policy JSON to one executable runtime configuration.

This is pure YAML/JSON preparation.  It never imports ``dual_arm_learning``,
Torch, MuJoCo or ROS, never calls Docker, and never starts a node.  The generic
Policy binding is produced by the learning-side prepare-policy command; all
model/processor/profile/schema identities are copied from that verified file.
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

ROOT = Path(__file__).resolve().parents[3]
RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
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
    document: dict[str, Any], bound: dict[str, Any], template_path: Path, project_root: Path
) -> None:
    bound_paths: dict[str, str] = {}
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


def _run_manifest(
    *,
    run_id: str,
    seed: int,
    bound: dict[str, Any],
    model: dict[str, Any],
    template_path: Path,
    binding_path: Path,
    runtime_path: Path,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "runtime_prepare",
        "run_id": run_id,
        "seed": seed,
        "component": "runtime",
        "policy": {
            "run_id": run_id,
            "endpoint": bound["endpoint"],
            "bundle_digest": model["bundle_digest"],
            "processor_digest": model["processor_digest"],
            "profile_sha256": bound["profile"]["sha256"],
            "observation_schema_sha256": bound["observation_schema"]["sha256"],
            "action_schema_sha256": bound["action_schema"]["sha256"],
        },
        "inputs": {
            "template": str(template_path),
            "template_sha256": _sha256(template_path),
            "policy_binding": str(binding_path),
            "policy_binding_sha256": _sha256(binding_path),
        },
        "outputs": {
            "runtime": str(runtime_path),
            "runtime_sha256": _sha256(runtime_path),
        },
    }


def prepare_run(
    *,
    template: Path,
    binding: Path,
    seed: int,
    run_id: str,
    output: Path,
    project_root: Path = ROOT,
) -> dict[str, Any]:
    """Write runtime.yaml and run_manifest.json into a new output directory."""
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise PrepareError("seed must be an integer")
    if RUN_ID.fullmatch(run_id) is None or ".." in run_id:
        raise PrepareError("run_id must be a safe 1..128 character path segment")
    template_path = Path(template).resolve()
    binding_path = Path(binding).resolve()
    document = _load_yaml(template_path, "runtime template")
    bound = _load_binding(binding_path)
    if bound["run_id"] != run_id:
        raise PrepareError(
            f"binding.run_id {bound['run_id']!r} differs from runtime run_id {run_id!r}"
        )
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
    manifest = _run_manifest(
        run_id=run_id,
        seed=seed,
        bound=bound,
        model=model,
        template_path=template_path,
        binding_path=binding_path,
        runtime_path=runtime_path,
    )
    (destination / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def main(argv: list[str] | None = None) -> int:
    _require_container()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--policy-binding", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    arguments = parser.parse_args(argv)
    try:
        manifest = prepare_run(
            template=arguments.template,
            binding=arguments.policy_binding,
            seed=arguments.seed,
            run_id=arguments.run_id,
            output=arguments.output,
            project_root=arguments.project_root,
        )
    except (PrepareError, OSError) as error:
        print(f"prepare_run: {error}", file=sys.stderr)
        return 1
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
