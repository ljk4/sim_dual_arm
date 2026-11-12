#!/usr/bin/env python3
"""Verify one ACT bundle and emit a generic Policy binding plus service config.

All model, processor, profile and schema identities come from the real bundle
and the shipped documents.  There is no hand-written digest and no fallback to
a fake policy.  The output directory is new; old bundles and runs are never
overwritten.  The runtime side consumes only the generic JSON.
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
for _path in (
    ROOT / "learning/lerobot/src",
    ROOT / "shared/common/python/run_logging/src",
    ROOT / "shared/common/python/contracts/src",
):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from dual_arm_learning.bundle import deployment_summary  # noqa: E402
from dual_arm_learning.policy_config import (  # noqa: E402
    PolicyConfigError,
    PolicyServiceConfig,
    load_policy_config,
)
from dual_arm_learning.scenario import (  # noqa: E402
    ScenarioSelectionError,
    select_scenario,
)

RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
DEFAULT_TEMPLATE = Path("shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml")


class PrepareError(ValueError):
    """The bundle, template or requested output is not deployable."""


def _require_container() -> None:
    marker = os.environ.get("PROJECT_CONTAINER")
    if marker != "0" and (marker == "1" or Path("/.dockerenv").exists()):
        return
    print(
        "error: prepare_policy must run in the learning environment; "
        "use learning/lerobot/scripts/run.sh prepare-policy ...",
        file=sys.stderr,
    )
    raise SystemExit(2)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _endpoint_port(value: object, where: str = "endpoint") -> int:
    if not isinstance(value, str) or not value:
        raise PrepareError(f"{where} must be a non-empty host:port string")
    host, separator, port_text = value.rpartition(":")
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


def _load(
    template: Path,
    *,
    bundle: Path,
    profile: Path | None,
    observation_schema: Path | None,
    action_schema: Path | None,
    device: str | None,
    require_formal: bool | None,
) -> PolicyServiceConfig:
    return load_policy_config(
        template,
        bundle_dir=Path(bundle).resolve(),
        robot_profile_path=None if profile is None else Path(profile).resolve(),
        observation_schema_path=(
            None if observation_schema is None else Path(observation_schema).resolve()
        ),
        action_schema_path=None if action_schema is None else Path(action_schema).resolve(),
        device=device,
        require_formal=require_formal,
    )


def prepare_policy(
    *,
    template: Path | None = None,
    bundle: Path | None = None,
    scenario: str | None = None,
    shared_root: Path | None = None,
    run_id: str,
    output: Path,
    endpoint: str = "learning:50051",
    profile: Path | None = None,
    observation_schema: Path | None = None,
    action_schema: Path | None = None,
    device: str | None = None,
    require_formal: bool | None = None,
    project_root: Path = ROOT,
) -> dict[str, Any]:
    """Write policy_binding.json, policy.yaml and run_manifest.json.

    A scenario selects ``recipes.policy_service`` and the profile identity from
    the binding; ``--bundle`` overrides its model source.  The bundle is always
    verified for real before any output is created, so a binding alone or an
    empty directory is never a deployable model.
    """
    if RUN_ID.fullmatch(run_id) is None or ".." in run_id:
        raise PrepareError("run_id must be a safe 1..128 character path segment")
    if template is not None and scenario is not None:
        raise PrepareError("--template and --scenario are mutually exclusive")
    selection = None
    bundle_source = "argument"
    if scenario is not None:
        if any(
            value is not None
            for value in (profile, observation_schema, action_schema)
        ):
            raise PrepareError(
                "--profile/--observation-schema/--action-schema cannot be combined "
                "with --scenario; the binding supplies the profile identity"
            )
        try:
            selection = select_scenario(
                scenario=scenario,
                recipe_field="policy_service",
                stage="inference",
                project_root=project_root,
                shared_root=shared_root,
                bundle=bundle,
            )
        except ScenarioSelectionError as error:
            raise PrepareError(str(error)) from error
        template = selection.recipe_path
        bundle = selection.resolved.bundle.path
        bundle_source = selection.resolved.bundle.source
        profile = selection.profile_path("robot")
        observation_schema = selection.profile_path("observation")
        action_schema = selection.profile_path("action")
    elif template is None:
        template = Path(project_root) / DEFAULT_TEMPLATE
    if bundle is None:
        raise PrepareError("--bundle is required unless --scenario supplies a model source")
    template_path = Path(template).resolve()
    template_sha256 = (
        selection.recipe_reference(selection.recipe_field).sha256
        if selection is not None
        else _sha256(template_path)
    )
    effective_bundle = Path(bundle).resolve()
    config = _load(
        template_path,
        bundle=effective_bundle,
        profile=profile,
        observation_schema=observation_schema,
        action_schema=action_schema,
        device=device,
        require_formal=require_formal,
    )
    endpoint_port = _endpoint_port(endpoint)
    if config.port != endpoint_port:
        raise PrepareError(
            f"ACT template service.port {config.port} must match endpoint port {endpoint_port}"
        )
    source = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    if not isinstance(source, dict) or not isinstance(source.get("model"), dict):
        raise PrepareError("ACT policy template has no model object")
    model = source["model"]
    model["bundle_dir"] = _container_path(config.bundle_dir, project_root)
    model["robot_profile_path"] = _container_path(config.robot_profile_path, project_root)
    model["observation_schema_path"] = _container_path(
        config.observation_schema_path, project_root
    )
    model["action_schema_path"] = _container_path(config.action_schema_path, project_root)
    model["device"] = config.device
    model["require_formal"] = config.require_formal
    if "service" not in source or not isinstance(source["service"], dict):
        raise PrepareError("ACT policy template has no service object")
    source["service"]["port"] = config.port
    if "FROM_VERIFIED" in yaml.safe_dump(source):
        raise PrepareError("ACT service template still contains an unverified placeholder")

    destination = Path(output).resolve()
    if destination.exists():
        raise PrepareError(f"output already exists; refusing to overwrite: {destination}")
    destination.mkdir(parents=True)
    policy_path = destination / "policy.yaml"
    policy_path.write_text(yaml.safe_dump(source, sort_keys=False), encoding="utf-8")
    policy_sha256 = _sha256(policy_path)
    summary = deployment_summary(config)
    binding = {
        "schema_version": 1,
        "kind": "policy-binding",
        "run_id": run_id,
        "endpoint": endpoint,
        "policy_api": "grpc",
        "model": {
            "bundle": _container_path(config.bundle_dir, project_root),
            "bundle_digest": config.model_bundle_digest,
            "model_sha256": summary.get("model_sha256", ""),
            "state_dict_sha256": summary.get("state_dict_sha256", ""),
            "config_sha256": summary.get("config_sha256", ""),
            "processor_digest": config.processor_digest,
            "checkpoint_step": summary.get("checkpoint_step", 0),
            "checkpoint_mode": summary.get("checkpoint_mode", ""),
            "dataset_seed": summary.get("dataset_seed", 0),
            "train_episodes": summary.get("train_episodes", 0),
            "validation_episodes": summary.get("validation_episodes", 0),
            "synthetic": summary.get("synthetic", False),
            "lerobot_revision": summary.get("lerobot_revision", ""),
            "lerobot_archive_sha256": summary.get("lerobot_archive_sha256", ""),
            "training_config_sha256": summary.get("training_config_sha256", ""),
        },
        "profile": {
            "path": _container_path(config.robot_profile_path, project_root),
            "sha256": _sha256(config.robot_profile_path),
            "semantic_sha256": config.robot_profile_hash,
        },
        "observation_schema": {
            "path": _container_path(config.observation_schema_path, project_root),
            "sha256": _sha256(config.observation_schema_path),
            "semantic_sha256": config.observation_schema_hash,
        },
        "action_schema": {
            "path": _container_path(config.action_schema_path, project_root),
            "sha256": _sha256(config.action_schema_path),
            "semantic_sha256": config.action_schema_hash,
        },
        "service": {
            "port": config.port,
            "task_id": config.task_id,
            "task_text": config.task_text,
            "predict_deadline_s": config.predict_deadline_s,
            "open_deadline_s": config.open_deadline_s,
            "close_deadline_s": config.close_deadline_s,
            "max_message_bytes": config.max_message_bytes,
            "chunk_size": config.chunk_size,
            "state_dim": len(config.joint_names),
            "action_period_ns": config.action_period_ns,
            "action_lead_ns": summary.get("action_lead_ns", 50_000_000),
            "cameras": list(config.camera_names),
            "camera_width": config.camera_width,
            "camera_height": config.camera_height,
        },
        "contract": summary,
    }
    binding_path = destination / "policy_binding.json"
    binding_path.write_text(
        json.dumps(binding, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    contract_path = destination / "act_contract.json"
    contract_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "contract": summary,
                "template": {"path": str(template_path), "sha256": template_sha256},
                "generated": {"path": str(policy_path), "sha256": policy_sha256},
            },
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    manifest = {
        "schema_version": 1,
        "kind": "policy_prepare",
        "run_id": run_id,
        "component": "learning",
        "endpoint": endpoint,
        "inputs": {
            "template": str(template_path),
            "template_sha256": template_sha256,
            "bundle": str(config.bundle_dir),
            "bundle_digest": config.model_bundle_digest,
        },
        "outputs": {
            "policy": str(policy_path),
            "policy_sha256": policy_sha256,
            "policy_binding": str(binding_path),
            "policy_binding_sha256": _sha256(binding_path),
            "act_contract": str(contract_path),
            "act_contract_sha256": _sha256(contract_path),
        },
        "model_source": {
            "source": bundle_source,
            "declared": (
                None if selection is None else selection.resolved.bundle.declared
            ),
            "path": str(config.bundle_dir),
            "bundle_digest": config.model_bundle_digest,
        },
    }
    if selection is not None:
        manifest["selection"] = selection.summary(include_bundle=True)
        overrides: dict[str, object] = {}
        if bundle_source == "override":
            overrides["bundle"] = {
                "source": "argument",
                "path": str(config.bundle_dir),
            }
        if device is not None:
            overrides["device"] = {"value": device, "source": "argument"}
        if require_formal is not None:
            overrides["require_formal"] = {
                "value": require_formal,
                "source": "argument",
            }
        if overrides:
            manifest["overrides"] = overrides
    (destination / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return manifest


def main(argv: list[str] | None = None) -> int:
    _require_container()
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--template", type=Path, default=None, help="direct ACT policy recipe path"
    )
    selection.add_argument(
        "--scenario", help="scenario id under <shared-root>/scenarios/<id>/binding.yaml"
    )
    parser.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    parser.add_argument(
        "--bundle",
        type=Path,
        help="verified model bundle; required unless the scenario binds one",
    )
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--observation-schema", type=Path)
    parser.add_argument("--action-schema", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda", "cuda:0"))
    parser.add_argument("--require-formal", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--endpoint", default=None)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    arguments = parser.parse_args(argv)
    if arguments.shared_root is not None and arguments.scenario is None:
        parser.error("--shared-root requires --scenario")
    if arguments.scenario is None and arguments.bundle is None:
        parser.error("--bundle is required unless --scenario binds a model source")
    try:
        manifest = prepare_policy(
            template=arguments.template,
            bundle=arguments.bundle,
            scenario=arguments.scenario,
            shared_root=arguments.shared_root,
            run_id=arguments.run_id,
            output=arguments.output,
            endpoint=(
                "learning:50051" if arguments.endpoint is None else arguments.endpoint
            ),
            profile=arguments.profile,
            observation_schema=arguments.observation_schema,
            action_schema=arguments.action_schema,
            device=arguments.device,
            require_formal=arguments.require_formal,
            project_root=arguments.project_root,
        )
    except (PrepareError, PolicyConfigError, ScenarioSelectionError, OSError, ValueError) as error:
        print(f"prepare_policy: {error}", file=sys.stderr)
        return 1
    print(json.dumps(manifest, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
