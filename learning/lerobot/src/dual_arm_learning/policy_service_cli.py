"""ROS-free entry for the named ACT Policy service.

The process loads and verifies one bundle, derives every wire identity from the
real files, warms the ACT adapter before the gRPC server is ready, then serves
the existing PolicyService/PolicyServer contract.  It never imports ROS, never
reads task truth and never falls back to a fake or constant provider.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
import uuid
from pathlib import Path

from .adapters.lerobot_act import load_act_adapter
from .bundle import BundleError, deployment_summary
from .policy_config import PolicyConfigError, PolicyServiceConfig, load_policy_config
from .policy_server import serve_forever
from .policy_types import PolicyError
from .scenario import LearningScenario, ScenarioSelectionError, select_scenario


def _project_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_startup_failure(
    directory: Path,
    *,
    stage: str,
    error: BaseException,
    config_path: Path,
    config: PolicyServiceConfig | None,
    bundle_dir: Path | None = None,
) -> None:
    """Leave a bounded, structured failure artefact even before the logger exists."""
    try:
        directory.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "status": "failed",
            "stage": stage,
            "error_type": type(error).__name__,
            "error": str(error),
            "wall_time_ns": time.time_ns(),
            "config_path": str(config_path),
            "bundle": (
                str(bundle_dir)
                if bundle_dir is not None
                else ("" if config is None else str(config.bundle_dir))
            ),
            "profile": "" if config is None else str(config.robot_profile_path),
            "observation_schema": (
                "" if config is None else str(config.observation_schema_path)
            ),
            "action_schema": "" if config is None else str(config.action_schema_path),
            "device": "" if config is None else config.device,
            "contract": None if config is None else deployment_summary(config),
        }
        (directory / "startup_failure.json").write_text(
            json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
    except Exception:  # noqa: BLE001 - preserve the original startup failure
        pass


def _scalar_fields(value: dict[str, object]) -> dict[str, object]:
    return {
        key: item
        for key, item in value.items()
        if isinstance(item, bool | int | float | str) or item is None
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument(
        "--config", type=Path, default=None, help="direct ACT policy recipe path"
    )
    selection.add_argument(
        "--scenario", help="scenario id under <shared-root>/scenarios/<id>/binding.yaml"
    )
    parser.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    parser.add_argument(
        "--bundle",
        "--bundle-dir",
        dest="bundle_dir",
        type=Path,
        help="verified model bundle; overrides models.bundle in the binding",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="override one existing policy/service recipe path (repeatable)",
    )
    parser.add_argument("--robot-profile", type=Path)
    parser.add_argument("--observation-schema", type=Path)
    parser.add_argument("--action-schema", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda", "cuda:0"))
    parser.add_argument("--log-directory", type=Path, default=None)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--snapshot-name", default="resolved_contract.json")
    parser.add_argument("--debug", action="store_true")
    return parser


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    return _parser().parse_args(argv)


def _snapshot(
    config: PolicyServiceConfig,
    adapter: object,
    summary: dict,
    *,
    config_path: Path,
    selection: LearningScenario | None = None,
    overrides: dict[str, object] | None = None,
) -> dict:
    record = {
        "schema_version": 1,
        "bundle": summary,
        "service": config.to_document(),
        "provider": adapter.summary(),
        "task_id": config.task_id,
        "task_text": config.task_text,
        "config": {"path": str(config_path.resolve()), "sha256": _sha256(config_path)},
        "source": {
            "bundle_root": str(config.bundle_dir),
            "profile": str(config.robot_profile_path),
            "observation_schema": str(config.observation_schema_path),
            "action_schema": str(config.action_schema_path),
            "device": config.device,
        },
    }
    if selection is not None:
        record["selection"] = selection.summary(include_bundle=True)
        record["model_source"] = {
            "source": selection.resolved.bundle.source,
            "declared": selection.resolved.bundle.declared,
            "path": str(config.bundle_dir),
            "bundle_digest": config.model_bundle_digest,
        }
    if overrides:
        record["overrides"] = overrides
    return record


def _serve(
    config: PolicyServiceConfig,
    adapter: object,
    startup: dict,
    snapshot: dict,
    args: object,
    config_path: Path,
) -> int:
    run_id = args.run_id or "act-policy-" + uuid.uuid4().hex[:12]
    code = serve_forever(
        config,
        adapter,
        log_directory=args.log_directory,
        prediction_directory=args.log_directory / "predictions",
        prediction_max_bytes=config.logging.max_total_bytes,
        run_id=run_id,
        startup_fields=startup,
        snapshot=snapshot,
        snapshot_name=args.snapshot_name,
    )
    if code != 0:
        _write_startup_failure(
            args.log_directory,
            stage="serve",
            error=RuntimeError(f"policy service exited {code}"),
            config_path=config_path,
            config=config,
        )
    return code


def _fail(
    args: object,
    stage: str,
    error: BaseException,
    config_path: Path,
    config: PolicyServiceConfig | None,
) -> int:
    _write_startup_failure(
        args.log_directory,
        stage=stage,
        error=error,
        config_path=config_path,
        config=config,
        bundle_dir=args.bundle_dir,
    )
    if args.debug:
        raise error
    print(f"act_policy: {type(error).__name__}: {error}", file=sys.stderr)
    return 1


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    project_root = _project_root()
    if args.log_directory is None:
        segment = args.scenario or "_direct"
        args.log_directory = (
            Path(os.environ.get("DATA_ROOT", "/data")) / segment / "logs" / "policy-service"
        )
    if args.scenario is None and args.shared_root is not None:
        parser.error("--shared-root requires --scenario")
    if args.scenario is not None and any(
        value is not None
        for value in (args.robot_profile, args.observation_schema, args.action_schema)
    ):
        parser.error(
            "--robot-profile/--observation-schema/--action-schema cannot be combined "
            "with --scenario; the binding supplies the profile identity"
        )
    config_path = (
        Path(f"scenario:{args.scenario}")
        if args.scenario is not None
        else (args.config if args.config.is_absolute() else project_root / args.config)
    )
    selection: LearningScenario | None = None
    overrides: dict[str, object] = {}
    config: PolicyServiceConfig | None = None
    stage = "config"
    try:
        if args.scenario is not None:
            selection = select_scenario(
                scenario=args.scenario,
                recipe_field="policy_service",
                stage="inference",
                project_root=project_root,
                shared_root=args.shared_root,
                bundle=args.bundle_dir,
            )
            config_path = selection.recipe_path
            config = load_policy_config(
                config_path,
                bundle_dir=selection.resolved.bundle.path,
                robot_profile_path=selection.profile_path("robot"),
                observation_schema_path=selection.profile_path("observation"),
                action_schema_path=selection.profile_path("action"),
                device=args.device,
                set_overrides=args.overrides,
            )
            if args.bundle_dir is not None:
                overrides["bundle_dir"] = {
                    "value": str(args.bundle_dir),
                    "source": "argument",
                }
            if args.device is not None:
                overrides["device"] = {"value": args.device, "source": "argument"}
            if args.overrides:
                overrides["set"] = {"value": list(args.overrides), "source": "argument"}
        else:
            config = load_policy_config(
                config_path,
                bundle_dir=args.bundle_dir,
                robot_profile_path=args.robot_profile,
                observation_schema_path=args.observation_schema,
                action_schema_path=args.action_schema,
                device=args.device,
                set_overrides=args.overrides,
            )
            if args.overrides:
                overrides["set"] = {"value": list(args.overrides), "source": "argument"}
        stage = "load_warm"
        print("=== Policy 服务配置与详细参数 ===", flush=True)
        if selection is not None:
            print(f"方案 (scenario): {selection.binding.scenario_id}", flush=True)
            print(f"Binding 文件:    {selection.path}", flush=True)
        print(f"配置文件路径:    {config_path.resolve()}", flush=True)
        print("--- 详细参数 ---", flush=True)
        print(json.dumps(config.to_document(), indent=2, ensure_ascii=False), flush=True)
        print("================================", flush=True)
        adapter = load_act_adapter(config)
        summary = deployment_summary(config)
        startup = _scalar_fields(
            {
                **summary,
                **adapter.summary(),
                "task_id": config.task_id,
                "task_text": config.task_text,
                "test_only": config.test_only,
            }
        )
        stage = "serve"
        snapshot = _snapshot(
            config,
            adapter,
            summary,
            config_path=config_path,
            selection=selection,
            overrides=overrides or None,
        )
        return _serve(config, adapter, startup, snapshot, args, config_path)
    except (
        PolicyConfigError,
        PolicyError,
        BundleError,
        ScenarioSelectionError,
        OSError,
        ValueError,
    ) as error:
        return _fail(args, stage, error, config_path, config)
    except Exception as error:  # noqa: BLE001 - startup must fail visibly
        return _fail(args, stage, error, config_path, config)


__all__ = ["main", "parse_arguments"]

if __name__ == "__main__":
    raise SystemExit(main())
