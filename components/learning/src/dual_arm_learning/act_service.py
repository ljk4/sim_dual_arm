"""ROS-free entry for the named ACT Policy service.

The process loads and verifies a 035 bundle, warms the ACT policy before the gRPC
server is ready, then serves the existing PolicyService/PolicyServer contract.
It never imports ROS, never reads task truth and never falls back to a fake or
constant provider.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
import uuid
from pathlib import Path

from robot_run_logging import LogSettings

from .act_contract import (
    ActContract,
    ActContractError,
    ActSettings,
    load_act_settings,
    resolve_act_contract,
)
from .act_provider import load_act_provider
from .bundle import BundleError
from .policy_config import PolicyServiceConfig
from .policy_server import serve_forever
from .policy_types import PolicyError


def _project_root() -> Path:
    return Path(__file__).resolve().parents[4]


def build_policy_config(settings: ActSettings, contract: ActContract) -> PolicyServiceConfig:
    """Derive every wire identity from the verified bundle and shipped files."""
    try:
        logging = LogSettings(**settings.logging)
    except TypeError as error:
        raise ActContractError(f"logging: {error}") from error
    return PolicyServiceConfig(
        schema_version=1,
        bind_address=settings.bind_address,
        port=settings.port,
        max_workers=settings.max_workers,
        max_message_bytes=settings.max_message_bytes,
        max_sessions=settings.max_sessions,
        max_open_requests=settings.max_open_requests,
        max_request_cache=settings.max_request_cache,
        predict_deadline_s=settings.predict_deadline_s,
        open_deadline_s=settings.open_deadline_s,
        close_deadline_s=settings.close_deadline_s,
        test_only=False,
        protocol_major=settings.protocol_major,
        stateful=True,
        model_bundle_digest=contract.bundle_digest,
        processor_digest=contract.processor_digest,
        observation_schema_hash=contract.observation_schema_hash,
        action_schema_hash=contract.action_schema_hash,
        robot_profile_hash=contract.profile_semantic_sha256,
        joint_names=contract.joint_names,
        joint_lower_bounds=contract.joint_lower_bounds,
        joint_upper_bounds=contract.joint_upper_bounds,
        action_joint_names=contract.joint_names,
        camera_names=contract.camera_names,
        camera_width=contract.camera_width,
        camera_height=contract.camera_height,
        action_period_ns=contract.action_period_ns,
        chunk_size=contract.chunk_size,
        task_id=settings.task_id,
        task_text=settings.task_text,
        command_space="joint_position",
        logging=logging,
    )


def _write_startup_failure(
    directory: Path,
    *,
    stage: str,
    error: BaseException,
    config_path: Path,
    settings: ActSettings | None,
    contract: ActContract | None,
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
            "bundle": "" if settings is None else str(settings.bundle),
            "profile": "" if settings is None else str(settings.profile),
            "observation_schema": "" if settings is None else str(settings.observation_schema),
            "action_schema": "" if settings is None else str(settings.action_schema),
            "device": "" if settings is None else settings.device,
            "contract": None if contract is None else contract.summary(),
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


def _with_overrides(settings: ActSettings, args: argparse.Namespace) -> ActSettings:
    changes: dict[str, object] = {}
    for name in ("bundle", "profile", "observation_schema", "action_schema", "device"):
        value = getattr(args, name)
        if value is not None:
            changes[name] = Path(value).resolve() if name != "device" else value
    return dataclasses.replace(settings, **changes) if changes else settings


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/learning/act_policy.yaml"))
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--observation-schema", type=Path)
    parser.add_argument("--action-schema", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda", "cuda:0"))
    parser.add_argument("--log-directory", type=Path, default=Path("var/logs/act-policy"))
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--snapshot-name", default="resolved_contract.json")
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    config_path = args.config if args.config.is_absolute() else _project_root() / args.config
    settings: ActSettings | None = None
    contract: ActContract | None = None
    stage = "config"
    try:
        settings = _with_overrides(load_act_settings(config_path), args)
        stage = "contract"
        contract = resolve_act_contract(
            bundle=settings.bundle,
            profile_path=settings.profile,
            observation_schema_path=settings.observation_schema,
            action_schema_path=settings.action_schema,
            require_formal=settings.require_formal,
        )
        policy_config = build_policy_config(settings, contract)
        stage = "load_warm"
        provider = load_act_provider(
            config=policy_config,
            contract=contract,
            device=settings.device,
            gripper_max_correction_m=settings.gripper_max_correction_m,
            prediction_directory=args.log_directory / "predictions",
            prediction_max_bytes=int(settings.logging.get("max_total_bytes", 268435456)),
        )
        startup = _scalar_fields(
            {
                **contract.summary(),
                **provider.summary(),
                "task_id": settings.task_id,
                "task_text": settings.task_text,
                "test_only": False,
            }
        )
        snapshot = {
            "schema_version": 1,
            "bundle": contract.summary(),
            "service": policy_config.to_document(),
            "provider": provider.summary(),
            "task_id": settings.task_id,
            "task_text": settings.task_text,
            "source": {
                "bundle_root": str(contract.bundle_root),
                "profile": str(settings.profile),
                "observation_schema": str(settings.observation_schema),
                "action_schema": str(settings.action_schema),
                "device": settings.device,
            },
        }
        run_id = args.run_id or "act-policy-" + uuid.uuid4().hex[:12]
        stage = "serve"
        code = serve_forever(
            policy_config,
            provider,
            log_directory=args.log_directory,
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
                settings=settings,
                contract=contract,
            )
        return code
    except (ActContractError, PolicyError, BundleError, OSError, ValueError) as error:
        _write_startup_failure(
            args.log_directory,
            stage=stage,
            error=error,
            config_path=config_path,
            settings=settings,
            contract=contract,
        )
        if args.debug:
            raise
        print(f"act_policy: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    except Exception as error:  # noqa: BLE001 - startup must fail visibly
        _write_startup_failure(
            args.log_directory,
            stage=stage,
            error=error,
            config_path=config_path,
            settings=settings,
            contract=contract,
        )
        if args.debug:
            raise
        print(f"act_policy: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


__all__ = ["build_policy_config", "main", "parse_arguments"]


if __name__ == "__main__":
    raise SystemExit(main())
