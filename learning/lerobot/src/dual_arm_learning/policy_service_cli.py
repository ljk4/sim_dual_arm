"""ROS-free entry for the named ACT Policy service.

The process loads and verifies one bundle, derives every wire identity from the
real files, warms the ACT adapter before the gRPC server is ready, then serves
the existing PolicyService/PolicyServer contract.  It never imports ROS, never
reads task truth and never falls back to a fake or constant provider.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from pathlib import Path

from .adapters.lerobot_act import load_act_adapter
from .bundle import BundleError, deployment_summary
from .policy_config import PolicyConfigError, PolicyServiceConfig, load_policy_config
from .policy_server import serve_forever
from .policy_types import PolicyError


def _project_root() -> Path:
    return Path(__file__).resolve().parents[4]


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


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("learning/lerobot/configs/policy_temporal.yaml"),
    )
    parser.add_argument("--bundle-dir", type=Path)
    parser.add_argument("--robot-profile", type=Path)
    parser.add_argument("--observation-schema", type=Path)
    parser.add_argument("--action-schema", type=Path)
    parser.add_argument("--device", choices=("cpu", "cuda", "cuda:0"))
    parser.add_argument("--log-directory", type=Path, default=Path("var/logs/act-policy"))
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--snapshot-name", default="resolved_contract.json")
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args(argv)


def _snapshot(config: PolicyServiceConfig, adapter: object, summary: dict) -> dict:
    return {
        "schema_version": 1,
        "bundle": summary,
        "service": config.to_document(),
        "provider": adapter.summary(),
        "task_id": config.task_id,
        "task_text": config.task_text,
        "source": {
            "bundle_root": str(config.bundle_dir),
            "profile": str(config.robot_profile_path),
            "observation_schema": str(config.observation_schema_path),
            "action_schema": str(config.action_schema_path),
            "device": config.device,
        },
    }


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
    args = parse_arguments(argv)
    config_path = args.config if args.config.is_absolute() else _project_root() / args.config
    config: PolicyServiceConfig | None = None
    stage = "config"
    try:
        config = load_policy_config(
            config_path,
            bundle_dir=args.bundle_dir,
            robot_profile_path=args.robot_profile,
            observation_schema_path=args.observation_schema,
            action_schema_path=args.action_schema,
            device=args.device,
        )
        stage = "load_warm"
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
        snapshot = _snapshot(config, adapter, summary)
        return _serve(config, adapter, startup, snapshot, args, config_path)
    except (PolicyConfigError, PolicyError, BundleError, OSError, ValueError) as error:
        return _fail(args, stage, error, config_path, config)
    except Exception as error:  # noqa: BLE001 - startup must fail visibly
        return _fail(args, stage, error, config_path, config)


__all__ = ["main", "parse_arguments"]

if __name__ == "__main__":
    raise SystemExit(main())
