"""ROS-free entry for the named ACT Policy service."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from robot_contracts.io import sha256_file as _sha256
from robot_contracts.project_paths import project_path

from ..adapters.lerobot_act import load_act_adapter
from ..scenario import (
    LearningScenario,
    ScenarioSelectionError,
    artifact_paths,
    latest_model,
    select_scenario,
)
from ..training.bundle import ModelError, deployment_summary
from .config import PolicyConfigError, PolicyServiceConfig, load_policy_config
from .server import serve_forever
from .types import PolicyError


def _project_root() -> Path:
    root = os.environ.get("PROJECT_ROOT")
    return Path(root) if root else Path(__file__).resolve().parents[5]


def _write_startup_failure(
    directory: Path,
    *,
    stage: str,
    error: BaseException,
    config_path: Path,
    config: PolicyServiceConfig | None,
    model_path: Path | None = None,
) -> None:
    """Leave a bounded, structured failure artefact even before the logger exists."""
    try:
        directory.mkdir(parents=True, exist_ok=True)
        payload = {
            "status": "failed",
            "stage": stage,
            "error_type": type(error).__name__,
            "error": str(error),
            "wall_time_ns": time.time_ns(),
            "config_path": str(config_path),
            "model": (
                str(model_path)
                if model_path is not None
                else ("" if config is None else str(config.model_path))
            ),
            "profile": "" if config is None else str(config.robot_profile_path),
            "observation_schema": ("" if config is None else str(config.observation_schema_path)),
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
    parser.add_argument("--config", type=Path, default=None, help="direct ACT policy recipe path")
    parser.add_argument(
        "--robot", help="robot id under <shared-root>/scenarios/<robot>/<task>/"
    )
    parser.add_argument("--task", help="task id for --robot")
    parser.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    parser.add_argument(
        "--model",
        dest="model_path",
        type=Path,
        help="model directory or model.safetensors; defaults to the newest act_* model",
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
        "model": summary,
        "service": config.to_document(),
        "provider": adapter.summary(),
        "task_id": config.task_id,
        "task_text": config.task_text,
        "config": {"path": str(config_path.resolve()), "sha256": _sha256(config_path)},
        "source": {
            "model": str(config.model_path),
            "profile": str(config.robot_profile_path),
            "observation_schema": str(config.observation_schema_path),
            "action_schema": str(config.action_schema_path),
            "device": config.device,
        },
    }
    if selection is not None:
        record["selection"] = selection.summary()
        record["model_source"] = {
            "source": selection.resolved.bundle.source,
            "declared": selection.resolved.bundle.declared,
            "path": str(config.model_path),
            "model_digest": config.model_bundle_digest,
        }
    if overrides:
        record["overrides"] = overrides
    return record


def _default_run_id() -> str:
    return f"policy_{time.strftime('%Y%m%d_%H%M%S')}"


def _serve(
    config: PolicyServiceConfig,
    adapter: object,
    startup: dict,
    snapshot: dict,
    args: object,
    config_path: Path,
) -> int:
    run_id = args.run_id or _default_run_id()
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
        model_path=args.model_path,
    )
    if args.debug:
        raise error
    print(f"act_policy: {type(error).__name__}: {error}", file=sys.stderr)
    return 1


def _target(args: argparse.Namespace, parser: argparse.ArgumentParser) -> tuple[str, str] | None:
    if args.robot is None and args.task is None:
        if args.config is None:
            parser.error("either --robot/--task or --config is required")
        return None
    if args.robot is None or args.task is None:
        parser.error("--robot and --task must be passed together")
    if args.config is not None:
        parser.error("--config cannot be combined with --robot/--task")
    return args.robot, args.task


def _default_log_directory(robot: str | None, task: str | None, run_id: str) -> Path:
    if robot is None or task is None:
        segment = "_direct"
    else:
        segment = f"{robot}/{task}"
    return Path(os.environ.get("DATA_ROOT", "/data")) / segment / "logs" / "policy-service" / run_id


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    project_root = _project_root()
    target = _target(args, parser)
    if args.shared_root is not None and target is None:
        parser.error("--shared-root requires --robot/--task")
    if not args.run_id:
        args.run_id = _default_run_id()
    if args.log_directory is None:
        args.log_directory = _default_log_directory(*(target or (None, None)), args.run_id)
    if target is not None and any(
        value is not None
        for value in (args.robot_profile, args.observation_schema, args.action_schema)
    ):
        parser.error(
            "--robot-profile/--observation-schema/--action-schema cannot be combined "
            "with --robot/--task; the binding supplies the profile identity"
        )
    config_path = (
        Path(f"scenario:{target[0]}/{target[1]}")
        if target is not None
        else project_path(args.config, project_root)
    )
    selection: LearningScenario | None = None
    overrides: dict[str, object] = {}
    config: PolicyServiceConfig | None = None
    stage = "config"
    try:
        if target is not None:
            robot, task = target
            model_path = args.model_path or latest_model(robot, task, project_root)
            if model_path is None:
                raise ModelError(
                    f"{artifact_paths(robot, task, project_root)['models']}: no act_* model found"
                )
            model_path = project_path(model_path, project_root)
            model_root = model_path.parent if model_path.is_file() else model_path
            selection = select_scenario(
                robot=robot,
                task=task,
                recipe_field="policy_service",
                stage="inference",
                project_root=project_root,
                shared_root=args.shared_root,
                model=model_root,
            )
            config_path = selection.recipe_path
            config = load_policy_config(
                config_path,
                model_path=model_path,
                robot_profile_path=selection.profile_path("robot"),
                observation_schema_path=selection.profile_path("observation"),
                action_schema_path=selection.profile_path("action"),
                device=args.device,
            )
            if args.model_path is not None:
                overrides["model_path"] = {"value": str(model_path), "source": "argument"}
            if args.device is not None:
                overrides["device"] = {"value": args.device, "source": "argument"}
        else:
            config = load_policy_config(
                config_path,
                model_path=(
                    None if args.model_path is None else project_path(args.model_path, project_root)
                ),
                robot_profile_path=(
                    None
                    if args.robot_profile is None
                    else project_path(args.robot_profile, project_root)
                ),
                observation_schema_path=(
                    None
                    if args.observation_schema is None
                    else project_path(args.observation_schema, project_root)
                ),
                action_schema_path=(
                    None
                    if args.action_schema is None
                    else project_path(args.action_schema, project_root)
                ),
                device=args.device,
            )
        if config.model_path is None and not config.test_only:
            raise ModelError("--model is required for a non-test policy service")
        stage = "load_warm"
        if selection is not None:
            print(
                f"机器人/任务: {selection.robot}/{selection.task} | "
                f"Binding: {selection.resolved.path}",
                flush=True,
            )
        print(f"模型: {config.model_path}", flush=True)
        print(
            f"服务: {config.bind_address}:{config.port} | "
            f"设备: {config.device} | 日志: {args.log_directory}",
            flush=True,
        )
        print(f"详细参数: {args.log_directory / args.snapshot_name}", flush=True)
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
        ModelError,
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
