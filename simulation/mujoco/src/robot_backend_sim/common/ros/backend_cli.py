"""Process entry point for the runnable backend.

Run directory creation, signals, run metadata and the SingleThreadedExecutor
loop live here so ``node.py`` stays focused on ROS conversion/lifecycle.
"""

from __future__ import annotations

import argparse
import os
import re
import secrets
import signal
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from robot_contracts.io import write_json_atomic
from robot_contracts.project_paths import container_data_root, logs_root, project_path
from robot_contracts.validation import UniqueKeyError, read_unique_yaml
from robot_run_logging import EventLogger, LogFailure, LogSettings

from robot_backend_sim.common.config.backend_config import (
    BackendConfigError,
    load_backend_document,
)
from robot_backend_sim.common.config.scenario_config import (
    ROBOT_REFERENCE_FIELDS,
    ScenarioError,
    resolve_scenario_backend,
)
from robot_backend_sim.common.config.scene_loader import load_scene_config_for
from robot_backend_sim.common.physics import assets
from robot_backend_sim.common.ros.node import BackendNode
from robot_backend_sim.common.runtime.worker import WorkerSummary

_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_EXIT_OK = 0
_EXIT_FAILURE = 1


class _Signals:
    def __init__(self) -> None:
        self._signum = 0

    def __call__(self, signum: int, _frame: object) -> None:
        if not self._signum:
            self._signum = signum

    @property
    def requested(self) -> bool:
        return bool(self._signum)

    @property
    def reason(self) -> str:
        return "" if not self._signum else signal.Signals(self._signum).name


def _validate_run_id(value: str) -> str:
    if not _RUN_ID_PATTERN.fullmatch(value) or value in {".", ".."}:
        raise ValueError("run id must contain 1..128 safe ASCII characters")
    return value


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _now_utc() -> str:
    from datetime import UTC, datetime

    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _random_seed() -> int:
    return secrets.randbelow(2**31)


def _safe_id_part(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip()).strip("._-")
    return cleaned or "unknown"


def _parser(argv: list[str] | None = None) -> argparse.ArgumentParser:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    inference_entry = "--inference-entry" in raw_argv
    parser = argparse.ArgumentParser(
        prog="scripts/inference_backend.sh" if inference_entry else None,
        description=__doc__,
    )
    parser.add_argument("--config", type=Path, help="direct backend recipe path")
    parser.add_argument(
        "--robot", help="robot id under <shared-root>/scenarios/<robot>/<backend>/<task>/"
    )
    parser.add_argument(
        "--backend", help="backend id under <shared-root>/scenarios/<robot>/<backend>/<task>/"
    )
    parser.add_argument(
        "--task", help="task id under <shared-root>/scenarios/<robot>/<backend>/<task>/"
    )
    if not inference_entry:
        parser.add_argument("--role", choices=("teacher", "policy"), default=None)
    parser.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="initial simulation seed (default: random)",
    )
    parser.add_argument(
        "--realtime-speed", type=float, help="simulation seconds per wall-clock second"
    )
    parser.add_argument("--namespace", default=None, help="override robot.namespace")
    parser.add_argument(
        "--no-visualization",
        action="store_true",
        help="disable MuJoCo state visualization topic",
    )
    parser.add_argument("--run-root", type=Path, default=None)
    if not inference_entry:
        parser.add_argument("--run-id", default=None)
    parser.add_argument("--inference-entry", action="store_true", help=argparse.SUPPRESS)
    return parser


def _parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parsed = _parser(argv).parse_args(argv)
    if getattr(parsed, "inference_entry", False):
        parsed.run_id = None
        parsed.role = None
    if parsed.seed is not None and not 0 <= parsed.seed <= 2**63 - 1:
        raise SystemExit("--seed must be within [0, 2^63-1]")
    return parsed


def _target(parsed: argparse.Namespace) -> tuple[str, str, str] | None:
    if any(value is not None for value in (parsed.robot, parsed.backend, parsed.task)):
        if not (parsed.robot and parsed.backend and parsed.task):
            raise SystemExit("--robot, --backend, and --task must be passed together")
        return parsed.robot, parsed.backend, parsed.task
    return None


def _default_run_root(robot: str | None, backend: str | None, task: str | None) -> Path:
    try:
        return logs_root(container_data_root(), robot, backend, task) / "backend"
    except ValueError as error:
        raise SystemExit(
            f"robot/backend/task id is not a safe artifact directory: {error}"
        ) from error


def _default_run_suffix() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _unique_run_id(run_root: Path, base: str) -> str:
    if not (run_root / base).exists():
        return base
    suffix = 2
    while (run_root / f"{base}_{suffix}").exists():
        suffix += 1
    return f"{base}_{suffix}"


def _inference_backend_run_id_base(
    target: tuple[str, str, str] | None,
    config: Any,
    parsed: argparse.Namespace,
    stamp: str | None = None,
) -> str:
    stamp = stamp or _default_run_suffix()
    if target is not None:
        robot_part = _safe_id_part(target[0])
        backend_part = _safe_id_part(target[1])
        task_part = _safe_id_part(target[2])
        head = f"inference_backend_{robot_part}_{backend_part}_{task_part}"[:80].rstrip("._-")
    else:
        scene_config = load_scene_config_for(config.scene_path)
        robot_part = _safe_id_part(scene_config.robot.assembly)
        task_env = getattr(scene_config.task, "environment", "") if hasattr(scene_config, "task") else ""
        task_part = _safe_id_part(str(task_env) or parsed.config.stem)
        head = f"inference_backend_{robot_part}_{task_part}"[:80].rstrip("._-")
    return f"{head}_{stamp}"


def _validate_selection_arguments(
    parsed: argparse.Namespace, target: tuple[str, str, str] | None
) -> None:
    if parsed.config is not None and target is not None:
        raise SystemExit("--config cannot be combined with --robot/--backend/--task")
    if target is not None:
        return
    if parsed.role is not None:
        raise SystemExit("--role requires --robot/--backend/--task")
    if parsed.shared_root is not None:
        raise SystemExit("--shared-root requires --robot/--backend/--task")
    if parsed.config is None:
        raise SystemExit("either --robot/--backend/--task or --config is required")


def _validate_arguments(
    parsed: argparse.Namespace, target: tuple[str, str, str] | None
) -> str:
    _validate_selection_arguments(parsed, target)
    if target is not None:
        parsed.role = parsed.role or "teacher"
        robot, backend, task = target
        return parsed.run_id or f"{robot}-{backend}-{task}-{parsed.role}_{_default_run_suffix()}"
    stem = re.sub(r"[^A-Za-z0-9._-]", "-", parsed.config.stem)
    return parsed.run_id or f"backend-{stem or 'run'}_{_default_run_suffix()}"


def _project_root() -> Path:
    root = os.environ.get("PROJECT_ROOT")
    return Path(root) if root else Path(__file__).resolve().parents[6]


def _resolve_config_path(config: Path) -> Path:
    if config.is_absolute() or config.is_file():
        return config.resolve()
    return project_path(config, _project_root()).resolve()


def _direct_document(parsed: argparse.Namespace, effective_seed: int) -> dict[str, Any]:
    path = _resolve_config_path(parsed.config)
    try:
        document = read_unique_yaml(path)
    except (UniqueKeyError, OSError) as error:
        raise SystemExit(f"cannot read backend config {path}: {error}") from error
    if not isinstance(document, dict):
        raise SystemExit(f"backend config must be one YAML object: {path}")
    robot = document.get("robot")
    if isinstance(robot, dict):
        for field in ROBOT_REFERENCE_FIELDS:
            value = robot.get(field)
            if isinstance(value, str):
                robot[field] = str((path.parent / value).resolve())
    run = document.get("run")
    if parsed.seed is not None:
        if not isinstance(run, dict) or "seed" not in run:
            raise SystemExit("--seed requires an existing run.seed field")
        run["seed"] = parsed.seed
    elif isinstance(run, dict) and "seed" in run:
        run["seed"] = effective_seed
    if parsed.realtime_speed is not None:
        if not isinstance(run, dict) or "realtime_speed" not in run:
            raise SystemExit("--realtime-speed requires an existing run.realtime_speed field")
        run["realtime_speed"] = parsed.realtime_speed
    return document


def _write_snapshot(run_dir: Path, document: dict[str, Any]) -> Path:
    path = run_dir / "backend.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _prepare_run(parsed: argparse.Namespace):
    target = _target(parsed)
    robot, backend, task = target if target is not None else (None, None, None)
    run_root = (parsed.run_root or _default_run_root(robot, backend, task)).resolve()
    inference_entry = bool(getattr(parsed, "inference_entry", False))
    if inference_entry:
        _validate_selection_arguments(parsed, target)
        if target is not None:
            parsed.role = "policy"
        raw_run_id = None
    else:
        raw_run_id = _validate_run_id(_validate_arguments(parsed, target))
    effective_seed = parsed.seed if parsed.seed is not None else _random_seed()
    selection: dict[str, Any]
    try:
        if target is not None:
            resolved = resolve_scenario_backend(
                robot=robot,
                backend=backend,
                task=task,
                role=parsed.role or "teacher",
                shared_root=parsed.shared_root,
                seed=effective_seed,
                realtime_speed=parsed.realtime_speed,
            )
            document, selection = resolved.document, resolved.selection
        else:
            document = _direct_document(parsed, effective_seed)
            selection = {"mode": "direct", "robot": None, "backend": None, "task": None, "role": None}
        if parsed.namespace is not None or parsed.no_visualization:
            robot_section = document.get("robot")
            if not isinstance(robot_section, dict):
                raise SystemExit("backend config must contain a robot mapping")
            if parsed.namespace is not None:
                robot_section["namespace"] = parsed.namespace
            if parsed.no_visualization:
                robot_section["visualization_enabled"] = False
        # Validate before creating the run directory so a bad recipe leaves no
        # orphan directory behind.
        config = load_backend_document(document, parsed.config or "scenario:selection")
    except (ScenarioError, BackendConfigError) as error:
        raise SystemExit(str(error)) from error
    if inference_entry:
        base_run_id = _validate_run_id(_inference_backend_run_id_base(target, config, parsed))
        run_id = _unique_run_id(run_root, base_run_id)
    else:
        assert raw_run_id is not None
        run_id = raw_run_id if parsed.run_id is not None else _unique_run_id(run_root, raw_run_id)
    run_dir = (run_root / run_id).resolve()
    if not _inside(run_dir, run_root) or run_dir == run_root:
        raise SystemExit("run directory escapes run root")
    try:
        run_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise SystemExit(f"run directory already exists: {run_dir}") from error
    snapshot = _write_snapshot(run_dir, document)
    config = config.model_copy(update={"source": snapshot})
    if target is not None:
        print(
            "=== 仿真后端启动 ===\n"
            f"方案: {robot}/{backend}/{task} | 角色: {parsed.role} | run_id: {run_id} | seed: {config.seed}\n"
            f"Binding: {selection['binding']['path']}\n"
            f"后端配方: {selection['recipe']['path']}\n"
            f"命名空间: {config.namespace} | 倍速: {config.realtime_speed}x | "
            f"可视化: {'开启' if config.visualization_enabled else '关闭'}\n"
            f"运行日志: {run_dir / 'log'}\n"
            "================================",
            flush=True,
        )
    else:
        print(
            "=== 仿真后端启动 (direct) ===\n"
            f"配置文件: {parsed.config} | run_id: {run_id} | seed: {config.seed}\n"
            f"配置快照: {snapshot}\n"
            f"命名空间: {config.namespace} | 倍速: {config.realtime_speed}x | "
            f"可视化: {'开启' if config.visualization_enabled else '关闭'}\n"
            f"运行日志: {run_dir / 'log'}\n"
            "================================",
            flush=True,
        )
    logger = EventLogger(
        run_dir / "log",
        run_id=run_id,
        component="backend",
        settings=LogSettings(
            queue_capacity=config.log.queue_capacity,
            max_event_bytes=config.log.max_event_bytes,
            max_file_bytes=config.log.max_file_bytes,
            max_total_bytes=config.log.max_total_bytes,
            close_timeout_s=config.log.close_timeout_s,
            flush_interval_s=config.log.flush_interval_s,
            console=config.log.console,
        ),
    )
    return config, run_id, run_dir, logger


def _robot_metadata(config) -> dict[str, str]:
    scene_config = load_scene_config_for(config.scene_path)
    assembly = scene_config.robot.assembly
    return {
        "robot_assembly": assembly,
        "robot_model": str(assets.robot_model_path(assembly)),
    }


def _emit_backend_start(logger, config, run_id: str) -> None:
    logger.emit(
        "backend.start",
        message="backend ready",
        fields={
            "namespace": config.namespace,
            "backend_kind": "mujoco",
            "capture_hz": config.capture_hz,
            "realtime_speed": config.realtime_speed,
            "velocity_feedforward": config.velocity_feedforward,
            "reliable_sensors": config.reliable_sensors,
            "profile": config.profile_path.name,
            "scene": config.scene_path.name,
            "control": config.control_path.name,
            "stream": config.stream_path.name,
            "follower": None if config.follower_path is None else config.follower_path.name,
            "run_id": run_id,
            "config": str(config.source),
            **_robot_metadata(config),
        },
    )


def _serve_node(
    config, logger, run_id: str, run_dir: Path, signals, executor
) -> tuple[object, str | None, int]:
    node: BackendNode | None = None
    summary: WorkerSummary | None = None
    failure: str | None = None
    exit_code = _EXIT_OK
    try:
        node = BackendNode(config, log=logger)
        executor.add_node(node)
        _emit_backend_start(logger, config, run_id)
        write_json_atomic(
            run_dir / "ready.json",
            {
                "ready": True,
                "run_id": run_id,
                "namespace": config.namespace,
                "role": config.task_mode or "robot",
                "seed": config.seed,
                "pid": os.getpid(),
                "ready_utc": _now_utc(),
            },
        )
        role_text = config.task_mode or "robot"
        print(
            f"仿真后端已就绪，正在等待连接 (namespace={config.namespace}, role={role_text}, "
            f"seed={config.seed}, run_id={run_id}) — 按 Ctrl-C 停止",
            flush=True,
        )
        while rclpy.ok() and not signals.requested and not node.should_stop:
            executor.spin_once(timeout_sec=0.05)
        if node.should_stop:
            failure = node.exit_reason
        elif signals.requested:
            failure = None
        summary, reason = node.close(signals.reason or node.exit_reason or "SHUTDOWN")
        exit_code = node.exit_code
        if summary.fault is not None:
            failure = summary.fault.reason
            exit_code = _EXIT_FAILURE
        elif reason:
            failure = reason if exit_code != _EXIT_OK else failure
        # A completed SIGINT/SIGTERM brake + close is a clean run; only faults
        # and close failures make the process nonzero.
    except (KeyboardInterrupt, ExternalShutdownException):
        failure = None
        exit_code = _EXIT_OK
        if node is not None:
            try:
                summary, _reason = node.close("INTERRUPTED")
            except BaseException:
                summary = None
    except BaseException as error:  # noqa: BLE001 - process boundary owns the failure
        failure = f"{type(error).__name__}: {error}"
        exit_code = _EXIT_FAILURE
        try:
            logger.emit(
                "backend.fatal",
                level="ERROR",
                reason="FATAL",
                message=failure,
                exception=error,
            )
        except Exception:
            pass
        if node is not None:
            try:
                summary, _reason = node.close("FATAL")
            except BaseException as close_error:  # noqa: BLE001 - preserve primary failure
                failure = f"{failure}; close failed: {close_error}"
        exit_code = _EXIT_FAILURE
    return (node, summary), failure, exit_code


def _summary_document(summary: WorkerSummary | None) -> dict | None:
    if summary is None:
        return None
    return {
        "closed": summary.closed,
        "physics_stopped": summary.physics_stopped,
        "reason": summary.reason,
        "fault": None
        if summary.fault is None
        else {"reason": summary.fault.reason, "detail": summary.fault.detail},
        "phase": summary.phase,
        "accepted_commands": summary.accepted_commands,
        "events_emitted": summary.events_emitted,
        "captures": summary.captures,
        "dropped_captures": summary.dropped_captures,
        "dropped_snapshots": summary.dropped_snapshots,
        "dropped_outputs": summary.dropped_outputs,
        "stale_outputs": summary.stale_outputs,
        "rendered": summary.rendered,
        "renderer_closed": summary.renderer_closed,
        "planner_stopped": summary.planner_stopped,
    }


def _teardown(executor, node, logger, previous, failure, exit_code) -> tuple[str | None, int]:
    for signum, handler in previous.items():
        signal.signal(signum, handler)
    if not executor.shutdown(timeout_sec=1.0):
        failure = failure or "EXECUTOR_CLOSE_TIMEOUT"
        exit_code = _EXIT_FAILURE
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    try:
        logger.close()
    except LogFailure as error:
        if exit_code == _EXIT_OK:
            exit_code = _EXIT_FAILURE
        failure = failure or f"LOGGER_CLOSE_FAILED: {error}"
        sys.stderr.write(f"backend logger close failed: {error}\n")
    return failure, exit_code


def run(argv: list[str] | None = None) -> int:
    parsed = _parse_arguments(argv)
    config, run_id, run_dir, logger = _prepare_run(parsed)
    signals = _Signals()
    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in previous:
        signal.signal(signum, signals)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    started = time.monotonic()
    (node, summary), failure, exit_code = _serve_node(
        config, logger, run_id, run_dir, signals, executor
    )
    failure, exit_code = _teardown(executor, node, logger, previous, failure, exit_code)
    status = {
        "run_id": run_id,
        "finished_utc": _now_utc(),
        "elapsed_s": round(time.monotonic() - started, 3),
        "success": exit_code == _EXIT_OK and failure is None,
        "exit_code": exit_code,
        "failure": failure,
        "summary": _summary_document(summary),
    }
    write_json_atomic(run_dir / "status.json", status)
    if exit_code != _EXIT_OK or failure is not None:
        print(
            f"错误: 仿真后端异常退出: {failure or f'exit_code={exit_code}'} "
            f"（详情见 {run_dir / 'status.json'} 与 {run_dir / 'log'}）",
            file=sys.stderr,
            flush=True,
        )
    print(f"仿真后端已停止 (run_id={run_id}, exit_code={exit_code})", flush=True)
    return exit_code


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
