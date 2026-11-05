"""Process entry point for the runnable native backend.

Run directory creation, signals, run metadata and the SingleThreadedExecutor
loop live here so ``node.py`` stays focused on ROS conversion/lifecycle.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import re
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from robot_run_logging import EventLogger, LogFailure, LogSettings

from . import assets
from .backend_config import load_backend_config
from .diagnostics import code_provenance
from .node import NativeBackendNode
from .worker import WorkerSummary

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


def _write_json(path: Path, document: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _now_utc() -> str:
    from datetime import UTC, datetime

    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    return parser


def run(argv: list[str] | None = None) -> int:
    parsed = _parser().parse_args(argv)
    config = load_backend_config(parsed.config)
    run_id = _validate_run_id(parsed.run_id)
    run_root = parsed.run_root.resolve()
    run_dir = (run_root / run_id).resolve()
    if not _inside(run_dir, run_root) or run_dir == run_root:
        raise SystemExit("run directory escapes run root")
    try:
        run_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise SystemExit(f"run directory already exists: {run_dir}") from error
    logger = EventLogger(
        run_dir / "log",
        run_id=run_id,
        component="native_backend",
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
    _write_json(
        run_dir / "run.json",
        {
            "run_id": run_id,
            "created_utc": _now_utc(),
            "config": str(config.source),
            "panda_model": str(assets.model_paths()[1]),
            "effective_configuration": json.loads(json.dumps(asdict(config), default=str)),
            "configuration_sources": {
                name: {
                    "path": str(path.resolve()),
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "content": path.read_text(),
                }
                for name, path in {
                    "backend": config.source,
                    "scene": config.scene_path,
                    "control": config.control_path,
                    "stream": config.stream_path,
                    "profile": config.profile_path,
                    "follower": config.follower_path,
                    "task": config.task_path,
                }.items()
                if path is not None
            },
            "code_provenance": code_provenance(),
            "namespace": config.namespace,
            "seed": config.seed,
            "capture_hz": config.capture_hz,
            "realtime_speed": config.realtime_speed,
            "velocity_feedforward": config.velocity_feedforward,
            "reliable_sensors": config.reliable_sensors,
            "versions": {
                "python": sys.version.split()[0],
                "executable": sys.executable,
                "packages": {
                    name: _package_version(name)
                    for name in ("mujoco", "scipy", "numpy", "mink", "ruckig")
                },
            },
        },
    )
    signals = _Signals()
    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in previous:
        signal.signal(signum, signals)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    node: NativeBackendNode | None = None
    summary: WorkerSummary | None = None
    failure: str | None = None
    exit_code = _EXIT_OK
    started = time.monotonic()
    try:
        node = NativeBackendNode(
            config,
            log=logger,
        )
        executor.add_node(node)
        logger.emit(
            "backend.start",
            message="native backend ready",
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
                "panda_model": str(assets.model_paths()[1]),
            },
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
    finally:
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
            sys.stderr.write(f"native backend logger close failed: {error}\n")
        status = {
            "run_id": run_id,
            "finished_utc": _now_utc(),
            "elapsed_s": round(time.monotonic() - started, 3),
            "success": exit_code == _EXIT_OK and failure is None,
            "exit_code": exit_code,
            "failure": failure,
            "summary": None
            if summary is None
            else {
                "closed": summary.closed,
                "physics_stopped": summary.physics_stopped,
                "reason": summary.reason,
                "fault": None if summary.fault is None else asdict(summary.fault),
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
            },
        }
        _write_json(run_dir / "status.json", status)
    return exit_code


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
