"""Process entry point for the runnable backend.

Run directory creation, signals, run metadata and the SingleThreadedExecutor
loop live here so ``node.py`` stays focused on ROS conversion/lifecycle.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import re
import signal
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from robot_contracts.overrides import OverrideError, apply_overrides
from robot_contracts.validation import UniqueKeyError, read_unique_yaml
from robot_run_logging import EventLogger, LogFailure, LogSettings

from robot_backend_sim.config.backend_config import (
    BackendConfigError,
    load_backend_config,
    load_backend_document,
)
from robot_backend_sim.config.scenario_config import (
    ROBOT_REFERENCE_FIELDS,
    ScenarioError,
    resolve_scenario_backend,
)
from robot_backend_sim.config.scene_loader import load_scene_config_for
from robot_backend_sim.physics import assets
from robot_backend_sim.ros.node import BackendNode
from robot_backend_sim.runtime.diagnostics import code_provenance
from robot_backend_sim.runtime.worker import WorkerSummary

_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_SCENARIO_SEGMENT_PATTERN = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9._-]{0,127}$")
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
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--config", type=Path, help="direct backend recipe path")
    selection.add_argument(
        "--scenario", help="scenario id under <shared-root>/scenarios/<id>/binding.yaml"
    )
    parser.add_argument("--role", choices=("teacher", "policy"), default=None)
    parser.add_argument("--shared-root", type=Path, help="shared tree containing scenarios/")
    parser.add_argument("--bundle", type=Path, help="verified bundle for a policy scenario")
    parser.add_argument("--seed", type=int, default=None, help="override run.seed")
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="override one existing configuration path (repeatable)",
    )
    parser.add_argument("--run-root", type=Path, default=None)
    parser.add_argument("--run-id", default=None)
    return parser


def _default_run_root(scenario: str | None) -> Path:
    segment = scenario or "_direct"
    if not _SCENARIO_SEGMENT_PATTERN.fullmatch(segment) or ".." in segment:
        raise SystemExit("scenario id is not a safe artifact directory")
    return Path(os.environ.get("DATA_ROOT", "/data")) / segment / "logs" / "backend"


def _configuration_sources(config) -> dict[str, dict[str, str]]:
    return {
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
    }


def _validate_arguments(parsed: argparse.Namespace) -> str:
    if parsed.scenario is not None:
        parsed.role = parsed.role or "teacher"
        return parsed.run_id or f"{parsed.scenario}-{parsed.role}"
    if parsed.role is not None:
        raise SystemExit("--role requires --scenario")
    if parsed.shared_root is not None:
        raise SystemExit("--shared-root requires --scenario")
    if parsed.bundle is not None:
        raise SystemExit("--bundle requires --scenario")
    stem = re.sub(r"[^A-Za-z0-9._-]", "-", parsed.config.stem)
    return parsed.run_id or f"backend-{stem or 'run'}"


def _direct_document(parsed: argparse.Namespace) -> dict[str, Any]:
    path = parsed.config.resolve()
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
    if parsed.seed is not None:
        run = document.get("run")
        if not isinstance(run, dict) or "seed" not in run:
            raise SystemExit("--seed requires an existing run.seed field")
        run["seed"] = parsed.seed
    return document


def _write_snapshot(run_dir: Path, document: dict[str, Any]) -> Path:
    path = run_dir / "backend.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def _prepare_run(parsed: argparse.Namespace):
    run_id = _validate_run_id(_validate_arguments(parsed))
    run_root = (parsed.run_root or _default_run_root(parsed.scenario)).resolve()
    run_dir = (run_root / run_id).resolve()
    if not _inside(run_dir, run_root) or run_dir == run_root:
        raise SystemExit("run directory escapes run root")
    selection: dict[str, Any]
    try:
        if parsed.scenario is not None:
            resolved = resolve_scenario_backend(
                scenario=parsed.scenario,
                role=parsed.role or "teacher",
                shared_root=parsed.shared_root,
                bundle=parsed.bundle,
                seed=parsed.seed,
                overrides=parsed.overrides,
            )
            document, selection = resolved.document, resolved.selection
        else:
            document = apply_overrides(_direct_document(parsed), parsed.overrides)
            selection = {"mode": "direct", "scenario_id": None, "role": None}
        # Validate before creating the run directory so a bad recipe leaves no
        # orphan directory behind.
        load_backend_document(document, parsed.config or "scenario:selection")
    except (ScenarioError, OverrideError, BackendConfigError) as error:
        raise SystemExit(str(error)) from error
    try:
        run_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise SystemExit(f"run directory already exists: {run_dir}") from error
    snapshot = _write_snapshot(run_dir, document)
    try:
        config = load_backend_config(snapshot)
    except BackendConfigError as error:
        raise SystemExit(str(error)) from error
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
    return config, run_id, run_dir, logger, selection


def _robot_metadata(config) -> dict[str, str]:
    scene_config = load_scene_config_for(config.scene_path)
    assembly = scene_config.robot.assembly
    return {
        "robot_assembly": assembly,
        "robot_model": str(assets.robot_model_path(assembly)),
    }


def _write_run_document(
    run_dir: Path, config, run_id: str, selection: dict[str, Any], overrides: list[str]
) -> None:
    _write_json(
        run_dir / "run.json",
        {
            "run_id": run_id,
            "created_utc": _now_utc(),
            "config": str(config.source),
            "selection": selection,
            "overrides": list(overrides),
            **_robot_metadata(config),
            "effective_configuration": config.canonical_dict(),
            "configuration_sources": _configuration_sources(config),
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


def _serve_node(config, logger, run_id: str, signals, executor) -> tuple[object, str | None, int]:
    node: BackendNode | None = None
    summary: WorkerSummary | None = None
    failure: str | None = None
    exit_code = _EXIT_OK
    try:
        node = BackendNode(config, log=logger)
        executor.add_node(node)
        _emit_backend_start(logger, config, run_id)
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
    parsed = _parser().parse_args(argv)
    config, run_id, run_dir, logger, selection = _prepare_run(parsed)
    _write_run_document(run_dir, config, run_id, selection, parsed.overrides)
    signals = _Signals()
    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in previous:
        signal.signal(signum, signals)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    started = time.monotonic()
    (node, summary), failure, exit_code = _serve_node(
        config, logger, run_id, signals, executor
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
    _write_json(run_dir / "status.json", status)
    return exit_code


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
