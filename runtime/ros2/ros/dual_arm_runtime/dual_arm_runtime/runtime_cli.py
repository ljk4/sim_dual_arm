"""Process entry point, signal handling and bounded runtime shutdown."""

from __future__ import annotations

import argparse
import json
import os
import re
import secrets
import signal
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from robot_run_logging import EventContext

from robot_contracts.project_paths import recordings_root, runs_root
from dual_arm_runtime.run_logging import (
    check_health,
    emit_best_effort,
    emit_event,
    initialize_run,
    runtime_summary,
    validate_run_id,
)
from dual_arm_runtime.runtime_config import (
    load_runtime_configuration,
    load_runtime_configuration_document,
)
from dual_arm_runtime.runtime_node import RuntimeNode
from dual_arm_runtime.scenario_runtime import resolve_scenario_runtime
from dual_arm_runtime.shutdown import (
    EXIT_RUNTIME_FAILURE,
    EXIT_SHUTDOWN_TIMEOUT,
    ShutdownSignal,
    install_signal_handlers,
    positive_seconds,
    restore_signal_handlers,
)


def _forced_shutdown_timeout(node: RuntimeNode | None, timeout_s: float) -> None:
    details = node.shutdown_details() if node is not None else {}
    print(
        json.dumps({"shutdown": "SHUTDOWN_TIMEOUT", "timeout_s": timeout_s, **details}),
        flush=True,
    )
    print(
        f"SHUTDOWN_TIMEOUT: worker still active after {timeout_s:.3f}s; "
        "forcing process exit without destroying ROS resources",
        file=sys.stderr,
        flush=True,
    )
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(EXIT_SHUTDOWN_TIMEOUT)


def _destroy_ros_resources(
    node: RuntimeNode | None, executor: SingleThreadedExecutor | None
) -> None:
    if executor is not None:
        if node is not None:
            executor.remove_node(node)
        executor.shutdown()
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _parse_arguments(argv: list[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    ros_split = raw_argv.index("--ros-args") if "--ros-args" in raw_argv else len(raw_argv)
    interactive = "--confirm" in raw_argv[:ros_split]

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path("/app"))
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--config", type=Path, help="direct runtime recipe path")
    selection.add_argument(
        "--robot", help="robot id under <shared-root>/scenarios/<robot>/<task>/binding.yaml"
    )
    parser.add_argument("--task", help="task id for --robot")
    parser.add_argument("--role", choices=("policy", "teacher"), default="policy")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument("--robot-namespace", help=argparse.SUPPRESS)
    parser.add_argument(
        "--policy-endpoint", metavar="HOST:PORT", help="Policy Service gRPC address"
    )
    parser.add_argument("--output", "--output-root", dest="output", type=Path, default=None)
    parser.add_argument(
        "--episode-dir",
        type=Path,
        default=None,
        help="exact final directory for the recorded episode (default: <output>/<episode-id>)",
    )
    if not interactive:
        parser.add_argument("--run-id", default=None)
        parser.add_argument("--episode-id", default=os.environ.get("RUNTIME_EPISODE_ID", ""))
    parser.add_argument("--runs-output", type=Path, default=None)
    parser.add_argument("--once", action="store_true")
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="prompt for interactive confirmation before resetting and starting",
    )
    parser.add_argument(
        "--hold-after-episode",
        action="store_true",
        help="with --once, keep the runtime alive after the episode until SIGINT/SIGTERM",
    )
    parser.add_argument(
        "--reset-seed",
        "--seed",
        dest="reset_seed",
        type=int,
        default=None,
        help="scene/reset seed for this runtime process (default: random)",
    )
    parser.add_argument(
        "--shutdown-timeout",
        type=positive_seconds,
        default=20.0,
        help="bounded seconds to wait for episode cleanup after a signal",
    )
    args, ros_args = parser.parse_known_args(raw_argv)
    if interactive:
        args.run_id = None
        args.episode_id = ""
    if ros_args and ros_args[0] != "--ros-args":
        parser.error(f"unrecognized arguments: {' '.join(ros_args)}")
    if args.robot is not None and args.task is None:
        parser.error("--robot requires --task")
    if args.task is not None and args.robot is None:
        parser.error("--task requires --robot")
    if args.robot_namespace is not None and args.robot is None:
        parser.error("--robot-namespace requires --robot/--task")
    if args.hold_after_episode and not args.once:
        parser.error("--hold-after-episode requires --once")
    if args.reset_seed is not None and not 0 <= args.reset_seed <= 2**63 - 1:
        parser.error("--reset-seed must be within [0, 2^63-1]")
    return args, ros_args


def _initialization_error(error: BaseException) -> None:
    print(
        json.dumps({"runtime_initialization_error": f"{type(error).__name__}: {error}"}),
        file=sys.stderr,
        flush=True,
    )
    raise SystemExit(EXIT_RUNTIME_FAILURE) from error


def _random_seed() -> int:
    return secrets.randbelow(2**31)


def _load_configuration(args: argparse.Namespace):
    try:
        fallback_seed = _random_seed() if args.reset_seed is None else None
        if args.robot is not None:
            if args.config is not None:
                raise ValueError("--robot/--task and --config are mutually exclusive")
            resolved = resolve_scenario_runtime(
                robot=args.robot,
                task=args.task,
                role=args.role,
                shared_root=args.shared_root,
                project_root=args.project_root,
                seed=args.reset_seed,
            )
            if fallback_seed is not None and isinstance(resolved.document.get("reset"), dict):
                resolved.document["reset"]["seed"] = fallback_seed
            if args.robot_namespace is not None:
                resolved.document["robot_namespace"] = args.robot_namespace
            return load_runtime_configuration_document(
                resolved.document,
                source=resolved.source,
                base_dir=resolved.base_dir,
                project_root=args.project_root,
                policy_endpoint=args.policy_endpoint,
                selection=resolved.selection,
            )
        if args.shared_root is not None:
            raise ValueError("--shared-root requires --robot/--task")
        return load_runtime_configuration(
            args.config,
            args.project_root,
            seed=args.reset_seed,
            default_seed=fallback_seed,
            policy_endpoint=args.policy_endpoint,
        )
    except Exception as error:
        _initialization_error(error)


class _RuntimeProcess:
    """Mutable process state for one runtime launch and its bounded shutdown."""

    def __init__(
        self,
        args: argparse.Namespace,
        configuration: Any,
        run_id: str,
        runs_output: Path,
        session: Any,
        signal_state: ShutdownSignal,
        previous_handlers: Any,
        ros_args: list[str],
    ) -> None:
        self.args = args
        self.configuration = configuration
        self.run_id = run_id
        self.runs_output = runs_output
        self.session = session
        self.signal_state = signal_state
        self.previous_handlers = previous_handlers
        self.ros_args = ros_args
        self.logger = session.logger
        self.node: RuntimeNode | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.rclpy_initialized = False
        self.exit_code = 0
        self.fatal_error = False
        self.fatal_exception: BaseException | None = None
        self.executor_usable = True
        self.startup_announced = False
        self.stopping_logged = False
        self.health_failure_seen = False
        self.hold_announced = False
        self.shutdown_deadline: float | None = None
        self.record_directory = ""
        self.episode_id = ""

    def set_fatal(self, error: BaseException) -> None:
        self.fatal_error = True
        if self.fatal_exception is None:
            self.fatal_exception = error

    def emit_stopping(self, code: str, detail: str = "") -> None:
        if self.stopping_logged:
            return
        self.stopping_logged = True
        node = self.node
        current_episode = ""
        if node is not None:
            try:
                current_episode = node.shutdown_details().get("episode_id", "")
            except Exception:
                current_episode = ""
        if code == "COMPLETE":
            level = "INFO"
        elif code == self.signal_state.reason and self.signal_state.requested:
            level = "WARNING"
        else:
            level = "ERROR"
        error_text = emit_best_effort(
            self.logger,
            "run.stopping",
            level=level,
            reason=code,
            message=detail,
            context=EventContext(episode_id=current_episode or None),
            fields={
                "signal": self.signal_state.reason or None,
                "episode_id": current_episode or None,
            },
            critical=True,
        )
        if error_text is not None:
            self.set_fatal(RuntimeError(error_text))
            if node is not None:
                node.append_failure(f"LOGGING_ERROR: {error_text}")

    def current_failure_code(self) -> str:
        if self.signal_state.requested:
            return self.signal_state.reason
        if self.fatal_error or (self.node is not None and self.node.failure_reason):
            return "RUNTIME_FAILURE"
        return "COMPLETE"

    def request_stop(self, reason: str, code: str, detail: str = "") -> None:
        if self.node is not None:
            self.node.request_shutdown(reason)
        if self.shutdown_deadline is None:
            self.shutdown_deadline = time.monotonic() + self.args.shutdown_timeout
        self.emit_stopping(code, detail)

    def start_ros(self) -> bool:
        try:
            rclpy.init(args=self.ros_args, signal_handler_options=SignalHandlerOptions.NO)
            self.rclpy_initialized = True
            self.node = RuntimeNode(
                self.configuration,
                self.args.output,
                auto_start=self.args.once,
                log=self.logger,
                run_id=self.run_id,
                initial_episode_id=self.args.episode_id or None,
                episode_output=self.args.episode_dir,
            )
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
        except BaseException as error:
            self.set_fatal(error)
            print(
                json.dumps({"runtime_error": f"INITIALIZATION: {type(error).__name__}: {error}"}),
                flush=True,
            )
            return False
        return True

    def announce_start(self) -> None:
        try:
            emit_event(
                self.logger,
                "run.started",
                level="INFO",
                reason="STARTUP",
                message="runtime process starting",
                fields={
                    "config_path": (
                        str(self.args.config)
                        if self.args.config is not None
                        else f"scenario:{self.args.robot}/{self.args.task}/{self.args.role}"
                    ),
                    "robot": self.args.robot,
                    "task": self.args.task,
                    "output_path": str(self.args.output),
                    "runs_output": str(self.runs_output),
                    "run_directory": str(self.session.run_directory),
                    "shutdown_timeout_s": self.args.shutdown_timeout,
                },
                critical=True,
            )
            self.startup_announced = True
        except Exception as error:
            self.set_fatal(error)
            self.request_stop(f"LOGGING_ERROR: {error}", "LOGGING_ERROR", detail=str(error))

    def _health_tick(self) -> None:
        if self.health_failure_seen:
            return
        try:
            check_health(self.logger)
        except Exception as error:
            self.health_failure_seen = True
            self.set_fatal(error)
            self.request_stop(f"LOGGING_ERROR: {error}", "LOGGING_ERROR", detail=str(error))

    def _episode_done(self) -> bool:
        node = self.node
        assert node is not None
        if not self.args.hold_after_episode:
            return True
        if not self.hold_announced:
            details = node.shutdown_details()
            print(
                "本局已结束；录制目录 "
                f"{details.get('directory', '')}；按 Ctrl-C 退出",
                flush=True,
            )
            self.hold_announced = True
        self._spin_once(0.1)
        return False

    def _spin_once(self, timeout: float) -> None:
        assert self.executor is not None
        try:
            self.executor.spin_once(timeout_sec=timeout)
        except KeyboardInterrupt:
            self.signal_state(signal.SIGINT, None)

    def _runtime_error(self, error: BaseException) -> None:
        self.set_fatal(error)
        self.executor_usable = False
        reason = f"RUNTIME_ERROR: {type(error).__name__}: {error}"
        self.request_stop(reason, "RUNTIME_ERROR", detail=reason)
        print(json.dumps({"runtime_error": reason}), flush=True)

    def _shutdown_step(self) -> bool:
        node = self.node
        assert node is not None
        if not node.worker_alive():
            return True
        if self.shutdown_deadline is None:
            self.shutdown_deadline = time.monotonic() + self.args.shutdown_timeout
        remaining = self.shutdown_deadline - time.monotonic()
        if remaining <= 0:
            _forced_shutdown_timeout(node, self.args.shutdown_timeout)
        if self.executor_usable:
            assert self.executor is not None
            try:
                self.executor.spin_once(timeout_sec=min(0.05, remaining))
            except KeyboardInterrupt:
                self.signal_state(signal.SIGINT, None)
            except Exception as error:
                self._runtime_error(error)
        elif not node.join_worker(min(0.05, remaining)):
            time.sleep(0.01)
        return False

    def run_loop(self) -> None:
        node = self.node
        assert node is not None
        while True:
            self._health_tick()
            if node.logging_failed and not node.shutdown_requested:
                self.set_fatal(RuntimeError(node.logging_failure))
                self.request_stop(
                    f"LOGGING_ERROR: {node.logging_failure}",
                    "LOGGING_ERROR",
                    detail=node.logging_failure,
                )
            if self.signal_state.requested and not node.shutdown_requested:
                self.request_stop(
                    self.signal_state.reason, self.signal_state.reason, detail="termination signal"
                )
            if (
                self.args.once
                and node.done.is_set()
                and not node.shutdown_requested
                and not self.signal_state.requested
            ):
                if self._episode_done():
                    break
                continue
            if node.shutdown_requested:
                if self._shutdown_step():
                    break
                continue
            try:
                assert self.executor is not None
                self.executor.spin_once(timeout_sec=0.1)
            except KeyboardInterrupt:
                self.signal_state(signal.SIGINT, None)
            except Exception as error:
                self._runtime_error(error)
                self.shutdown_deadline = time.monotonic() + self.args.shutdown_timeout

    def _await_worker(self) -> None:
        node = self.node
        assert node is not None
        if not node.worker_alive():
            return
        if self.shutdown_deadline is None:
            self.shutdown_deadline = time.monotonic() + self.args.shutdown_timeout
        remaining = self.shutdown_deadline - time.monotonic()
        if remaining <= 0 or not node.join_worker(remaining):
            _forced_shutdown_timeout(node, self.args.shutdown_timeout)

    def _set_exit_code(self) -> None:
        node = self.node
        if self.signal_state.requested:
            self.exit_code = self.signal_state.exit_code
        elif self.fatal_error or (node is not None and node.failure_reason):
            self.exit_code = EXIT_RUNTIME_FAILURE

    def execute(self) -> None:
        try:
            if not self.start_ros():
                return
            self.announce_start()
            self.run_loop()
            self._await_worker()
            self._set_exit_code()
            if not self.stopping_logged:
                self.emit_stopping(self.current_failure_code(), "process leaving main")
        except BaseException as error:
            self.set_fatal(error)
            print(json.dumps({"runtime_error": f"{type(error).__name__}: {error}"}), flush=True)
            if self.node is not None:
                self.request_stop(
                    f"RUNTIME_ERROR: {type(error).__name__}: {error}",
                    "RUNTIME_ERROR",
                    detail=str(error),
                )

    def _teardown(self) -> tuple[BaseException | None, bool]:
        teardown_error: BaseException | None = None
        node = self.node
        try:
            if node is not None:
                try:
                    details = node.shutdown_details()
                    self.episode_id = details.get("episode_id", "")
                    self.record_directory = details.get("directory", "")
                except Exception:
                    pass
                if node.worker_alive():
                    node.request_shutdown("UNEXPECTED_MAIN_EXIT")
                    if not node.join_worker(self.args.shutdown_timeout):
                        _forced_shutdown_timeout(node, self.args.shutdown_timeout)
            try:
                if self.rclpy_initialized:
                    _destroy_ros_resources(node, self.executor)
            except BaseException as error:
                teardown_error = error
                self.set_fatal(error)
                print(
                    json.dumps({"runtime_error": f"TEARDOWN: {type(error).__name__}: {error}"}),
                    flush=True,
                )
        finally:
            restore_signal_handlers(self.previous_handlers)
        return teardown_error, self.stopping_logged

    def _finalize_logging(self, teardown_error: BaseException | None) -> bool:
        if not self.stopping_logged:
            self.emit_stopping(self.current_failure_code(), "process leaving main")
        self._set_exit_code()
        node = self.node
        if (
            not self.startup_announced
            and not self.signal_state.requested
            and not self.fatal_error
            and not (node is not None and node.failure_reason)
        ):
            self.exit_code = EXIT_RUNTIME_FAILURE
        finalizing_code = self.current_failure_code()
        if finalizing_code == "COMPLETE":
            finalizing_reason, finalizing_level = "CLOSE_PENDING", "INFO"
        elif self.signal_state.requested:
            finalizing_reason, finalizing_level = finalizing_code, "WARNING"
        else:
            finalizing_reason, finalizing_level = finalizing_code, "ERROR"
        node = self.node
        failure_detail = node.failure_reason if node is not None else ""
        if teardown_error is not None:
            failure_detail = (
                f"{failure_detail}; TEARDOWN: {type(teardown_error).__name__}: {teardown_error}"
            )
        try:
            emit_event(
                self.logger,
                "run.finalizing",
                level=finalizing_level,
                reason=finalizing_reason,
                message=failure_detail,
                context=EventContext(episode_id=self.episode_id or None),
                fields={
                    "exit_code_before_log_close": self.exit_code,
                    "log_close_pending": True,
                    "episode_id": self.episode_id or None,
                    "record_directory": self.record_directory or None,
                    "started": self.startup_announced,
                    "signal": self.signal_state.reason or None,
                },
                exception=self.fatal_exception or teardown_error,
                critical=True,
            )
        except Exception as error:
            if self.exit_code == 0:
                self.exit_code = EXIT_RUNTIME_FAILURE
            print(
                json.dumps({"logging_error": f"run.finalizing: {type(error).__name__}: {error}"}),
                file=sys.stderr,
                flush=True,
            )
        logging_closed = False
        try:
            self.logger.close()
        except Exception as error:
            if self.exit_code == 0:
                self.exit_code = EXIT_RUNTIME_FAILURE
            print(
                json.dumps({"logging_error": f"close: {type(error).__name__}: {error}"}),
                file=sys.stderr,
                flush=True,
            )
        else:
            logging_closed = True
        return logging_closed

    def finalize(self) -> tuple[bool, str]:
        teardown_error, _stopping = self._teardown()
        logging_closed = self._finalize_logging(teardown_error)
        if self.signal_state.requested:
            process_outcome = "interrupted"
        elif self.exit_code == 0:
            process_outcome = "succeeded"
        else:
            process_outcome = "failed"
        return logging_closed, process_outcome


def _print_configuration_summary(
    configuration: object, args: argparse.Namespace, run_directory: Path
) -> None:
    """Print only the operator-relevant parameters; the full snapshot is on disk."""
    selection = getattr(configuration, "selection", None)
    if isinstance(selection, dict) and selection.get("mode") == "scenario":
        print(
            f"方案: {selection.get('robot')}/{selection.get('task')} "
            f"({selection.get('role')})",
            flush=True,
        )
        binding = selection.get("binding") or {}
        if binding.get("path"):
            print(f"Binding: {binding['path']}", flush=True)
    recipe = configuration.recipe
    stream = recipe.policy if recipe.action_source == "policy" else recipe.teacher
    print(f"配置: {configuration.configuration_source}", flush=True)
    print(
        f"动作来源: {recipe.action_source} | 命名空间: {recipe.robot_namespace} | "
        f"终止模式: {getattr(stream, 'termination_mode', '-')} | "
        f"seed: {recipe.reset.seed}",
        flush=True,
    )
    print(f"输出目录: {args.output}", flush=True)
    print(f"详细参数: {run_directory / 'resolved' / 'runtime.json'}", flush=True)


def _unique_run_id(output_dir: Path, runs_output: Path, base: str) -> str:
    def _exists(name: str) -> bool:
        return (
            (output_dir / name).exists()
            or (output_dir / f".partial-{name}").exists()
            or (runs_output / name).exists()
        )

    candidate = base
    if _exists(candidate):
        suffix = 2
        while _exists(f"{base}_{suffix}"):
            suffix += 1
        candidate = f"{base}_{suffix}"
    return candidate


def _safe_id_part(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip()).strip("._-")
    return cleaned or "unknown"


def _interactive_run_id_base(
    args: argparse.Namespace, configuration: Any, stamp: str | None = None
) -> str:
    stamp = stamp or time.strftime("%Y%m%d_%H%M%S")
    if args.robot is not None and args.task is not None:
        robot_part = _safe_id_part(args.robot)
        task_part = _safe_id_part(args.task)
    else:
        robot_part = _safe_id_part(configuration.profile.robot_id)
        recipe = configuration.recipe
        task_name = (
            recipe.policy.task_id
            if recipe.policy is not None
            else recipe.teacher.task_id
            if recipe.teacher is not None
            else ""
        )
        task_part = _safe_id_part(task_name or args.config.stem)
    head = f"client_interactive_{robot_part}_{task_part}"[:80].rstrip("._-")
    return f"{head}_{stamp}"


def _confirm_start(args: argparse.Namespace, configuration: Any, run_id: str) -> None:
    display = (
        f"scenario:{args.robot}/{args.task}/{args.role}"
        if args.robot is not None
        else str(args.config)
    )
    print(f"配置: {display}", flush=True)
    if args.policy_endpoint:
        print(f"策略服务: {args.policy_endpoint}", flush=True)
    print(f"run_id: {run_id}", flush=True)
    print(f"seed: {configuration.recipe.reset.seed}", flush=True)
    print("输入 y 复原环境并开始本次任务，其他键取消: ", end="", flush=True)
    try:
        answer = input().strip()
    except EOFError as error:
        raise SystemExit("interactive: no answer; canceled") from error
    if answer not in {"y", "Y", "yes", "YES"}:
        print("interactive: canceled", flush=True)
        raise SystemExit(0)


def main() -> None:
    args, ros_args = _parse_arguments()
    configuration = _load_configuration(args)
    try:
        if args.output is None:
            args.output = recordings_root(args.robot, args.task)
        runs_output = (
            args.runs_output
            if args.runs_output is not None
            else runs_root(args.robot, args.task)
        )
        if args.confirm:
            run_id = validate_run_id(
                _unique_run_id(
                    args.output,
                    runs_output,
                    _interactive_run_id_base(args, configuration),
                )
            )
            args.episode_id = run_id
        else:
            if args.run_id is None:
                prefix = args.role or "policy"
                run_id = _unique_run_id(
                    args.output, runs_output, f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}"
                )
            else:
                run_id = validate_run_id(args.run_id)
            if args.episode_id:
                validate_run_id(args.episode_id)
            elif args.once:
                args.episode_id = run_id
    except ValueError as error:
        raise SystemExit(f"invalid run/episode id: {error}") from error
    if args.confirm:
        _confirm_start(args, configuration, run_id)
    try:
        session = initialize_run(
            configuration, run_id=run_id, runs_output=runs_output, output=args.output
        )
    except Exception as error:
        _initialization_error(error)
    _print_configuration_summary(configuration, args, session.run_directory)
    signal_state = ShutdownSignal()
    process = _RuntimeProcess(
        args,
        configuration,
        run_id,
        runs_output,
        session,
        signal_state,
        install_signal_handlers(signal_state),
        ros_args,
    )
    try:
        process.execute()
    finally:
        logging_closed, process_outcome = process.finalize()
    summary = runtime_summary(
        session,
        record_directory=process.record_directory,
        exit_code=process.exit_code,
        process_outcome=process_outcome,
        logging_closed=logging_closed,
    )
    summary_path = session.run_directory / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"运行结束：{process_outcome} | 退出码 {process.exit_code} | "
        f"日志 {session.run_directory}",
        flush=True,
    )
    if process.exit_code:
        raise SystemExit(process.exit_code)


if __name__ == "__main__":
    main()
