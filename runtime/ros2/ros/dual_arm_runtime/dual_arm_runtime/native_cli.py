"""Process entry point, signal handling and bounded native runtime shutdown."""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from robot_run_logging import EventContext

from dual_arm_runtime.native_config import load_native_configuration
from dual_arm_runtime.native_node import NativeRuntimeNode
from dual_arm_runtime.run_logging import (
    check_health,
    emit_best_effort,
    emit_event,
    initialize_run,
    new_run_id,
    runtime_summary,
    validate_run_id,
)
from dual_arm_runtime.shutdown import (
    EXIT_RUNTIME_FAILURE,
    EXIT_SHUTDOWN_TIMEOUT,
    ShutdownSignal,
    install_signal_handlers,
    positive_seconds,
    restore_signal_handlers,
)


def _forced_shutdown_timeout(node: NativeRuntimeNode | None, timeout_s: float) -> None:
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
    node: NativeRuntimeNode | None, executor: SingleThreadedExecutor | None
) -> None:
    if executor is not None:
        if node is not None:
            executor.remove_node(node)
        executor.shutdown()
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path("/workspace"))
    parser.add_argument(
        "--config", type=Path, default=Path("/workspace/runtime/ros2/configs/native_collect.yaml")
    )
    parser.add_argument("--output", type=Path, default=Path("/data/raw"))
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--episode-id", default=os.environ.get("NATIVE_EPISODE_ID", ""))
    parser.add_argument("--runs-output", type=Path, default=None)
    parser.add_argument("--once", action="store_true")
    parser.add_argument(
        "--shutdown-timeout",
        type=positive_seconds,
        default=20.0,
        help="bounded seconds to wait for native episode cleanup after a signal",
    )
    args, ros_args = parser.parse_known_args()
    try:
        run_id = new_run_id() if args.run_id is None else validate_run_id(args.run_id)
        if args.episode_id:
            validate_run_id(args.episode_id)
    except ValueError as error:
        parser.error(str(error))
    runs_output = args.runs_output if args.runs_output is not None else args.output.parent / "runs"
    try:
        configuration = load_native_configuration(args.config, args.project_root)
    except Exception as error:
        print(
            json.dumps({"runtime_initialization_error": f"{type(error).__name__}: {error}"}),
            file=sys.stderr,
            flush=True,
        )
        raise SystemExit(EXIT_RUNTIME_FAILURE) from error
    try:
        session = initialize_run(
            configuration, run_id=run_id, runs_output=runs_output, output=args.output
        )
    except Exception as error:
        print(
            json.dumps({"runtime_initialization_error": f"{type(error).__name__}: {error}"}),
            file=sys.stderr,
            flush=True,
        )
        raise SystemExit(EXIT_RUNTIME_FAILURE) from error
    logger = session.logger
    signal_state = ShutdownSignal()
    previous_handlers = install_signal_handlers(signal_state)
    rclpy_initialized = False
    node: NativeRuntimeNode | None = None
    executor: SingleThreadedExecutor | None = None
    exit_code = 0
    fatal_error = False
    fatal_exception: BaseException | None = None
    executor_usable = True
    startup_announced = False
    stopping_logged = False
    health_failure_seen = False
    shutdown_deadline: float | None = None
    record_directory = ""
    episode_id = ""

    def set_fatal(error: BaseException) -> None:
        nonlocal fatal_error, fatal_exception
        fatal_error = True
        if fatal_exception is None:
            fatal_exception = error

    def emit_stopping(code: str, detail: str = "") -> None:
        nonlocal stopping_logged
        if stopping_logged:
            return
        stopping_logged = True
        current_episode = ""
        if node is not None:
            try:
                current_episode = node.shutdown_details().get("episode_id", "")
            except Exception:
                current_episode = ""
        if code == "COMPLETE":
            level = "INFO"
        elif code == signal_state.reason and signal_state.requested:
            level = "WARNING"
        else:
            level = "ERROR"
        error_text = emit_best_effort(
            logger,
            "run.stopping",
            level=level,
            reason=code,
            message=detail,
            context=EventContext(episode_id=current_episode or None),
            fields={"signal": signal_state.reason or None, "episode_id": current_episode or None},
            critical=True,
        )
        if error_text is not None:
            set_fatal(RuntimeError(error_text))
            if node is not None:
                node.append_failure(f"LOGGING_ERROR: {error_text}")

    def current_failure_code() -> str:
        if signal_state.requested:
            return signal_state.reason
        if fatal_error or (node is not None and node.failure_reason):
            return "RUNTIME_FAILURE"
        return "COMPLETE"

    def request_stop(reason: str, code: str, detail: str = "") -> None:
        nonlocal shutdown_deadline
        if node is not None:
            node.request_shutdown(reason)
        if shutdown_deadline is None:
            shutdown_deadline = time.monotonic() + args.shutdown_timeout
        emit_stopping(code, detail)

    try:
        try:
            rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
            rclpy_initialized = True
            node = NativeRuntimeNode(
                configuration,
                args.output,
                auto_start=args.once,
                log=logger,
                run_id=run_id,
                initial_episode_id=args.episode_id or None,
            )
            executor = SingleThreadedExecutor()
            executor.add_node(node)
        except BaseException as error:
            set_fatal(error)
            print(
                json.dumps({"runtime_error": f"INITIALIZATION: {type(error).__name__}: {error}"}),
                flush=True,
            )
        else:
            try:
                emit_event(
                    logger,
                    "run.started",
                    level="INFO",
                    reason="STARTUP",
                    message="native runtime process starting",
                    fields={
                        "config_path": str(args.config),
                        "output_path": str(args.output),
                        "runs_output": str(runs_output),
                        "run_directory": str(session.run_directory),
                        "shutdown_timeout_s": args.shutdown_timeout,
                    },
                    critical=True,
                )
                startup_announced = True
            except Exception as error:
                set_fatal(error)
                request_stop(f"LOGGING_ERROR: {error}", "LOGGING_ERROR", detail=str(error))
            while True:
                if not health_failure_seen:
                    try:
                        check_health(logger)
                    except Exception as error:
                        health_failure_seen = True
                        set_fatal(error)
                        request_stop(f"LOGGING_ERROR: {error}", "LOGGING_ERROR", detail=str(error))
                if node.logging_failed and not node.shutdown_requested:
                    set_fatal(RuntimeError(node.logging_failure))
                    request_stop(
                        f"LOGGING_ERROR: {node.logging_failure}",
                        "LOGGING_ERROR",
                        detail=node.logging_failure,
                    )
                if signal_state.requested and not node.shutdown_requested:
                    request_stop(
                        signal_state.reason, signal_state.reason, detail="termination signal"
                    )
                if args.once and node.done.is_set() and not node.shutdown_requested:
                    break
                if node.shutdown_requested:
                    if not node.worker_alive():
                        break
                    if shutdown_deadline is None:
                        shutdown_deadline = time.monotonic() + args.shutdown_timeout
                    remaining = shutdown_deadline - time.monotonic()
                    if remaining <= 0:
                        _forced_shutdown_timeout(node, args.shutdown_timeout)
                    if executor_usable:
                        try:
                            executor.spin_once(timeout_sec=min(0.05, remaining))
                        except KeyboardInterrupt:
                            signal_state(signal.SIGINT, None)
                        except Exception as error:
                            set_fatal(error)
                            executor_usable = False
                            reason = f"RUNTIME_ERROR: {type(error).__name__}: {error}"
                            request_stop(reason, "RUNTIME_ERROR", detail=reason)
                            print(json.dumps({"runtime_error": reason}), flush=True)
                    elif not node.join_worker(min(0.05, remaining)):
                        time.sleep(0.01)
                    continue
                try:
                    executor.spin_once(timeout_sec=0.1)
                except KeyboardInterrupt:
                    signal_state(signal.SIGINT, None)
                except Exception as error:
                    set_fatal(error)
                    executor_usable = False
                    reason = f"RUNTIME_ERROR: {type(error).__name__}: {error}"
                    request_stop(reason, "RUNTIME_ERROR", detail=reason)
                    print(json.dumps({"runtime_error": reason}), flush=True)
                    shutdown_deadline = time.monotonic() + args.shutdown_timeout
            if node.worker_alive():
                if shutdown_deadline is None:
                    shutdown_deadline = time.monotonic() + args.shutdown_timeout
                remaining = shutdown_deadline - time.monotonic()
                if remaining <= 0 or not node.join_worker(remaining):
                    _forced_shutdown_timeout(node, args.shutdown_timeout)
            if signal_state.requested:
                exit_code = signal_state.exit_code
            elif fatal_error or node.failure_reason:
                exit_code = EXIT_RUNTIME_FAILURE
            if not stopping_logged:
                emit_stopping(current_failure_code(), "process leaving main")
    except BaseException as error:
        set_fatal(error)
        print(json.dumps({"runtime_error": f"{type(error).__name__}: {error}"}), flush=True)
        if node is not None:
            request_stop(
                f"RUNTIME_ERROR: {type(error).__name__}: {error}",
                "RUNTIME_ERROR",
                detail=str(error),
            )
    finally:
        teardown_error: BaseException | None = None
        try:
            if node is not None:
                try:
                    details = node.shutdown_details()
                    episode_id = details.get("episode_id", "")
                    record_directory = details.get("directory", "")
                except Exception:
                    pass
                if node.worker_alive():
                    node.request_shutdown("UNEXPECTED_MAIN_EXIT")
                    if not node.join_worker(args.shutdown_timeout):
                        _forced_shutdown_timeout(node, args.shutdown_timeout)
            try:
                if rclpy_initialized:
                    _destroy_ros_resources(node, executor)
            except BaseException as error:
                teardown_error = error
                set_fatal(error)
                print(
                    json.dumps({"runtime_error": f"TEARDOWN: {type(error).__name__}: {error}"}),
                    flush=True,
                )
            if not stopping_logged:
                emit_stopping(current_failure_code(), "process leaving main")
            if signal_state.requested:
                exit_code = signal_state.exit_code
            elif fatal_error or (node is not None and node.failure_reason):
                exit_code = EXIT_RUNTIME_FAILURE
            elif not startup_announced:
                exit_code = EXIT_RUNTIME_FAILURE
            finalizing_code = current_failure_code()
            if finalizing_code == "COMPLETE":
                finalizing_reason, finalizing_level = "CLOSE_PENDING", "INFO"
            elif signal_state.requested:
                finalizing_reason, finalizing_level = finalizing_code, "WARNING"
            else:
                finalizing_reason, finalizing_level = finalizing_code, "ERROR"
            failure_detail = node.failure_reason if node is not None else ""
            if teardown_error is not None:
                failure_detail = (
                    f"{failure_detail}; TEARDOWN: {type(teardown_error).__name__}: {teardown_error}"
                )
            try:
                emit_event(
                    logger,
                    "run.finalizing",
                    level=finalizing_level,
                    reason=finalizing_reason,
                    message=failure_detail,
                    context=EventContext(episode_id=episode_id or None),
                    fields={
                        "exit_code_before_log_close": exit_code,
                        "log_close_pending": True,
                        "episode_id": episode_id or None,
                        "record_directory": record_directory or None,
                        "started": startup_announced,
                        "signal": signal_state.reason or None,
                    },
                    exception=fatal_exception or teardown_error,
                    critical=True,
                )
            except Exception as error:
                if exit_code == 0:
                    exit_code = EXIT_RUNTIME_FAILURE
                print(
                    json.dumps(
                        {"logging_error": f"run.finalizing: {type(error).__name__}: {error}"}
                    ),
                    file=sys.stderr,
                    flush=True,
                )
            logging_closed = False
            try:
                logger.close()
            except Exception as error:
                if exit_code == 0:
                    exit_code = EXIT_RUNTIME_FAILURE
                print(
                    json.dumps({"logging_error": f"close: {type(error).__name__}: {error}"}),
                    file=sys.stderr,
                    flush=True,
                )
            else:
                logging_closed = True
            if signal_state.requested:
                process_outcome = "interrupted"
            elif exit_code == 0:
                process_outcome = "succeeded"
            else:
                process_outcome = "failed"
        finally:
            restore_signal_handlers(previous_handlers)
    print(
        json.dumps(
            runtime_summary(
                session,
                record_directory=record_directory,
                exit_code=exit_code,
                process_outcome=process_outcome,
                logging_closed=logging_closed,
            )
        ),
        flush=True,
    )
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
