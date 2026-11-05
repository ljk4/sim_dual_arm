"""Real ROS subprocess checks for signal-driven runtime shutdown.

Run inside the Jazzy test image. The mock backend, runtime and status watcher are
separate processes; no rclpy module is replaced or mocked.
"""

import argparse
import json
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
RUNTIME_SOURCE = ROOT / "runtime/ros2/ros/dual_arm_runtime"
DOUBLES_SOURCE = ROOT / "tests/doubles/ros/mock_robot_backend"
BOOTSTRAP = ROOT / "tests/integration/runtime_shutdown_bootstrap.py"
CONFIG = ROOT / "runtime/ros2/configs/mock_collect.yaml"
PROFILE = ROOT / "profiles/robots/mock_dual_arm_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas/v1"
FINISH_HOLD_ENV = "DSH003_TEST_FINISH_HOLD_S"
FINISH_MARKER_ENV = "DSH003_TEST_FINISH_MARKER"
FAIL_SPIN_ENV = "DSH003_TEST_FAIL_SPIN_AFTER"
FAIL_STATUS_ENV = "DSH003_TEST_FAIL_STATUS_PHASE"
FAIL_FINISH_ENV = "DSH003_TEST_FAIL_FINISH"
CLEANUP_MARKER_ENV = "DSH003_TEST_CLEANUP_MARKER"


def _environment(domain_id: int) -> dict[str, str]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["PYTHONUNBUFFERED"] = "1"
    paths = [str(RUNTIME_SOURCE), str(DOUBLES_SOURCE)]
    if environment.get("PYTHONPATH"):
        paths.append(environment["PYTHONPATH"])
    environment["PYTHONPATH"] = os.pathsep.join(paths)
    return environment


class _Harness:
    """Own every subprocess and log file; stop() is safe in a test finally block."""

    def __init__(self, tmp_path: Path) -> None:
        self.tmp = tmp_path
        self.output = tmp_path / "raw"
        self.domain_id = random.SystemRandom().randint(1, 200)
        self.environment = _environment(self.domain_id)
        self.backend = None
        self.runtime = None
        self.watcher = None
        self.runtime_log = None
        self.watcher_log = None
        self._processes = []
        self._handles = []

    def _spawn(self, arguments: list[str], name: str):
        path = self.tmp / name
        handle = path.open("w")
        self._handles.append(handle)
        process = subprocess.Popen(
            arguments,
            cwd=ROOT,
            env=self.environment,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        self._processes.append(process)
        return process, path

    def start_backend(self) -> None:
        self.backend, self.backend_log = self._spawn(
            [
                sys.executable,
                "-m",
                "mock_robot_backend.node",
                "--profile",
                str(PROFILE),
                "--schemas",
                str(SCHEMAS),
            ],
            "backend.log",
        )

    def start_runtime(
        self,
        *,
        once: bool = True,
        shutdown_timeout: float = 15.0,
        extra_environment: dict[str, str] | None = None,
        bootstrap: bool = False,
    ) -> None:
        if extra_environment:
            self.environment.update(extra_environment)
        if bootstrap:
            command = [sys.executable, str(BOOTSTRAP)]
        else:
            command = [sys.executable, "-m", "dual_arm_runtime.node"]
        arguments = [
            *command,
            "--project-root",
            str(ROOT),
            "--config",
            str(CONFIG),
            "--output",
            str(self.output),
            "--shutdown-timeout",
            str(shutdown_timeout),
        ]
        if once:
            arguments.append("--once")
        self.runtime, self.runtime_log = self._spawn(arguments, "runtime.log")

    def start_watcher(
        self,
        *,
        phase: str = "",
        command: bool = False,
        service: str = "",
        signal_number: signal.Signals = signal.SIGINT,
        timeout: float = 60.0,
    ) -> None:
        arguments = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--watch",
            "--pid",
            str(self.runtime.pid),
            "--signal",
            signal_number.name,
            "--timeout",
            str(timeout),
        ]
        if phase:
            arguments.extend(["--phase", phase])
        if command:
            arguments.append("--command")
        if service:
            arguments.extend(["--service", service])
        self.watcher, self.watcher_log = self._spawn(arguments, "watcher.log")

    def wait_runtime(self, timeout: float = 60.0) -> int:
        return self.runtime.wait(timeout=timeout)

    def wait_watcher(self, timeout: float = 10.0) -> int:
        return self.watcher.wait(timeout=timeout)

    def read_log(self, path: Path) -> str:
        return path.read_text() if path is not None and path.exists() else ""

    def stop(self) -> None:
        for process in reversed(self._processes):
            if process.poll() is None:
                process.kill()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for handle in self._handles:
            handle.close()


def _episodes(output: Path) -> tuple[list[Path], list[Path]]:
    if not output.exists():
        return [], []
    return sorted(output.glob("episode-*")), sorted(output.glob(".partial-*"))


def _manifest(directory: Path) -> dict:
    return json.loads((directory / "manifest.json").read_text())


def _wait_for_file(path: Path, timeout: float = 30.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            return
        time.sleep(0.02)
    pytest.fail(f"Timed out waiting for {path}")


def test_once_completes_and_exits_zero(tmp_path):
    harness = _Harness(tmp_path)
    try:
        harness.start_backend()
        harness.start_runtime(once=True)
        assert harness.wait_runtime(timeout=60) == 0, harness.read_log(harness.runtime_log)
        committed, partial = _episodes(harness.output)
        assert len(committed) == 1
        assert partial == []
        manifest = _manifest(committed[0])
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "completed"
        assert manifest["task"]["outcome"] == "not_evaluated"
    finally:
        harness.stop()


@pytest.mark.parametrize(
    ("signal_number", "expected_code"),
    [(signal.SIGINT, 130), (signal.SIGTERM, 143)],
)
def test_action_interrupt_writes_interrupted_manifest(tmp_path, signal_number, expected_code):
    harness = _Harness(tmp_path)
    try:
        harness.start_backend()
        harness.start_runtime(once=True)
        harness.start_watcher(phase="ACTIVE", command=True, signal_number=signal_number)
        assert harness.wait_runtime(timeout=60) == expected_code, harness.read_log(
            harness.runtime_log
        )
        assert harness.wait_watcher() == 0, harness.read_log(harness.watcher_log)
        committed, partial = _episodes(harness.output)
        assert partial == []
        assert len(committed) == 1
        manifest = _manifest(committed[0])
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "interrupted"
        assert signal_number.name in manifest["runtime"]["reason"]
        assert manifest["task"]["outcome"] == "not_evaluated"
    finally:
        harness.stop()


def test_sigint_inside_finish_updates_runtime_and_repeats_safely(tmp_path):
    harness = _Harness(tmp_path)
    marker = tmp_path / "finish-entered"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            bootstrap=True,
            extra_environment={
                FINISH_HOLD_ENV: "2.0",
                FINISH_MARKER_ENV: str(marker),
            },
        )
        _wait_for_file(marker)
        assert harness.runtime.poll() is None
        harness.runtime.send_signal(signal.SIGINT)
        time.sleep(0.05)
        assert harness.runtime.poll() is None, "first signal ended the runtime too early"
        harness.runtime.send_signal(signal.SIGINT)
        assert harness.wait_runtime(timeout=60) == 130, harness.read_log(harness.runtime_log)
        committed, partial = _episodes(harness.output)
        assert partial == []
        assert len(committed) == 1
        manifest = _manifest(committed[0])
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "interrupted"
        assert "SIGINT" in manifest["runtime"]["reason"]
        assert manifest["task"]["outcome"] == "not_evaluated"
    finally:
        harness.stop()


def test_shutdown_timeout_is_nonzero_and_preserves_partial(tmp_path):
    harness = _Harness(tmp_path)
    marker = tmp_path / "finish-entered"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            shutdown_timeout=0.5,
            bootstrap=True,
            extra_environment={
                FINISH_HOLD_ENV: "10.0",
                FINISH_MARKER_ENV: str(marker),
            },
        )
        _wait_for_file(marker)
        harness.runtime.send_signal(signal.SIGINT)
        assert harness.wait_runtime(timeout=5) == 124
        log = harness.read_log(harness.runtime_log)
        assert "SHUTDOWN_TIMEOUT" in log
        committed, partial = _episodes(harness.output)
        assert committed == []
        assert len(partial) == 1
        assert (partial[0] / "recording").is_dir()
        assert not (partial[0] / "manifest.json").exists()
    finally:
        harness.stop()


def test_executor_exception_in_idle_exits_nonzero(tmp_path):
    harness = _Harness(tmp_path)
    try:
        harness.start_runtime(
            once=False,
            bootstrap=True,
            extra_environment={FAIL_SPIN_ENV: "5"},
        )
        assert harness.wait_runtime(timeout=20) == 1, harness.read_log(harness.runtime_log)
        log = harness.read_log(harness.runtime_log)
        assert "injected executor failure" in log
        assert "SHUTDOWN_TIMEOUT" not in log
    finally:
        harness.stop()


def test_shutdown_executor_failure_is_not_retried_and_preserves_record(tmp_path):
    harness = _Harness(tmp_path)
    finish_marker, spin_marker = tmp_path / "finish-entered", tmp_path / "spin-failures"
    try:
        harness.start_backend()
        harness.start_runtime(
            bootstrap=True,
            extra_environment={
                FINISH_HOLD_ENV: "2.0",
                FINISH_MARKER_ENV: str(finish_marker),
                "DSH003_TEST_FAIL_SHUTDOWN_SPIN": str(spin_marker),
            },
        )
        _wait_for_file(finish_marker)
        harness.runtime.send_signal(signal.SIGINT)
        assert harness.wait_runtime(timeout=10) == 130, harness.read_log(harness.runtime_log)
        assert spin_marker.read_text().splitlines() == ["failed-spin"]
        assert "SHUTDOWN_TIMEOUT" not in harness.read_log(harness.runtime_log)
        committed, partial = _episodes(harness.output)
        assert partial == []
        assert len(committed) == 1
        manifest = _manifest(committed[0])
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "failed"
        assert "SIGINT" in manifest["runtime"]["reason"]
        assert "ExternalShutdownException" in manifest["runtime"]["reason"]
        assert manifest["task"]["outcome"] == "not_evaluated"
    finally:
        harness.stop()


def test_status_and_finish_failures_do_not_skip_cleanup(tmp_path):
    harness = _Harness(tmp_path)
    cleanup_marker = tmp_path / "cleanup-ran"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            bootstrap=True,
            extra_environment={
                FAIL_STATUS_ENV: "ENDING",
                FAIL_FINISH_ENV: "1",
                CLEANUP_MARKER_ENV: str(cleanup_marker),
            },
        )
        assert harness.wait_runtime(timeout=60) == 1, harness.read_log(harness.runtime_log)
        _wait_for_file(cleanup_marker, timeout=5)
        log = harness.read_log(harness.runtime_log)
        assert "STATUS_ERROR: injected ENDING status failure" in log
        assert "RECORDING_ERROR: injected finish failure" in log
    finally:
        harness.stop()


def test_idle_sigterm_exits_143_without_episode(tmp_path):
    harness = _Harness(tmp_path)
    try:
        harness.start_runtime(once=False)
        harness.start_watcher(
            service="/runtime/episode/start",
            signal_number=signal.SIGTERM,
            timeout=30,
        )
        assert harness.wait_runtime(timeout=30) == 143, harness.read_log(harness.runtime_log)
        assert harness.wait_watcher() == 0, harness.read_log(harness.watcher_log)
        committed, partial = _episodes(harness.output)
        assert committed == []
        assert partial == []
    finally:
        harness.stop()


def test_preparing_sigint_exits_130_without_backend(tmp_path):
    harness = _Harness(tmp_path)
    try:
        harness.start_runtime(once=True)
        harness.start_watcher(phase="PREPARING", signal_number=signal.SIGINT, timeout=30)
        assert harness.wait_runtime(timeout=30) == 130, harness.read_log(harness.runtime_log)
        assert harness.wait_watcher() == 0, harness.read_log(harness.watcher_log)
        committed, partial = _episodes(harness.output)
        assert committed == []
        assert partial == []
    finally:
        harness.stop()


def _watch_main(arguments: list[str]) -> int:
    """Small ROS-side watcher used by the tests; the runtime under test is a peer."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--watch", action="store_true", required=True)
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--signal", default="SIGINT")
    parser.add_argument("--phase", action="append", default=[])
    parser.add_argument("--command", action="store_true")
    parser.add_argument("--service", default="")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(arguments)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from robot_api_msgs.msg import RuntimeStatus, TrajectoryCommand
    from robot_api_msgs.srv import StartEpisode
    from robot_ros_support.qos import EVENT_QOS, INFO_QOS

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("dsh003_shutdown_watcher")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    observed = {"phases": [], "command": False, "service": False}
    node.create_subscription(
        RuntimeStatus,
        "/runtime/status",
        lambda message: observed["phases"].append(message.phase),
        INFO_QOS,
    )
    if args.command:
        node.create_subscription(
            TrajectoryCommand,
            "/runtime/control/commands",
            lambda _: observed.update(command=True),
            EVENT_QOS,
        )
    client = None
    if args.service:
        client = node.create_client(StartEpisode, args.service)
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if client is not None and client.service_is_ready():
                observed["service"] = True
            phase_ready = not args.phase or bool(set(args.phase) & set(observed["phases"]))
            command_ready = not args.command or observed["command"]
            service_ready = not args.service or observed["service"]
            if phase_ready and command_ready and service_ready:
                os.kill(args.pid, getattr(signal, args.signal))
                print(json.dumps({"watcher": "sent", **observed}), flush=True)
                return 0
        print(json.dumps({"watcher": "timeout", **observed}), flush=True)
        return 2
    finally:
        if client is not None:
            node.destroy_client(client)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    if "--watch" in sys.argv[1:]:
        return _watch_main(sys.argv[1:])
    return pytest.main([__file__, *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
