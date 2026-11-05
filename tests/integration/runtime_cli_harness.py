"""Small real-ROS fixture for the human runtime CLI tests."""

from __future__ import annotations

import os
import random
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNTIME_SOURCE = ROOT / "runtime/ros2/ros/dual_arm_runtime"
DOUBLES_SOURCE = ROOT / "tests/doubles/ros/mock_robot_backend"
CONFIG = ROOT / "runtime/ros2/configs/mock_collect.yaml"
PROFILE = ROOT / "profiles/robots/mock_dual_arm_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas/v1"
RUN_ID = "manual-cli-test"
DEFAULT_DOMAIN_ID = random.SystemRandom().randint(1, 200)


def environment(domain_id: int) -> dict[str, str]:
    value = os.environ.copy()
    value["ROS_DOMAIN_ID"] = str(domain_id)
    value["PYTHONUNBUFFERED"] = "1"
    paths = [str(RUNTIME_SOURCE), str(DOUBLES_SOURCE)]
    if value.get("PYTHONPATH"):
        paths.append(value["PYTHONPATH"])
    value["PYTHONPATH"] = os.pathsep.join(paths)
    return value


class RuntimeCliHarness:
    """Own the backend/runtime subprocesses; stop() is safe in a test finally."""

    def __init__(
        self,
        tmp_path: Path,
        run_id: str = RUN_ID,
        domain_id: int = DEFAULT_DOMAIN_ID,
    ) -> None:
        self.tmp = tmp_path
        self.run_id = run_id
        self.raw_root = tmp_path / "raw"
        self.output = self.raw_root / run_id
        self.runs_root = tmp_path / "runs"
        self.domain_id = domain_id
        self.environment = environment(self.domain_id)
        self.backend = None
        self.runtime = None
        self.backend_log = None
        self.runtime_log = None
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

    def start_runtime(self) -> None:
        self.runtime, self.runtime_log = self._spawn(
            [
                sys.executable,
                "-m",
                "dual_arm_runtime.node",
                "--project-root",
                str(ROOT),
                "--config",
                str(CONFIG),
                "--output",
                str(self.output),
                "--run-id",
                self.run_id,
                "--runs-output",
                str(self.runs_root),
                "--shutdown-timeout",
                "15",
            ],
            "runtime.log",
        )

    def read_log(self, path: Path | None) -> str:
        return path.read_text() if path is not None and path.exists() else ""

    def stop_runtime(self) -> None:
        if self.runtime is not None and self.runtime.poll() is None:
            self.runtime.kill()
        if self.runtime is not None:
            try:
                self.runtime.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.runtime.kill()
                self.runtime.wait(timeout=5)

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


@contextmanager
def client_domain(harness: RuntimeCliHarness):
    """Run the in-process client on the same ROS domain as the harness processes."""
    previous = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(harness.domain_id)
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous


def run_cli(harness: RuntimeCliHarness, operation: str, timeout: float, **options):
    """Execute the real CLI in this process against the harness's raw root."""
    from dual_arm_runtime import cli

    argv = ["--run-id", harness.run_id, "--timeout", str(timeout), operation]
    for name, value in options.items():
        option = "--" + name.replace("_", "-")
        if isinstance(value, bool):
            if value:
                argv.append(option)
        else:
            argv.extend([option, str(value)])
    try:
        with client_domain(harness):
            return cli.execute(cli.parse_args(argv), raw_root=harness.raw_root)
    except cli.CliError as error:
        return {
            "operation": operation,
            "run_id": harness.run_id,
            "success": False,
            "reason": error.reason,
            "detail": error.detail,
            **error.fields,
        }


def wait_ready(harness: RuntimeCliHarness, timeout: float = 30.0) -> dict:
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = run_cli(harness, "status", min(2.0, max(0.2, deadline - time.monotonic())))
        if last.get("success"):
            return last
        time.sleep(0.1)
    raise AssertionError(
        f"runtime CLI never became ready: {last}\n{harness.read_log(harness.runtime_log)}"
    )


def wait_active(harness: RuntimeCliHarness, episode_id: str, timeout: float = 15.0) -> dict:
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = run_cli(harness, "status", min(2.0, max(0.2, deadline - time.monotonic())))
        status = last.get("status") or {}
        if status.get("episode_id") == episode_id and status.get("phase") == "ACTIVE":
            return last
        if status.get("episode_id") == episode_id and status.get("phase") == "FAULT":
            raise AssertionError(f"episode faulted before end: {last}")
        time.sleep(0.05)
    raise AssertionError(f"episode never became ACTIVE: {last}")
