"""Real ROS/DDS integration for the native runtime and its v3 MCAP audit.

Run inside the supervisor's fixed Docker/EGL image.  The test starts the real
DSH-027 backend and the real native runtime process, waits for a configured
diagnostic motion, then audits the committed recording independently.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest
import yaml

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from robot_api_msgs.msg import RuntimeStatus
    from robot_api_msgs.srv import EndEpisode, StartEpisode
    from robot_ros_support.qos import INFO_QOS
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[2]
RUNTIME_SRC = ROOT / "components/runtime/ros/dual_arm_runtime"
CONTRACTS_SRC = ROOT / "contracts/python/src"
ROS_SUPPORT_SRC = ROOT / "libraries/ros_support/src"
RUN_LOGGING_SRC = ROOT / "libraries/run_logging/src"
BACKEND_SRC = ROOT / "components/backend_sim/src"
ASSETS_ROOT = Path(os.environ.get("SIM_ASSETS_ROOT", ROOT / "third_party/src/mujoco_menagerie"))
SUMMARY = Path(os.environ.get("SIM_ASSET_SUMMARY", ROOT / ".build/assets/panda.json"))
SOURCE = ROOT / "third_party/panda-source.json"
DOMAIN = os.environ.get("ROS_DOMAIN_ID", "77")


def _pythonpath(*sources: Path) -> str:
    return os.pathsep.join([*(str(source) for source in sources), os.environ.get("PYTHONPATH", "")])


def _absolute_config(
    tmp_path: Path,
    source: Path,
    *,
    native: bool,
    overrides: dict | None = None,
) -> Path:
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if native:
        for field in (
            "robot_profile",
            "observation_schema",
            "action_schema",
            "control_constraints",
        ):
            document[field] = str((source.parent / document[field]).resolve())
        if "follower_constraints" in document:
            document["follower_constraints"] = str(
                (source.parent / document["follower_constraints"]).resolve()
            )
        document["robot_namespace"] = "/robot"
        if overrides:
            document.update(overrides)
    else:
        document["robot"].update(
            {
                "namespace": "/robot",
                "profile": str(ROOT / "profiles/robots/panda_stack_v1/profile.json"),
                "schemas": str(ROOT / "contracts/schemas"),
                "scene": str(ROOT / "configs/simulation/panda_stack.yaml"),
                "control": str(ROOT / "configs/simulation/panda_control.yaml"),
                "stream": str(ROOT / "configs/simulation/panda_stream.yaml"),
            }
        )
        if document["robot"].get("follower"):
            document["robot"]["follower"] = str(
                (source.parent / document["robot"]["follower"]).resolve()
            )
        document["run"]["realtime_speed"] = 1.0
    path = tmp_path / ("native.yaml" if native else "backend.yaml")
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def _env(*sources: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONPATH"] = _pythonpath(*sources)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["ROS_LOCALHOST_ONLY"] = "0"
    env["RMW_IMPLEMENTATION"] = env.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    env["PYTHONUNBUFFERED"] = "1"
    return env


class RuntimeClient:
    def __init__(self) -> None:
        self.node = Node(f"native_runtime_test_{uuid.uuid4().hex[:8]}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.statuses: list[RuntimeStatus] = []
        self.node.create_subscription(
            RuntimeStatus, "/runtime/status", self.statuses.append, INFO_QOS
        )
        self.start_client = self.node.create_client(StartEpisode, "/runtime/episode/start")
        self.end_client = self.node.create_client(EndEpisode, "/runtime/episode/end")
        self.process = None
        self.output = lambda: ""

    def attach(self, process, output) -> None:
        self.process = process
        self.output = output

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def wait_phase(self, phases: set[str], timeout: float = 60.0) -> RuntimeStatus:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process is not None and self.process.poll() is not None:
                raise AssertionError(f"process exited before status {phases}: {self.output()}")
            self.spin(0.05)
            if self.statuses and self.statuses[-1].phase in phases:
                return self.statuses[-1]
        raise AssertionError(
            f"status never reached {phases}: last={self.statuses[-1] if self.statuses else None}"
        )

    def call(self, client, request, timeout: float = 20.0):
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise AssertionError("service unavailable")
        future = client.call_async(request)
        while time.monotonic() < deadline:
            self.spin(0.02)
            if future.done():
                return future.result()
        raise AssertionError("service call timed out")

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()


class Processes:
    def __init__(
        self,
        tmp_path: Path,
        *,
        native_overrides: dict | None = None,
        backend_source: Path = ROOT / "configs/simulation/backend.yaml",
        runtime_source: Path = ROOT / "configs/runtime/native_collect.yaml",
    ) -> None:
        self.tmp_path = tmp_path
        self.runtime_source = runtime_source
        self.native_overrides = native_overrides
        self.backend_log = (tmp_path / "backend.log").open("wb")
        self.runtime_log = (tmp_path / "runtime.log").open("wb")
        self.backend = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "robot_backend_sim.node",
                "--config",
                str(_absolute_config(tmp_path, backend_source, native=False)),
                "--assets-root",
                str(ASSETS_ROOT),
                "--asset-summary",
                str(SUMMARY),
                "--asset-source",
                str(SOURCE),
                "--run-root",
                str(tmp_path / "backend-runs"),
                "--run-id",
                f"backend-{uuid.uuid4().hex[:8]}",
            ],
            cwd=str(ROOT),
            env=_env(BACKEND_SRC, CONTRACTS_SRC, ROS_SUPPORT_SRC, RUN_LOGGING_SRC),
            stdout=self.backend_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.runtime: subprocess.Popen | None = None
        self.run_id = f"native-{uuid.uuid4().hex[:8]}"
        self.output = tmp_path / "raw"

    def start_runtime(self, *, once: bool) -> subprocess.Popen:
        argv = [
            sys.executable,
            "-m",
            "dual_arm_runtime.native_node",
            "--project-root",
            str(ROOT),
            "--config",
            str(
                _absolute_config(
                    self.tmp_path,
                    self.runtime_source,
                    native=True,
                    overrides=self.native_overrides,
                )
            ),
            "--output",
            str(self.output),
            "--runs-output",
            str(self.tmp_path / "runtime-runs"),
            "--run-id",
            self.run_id,
            "--shutdown-timeout",
            "20",
        ]
        if once:
            argv.append("--once")
        self.runtime = subprocess.Popen(
            argv,
            cwd=str(ROOT),
            env=_env(RUNTIME_SRC, CONTRACTS_SRC, ROS_SUPPORT_SRC, RUN_LOGGING_SRC),
            stdout=self.runtime_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        return self.runtime

    def _output(self) -> str:
        self.runtime_log.flush()
        self.backend_log.flush()
        runtime = ""
        backend = ""
        try:
            runtime = (self.tmp_path / "runtime.log").read_text(encoding="utf-8", errors="replace")
        except OSError:
            pass
        try:
            backend = (self.tmp_path / "backend.log").read_text(encoding="utf-8", errors="replace")
        except OSError:
            pass
        return f"runtime:\n{runtime[-8000:]}\nbackend:\n{backend[-4000:]}"

    def stop(self) -> None:
        for process in (self.runtime, self.backend):
            if process is None or process.poll() is not None:
                continue
            os.killpg(process.pid, signal.SIGTERM)
        for process in (self.runtime, self.backend):
            if process is None:
                continue
            try:
                process.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=10)
        self.runtime_log.close()
        self.backend_log.close()

    def failed(self, process: subprocess.Popen | None) -> None:
        code = None if process is None else process.poll()
        raise AssertionError(f"process failed (exit={code}):\n{self._output()}")


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _runtime_json(log_path: Path) -> tuple[dict, dict]:
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    episode = None
    summary = None
    for line in lines:
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and "recording" in value and "runtime" in value:
            episode = value
        if isinstance(value, dict) and "run_id" in value and "record_directory" in value:
            summary = value
    assert episode is not None, lines[-20:]
    assert summary is not None, lines[-20:]
    return episode, summary


def test_normal_diagnostic_motion_and_independent_v3_audit(tmp_path):
    from dual_arm_runtime.audit import inspect_episode

    processes = Processes(tmp_path)
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=True)
        client.attach(runtime, processes._output)
        try:
            code = runtime.wait(timeout=180)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        assert code == 0, processes._output()
        episode, summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == "completed", processes._output()
        assert episode["recording"]["complete"] is True, processes._output()
        assert episode["task"]["outcome"] == "not_evaluated"
        directory = Path(summary["record_directory"])
        assert directory.is_dir()
        report = inspect_episode(directory, ROOT / "contracts/schemas")
        assert report["schema_version"] == 3
        assert report["native_format"] == "v3"
        assert report["recording"]["complete"] is True
        assert report["runtime"]["outcome"] == "completed"
        assert report["task"]["outcome"] == "not_evaluated"
        assert report["configured_motions"] == 2
        assert report["commands"] >= 2
        assert report["completed_commands"] >= 2
        assert report["accepted_references"] >= 2
        assert report["max_joint_motion"] > 0.01
        manifest = json.loads((directory / "manifest.json").read_text())
        configuration = manifest["configuration"]
        assert configuration["reset_used"] is True
        assert configuration["reset_seed"] == 0
        assert configuration["recipe"]["control_constraints"]["max_accelerations"]
        assert len(configuration["recipe"]["motions"][0]["positions"]) == 16
    finally:
        client.close()
        processes.stop()


def test_end_episode_stops_runtime_but_keeps_complete_recording(tmp_path):
    from dual_arm_runtime.audit import inspect_episode

    processes = Processes(tmp_path)
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=False)
        client.attach(runtime, processes._output)
        client.wait_phase({"IDLE"}, timeout=60)
        response = client.call(client.start_client, StartEpisode.Request(task_id="native-test"))
        assert response.success, response.reason
        active = client.wait_phase({"ACTIVE", "FAULT", "IDLE"}, timeout=90)
        assert active.phase == "ACTIVE", processes._output()
        time.sleep(0.2)
        stopped = client.call(client.end_client, EndEpisode.Request(reason="TEST_END"))
        assert stopped.success, stopped.reason
        client.wait_phase({"IDLE", "FAULT"}, timeout=90)
        if runtime.poll() is None:
            os.killpg(runtime.pid, signal.SIGTERM)
        try:
            code = runtime.wait(timeout=60)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        assert code in (0, 143), processes._output()
        episode, summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == "stopped", processes._output()
        assert episode["recording"]["complete"] is True, processes._output()
        report = inspect_episode(Path(summary["record_directory"]), ROOT / "contracts/schemas")
        assert report["recording"]["complete"] is True
        assert report["runtime"]["outcome"] == "stopped"
        assert report["task"]["outcome"] == "not_evaluated"
    finally:
        client.close()
        processes.stop()


def test_signal_interruption_keeps_complete_recording_and_exit_143(tmp_path):
    from dual_arm_runtime.audit import inspect_episode

    processes = Processes(tmp_path)
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=False)
        client.attach(runtime, processes._output)
        client.wait_phase({"IDLE"}, timeout=60)
        response = client.call(client.start_client, StartEpisode.Request(task_id="native-signal"))
        assert response.success, response.reason
        active = client.wait_phase({"ACTIVE", "FAULT", "IDLE"}, timeout=90)
        assert active.phase == "ACTIVE", processes._output()
        time.sleep(0.3)
        os.killpg(runtime.pid, signal.SIGTERM)
        try:
            code = runtime.wait(timeout=90)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        assert code == 143, processes._output()
        episode, summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == "interrupted", processes._output()
        assert episode["recording"]["complete"] is True, processes._output()
        report = inspect_episode(Path(summary["record_directory"]), ROOT / "contracts/schemas")
        assert report["recording"]["complete"] is True
        assert report["runtime"]["outcome"] == "interrupted"
        assert report["task"]["outcome"] == "not_evaluated"
    finally:
        client.close()
        processes.stop()


def test_backend_disconnect_is_reported_and_does_not_fake_success(tmp_path):
    processes = Processes(tmp_path)
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=False)
        client.attach(runtime, processes._output)
        client.wait_phase({"IDLE"}, timeout=60)
        response = client.call(client.start_client, StartEpisode.Request(task_id="backend-loss"))
        assert response.success, response.reason
        active = client.wait_phase({"ACTIVE", "FAULT", "IDLE"}, timeout=90)
        assert active.phase == "ACTIVE", processes._output()
        os.killpg(processes.backend.pid, signal.SIGKILL)
        processes.backend.wait(timeout=20)
        client.wait_phase({"FAULT"}, timeout=60)
        if runtime.poll() is None:
            os.killpg(runtime.pid, signal.SIGTERM)
        try:
            runtime.wait(timeout=60)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        episode, _summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == "failed", processes._output()
        assert episode["recording"]["complete"] in (True, False)
    finally:
        client.close()
        processes.stop()


def test_lease_expiry_while_process_is_paused_is_not_faked_as_success(tmp_path):
    processes = Processes(tmp_path)
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=False)
        client.attach(runtime, processes._output)
        client.wait_phase({"IDLE"}, timeout=60)
        response = client.call(client.start_client, StartEpisode.Request(task_id="lease-loss"))
        assert response.success, response.reason
        active = client.wait_phase({"ACTIVE", "FAULT", "IDLE"}, timeout=90)
        assert active.phase == "ACTIVE", processes._output()
        os.kill(runtime.pid, signal.SIGSTOP)
        time.sleep(3.0)
        os.kill(runtime.pid, signal.SIGCONT)
        try:
            client.wait_phase({"FAULT"}, timeout=60)
        finally:
            if runtime.poll() is None:
                os.killpg(runtime.pid, signal.SIGTERM)
        try:
            runtime.wait(timeout=60)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        episode, _summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == "failed", processes._output()
        assert episode["task"]["outcome"] == "not_evaluated"
        assert episode["recording"]["complete"] in (True, False)
    finally:
        client.close()
        processes.stop()


def test_writer_backpressure_fault_leaves_partial_evidence(tmp_path):
    processes = Processes(
        tmp_path,
        native_overrides={"recording": {"queue_capacity": 1, "max_pending_bytes": 1024}},
    )
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=True)
        client.attach(runtime, processes._output)
        try:
            code = runtime.wait(timeout=120)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        assert code != 0, processes._output()
        episode, _summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["recording"]["complete"] is False, processes._output()
        assert episode["runtime"]["outcome"] == "failed", processes._output()
        assert list(processes.output.glob(".partial-episode-*")), processes._output()
    finally:
        client.close()
        processes.stop()


def test_logger_failure_exits_nonzero_without_success_manifest(tmp_path):
    processes = Processes(
        tmp_path,
        native_overrides={
            "logging": {
                "queue_capacity": 1,
                "max_event_bytes": 64,
                "max_file_bytes": 128,
                "max_total_bytes": 1,
                "close_timeout_s": 1.0,
                "flush_interval_s": 0.01,
                "console": False,
            }
        },
    )
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=True)
        client.attach(runtime, processes._output)
        try:
            code = runtime.wait(timeout=120)
        except subprocess.TimeoutExpired:
            processes.failed(runtime)
        assert code != 0, processes._output()
        assert not list(processes.output.glob("episode-*/manifest.json")), processes._output()
    finally:
        client.close()
        processes.stop()


def test_explicit_episode_identity_reaches_recorded_context(tmp_path, monkeypatch):
    """One small diagnostic, never a full grasp/stack pilot."""
    episode_id = "component-requested-episode"
    monkeypatch.setenv("NATIVE_EPISODE_ID", episode_id)
    processes = Processes(tmp_path)
    try:
        process = processes.start_runtime(once=True)
        assert process.wait(timeout=40) == 0, processes._output()
        episode, _ = _runtime_json(tmp_path / "runtime.log")
        assert episode["episode_id"] == episode_id
        manifest = json.loads((Path(episode["directory"]) / "manifest.json").read_text())
        assert manifest["configuration"]["run_context"]["episode_id"] == episode_id
    finally:
        processes.stop()
