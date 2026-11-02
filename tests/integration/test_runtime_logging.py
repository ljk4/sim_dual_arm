"""Real ROS runtime checks for event correlation, metadata and log failures.

The supervisor runs this file in the Jazzy test image. Fault injection lives in
``runtime_shutdown_bootstrap.py`` and only replaces EventLogger/ControlClient
boundaries; production code has no test switches.
"""

from __future__ import annotations

import json
import signal
import sys
import time
from itertools import pairwise
from pathlib import Path

import jsonschema
import pytest
from dual_arm_runtime.audit import inspect_episode
from test_runtime_shutdown import CONFIG, ROOT, SCHEMAS, _episodes, _Harness, _manifest

SCHEMA = json.loads((SCHEMAS / "run_event.schema.json").read_text(encoding="utf-8"))
VALIDATOR = jsonschema.Draft202012Validator(SCHEMA)
MANIFEST_SCHEMAS = ROOT / "contracts/schemas"
FAIL_WRITE_ENV = "DSH008_TEST_FAIL_WRITE_EVENT"
FAIL_EMIT_ENV = "DSH008_TEST_FAIL_EMIT_EVENT"
FAIL_EMIT_AFTER_ENV = "DSH008_TEST_FAIL_EMIT_AFTER"
CLOSE_ERROR_ENV = "DSH008_TEST_CLOSE_ERROR"
METADATA_ERROR_ENV = "DSH008_TEST_METADATA_ERROR"
COMMAND_MARKER_ENV = "DSH008_TEST_COMMAND_MARKER"
CONTROL_CALL_MARKER_ENV = "DSH008_TEST_CONTROL_CALL_MARKER"
FAIL_CONTROL_CALL_ENV = "DSH008_TEST_FAIL_CONTROL_CALL"


class _LoggingHarness(_Harness):
    """The shutdown harness with deployment run arguments and a ROS driver."""

    def start_runtime(
        self,
        *,
        once: bool = True,
        shutdown_timeout: float = 15.0,
        extra_environment: dict[str, str] | None = None,
        bootstrap: bool = True,
        run_id: str | None = None,
        runs_output: Path | None = None,
    ) -> None:
        if extra_environment:
            self.environment.update(extra_environment)
        command = (
            [sys.executable, str(ROOT / "tests/integration/runtime_shutdown_bootstrap.py")]
            if bootstrap
            else [sys.executable, "-m", "dual_arm_runtime.node"]
        )
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
        if run_id is not None:
            arguments.extend(["--run-id", run_id])
        if runs_output is not None:
            arguments.extend(["--runs-output", str(runs_output)])
        if once:
            arguments.append("--once")
        self.runtime, self.runtime_log = self._spawn(arguments, "runtime.log")

    def start_driver(self) -> None:
        self.driver, self.driver_log = self._spawn(
            [sys.executable, str(Path(__file__).resolve()), "--drive-episodes"],
            "driver.log",
        )


def _run_dir(harness: _LoggingHarness, run_id: str) -> Path:
    return harness.output.parent / "runs" / run_id


def _event_files(run_dir: Path) -> list[Path]:
    return sorted(run_dir.glob("logs/runtime/*/events.*.jsonl"))


def _events(run_dir: Path) -> list[dict[str, object]]:
    events = []
    for path in _event_files(run_dir):
        for line in path.read_bytes().splitlines():
            events.append(json.loads(line))
    return events


def _logs(run_dir: Path) -> list[Path]:
    return sorted(run_dir.glob("logs/runtime/*/summary.json"))


def _summary(run_dir: Path) -> dict[str, object]:
    paths = _logs(run_dir)
    assert len(paths) == 1, paths
    return json.loads(paths[0].read_text(encoding="utf-8"))


def _read_json_lines(path: Path) -> list[str]:
    if not path.exists():
        return []
    return path.read_text(encoding="utf-8").splitlines()


def _assert_event_schema(run_dir: Path) -> list[dict[str, object]]:
    events = _events(run_dir)
    assert events
    for event in events:
        VALIDATOR.validate(event)
    for previous, current in pairwise(events):
        assert current["event_seq"] == previous["event_seq"] + 1
    return events


def _events_named(events, name: str) -> list[dict[str, object]]:
    return [event for event in events if event["event"] == name]


def _final_summary(harness: _LoggingHarness) -> dict[str, object]:
    for line in reversed(harness.read_log(harness.runtime_log).splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and "process_outcome" in value:
            return value
    raise AssertionError("runtime final summary not found")


def test_once_records_command_correlation_and_run_metadata(tmp_path):
    harness = _LoggingHarness(tmp_path)
    run_id = "run-once"
    try:
        harness.start_backend()
        harness.start_runtime(once=True, run_id=run_id)
        assert harness.wait_runtime(timeout=60) == 0, harness.read_log(harness.runtime_log)

        run_dir = _run_dir(harness, run_id)
        events = _assert_event_schema(run_dir)
        started = _events_named(events, "run.started")
        finalizing = _events_named(events, "run.finalizing")
        assert len(started) == len(finalizing) == 1
        assert started[0]["fields"]["run_directory"] == str(run_dir)
        assert finalizing[0]["fields"]["exit_code_before_log_close"] == 0
        assert finalizing[0]["fields"]["log_close_pending"] is True
        assert finalizing[0]["reason"] == "CLOSE_PENDING"
        assert finalizing[0]["level"] == "INFO"
        assert _events_named(events, "run.finished") == []
        final_summary = _final_summary(harness)
        assert final_summary["exit_code"] == 0
        assert final_summary["process_outcome"] == "succeeded"
        assert final_summary["logging_closed"] is True

        episode = _events_named(events, "episode.finished")
        assert len(episode) == 1
        episode_id = episode[0]["episode_id"]
        assert episode[0]["fields"]["recording_complete"] is True
        assert episode[0]["fields"]["runtime_outcome"] == "completed"
        assert episode[0]["fields"]["task_outcome"] == "not_evaluated"

        sending = _events_named(events, "command.sending")
        assert len(sending) == 3
        for event in sending:
            assert event["episode_id"] == episode_id
            assert event["command_id"].startswith(f"{episode_id}-")
            assert event["clock_epoch"] is not None
            assert isinstance(event["fields"]["observation_id"], int)
            assert event["fields"]["command_seq"] in (1, 2, 3)
            assert "positions" not in event["fields"]

        assert len(_events_named(events, "command.action_accepted")) == 3
        assert len(_events_named(events, "command.action_result")) == 3
        assert len(_events_named(events, "command.accepted")) == 3
        assert len(_events_named(events, "command.started")) == 3
        assert len(_events_named(events, "command.completed")) == 3

        committed, partial = _episodes(harness.output)
        assert len(committed) == 1
        assert partial == []
        manifest = _manifest(committed[0])
        assert manifest["schema_version"] == 2
        assert manifest["run_id"] == run_id
        assert manifest["source"] == "mock"
        assert manifest["test_only"] is True
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "completed"
        assert manifest["task"]["outcome"] == "not_evaluated"
        assert manifest["configuration"]["run_id"] == run_id
        assert manifest["configuration"]["run_context"]["episode_id"] == episode_id
        assert manifest["configuration"]["recipe"]["logging"]["queue_capacity"] == 256
        report = inspect_episode(committed[0], MANIFEST_SCHEMAS)
        assert report["recording"]["complete"] is True
        assert report["runtime"]["outcome"] == "completed"
        assert report["task"]["outcome"] == "not_evaluated"
        assert report["commands_without_terminal"] == 0
        assert report["terminal_events"] == 3
        # Mock demonstration still requires real joint motion; this is no longer
        # a generic-audit requirement for stopped/static v2 recordings.
        assert report["max_joint_motion"] > 1e-6
        assert _summary(run_dir)["state"] == "complete"
        assert not (run_dir / "status.json").exists()
    finally:
        harness.stop()


@pytest.mark.parametrize(
    ("signal_number", "expected_code"),
    [(signal.SIGINT, 130), (signal.SIGTERM, 143)],
)
def test_signal_records_stopping_and_finished_events(tmp_path, signal_number, expected_code):
    harness = _LoggingHarness(tmp_path)
    run_id = "run-signal"
    try:
        harness.start_backend()
        harness.start_runtime(once=True, run_id=run_id)
        harness.start_watcher(phase="ACTIVE", signal_number=signal_number, timeout=60)
        assert harness.wait_runtime(timeout=60) == expected_code, harness.read_log(
            harness.runtime_log
        )
        assert harness.wait_watcher() == 0, harness.read_log(harness.watcher_log)

        run_dir = _run_dir(harness, run_id)
        events = _assert_event_schema(run_dir)
        stopping = _events_named(events, "run.stopping")
        finalizing = _events_named(events, "run.finalizing")
        assert len(stopping) == 1
        assert stopping[0]["reason"] == signal_number.name
        assert len(finalizing) == 1
        assert finalizing[0]["fields"]["exit_code_before_log_close"] == expected_code
        assert finalizing[0]["fields"]["log_close_pending"] is True
        assert finalizing[0]["reason"] == signal_number.name
        assert _events_named(events, "run.finished") == []
        final_summary = _final_summary(harness)
        assert final_summary["exit_code"] == expected_code
        assert final_summary["process_outcome"] == "interrupted"
        assert final_summary["logging_closed"] is True
        assert _summary(run_dir)["state"] == "complete"
    finally:
        harness.stop()


def test_two_episodes_share_run_without_context_leak(tmp_path):
    harness = _LoggingHarness(tmp_path)
    run_id = "run-two-episodes"
    try:
        harness.start_backend()
        harness.start_runtime(once=False, run_id=run_id)
        harness.start_driver()
        assert harness.driver.wait(timeout=60) == 0, harness.read_log(harness.driver_log)
        result_lines = [
            line
            for line in _read_json_lines(harness.driver_log)
            if line.startswith("DSH008_DRIVER_RESULT ")
        ]
        assert len(result_lines) == 1, harness.read_log(harness.driver_log)
        result = json.loads(result_lines[0].removeprefix("DSH008_DRIVER_RESULT "))
        episode_ids = result["episodes"]
        assert len(episode_ids) == 2
        assert len(set(episode_ids)) == 2

        harness.runtime.send_signal(signal.SIGTERM)
        assert harness.wait_runtime(timeout=30) == 143, harness.read_log(harness.runtime_log)

        run_dir = _run_dir(harness, run_id)
        events = _assert_event_schema(run_dir)
        assert len(_events_named(events, "run.started")) == 1
        finalizing = _events_named(events, "run.finalizing")
        assert len(finalizing) == 1
        assert finalizing[0]["fields"]["exit_code_before_log_close"] == 143
        assert _events_named(events, "run.finished") == []
        assert _final_summary(harness)["process_outcome"] == "interrupted"
        assert len(_events_named(events, "episode.preparing")) == 2
        assert len(_events_named(events, "episode.finished")) == 2

        committed, partial = _episodes(harness.output)
        assert partial == []
        assert len(committed) == 2
        manifests = {_manifest(path)["episode_id"]: _manifest(path) for path in committed}
        assert set(manifests) == set(episode_ids)
        for episode_id, manifest in manifests.items():
            assert manifest["recording"]["complete"] is True
            assert manifest["runtime"]["outcome"] == "completed"
            assert manifest["task"]["outcome"] == "not_evaluated"
            assert manifest["configuration"]["run_id"] == run_id
            assert manifest["configuration"]["run_context"]["episode_id"] == episode_id

        # Every command is correlated with exactly one episode, and the first
        # episode's finish precedes the second episode's preparation.
        by_episode = {episode_id: [] for episode_id in episode_ids}
        for event in events:
            if event["event"] == "command.sending":
                by_episode[event["episode_id"]].append(event)
        assert {episode_id: len(commands) for episode_id, commands in by_episode.items()} == {
            episode_ids[0]: 3,
            episode_ids[1]: 3,
        }
        first_finish = next(
            index
            for index, event in enumerate(events)
            if event["event"] == "episode.finished" and event["episode_id"] == episode_ids[0]
        )
        second_prepare = next(
            index
            for index, event in enumerate(events)
            if event["event"] == "episode.preparing" and event["episode_id"] == episode_ids[1]
        )
        assert first_finish < second_prepare
    finally:
        harness.stop()


def test_async_writer_failure_stops_commands_and_still_cleans_up(tmp_path):
    harness = _LoggingHarness(tmp_path)
    commands = tmp_path / "commands.txt"
    calls = tmp_path / "control-calls.txt"
    cleanup = tmp_path / "cleanup-ran"
    run_id = "run-async-failure"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            run_id=run_id,
            extra_environment={
                FAIL_WRITE_ENV: "command.sending",
                COMMAND_MARKER_ENV: str(commands),
                CONTROL_CALL_MARKER_ENV: str(calls),
                "DSH003_TEST_CLEANUP_MARKER": str(cleanup),
            },
        )
        assert harness.wait_runtime(timeout=60) == 1, harness.read_log(harness.runtime_log)
        assert len(_read_json_lines(commands)) == 1
        attempted = _read_json_lines(calls)
        assert "stop" in attempted
        assert "release_control" in attempted
        assert cleanup.exists()
        summary = _summary(_run_dir(harness, run_id))
        assert summary["state"] == "failed"
        events = _events(_run_dir(harness, run_id))
        assert _events_named(events, "run.finished") == []
        assert _events_named(events, "run.finalizing") == []
    finally:
        harness.stop()


def test_critical_emit_failure_stops_next_command_and_still_cleans_up(tmp_path):
    harness = _LoggingHarness(tmp_path)
    commands = tmp_path / "commands.txt"
    calls = tmp_path / "control-calls.txt"
    cleanup = tmp_path / "cleanup-ran"
    run_id = "run-emit-failure"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            run_id=run_id,
            extra_environment={
                FAIL_EMIT_ENV: "command.sending",
                FAIL_EMIT_AFTER_ENV: "2",
                COMMAND_MARKER_ENV: str(commands),
                CONTROL_CALL_MARKER_ENV: str(calls),
                "DSH003_TEST_CLEANUP_MARKER": str(cleanup),
            },
        )
        assert harness.wait_runtime(timeout=60) == 1, harness.read_log(harness.runtime_log)
        assert len(_read_json_lines(commands)) == 1
        attempted = _read_json_lines(calls)
        assert "stop" in attempted
        assert "release_control" in attempted
        assert cleanup.exists()
        run_dir = _run_dir(harness, run_id)
        events = _assert_event_schema(run_dir)
        assert len(_events_named(events, "command.sending")) == 1
        assert len(_events_named(events, "command.action_accepted")) == 1
        finalizing = _events_named(events, "run.finalizing")
        assert len(finalizing) == 1
        assert finalizing[0]["reason"] == "RUNTIME_FAILURE"
        assert finalizing[0]["level"] == "ERROR"
        assert finalizing[0]["fields"]["exit_code_before_log_close"] == 1
        assert _events_named(events, "run.finished") == []
        final_summary = _final_summary(harness)
        assert final_summary["exit_code"] == 1
        assert final_summary["process_outcome"] == "failed"
        assert final_summary["logging_closed"] is True
        assert _summary(run_dir)["state"] == "complete"
    finally:
        harness.stop()


def test_idle_log_failure_exits_nonzero_without_episode(tmp_path):
    harness = _LoggingHarness(tmp_path)
    run_id = "run-idle-failure"
    try:
        harness.start_runtime(
            once=False,
            run_id=run_id,
            extra_environment={FAIL_WRITE_ENV: "run.started"},
        )
        assert harness.wait_runtime(timeout=20) == 1, harness.read_log(harness.runtime_log)
        run_dir = _run_dir(harness, run_id)
        assert _summary(run_dir)["state"] == "failed"
        committed, partial = _episodes(harness.output)
        assert committed == []
        assert partial == []
    finally:
        harness.stop()


def test_control_failure_is_error_and_finalization_never_claims_complete(tmp_path):
    harness = _LoggingHarness(tmp_path)
    calls = tmp_path / "control-calls.txt"
    run_id = "run-control-failure"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            run_id=run_id,
            extra_environment={
                FAIL_CONTROL_CALL_ENV: "stop",
                CONTROL_CALL_MARKER_ENV: str(calls),
            },
        )
        assert harness.wait_runtime(timeout=60) == 1, harness.read_log(harness.runtime_log)
        attempted = _read_json_lines(calls)
        assert "stop" in attempted
        assert "release_control" in attempted
        run_dir = _run_dir(harness, run_id)
        events = _assert_event_schema(run_dir)
        stop_failed = _events_named(events, "control.stop_failed")
        assert len(stop_failed) == 1
        assert stop_failed[0]["level"] == "ERROR"
        assert stop_failed[0]["reason"] == "CONTROL_STOP_FAILED"
        assert stop_failed[0]["exception"]["type"] == "RuntimeError"
        assert "injected stop failure" in stop_failed[0]["exception"]["message"]
        episode_finished = _events_named(events, "episode.finished")
        assert len(episode_finished) == 1
        assert episode_finished[0]["level"] == "ERROR"
        assert episode_finished[0]["reason"] == "EPISODE_FAILED"
        assert episode_finished[0]["fields"]["recording_complete"] is True
        assert episode_finished[0]["fields"]["runtime_outcome"] == "failed"
        assert episode_finished[0]["fields"]["task_outcome"] == "not_evaluated"
        stopping = _events_named(events, "run.stopping")
        finalizing = _events_named(events, "run.finalizing")
        assert len(stopping) == len(finalizing) == 1
        assert stopping[0]["reason"] == "RUNTIME_FAILURE"
        assert finalizing[0]["reason"] == "RUNTIME_FAILURE"
        assert finalizing[0]["level"] == "ERROR"
        assert _events_named(events, "run.finished") == []
        final_summary = _final_summary(harness)
        assert final_summary["exit_code"] == 1
        assert final_summary["process_outcome"] == "failed"
        assert final_summary["logging_closed"] is True
        assert _summary(run_dir)["state"] == "complete"
    finally:
        harness.stop()


def test_close_failure_alone_changes_normal_exit_to_one(tmp_path):
    harness = _LoggingHarness(tmp_path)
    run_id = "run-close-failure"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=True,
            run_id=run_id,
            extra_environment={CLOSE_ERROR_ENV: "1"},
        )
        assert harness.wait_runtime(timeout=60) == 1, harness.read_log(harness.runtime_log)
        run_dir = _run_dir(harness, run_id)
        events = _events(run_dir)
        finalizing = _events_named(events, "run.finalizing")
        assert len(finalizing) == 1
        assert finalizing[0]["fields"]["exit_code_before_log_close"] == 0
        assert finalizing[0]["fields"]["log_close_pending"] is True
        assert _events_named(events, "run.finished") == []
        final_summary = _final_summary(harness)
        assert final_summary["exit_code"] == 1
        assert final_summary["process_outcome"] == "failed"
        assert final_summary["logging_closed"] is False
        assert _summary(run_dir)["state"] == "complete"
        committed, partial = _episodes(harness.output)
        assert partial == []
        assert len(committed) == 1
        manifest = _manifest(committed[0])
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "completed"
        assert manifest["task"]["outcome"] == "not_evaluated"
        log = harness.read_log(harness.runtime_log)
        assert '"logging_error"' in log
        assert "close" in log
    finally:
        harness.stop()


def test_initialization_failure_leaves_no_completion_evidence(tmp_path):
    harness = _LoggingHarness(tmp_path)
    run_id = "run-init-failure"
    try:
        harness.start_runtime(
            once=True,
            run_id=run_id,
            extra_environment={METADATA_ERROR_ENV: "1"},
        )
        assert harness.wait_runtime(timeout=20) == 1, harness.read_log(harness.runtime_log)
        run_dir = _run_dir(harness, run_id)
        assert run_dir.is_dir()
        assert not (run_dir / "logs").exists()
        assert not list(run_dir.glob("**/summary.json"))
        assert not (run_dir / "status.json").exists()
        assert "runtime_initialization_error" in harness.read_log(harness.runtime_log)
    finally:
        harness.stop()


def _drive_episodes() -> int:
    """Service driver used by the two-episode integration test."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from robot_api_msgs.msg import RuntimeStatus
    from robot_api_msgs.srv import StartEpisode
    from robot_ros_support.qos import INFO_QOS

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("dsh008_episode_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    statuses: list[tuple[str, str]] = []
    node.create_subscription(
        RuntimeStatus,
        "/runtime/status",
        lambda message: statuses.append((message.episode_id, message.phase)),
        INFO_QOS,
    )
    client = node.create_client(StartEpisode, "/runtime/episode/start")
    episodes: list[str] = []
    try:
        deadline = time.monotonic() + 30.0
        while not client.service_is_ready():
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() >= deadline:
                print("DSH008_DRIVER_ERROR service unavailable", flush=True)
                return 2

        def start_episode(task_id):
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                request = StartEpisode.Request()
                request.task_id = task_id
                future = client.call_async(request)
                while not future.done():
                    executor.spin_once(timeout_sec=0.05)
                    if time.monotonic() >= deadline:
                        print("DSH008_DRIVER_ERROR start timeout", flush=True)
                        return None
                response = future.result()
                if response is not None and response.success:
                    return response.episode_id
                # The previous worker may still be between IDLE and done.
                if response is not None and response.reason == "CONTROL_BUSY":
                    executor.spin_once(timeout_sec=0.05)
                    continue
                print(
                    "DSH008_DRIVER_ERROR start rejected "
                    + json.dumps({"response": str(response), "statuses": statuses}),
                    flush=True,
                )
                return None
            print("DSH008_DRIVER_ERROR start retry timeout", flush=True)
            return None

        for task_id in ("first", "second"):
            episode_id = start_episode(task_id)
            if episode_id is None:
                return 2
            episodes.append(episode_id)
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if any(
                    current_id == episode_id and phase in ("IDLE", "FAULT")
                    for current_id, phase in statuses
                ):
                    break
            else:
                print(
                    "DSH008_DRIVER_ERROR episode timeout "
                    + json.dumps({"episode": episode_id, "statuses": statuses}),
                    flush=True,
                )
                return 2
        print(
            "DSH008_DRIVER_RESULT " + json.dumps({"episodes": episodes, "statuses": statuses}),
            flush=True,
        )
        return 0
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    if "--drive-episodes" in sys.argv[1:]:
        return _drive_episodes()
    return pytest.main([__file__, *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
