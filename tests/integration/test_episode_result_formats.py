"""Result-format compatibility and v2 dimension checks; run inside the ROS image."""

import json
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path

import pytest
from dual_arm_runtime.audit import inspect_episode
from dual_arm_runtime.configuration import load_configuration
from dual_arm_runtime.recording import EpisodeRecorder
from robot_contracts.episodes import RuntimeOutcome, RuntimeResult
from robot_contracts.models import ContractError, RunContext
from test_recording import ROOT, SCHEMAS, write_episode
from test_runtime_shutdown import _episodes, _Harness, _manifest

CONTROL_CALL_MARKER_ENV = "DSH008_TEST_CONTROL_CALL_MARKER"


def _write_manifest(directory: Path, manifest: dict) -> None:
    (directory / "manifest.json").write_text(json.dumps(manifest))


def _as_v1(manifest: dict) -> dict:
    return {
        "schema_version": 1,
        "complete": True,
        "test_only": True,
        "source": "mock",
        "episode_id": manifest["episode_id"],
        "robot_profile_hash": manifest["robot_profile_hash"],
        "observation_schema_hash": manifest["observation_schema_hash"],
        "action_schema_hash": manifest["action_schema_hash"],
        "started_at_ns": manifest["started_at_ns"],
        "ended_at_ns": manifest["ended_at_ns"],
        "message_counts": manifest["message_counts"],
        "files": manifest["files"],
        "failure_reason": "",
        "configuration": manifest["configuration"],
    }


def _empty_recorder(tmp_path: Path) -> EpisodeRecorder:
    config = load_configuration(ROOT, ROOT / "runtime/ros2/configs/mock_collect.yaml")
    context = RunContext(
        config.profile.robot_id,
        config.profile.digest,
        "backend-test",
        "mock-system",
        0,
        "episode-empty",
        1,
    )
    return EpisodeRecorder(
        tmp_path,
        context.episode_id,
        {**config.snapshot, "run_context": asdict(context)},
        SCHEMAS,
        {"/robot/info": "robot_api_msgs/msg/BackendInfo"},
        run_id="test-empty",
        source="mock",
        test_only=True,
    )


def test_v2_empty_recording_is_valid_but_task_is_not_evaluated(tmp_path):
    directory = _empty_recorder(tmp_path).finish()
    report = inspect_episode(directory, SCHEMAS)
    assert report["recording"]["complete"] is True
    assert report["runtime"]["outcome"] == "completed"
    assert report["task"]["outcome"] == "not_evaluated"
    assert report["observations"] == report["commands"] == report["frames"] == 0
    assert report["commands_without_terminal"] == report["terminal_events"] == 0


def test_v2_static_observation_and_zero_commands_pass_generic_audit(tmp_path):
    directory = write_episode(
        tmp_path,
        moving=False,
        with_commands=False,
        runtime=RuntimeResult(RuntimeOutcome.STOPPED, "operator stopped"),
    )
    report = inspect_episode(directory, SCHEMAS)
    assert report["recording"]["complete"] is True
    assert report["runtime"]["outcome"] == "stopped"
    assert report["task"]["outcome"] == "not_evaluated"
    assert report["observations"] == 2
    assert report["commands"] == 0


def test_v2_unconfirmed_intent_and_prestart_cancel_are_legal_prefixes(tmp_path):
    pending = write_episode(
        tmp_path / "pending",
        phases=("ACCEPTED", "STARTED"),
    )
    pending_report = inspect_episode(pending, SCHEMAS)
    assert pending_report["commands"] == 1
    assert pending_report["commands_without_terminal"] == 1
    assert pending_report["terminal_events"] == 0

    canceled = write_episode(
        tmp_path / "canceled",
        phases=("ACCEPTED", "CANCELED"),
    )
    canceled_report = inspect_episode(canceled, SCHEMAS)
    assert canceled_report["commands_without_terminal"] == 0
    assert canceled_report["terminal_events"] == 1


@pytest.mark.parametrize(
    "phases",
    [
        ("STARTED", "ACCEPTED"),
        ("ACCEPTED", "ACCEPTED"),
        ("ACCEPTED", "STARTED", "COMPLETED", "COMPLETED"),
        ("ACCEPTED", "COMPLETED", "STARTED"),
    ],
)
def test_v2_illegal_command_event_orders_never_commit(tmp_path, phases):
    directory = write_episode(tmp_path, phases=phases)
    assert directory.name.startswith(".partial-")
    manifest = _manifest(directory)
    assert manifest["recording"]["complete"] is False
    assert manifest["runtime"]["outcome"] == "failed"
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(directory, SCHEMAS)


def test_v1_fixture_reads_with_old_strict_rules(tmp_path):
    v2_directory = write_episode(tmp_path)
    manifest = _as_v1(_manifest(v2_directory))
    _write_manifest(v2_directory, manifest)
    report = inspect_episode(v2_directory, SCHEMAS)
    assert report["schema_version"] == 1
    assert report["complete"] is True
    assert report["recording"]["complete"] is True
    assert report["runtime"]["outcome"] == "completed"
    assert report["task"]["outcome"] == "not_evaluated"


def test_v1_complete_false_is_rejected_even_with_complete_files(tmp_path):
    directory = write_episode(tmp_path)
    manifest = _as_v1(_manifest(directory))
    manifest["complete"] = False
    manifest["failure_reason"] = "EPISODE_STOPPED"
    _write_manifest(directory, manifest)
    with pytest.raises(ContractError, match="EPISODE_STOPPED"):
        inspect_episode(directory, SCHEMAS)


def test_unknown_version_and_v2_legacy_top_level_alias_are_rejected(tmp_path):
    unknown = write_episode(tmp_path / "unknown")
    manifest = _manifest(unknown)
    manifest["schema_version"] = 99
    _write_manifest(unknown, manifest)
    with pytest.raises(ContractError, match="UNSUPPORTED_SCHEMA_VERSION"):
        inspect_episode(unknown, SCHEMAS)

    mixed = write_episode(tmp_path / "mixed")
    manifest = _manifest(mixed)
    manifest["complete"] = True
    _write_manifest(mixed, manifest)
    with pytest.raises(ContractError, match="SCHEMA_MISMATCH"):
        inspect_episode(mixed, SCHEMAS)


@pytest.mark.parametrize("drop_terminal", [False, True])
def test_public_end_service_stops_then_next_episode_completes(tmp_path, drop_terminal):
    harness = _Harness(tmp_path)
    control_calls = tmp_path / "control-calls.txt"
    environment = {
        CONTROL_CALL_MARKER_ENV: str(control_calls),
        "DSH010_TEST_DELAY_TERMINAL_S": "0.3",
    }
    if drop_terminal:
        environment["DSH010_TEST_DROP_TERMINAL"] = "1"
    try:
        harness.start_backend()
        harness.start_runtime(
            once=False,
            bootstrap=True,
            extra_environment=environment,
        )
        harness.driver, harness.driver_log = harness._spawn(
            [sys.executable, str(Path(__file__).resolve()), "--drive-end"],
            "driver.log",
        )
        assert harness.driver.wait(timeout=90) == 0, harness.read_log(harness.driver_log)
        result_lines = [
            line
            for line in harness.read_log(harness.driver_log).splitlines()
            if line.startswith("DSH010_END_RESULT ")
        ]
        assert len(result_lines) == 1, harness.read_log(harness.driver_log)
        result = json.loads(result_lines[0].removeprefix("DSH010_END_RESULT "))
        assert result["end_reason"] == "ENDING"
        assert 1 <= result["first_commands"] < 3

        committed, partial = _episodes(harness.output)
        assert partial == []
        manifests = {_manifest(path)["episode_id"]: _manifest(path) for path in committed}
        assert set(manifests) == {result["first_episode"], result["second_episode"]}
        first = manifests[result["first_episode"]]
        assert first["recording"]["complete"] is True
        if drop_terminal:
            assert first["runtime"]["outcome"] == "failed"
            assert "Missing terminal events after stop" in first["runtime"]["reason"]
            assert "operator requested" in first["runtime"]["reason"]
            assert "LOGGING_ERROR" not in first["runtime"]["reason"]
        else:
            assert first["runtime"] == {
                "outcome": "stopped",
                "reason": "operator requested",
            }
        assert first["task"]["outcome"] == "not_evaluated"
        second = manifests[result["second_episode"]]
        assert second["recording"]["complete"] is True
        assert second["runtime"] == {"outcome": "completed", "reason": ""}
        assert second["task"]["outcome"] == "not_evaluated"

        first_report = inspect_episode(
            next(path for path in committed if path.name == result["first_episode"]),
            SCHEMAS,
        )
        assert first_report["commands"] == 1
        if drop_terminal:
            assert first_report["commands_without_terminal"] == 1
            assert first_report["terminal_events"] == 0
        else:
            assert first_report["commands_without_terminal"] == 0
            assert first_report["terminal_events"] == 1
        second_report = inspect_episode(
            next(path for path in committed if path.name == result["second_episode"]),
            SCHEMAS,
        )
        assert second_report["commands"] == 3
        assert second_report["commands_without_terminal"] == 0
        assert second_report["terminal_events"] == 3

        calls = control_calls.read_text().splitlines()
        assert "stop" in calls
        assert "release_control" in calls
        harness.runtime.send_signal(signal.SIGTERM)
        assert harness.wait_runtime(timeout=30) == 143, harness.read_log(harness.runtime_log)
    finally:
        harness.stop()


def _drive_end() -> int:
    """Public ROS driver: end the first episode, then run a second one."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from robot_api_msgs.msg import CommandEvent, RuntimeStatus, TrajectoryCommand
    from robot_api_msgs.srv import EndEpisode, StartEpisode
    from robot_ros_support.conversions import context_from_message
    from robot_ros_support.qos import EVENT_QOS, INFO_QOS

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("dsh010_end_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    statuses: list[tuple[str, str, str]] = []
    commands: list[tuple[str, str]] = []
    started_episodes: set[str] = set()

    def on_command_event(message: CommandEvent) -> None:
        if message.phase == "STARTED":
            started_episodes.add(message.context.episode_id)

    node.create_subscription(
        RuntimeStatus,
        "/runtime/status",
        lambda message: statuses.append((message.episode_id, message.phase, message.reason)),
        INFO_QOS,
    )
    node.create_subscription(
        TrajectoryCommand,
        "/runtime/control/commands",
        lambda message: commands.append(
            (context_from_message(message.context).episode_id, message.command_id)
        ),
        EVENT_QOS,
    )
    node.create_subscription(
        CommandEvent,
        "/cell_01/robot/command_events",
        on_command_event,
        EVENT_QOS,
    )
    start_client = node.create_client(StartEpisode, "/runtime/episode/start")
    end_client = node.create_client(EndEpisode, "/runtime/episode/end")

    def spin_until(predicate, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call(client, request):
        future = client.call_async(request)
        if not spin_until(future.done):
            raise RuntimeError("service call timed out")
        return future.result()

    def start_episode(task_id):
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            request = StartEpisode.Request()
            request.task_id = task_id
            response = call(start_client, request)
            if response is not None and response.success:
                return response
            if response is not None and response.reason == "CONTROL_BUSY":
                # The previous IDLE status can be observed just before its worker
                # thread exits; retry exactly as the public API requires.
                settle = time.monotonic() + 0.05
                while time.monotonic() < settle:
                    executor.spin_once(timeout_sec=0.01)
                continue
            raise RuntimeError(f"start rejected: {response}")
        raise RuntimeError(f"start retry timed out for {task_id}")

    try:
        if not spin_until(
            lambda: start_client.service_is_ready() and end_client.service_is_ready(),
            timeout=30.0,
        ):
            raise RuntimeError("episode services unavailable")

        first_response = start_episode("first")
        first_episode = first_response.episode_id

        if not spin_until(
            lambda: first_episode in started_episodes,
            timeout=30.0,
        ):
            raise RuntimeError("first episode never received a STARTED acknowledgement")

        end_request = EndEpisode.Request()
        end_request.reason = "operator requested"
        end_response = call(end_client, end_request)
        if end_response is None or not end_response.success:
            raise RuntimeError(f"end rejected: {end_response}")
        if end_response.reason != "ENDING":
            raise RuntimeError(f"unexpected end reason: {end_response.reason}")

        if not spin_until(
            lambda: any(
                episode_id == first_episode and phase in ("IDLE", "FAULT")
                for episode_id, phase, _ in statuses
            ),
            timeout=30.0,
        ):
            raise RuntimeError("first episode did not finish")

        second_response = start_episode("second")
        second_episode = second_response.episode_id
        if not spin_until(
            lambda: any(
                episode_id == second_episode and phase in ("IDLE", "FAULT")
                for episode_id, phase, _ in statuses
            ),
            timeout=60.0,
        ):
            raise RuntimeError("second episode did not finish")

        result = {
            "first_episode": first_episode,
            "second_episode": second_episode,
            "first_commands": len(
                [episode_id for episode_id, _ in commands if episode_id == first_episode]
            ),
            "end_reason": end_response.reason,
        }
        print("DSH010_END_RESULT " + json.dumps(result), flush=True)
        return 0
    except Exception as error:
        print(f"DSH010_END_ERROR {type(error).__name__}: {error}", flush=True)
        return 2
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    if "--drive-end" in sys.argv[1:]:
        return _drive_end()
    return pytest.main([__file__, *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
