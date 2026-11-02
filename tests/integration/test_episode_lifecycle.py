"""Behavior tests for the extracted episode lifecycle with explicit doubles."""

import json
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_runtime.configuration import ScriptTarget
from dual_arm_runtime.episode import EpisodeRunner
from dual_arm_runtime.observation import ObservationRef, SampleRef
from dual_arm_runtime.recording_queue import RecordingStats
from robot_contracts.episodes import RuntimeOutcome, RuntimeResult
from robot_contracts.models import ContractError, RunContext


def _configuration(targets):
    return SimpleNamespace(
        recipe=SimpleNamespace(
            script_targets=tuple(targets),
            startup_timeout_s=5.0,
            call_timeout_s=5.0,
            command_acceptance_ns=1_000,
        ),
        profile=SimpleNamespace(joint_names=("joint_a",)),
        snapshot={"profile_hash": "profile-hash"},
        schema_root=Path("schemas"),
    )


class _IO:
    def __init__(self):
        self.info_topic = "/cell/robot/info"
        self.backend_context = RunContext("robot", "profile", "backend", "mock", 1)
        self.backend_error = None
        self.backend_gate = None
        self.backend_waiting = threading.Event()
        self.stop_requested = None
        self.bind_contexts = []
        self.context = None
        self.observation_id = 0
        self.observation_error = None
        self.terminal_events = set()
        self.accepted_commands = set()
        self.terminal_calls = []
        self.after_stop_waits = []
        self.after_stop_error = None
        self.commands = []
        self.statuses = []
        self.status_error_phase = ""
        self.errors = []
        self.time_ns = 0

    def wait_backend(self, episode_id, timeout_s, stop):
        self.stop_requested = stop
        self.backend_waiting.set()
        if self.backend_gate is not None and not self.backend_gate.wait(timeout_s):
            raise ContractError("NOT_READY", "Backend info")
        if self.backend_error is not None:
            raise self.backend_error
        if stop.is_set():
            raise ContractError("NOT_READY", "Backend info")
        return (
            RunContext(
                self.backend_context.robot_id,
                self.backend_context.robot_profile_hash,
                self.backend_context.backend_instance_id,
                self.backend_context.time_domain_id,
                self.backend_context.clock_epoch,
                episode_id,
                1,
            ),
            SimpleNamespace(backend_kind="mock", test_only=True),
        )

    def bind_context(self, context):
        self.bind_contexts.append(context)
        self.context = context
        self.observation_id = 0
        self.terminal_events.clear()
        self.accepted_commands.clear()

    def wait_observation(self, after_id, timeout_s, stop):
        if self.observation_error is not None:
            raise self.observation_error
        self.observation_id += 1
        return ObservationRef(
            self.context,
            self.observation_id,
            SampleRef(self.observation_id, self.time_ns),
            {},
        )

    def wait_terminal_events(self, command_ids, timeout_s, stop):
        self.terminal_calls.append(tuple(command_ids))

    def wait_terminal_events_after_stop(self, command_ids, timeout_s):
        expected = set(command_ids) & self.accepted_commands
        self.after_stop_waits.append((tuple(command_ids), tuple(sorted(expected)), timeout_s))
        if self.after_stop_error is not None:
            raise self.after_stop_error

    def robot_time_ns(self):
        self.time_ns += 1
        return self.time_ns

    def publish_command(self, command, timestamp_ns):
        self.commands.append((command, timestamp_ns))
        return command

    def publish_status(self, phase, episode_id, reason, record_directory):
        if phase == self.status_error_phase:
            raise RuntimeError(f"injected {phase} status failure")
        self.statuses.append((phase, episode_id, reason, record_directory))

    def record_topics(self):
        return {self.info_topic: "BackendInfo", "/runtime/control/commands": "Trajectory"}

    def log_error(self, message):
        self.errors.append(message)

    def notify(self):
        pass


class _Control:
    def __init__(self):
        self.stop_requested = None
        self.lease_id = "lease-1"
        self.acquisitions = []
        self.commands = []
        self.acquire_error = None
        self.execute_hook = None
        self.cleanup_calls = 0

    def acquire(self, context):
        self.acquisitions.append(context)
        if self.acquire_error is not None:
            raise self.acquire_error

    def execute(self, command_message):
        self.commands.append(command_message)
        if self.execute_hook is not None:
            self.execute_hook(command_message)

    def renew(self):
        pass

    def cleanup(self):
        self.cleanup_calls += 1


class _Recorder:
    def __init__(self, directory, finish_error=None):
        self.directory = directory
        self.finish_error = finish_error
        self.appended = []
        self.finished = []

    def append(self, topic, message, timestamp_ns):
        self.appended.append((topic, message, timestamp_ns))

    def check_health(self):
        pass

    def stats(self):
        return RecordingStats(
            accepted=len(self.appended),
            written=len(self.appended),
            pending_items=0,
            pending_bytes=0,
            peak_items=len(self.appended),
            peak_bytes=0,
            failure_reason="",
            written_counts=(),
        )

    def finish(
        self,
        runtime=None,
        *,
        current_runtime=None,
    ):
        runtime_result = current_runtime() if current_runtime is not None else runtime
        if runtime_result is None:
            runtime_result = RuntimeResult.completed()
        self.finished.append((runtime_result, current_runtime))
        if self.finish_error is not None:
            raise self.finish_error
        self.directory.mkdir(parents=True, exist_ok=True)
        (self.directory / "manifest.json").write_text(
            json.dumps(
                {
                    "recording": {"complete": True, "failure_reason": ""},
                    "runtime": {
                        "outcome": runtime_result.outcome.value,
                        "reason": runtime_result.reason,
                    },
                    "task": {"outcome": "not_evaluated", "reason": ""},
                }
            )
        )
        final = self.directory.with_name(self.directory.name.removeprefix(".partial-"))
        self.directory.rename(final)
        self.directory = final
        return self.directory


def _make_runner(tmp_path, *, io=None, control=None, targets=None, finish_error=None):
    io = io or _IO()
    control = control or _Control()
    recorders = []

    def control_factory(stop, fault):
        control.stop_requested = stop
        control.report_fault = fault
        return control

    def recorder_factory(episode_id, snapshot, topics):
        recorder = _Recorder(tmp_path / f".partial-{episode_id}", finish_error=finish_error)
        recorder.snapshot = snapshot
        recorder.topics = topics
        recorders.append(recorder)
        return recorder

    runner = EpisodeRunner(
        _configuration(targets or [ScriptTarget(1_000_000, (0.25,))]),
        tmp_path,
        io,
        control_factory=control_factory,
        recorder_factory=recorder_factory,
    )
    return runner, io, control, recorders


def _wait_for_worker(runner):
    assert runner.done.wait(5.0)
    assert runner.join_worker(1.0)


def test_single_run_completes_and_reads_final_manifest(tmp_path):
    runner, io, control, recorders = _make_runner(
        tmp_path,
        targets=[
            ScriptTarget(1_000_000, (0.25,)),
            ScriptTarget(2_000_000, (0.5,)),
        ],
    )

    episode_id = runner.start("task-1")
    _wait_for_worker(runner)

    assert runner.failure_reason == ""
    assert runner.done.is_set()
    assert control.stop_requested is runner.stop_requested
    assert io.stop_requested is runner.stop_requested
    assert [context.episode_id for context in control.acquisitions] == [episode_id]
    assert [command.command_id for command in control.commands] == [
        f"{episode_id}-1",
        f"{episode_id}-2",
    ]
    assert [command.targets[0].positions for command in control.commands] == [
        (0.25,),
        (0.5,),
    ]
    assert [status[0] for status in io.statuses] == [
        "PREPARING",
        "ACTIVE",
        "ENDING",
        "IDLE",
    ]
    assert io.statuses[-1][1] == episode_id
    assert io.statuses[-1][3] == runner.record_directory
    assert io.bind_contexts[0].episode_id == episode_id
    assert io.context is not None and io.context.episode_id == episode_id
    assert len(recorders) == 1
    assert any(topic == io.info_topic for topic, _, _ in recorders[0].appended)
    assert recorders[0].finished[0][0].outcome is RuntimeOutcome.COMPLETED
    assert recorders[0].topics == io.record_topics()
    manifest = json.loads((Path(runner.record_directory) / "manifest.json").read_text())
    assert manifest["recording"]["complete"] is True
    assert manifest["runtime"]["outcome"] == "completed"
    assert manifest["task"]["outcome"] == "not_evaluated"


def test_concurrent_start_is_rejected_while_worker_runs(tmp_path):
    io = _IO()
    io.backend_gate = threading.Event()
    runner, _, _, _ = _make_runner(tmp_path, io=io)

    runner.start("first")
    assert io.backend_waiting.wait(1.0)
    with pytest.raises(ContractError) as rejected:
        runner.start("second")
    assert rejected.value.reason == "CONTROL_BUSY"

    io.backend_gate.set()
    _wait_for_worker(runner)
    assert runner.failure_reason == ""


def test_start_after_shutdown_is_rejected(tmp_path):
    runner, io, control, recorders = _make_runner(tmp_path)

    runner.request_shutdown("SIGINT")

    assert runner.shutdown_requested is True
    assert runner.shutdown_reason == "SIGINT"
    with pytest.raises(ContractError) as rejected:
        runner.start("late")
    assert rejected.value.reason == "SHUTTING_DOWN"
    assert io.bind_contexts == []
    assert control.cleanup_calls == 0
    assert recorders == []


def test_previous_episode_failure_and_indices_do_not_leak(tmp_path):
    control = _Control()
    control.acquire_error = RuntimeError("FIRST_FAILURE")
    runner, io, _, recorders = _make_runner(tmp_path, control=control)

    first_id = runner.start("first")
    _wait_for_worker(runner)
    first_directory = runner.record_directory
    assert "FIRST_FAILURE" in runner.failure_reason
    assert first_directory
    first_manifest = json.loads((Path(first_directory) / "manifest.json").read_text())
    assert first_manifest["recording"]["complete"] is True
    assert first_manifest["runtime"]["outcome"] == "failed"

    control.acquire_error = None
    io.terminal_events.update({"old-command"})
    io.observation_id = 99
    second_id = runner.start("second")
    _wait_for_worker(runner)

    assert second_id != first_id
    assert runner.failure_reason == ""
    assert runner.record_directory != first_directory
    assert io.terminal_events == set()
    assert io.observation_id == 1
    assert io.bind_contexts[-1].episode_id == second_id
    assert [status[0] for status in io.statuses[-4:]] == [
        "PREPARING",
        "ACTIVE",
        "ENDING",
        "IDLE",
    ]
    assert all(status[2] == "" for status in io.statuses[-4:])
    assert len(recorders) == 2
    assert recorders[1].finished[0][0].outcome is RuntimeOutcome.COMPLETED


def test_start_failure_still_cleans_up_and_marks_done(tmp_path):
    io = _IO()
    io.backend_error = ContractError("NOT_READY", "Backend info")
    runner, _, control, recorders = _make_runner(tmp_path, io=io)

    runner.start("no-backend")
    assert runner.done.wait(5.0)
    assert runner.join_worker(1.0)

    assert "NOT_READY" in runner.failure_reason
    assert control.cleanup_calls == 1
    assert recorders == []
    assert [status[0] for status in io.statuses] == ["PREPARING", "ENDING", "FAULT"]


@pytest.mark.parametrize("phase", ["PREPARING", "ENDING"])
def test_status_failure_does_not_skip_cleanup_or_recording(tmp_path, phase):
    io = _IO()
    io.status_error_phase = phase
    runner, _, control, recorders = _make_runner(tmp_path, io=io)

    runner.start("status-failure")
    _wait_for_worker(runner)

    assert f"injected {phase} status failure" in runner.failure_reason
    assert control.cleanup_calls == 1
    if phase == "PREPARING":
        assert recorders == []
        assert [status[0] for status in io.statuses] == ["ENDING", "FAULT"]
    else:
        assert len(recorders) == 1
        assert recorders[0].finished[0][0].outcome is RuntimeOutcome.FAILED
        assert io.statuses[-1][0] == "FAULT"
        manifest = json.loads((Path(runner.record_directory) / "manifest.json").read_text())
        assert manifest["recording"]["complete"] is True
        assert manifest["runtime"]["outcome"] == "failed"


def test_manual_end_stops_after_inflight_and_next_episode_resets(tmp_path):
    control = _Control()
    started, release = threading.Event(), threading.Event()

    def hold_first_command(command):
        if len(control.commands) == 1:
            io.accepted_commands.add(command.command_id)
            started.set()
            assert release.wait(2.0)

    control.execute_hook = hold_first_command
    runner, io, _, recorders = _make_runner(
        tmp_path,
        control=control,
        targets=[
            ScriptTarget(1_000_000, (0.25,)),
            ScriptTarget(1_000_000, (0.5,)),
        ],
    )

    first_id = runner.start("first")
    assert started.wait(2.0)
    assert runner.end_episode("operator requested") is True
    # Repeated requests do not replace the first reason or start another finish.
    assert runner.end_episode("a different later reason") is True
    release.set()
    _wait_for_worker(runner)

    assert runner.failure_reason == ""
    assert len(control.commands) == 1, "no next script segment after EndEpisode"
    assert control.cleanup_calls == 1
    first_manifest = json.loads((Path(runner.record_directory) / "manifest.json").read_text())
    assert first_manifest["recording"]["complete"] is True
    assert first_manifest["runtime"] == {
        "outcome": "stopped",
        "reason": "operator requested",
    }
    assert first_manifest["task"]["outcome"] == "not_evaluated"
    assert io.statuses[-1][0] == "IDLE"
    sent, expected, timeout_s = io.after_stop_waits[-1]
    assert sent == (f"{first_id}-1",)
    assert expected == sent, "the accepted in-flight command must be awaited"
    assert timeout_s == 5.0
    assert runner.end_episode("after completion") is False

    control.execute_hook = None
    second_id = runner.start("second")
    _wait_for_worker(runner)

    assert second_id != first_id
    assert runner.failure_reason == ""
    assert len(control.commands) == 3
    assert control.cleanup_calls == 2
    assert io.after_stop_waits[-1][1] == ()
    second_manifest = json.loads((Path(runner.record_directory) / "manifest.json").read_text())
    assert second_manifest["recording"]["complete"] is True
    assert second_manifest["runtime"] == {"outcome": "completed", "reason": ""}
    assert second_manifest["task"]["outcome"] == "not_evaluated"
    assert len(recorders) == 2


def test_missing_terminal_after_stop_is_reported_and_finish_still_completes(tmp_path):
    io = _IO()
    control = _Control()
    control.execute_hook = lambda command: io.accepted_commands.add(command.command_id)
    io.after_stop_error = ContractError(
        "CALL_TIMEOUT",
        "Missing terminal events after stop: episode-command-1",
    )
    runner, _, control, recorders = _make_runner(tmp_path, io=io, control=control)

    runner.start("missing-terminal")
    _wait_for_worker(runner)

    assert "TERMINAL_WAIT_ERROR" in runner.failure_reason
    assert "Missing terminal events after stop" in runner.failure_reason
    assert control.cleanup_calls == 1
    assert io.statuses[-1][0] == "FAULT"
    sent, expected, timeout_s = io.after_stop_waits[-1]
    assert expected == sent
    assert timeout_s == 5.0
    manifest = json.loads((Path(runner.record_directory) / "manifest.json").read_text())
    assert manifest["recording"]["complete"] is True
    assert manifest["runtime"]["outcome"] == "failed"
    assert len(recorders) == 1


def test_early_stop_without_accepted_command_does_not_wait_for_fake_ack(tmp_path):
    control = _Control()
    started, release = threading.Event(), threading.Event()

    def hold_unaccepted_command(_command):
        started.set()
        assert release.wait(2.0)

    control.execute_hook = hold_unaccepted_command
    runner, io, _, _ = _make_runner(tmp_path, control=control)

    first_id = runner.start("early")
    assert started.wait(2.0)
    assert runner.end_episode("early stop") is True
    release.set()
    _wait_for_worker(runner)

    assert runner.failure_reason == ""
    sent, expected, _timeout = io.after_stop_waits[-1]
    assert sent == (f"{first_id}-1",)
    assert expected == (), "an unaccepted intent has no terminal ACK to wait for"
    manifest = json.loads((Path(runner.record_directory) / "manifest.json").read_text())
    assert manifest["runtime"]["outcome"] == "stopped"
    assert manifest["recording"]["complete"] is True


def test_recording_finish_failure_preserves_primary_failure(tmp_path):
    control = _Control()
    control.acquire_error = RuntimeError("PRIMARY_FAILURE")
    runner, io, _, recorders = _make_runner(
        tmp_path,
        control=control,
        finish_error=RuntimeError("finish boom"),
    )

    runner.start("primary")
    _wait_for_worker(runner)

    assert runner.failure_reason.startswith("PRIMARY_FAILURE; RECORDING_ERROR: finish boom")
    assert control.cleanup_calls == 1
    assert len(recorders) == 1
    assert recorders[0].finished[0][0].outcome is RuntimeOutcome.FAILED
    assert io.statuses[-1][0] == "FAULT"
    assert runner.record_directory == ""
