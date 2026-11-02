"""Exercise production feedback freezing under real ROS message conversion."""

from dataclasses import replace
from threading import Condition
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from dual_arm_runtime.native_node import NativeRuntimeNode  # noqa: E402
from robot_contracts.models import ContractError  # noqa: E402
from test_native_protocol_ros import execution  # noqa: E402


class _RecorderHarness:
    record_execution = NativeRuntimeNode.record_execution
    _execution_identity = NativeRuntimeNode._execution_identity

    def __init__(self):
        self._condition = Condition()
        self._execution_signature = {}
        self._terminal_execution = {}
        self._latest_execution = {}
        self._execution_duplicates = 0
        self.recorded = []
        self._root = "/robot"
        self._episode = SimpleNamespace(record=lambda *args: self.recorded.append(args))


def test_late_nonterminal_cannot_replace_a_frozen_terminal_identity():
    terminal = execution()
    late = replace(
        terminal,
        phase="STARTED",
        reason="",
        stop_reason="",
        has_finished=False,
        finished_sim_time_ns=None,
        finished_wall_time_ns=None,
        motion_phase="RUNNING",
    )
    harness = _RecorderHarness()
    harness.record_execution(terminal)
    harness.record_execution(late)
    harness.record_execution(terminal)
    assert harness._latest_execution[terminal.command_id] == terminal
    assert len(harness.recorded) == 1
    assert harness._execution_duplicates == 2


def test_changed_terminal_still_fails_after_an_out_of_order_nonterminal():
    harness = _RecorderHarness()
    terminal = execution()
    harness.record_execution(terminal)
    with pytest.raises(ContractError, match="terminal execution changed"):
        harness.record_execution(replace(terminal, reason="CHANGED_TERMINAL"))


def test_stop_while_submit_ack_is_pending_keeps_command_for_terminal_query():
    from threading import Event

    from dual_arm_runtime.native_episode import NativeEpisodeRunner
    from robot_contracts.models import RunContext

    context = RunContext("robot", "profile", "backend", "time", 1, "episode", 1)
    observation = SimpleNamespace(observation_id=1, stamp_ns=50_000_000)

    def stopped_submit(request):
        raise ContractError("EPISODE_STOPPED", "stop before ACK arrived")

    harness = SimpleNamespace(
        _configuration=SimpleNamespace(
            recipe=SimpleNamespace(
                motions=(SimpleNamespace(name="first", duration_ns=1_000_000_000,
                                         positions=(0.0,)),),
                call_timeout_s=1.0, command_acceptance_ns=500_000_000,
            ),
            profile=SimpleNamespace(joint_names=("joint",)),
        ),
        _io=SimpleNamespace(
            wait_observation=lambda *args: observation,
            robot_time_ns=lambda: 50_000_000,
            publish_native_command=lambda *args: None,
        ),
        _control=SimpleNamespace(lease_id="lease", submit=stopped_submit),
        _stop_requested=Event(), _sent_command_ids=[],
        check_recording_health=lambda: None, _emit_best_effort=lambda *a, **kw: None,
    )
    with pytest.raises(ContractError, match="EPISODE_STOPPED"):
        NativeEpisodeRunner._run_motions(harness, "episode", context)
    assert harness._sent_command_ids == ["episode-1"]


def test_terminal_fact_is_published_only_after_recorder_enqueue():
    harness = _RecorderHarness()
    terminal = execution()
    seen = []

    def enqueue(*args):
        seen.append(terminal.command_id not in harness._terminal_execution)

    harness._episode.record = enqueue
    harness.record_execution(terminal)
    assert seen == [True]
    assert harness._terminal_execution[terminal.command_id] == terminal


def test_finite_motion_stop_requires_reference_and_terminal_without_stream_events():
    from robot_contracts.native import NATIVE_MOTION

    harness = SimpleNamespace(
        _terminal_execution={"cmd": execution()},
        _expected_requests={"cmd": SimpleNamespace(kind=NATIVE_MOTION)},
        _accepted_digests={"cmd": {"digest"}},
    )
    assert NativeRuntimeNode._control_facts_complete(harness, "cmd")
    harness._accepted_digests.clear()
    assert not NativeRuntimeNode._control_facts_complete(harness, "cmd")
