import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from dual_arm_runtime.native_config import load_native_configuration
from dual_arm_runtime.native_stream import (
    PolicyStreamRunner,
    PolicyTargetSource,
    follower_limits,
)
from dual_arm_runtime.policy_client import PolicyPrediction
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import ContractError, RunContext
from robot_contracts.reference import ReferenceSample

ROOT = Path(__file__).resolve().parents[2]
POLICY_CONFIG = ROOT / "tests/fixtures/simulation/native_policy_unit.yaml"


class _Image:
    def __init__(self, camera):
        self.width = 320
        self.height = 240
        self.encoding = "rgb8"
        self.step = 960
        self.data = b"\x01" * (320 * 240 * 3)
        self.header = SimpleNamespace(frame_id=camera.frame_id)


class _Payload:
    def __init__(self, camera, sequence):
        self.payload = SimpleNamespace(
            image=_Image(camera), calibration_digest=camera.calibration_digest
        )
        self.sequence = sequence


class _IO:
    def __init__(self, profile, context=None):
        self.profile = profile
        self.context = context
        self.count = 0
        self.commands = []
        self.references = []
        self.executions = []

    def robot_time_ns(self):
        return 1_000_000_000

    def wait_observation(self, after, timeout, stop):
        self.count += 1
        stamp = 1_000_000_000 + (self.count - 1) * 50_000_000
        state = SimpleNamespace(
            joints=SimpleNamespace(
                name=list(self.profile.joint_names),
                position=[joint.initial for joint in self.profile.joints],
                velocity=[0.0] * 16,
            )
        )
        frames = {camera.name: _Payload(camera, self.count) for camera in self.profile.cameras}
        return SimpleNamespace(
            context=self.context,
            observation_id=self.count,
            stamp_ns=stamp,
            state=SimpleNamespace(payload=state, sequence=self.count),
            state_seq=self.count,
            frames=frames,
        )

    def publish_native_command(self, request, timestamp_ns):
        self.commands.append(request)

    def record_follower_reference(self, reference):
        self.references.append(reference)

    def record_execution(self, execution):
        self.executions.append(execution)

    def wait_native_terminal(self, command_id, timeout_s, stop):
        return SimpleNamespace(phase="COMPLETED", reason="", chunk_seq=8)


class _Control:
    lease_id = "lease-1"

    def __init__(self, profile, *, start_ns=1_250_000_000):
        self.profile = profile
        self.start_ns = start_ns
        self.anchors = 0

    def get_anchor(self, context, start_sim_time_ns=None):
        self.anchors += 1
        start = self.start_ns + (self.anchors - 1) * 250_000_000
        return ChunkAnchor(
            "stream-1",
            self.anchors - 1,
            start,
            ReferenceSample(
                tuple(joint.initial for joint in self.profile.joints),
                (0.0,) * 16,
                (0.0,) * 16,
            ),
            self.profile.digest,
        )

    def submit_follower(self, request):
        sequence = request.anchor.predecessor_seq + 1
        reference = SimpleNamespace(
            chunk_seq=sequence,
            accepted_sim_time_ns=request.observation_time_ns,
            accepted_wall_time_ns=1,
            reference_digest=f"digest-{sequence}",
            request=request,
        )
        return SimpleNamespace(
            reference=reference,
            execution=SimpleNamespace(phase="STARTED"),
            queried=False,
            submit_attempts=1,
        )


class _Client:
    session_id = "session-1"

    def __init__(self, policy, profile):
        self.policy = policy
        self.profile = profile

    def predict(self, context, session_id, request_id, observation, deadline_s=None):
        times = tuple((index + 1) * 50_000_000 for index in range(60))
        positions = tuple(tuple(observation.positions) for _ in times)
        return PolicyPrediction(
            session_id,
            request_id,
            context,
            observation.observation_id,
            self.policy.model_bundle_digest,
            self.profile.joint_names,
            times,
            positions,
            1_000,
        )


class _Log:
    def __init__(self):
        self.events = []

    def emit(self, event, **fields):
        self.events.append((event, fields))
        return True

    def check_health(self):
        return None


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class _MissingIO(_IO):
    """Yield one exact capture, then report the existing bounded wall timeout."""

    def wait_observation(self, after, timeout, stop):
        if self.count >= 1:
            raise ContractError("WALL_TIMEOUT", "exact complete observation did not arrive")
        return super().wait_observation(after, timeout, stop)


class _FrozenIO(_IO):
    """Every new observation carries the same simulation stamp."""

    def __init__(self, profile, clock, step_s=0.1):
        super().__init__(profile)
        self.clock = clock
        self.step_s = step_s

    def wait_observation(self, after, timeout, stop):
        observation = super().wait_observation(after, timeout, stop)
        self.clock.now += self.step_s
        state = SimpleNamespace(
            payload=observation.state.payload,
            sequence=observation.state_seq,
            stamp_ns=1_000_000_000,
        )
        return SimpleNamespace(
            context=observation.context,
            observation_id=observation.observation_id,
            stamp_ns=1_000_000_000,
            state=state,
            state_seq=observation.state_seq,
            frames=observation.frames,
        )


def policy_configuration(tmp_path, *, settle_ns, startup_timeout_s=None):
    value = yaml.safe_load(POLICY_CONFIG.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_constraints",
    ):
        value[field] = str((POLICY_CONFIG.parent / value[field]).resolve())
    value["policy"]["startup_settle_sim_ns"] = settle_ns
    value["policy"]["max_chunks"] = 8
    if startup_timeout_s is not None:
        value["startup_timeout_s"] = startup_timeout_s
    path = tmp_path / "native_policy.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return load_native_configuration(path, ROOT)


def _runner(configuration, io, control, log=None, *, stop=None, clock=None):
    profile = configuration.profile
    context = RunContext("panda_stack", profile.digest, "backend", "sim", 1, "episode", 1)
    io.context = context
    return PolicyStreamRunner(
        configuration,
        context,
        "episode",
        "task",
        io,
        control,
        _Client(configuration.recipe.policy, profile),
        follower_limits(configuration, [1.0] * 16),
        stop_requested=stop or threading.Event(),
        log=log,
        clock=clock or (lambda: 0.0),
    )


def test_eight_chunks_use_fresh_captures_keep_raw_60_and_settle_only_last(tmp_path):
    configuration = policy_configuration(tmp_path, settle_ns=0)
    profile = configuration.profile
    policy = configuration.recipe.policy
    context = RunContext("panda_stack", profile.digest, "backend", "sim", 1, "episode", 1)
    io = _IO(profile)
    control = _Control(profile)
    runner = PolicyStreamRunner(
        configuration,
        context,
        "episode",
        "task",
        io,
        control,
        _Client(policy, profile),
        follower_limits(configuration, [1.0] * 16),
        stop_requested=threading.Event(),
    )
    runner.run()
    assert runner.chunks == policy.max_chunks == 8
    assert len(io.commands) == 8
    assert [command.trajectory.observation_id for command in io.commands] == list(range(1, 9))
    assert all(len(command.trajectory.targets) == 60 for command in io.commands)
    assert all(command.target_count == policy.target_count for command in io.commands)
    assert io.commands[0].kind == 2
    assert all(command.kind == 3 for command in io.commands[1:])
    assert not any(command.settle_at_end for command in io.commands[:-1])
    assert io.commands[-1].settle_at_end is True
    assert all(
        command.window is not None
        and len(command.window.selected_indices) == policy.target_count
        for command in io.commands
    )
    assert all(command.trajectory.targets[0].time_ns == 50_000_000 for command in io.commands)


def test_zero_startup_settle_consumes_no_extra_observation(tmp_path, monkeypatch):
    configuration = policy_configuration(tmp_path, settle_ns=0)
    profile = configuration.profile
    assert configuration.recipe.policy.startup_settle_sim_ns == 0
    io = _IO(profile)
    calls = []
    original_begin = PolicyTargetSource.begin

    def begin(self, context, episode_id, task_id):
        calls.append(io.count)
        return original_begin(self, context, episode_id, task_id)

    monkeypatch.setattr(PolicyTargetSource, "begin", begin)
    runner = _runner(configuration, io, _Control(profile))
    runner.run()
    assert calls == [0]
    assert io.count == 8
    assert len(io.commands) == 8
    assert [command.trajectory.observation_id for command in io.commands] == list(range(1, 9))


def test_two_second_settle_uses_sim_time_and_finishes_before_source_begin(tmp_path, monkeypatch):
    configuration = policy_configuration(tmp_path, settle_ns=2_000_000_000)
    profile = configuration.profile
    io = _IO(profile)
    log = _Log()
    calls = []
    original_begin = PolicyTargetSource.begin

    def begin(self, context, episode_id, task_id):
        calls.append(io.count)
        return original_begin(self, context, episode_id, task_id)

    monkeypatch.setattr(PolicyTargetSource, "begin", begin)
    runner = _runner(
        configuration,
        io,
        _Control(profile, start_ns=3_250_000_000),
        log,
    )
    runner.run()

    assert calls == [41]
    assert io.commands[0].trajectory.observation_id == 42
    assert runner.chunks == 8
    started = [
        fields
        for event, fields in log.events
        if event == "native.policy.startup_settle.started"
    ]
    completed = [
        fields
        for event, fields in log.events
        if event == "native.policy.startup_settle.completed"
    ]
    assert len(started) == 1
    assert len(completed) == 1
    assert started[0]["fields"]["configured_sim_ns"] == 2_000_000_000
    assert started[0]["fields"]["observation_id"] == 1
    assert completed[0]["fields"] == {
        "configured_sim_ns": 2_000_000_000,
        "actual_sim_ns": 2_000_000_000,
        "first_observation_id": 1,
        "first_stamp_ns": 1_000_000_000,
        "last_observation_id": 41,
        "last_stamp_ns": 3_000_000_000,
        "observations": 41,
    }


def test_startup_settle_stop_before_first_capture_is_bounded(tmp_path, monkeypatch):
    configuration = policy_configuration(tmp_path, settle_ns=2_000_000_000)
    profile = configuration.profile
    io = _IO(profile)
    stop = threading.Event()
    stop.set()
    calls = []
    original_begin = PolicyTargetSource.begin

    def begin(self, context, episode_id, task_id):
        calls.append(io.count)
        return original_begin(self, context, episode_id, task_id)

    monkeypatch.setattr(PolicyTargetSource, "begin", begin)
    runner = _runner(configuration, io, _Control(profile), stop=stop)
    with pytest.raises(ContractError) as error:
        runner.run()
    assert error.value.reason == "EPISODE_STOPPED"
    assert io.count == 0
    assert calls == []


def test_startup_settle_missing_capture_uses_existing_wall_timeout(tmp_path):
    configuration = policy_configuration(tmp_path, settle_ns=2_000_000_000)
    profile = configuration.profile
    io = _MissingIO(profile)
    runner = _runner(configuration, io, _Control(profile))
    with pytest.raises(ContractError) as error:
        runner.run()
    assert error.value.reason == "WALL_TIMEOUT"
    assert io.count == 1


def test_startup_settle_frozen_sim_clock_is_bounded_by_startup_timeout(tmp_path):
    configuration = policy_configuration(
        tmp_path,
        settle_ns=2_000_000_000,
        startup_timeout_s=0.25,
    )
    profile = configuration.profile
    clock = _Clock()
    io = _FrozenIO(profile, clock)
    runner = _runner(configuration, io, _Control(profile), clock=clock)
    with pytest.raises(ContractError) as error:
        runner.run()
    assert error.value.reason == "STREAM_STARTUP_TIMEOUT"
    assert clock.now >= 0.25


class _WrongContextIO(_IO):
    def wait_observation(self, after, timeout, stop):
        observation = super().wait_observation(after, timeout, stop)
        return SimpleNamespace(
            context=RunContext("other", self.profile.digest, "backend", "sim", 1, "episode", 1),
            observation_id=observation.observation_id,
            stamp_ns=observation.stamp_ns,
            state=observation.state,
            state_seq=observation.state_seq,
            frames=observation.frames,
        )


class _RegressionIO(_IO):
    def wait_observation(self, after, timeout, stop):
        observation = super().wait_observation(after, timeout, stop)
        stamp_ns = 1_000_000_000 if self.count == 1 else 999_999_999
        state = SimpleNamespace(
            payload=observation.state.payload,
            sequence=observation.state_seq,
            stamp_ns=stamp_ns,
        )
        return SimpleNamespace(
            context=observation.context,
            observation_id=observation.observation_id,
            stamp_ns=stamp_ns,
            state=state,
            state_seq=observation.state_seq,
            frames=observation.frames,
        )


def test_startup_settle_rejects_another_session_context(tmp_path):
    configuration = policy_configuration(tmp_path, settle_ns=2_000_000_000)
    profile = configuration.profile
    runner = _runner(configuration, _WrongContextIO(profile), _Control(profile))
    with pytest.raises(ContractError) as error:
        runner.run()
    assert error.value.reason == "STREAM_CONTEXT_MISMATCH"


def test_startup_settle_rejects_simulation_time_regression(tmp_path):
    configuration = policy_configuration(tmp_path, settle_ns=2_000_000_000)
    profile = configuration.profile
    runner = _runner(configuration, _RegressionIO(profile), _Control(profile))
    with pytest.raises(ContractError) as error:
        runner.run()
    assert error.value.reason == "STREAM_OBSERVATION_REGRESSION"
