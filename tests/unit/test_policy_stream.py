import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from dual_arm_runtime.chunk_stream import (
    PolicyStreamRunner,
    PolicyTargetSource,
    follower_limits,
)
from dual_arm_runtime.policy_client import PolicyPrediction
from dual_arm_runtime.runtime_config import load_runtime_configuration
from dual_arm_runtime.target_source import Prediction
from dual_arm_runtime.teacher_source import TeacherStreamRunner, TeacherTargetSource
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import ContractError, RunContext, Target
from robot_contracts.trajectory.spline import ReferenceSample

ROOT = Path(__file__).resolve().parents[2]
POLICY_CONFIG = ROOT / "tests/fixtures/simulation/policy_unit.yaml"
TEACHER_CONFIG = ROOT / "shared/scenarios/panda_stack/configs/runtime/teacher_collect.yaml"


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

    def publish_command(self, request, timestamp_ns):
        self.commands.append(request)

    def record_follower_reference(self, reference):
        self.references.append(reference)

    def record_execution(self, execution):
        self.executions.append(execution)

    def wait_terminal(self, command_id, timeout_s, stop):
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
        "follower_config",
    ):
        value[field] = str((POLICY_CONFIG.parent / value[field]).resolve())
    value["policy"]["startup_settle_sim_ns"] = settle_ns
    value["policy"]["max_chunks"] = 8
    if startup_timeout_s is not None:
        value["startup_timeout_s"] = startup_timeout_s
    path = tmp_path / "policy.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return load_runtime_configuration(path, ROOT)


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


def test_stopped_successor_reports_physical_failure_instead_of_anchor_symptom(tmp_path):
    configuration = policy_configuration(tmp_path, settle_ns=0)
    profile = configuration.profile
    io = _IO(profile)

    class StoppedControl(_Control):
        def get_anchor(self, context, start_sim_time_ns=None):
            if self.anchors:
                raise ContractError("STREAM_ANCHOR_UNAVAILABLE", "stopping motion")
            return super().get_anchor(context, start_sim_time_ns)

        def query(self, context, command_id, *, ignore_stop=False):
            assert command_id == "episode-policy" and ignore_stop
            return SimpleNamespace(
                phase="STOPPING", stop_reason="TRACKING_TIMEOUT", reason="TRACKING_TIMEOUT"
            )

    runner = _runner(configuration, io, StoppedControl(profile))
    with pytest.raises(ContractError) as caught:
        runner.run()
    assert caught.value.reason == "TRACKING_TIMEOUT"
    assert len(io.commands) == 1
    assert io.executions[-1].stop_reason == "TRACKING_TIMEOUT"


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
    target_count = configuration.recipe.follower.target_count
    assert all(command.target_count == target_count for command in io.commands)
    assert io.commands[0].kind == 2
    assert all(command.kind == 3 for command in io.commands[1:])
    assert not any(command.settle_at_end for command in io.commands[:-1])
    assert io.commands[-1].settle_at_end is True
    assert all(
        command.window is not None
        and len(command.window.selected_indices) == target_count
        for command in io.commands
    )
    assert all(command.trajectory.targets[0].time_ns == 50_000_000 for command in io.commands)


@pytest.mark.parametrize("task_start_ns", [999_999_999, 1_000_000_000, 1_050_000_000])
def test_first_task_prediction_uses_capture_strictly_after_start(
    tmp_path, monkeypatch, task_start_ns
):
    configuration = policy_configuration(tmp_path, settle_ns=0)
    io = _IO(configuration.profile)
    original_begin = PolicyTargetSource.begin

    def begin(self, *args):
        original_begin(self, *args)
        return task_start_ns

    monkeypatch.setattr(PolicyTargetSource, "begin", begin)
    runner = _runner(configuration, io, _Control(configuration.profile))

    runner.run()

    first = io.commands[0]
    assert first.observation_time_ns > task_start_ns
    assert first.observation_time_ns - task_start_ns <= 50_000_000
    assert runner.chunks == 8


def test_same_time_capture_cannot_replace_missing_post_start_capture(tmp_path, monkeypatch):
    configuration = policy_configuration(tmp_path, settle_ns=0)
    io = _MissingIO(configuration.profile)
    original_begin = PolicyTargetSource.begin

    def begin(self, *args):
        original_begin(self, *args)
        return 1_000_000_000

    monkeypatch.setattr(PolicyTargetSource, "begin", begin)
    runner = _runner(configuration, io, _Control(configuration.profile))

    with pytest.raises(ContractError, match="WALL_TIMEOUT"):
        runner.run()

    assert io.commands == []


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
        if event == "policy.startup_settle.started"
    ]
    completed = [
        fields
        for event, fields in log.events
        if event == "policy.startup_settle.completed"
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


def _retiming_configuration(tmp_path):
    follower_doc = yaml.safe_load(
        (POLICY_CONFIG.parent / yaml.safe_load(POLICY_CONFIG.read_text())["follower_config"])
        .resolve()
        .read_text(encoding="utf-8")
    )
    follower_doc["target_count"] = 5
    follower_path = tmp_path / "follower_5.yaml"
    follower_path.write_text(yaml.safe_dump(follower_doc), encoding="utf-8")
    value = yaml.safe_load(POLICY_CONFIG.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
    ):
        value[field] = str((POLICY_CONFIG.parent / value[field]).resolve())
    value["follower_config"] = str(follower_path)
    value["policy"]["startup_settle_sim_ns"] = 0
    value["policy"]["max_chunks"] = 8
    path = tmp_path / "policy.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return load_runtime_configuration(path, ROOT)


def test_gripper_transition_is_retimed_before_submission(tmp_path):
    configuration = _retiming_configuration(tmp_path)
    profile = configuration.profile
    gripper = profile.joint_names.index("right_gripper_width")
    initial = [joint.initial for joint in profile.joints]

    class JumpClient(_Client):
        def predict(self, context, session_id, request_id, observation, deadline_s=None):
            times = tuple((index + 1) * 50_000_000 for index in range(60))
            positions = []
            for index in times:
                values = list(observation.positions)
                if 5 <= (index // 50_000_000) - 1 <= 9:
                    values[gripper] = 0.079865
                positions.append(tuple(values))
            return PolicyPrediction(
                session_id,
                request_id,
                context,
                observation.observation_id,
                self.policy.model_bundle_digest,
                self.profile.joint_names,
                times,
                tuple(positions),
                1_000,
            )

    class HeldGripperControl(_Control):
        def get_anchor(self, context, start_sim_time_ns=None):
            anchor = super().get_anchor(context, start_sim_time_ns)
            positions = list(anchor.reference.positions)
            velocities = list(anchor.reference.velocities)
            positions[gripper] = 0.022205447806916776
            velocities[gripper] = 0.04
            return ChunkAnchor(
                anchor.stream_id,
                anchor.predecessor_seq,
                anchor.start_sim_time_ns,
                ReferenceSample(tuple(positions), tuple(velocities),
                                anchor.reference.accelerations),
                anchor.robot_profile_hash,
            )

    io = _IO(profile)
    context = RunContext("panda_stack", profile.digest, "backend", "sim", 1, "episode", 1)
    io.context = context
    log = _Log()
    runner = PolicyStreamRunner(
        configuration,
        context,
        "episode",
        "task",
        io,
        HeldGripperControl(profile),
        JumpClient(configuration.recipe.policy, profile),
        follower_limits(configuration, configuration.control_document["max_accelerations"]),
        stop_requested=threading.Event(),
        log=log,
        clock=lambda: 0.0,
    )
    runner.run()

    first = io.commands[0]
    assert first.trajectory.targets[5].positions[gripper] == pytest.approx(
        0.024205447806916774)
    assert first.trajectory.targets[5].positions[0] == pytest.approx(initial[0])
    # Every unselected prediction point stays exactly as predicted.
    for index in list(range(0, 5)) + list(range(10, 60)):
        assert first.trajectory.targets[index].positions[gripper] == pytest.approx(0.08)
    retimed = [fields for event, fields in log.events if event == "follower.chunk_retimed"]
    assert retimed
    assert retimed[0]["fields"]["axis"] == "right_gripper_width"
    assert retimed[0]["fields"]["original_first"] == pytest.approx(0.079865)
    assert retimed[0]["fields"]["adapted_first"] == pytest.approx(0.024205447806916774)


def test_teacher_targets_are_not_rewritten_by_policy_adaptation():
    configuration = load_runtime_configuration(TEACHER_CONFIG, ROOT)
    assert configuration.recipe.action_source == "teacher"
    profile = configuration.profile
    context = RunContext(profile.robot_id, profile.digest, "backend", "sim", 1, "episode", 1)
    io = _IO(profile, context)
    control = _Control(profile)
    observation = io.wait_observation(0, 1.0, threading.Event())
    anchor = control.get_anchor(context)
    axis = profile.joint_names.index("right_gripper_width")
    positions = list(anchor.reference.positions)
    velocities = list(anchor.reference.velocities)
    positions[axis] = 0.022205447806916776
    velocities[axis] = 0.04
    anchor = ChunkAnchor(
        anchor.stream_id,
        anchor.predecessor_seq,
        anchor.start_sim_time_ns,
        ReferenceSample(tuple(positions), tuple(velocities),
                        (0.0,) * len(positions)),
        anchor.robot_profile_hash,
    )
    points = []
    for _ in range(60):
        point = list(profile.initial_positions)
        point[axis] = 0.079865
        points.append(tuple(point))
    prediction = Prediction(
        context=context,
        observation_id=observation.observation_id,
        joint_names=profile.joint_names,
        times_from_observation_ns=tuple((index + 1) * 50_000_000 for index in range(60)),
        positions=tuple(points),
        target_count=configuration.recipe.follower.target_count,
        chunk_digest="teacher-original-chunk",
    )

    class _StaticTeacherSource(TeacherTargetSource):
        def predict(self, observation, chunk_index, request_id, *, handoff_time_ns=0):
            del observation, chunk_index, request_id, handoff_time_ns
            return prediction

    source = _StaticTeacherSource(
        object(),
        configuration.recipe.teacher,
        profile,
        target_count=configuration.recipe.follower.target_count,
        task_seed=10000,
    )
    log = _Log()
    runner = TeacherStreamRunner(
        configuration,
        context,
        "episode",
        "task",
        io,
        control,
        source,
        follower_limits(configuration, configuration.control_document["max_accelerations"]),
        stop_requested=threading.Event(),
        log=log,
        clock=lambda: 0.0,
    )
    assert runner._kind() == "teacher"
    runner._run_chunk(1, observation, anchor)

    original = tuple(
        Target(time_ns, positions)
        for time_ns, positions in zip(
            prediction.times_from_observation_ns, prediction.positions, strict=True)
    )
    submitted = io.commands[-1]
    assert submitted.trajectory.targets == original
    assert submitted.trajectory.joint_names == prediction.joint_names
    assert submitted.anchor == anchor
    assert submitted.trajectory.command_id == "episode-teacher"
    assert submitted.observation_time_ns == observation.stamp_ns
    assert not [event for event, _ in log.events if event == "follower.chunk_retimed"]
    accepted = [fields for event, fields in log.events if event == "follower.chunk_accepted"]
    assert accepted
    assert accepted[0]["fields"]["chunk_digest"] == "teacher-original-chunk"

    class _RejectingControl(_Control):
        def submit_follower(self, request):
            raise ContractError("FOLLOWER_DEVIATION_EXCEEDED", "teacher request rejected")

    rejecting_log = _Log()
    rejecting = TeacherStreamRunner(
        configuration,
        context,
        "episode",
        "task",
        io,
        _RejectingControl(profile),
        source,
        follower_limits(configuration, configuration.control_document["max_accelerations"]),
        stop_requested=threading.Event(),
        log=rejecting_log,
        clock=lambda: 0.0,
    )
    with pytest.raises(ContractError) as error:
        rejecting._run_chunk(1, observation, anchor)
    assert error.value.reason == "FOLLOWER_DEVIATION_EXCEEDED"
    assert io.commands[-1].trajectory.targets == original
    assert not [event for event, _ in rejecting_log.events
                if event == "follower.chunk_retimed"]
