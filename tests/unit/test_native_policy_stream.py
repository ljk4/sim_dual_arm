import threading
from pathlib import Path
from types import SimpleNamespace

from dual_arm_runtime.native_config import load_native_configuration
from dual_arm_runtime.native_stream import PolicyStreamRunner, follower_limits
from dual_arm_runtime.policy_client import PolicyPrediction
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import RunContext
from robot_contracts.reference import ReferenceSample

ROOT = Path(__file__).resolve().parents[2]


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
    def __init__(self, profile):
        self.profile = profile
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

    def __init__(self, profile):
        self.profile = profile
        self.anchors = 0

    def get_anchor(self, context, start_sim_time_ns=None):
        self.anchors += 1
        start = 1_250_000_000 + (self.anchors - 1) * 250_000_000
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


def test_eight_chunks_use_fresh_captures_keep_raw_60_and_settle_only_last():
    configuration = load_native_configuration(ROOT / "configs/runtime/native_policy.yaml", ROOT)
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
    assert all(command.target_count == 5 for command in io.commands)
    assert io.commands[0].kind == 2
    assert all(command.kind == 3 for command in io.commands[1:])
    assert not any(command.settle_at_end for command in io.commands[:-1])
    assert io.commands[-1].settle_at_end is True
    assert all(
        command.window is not None and len(command.window.selected_indices) == 5
        for command in io.commands
    )
    assert all(command.trajectory.targets[0].time_ns == 50_000_000 for command in io.commands)
