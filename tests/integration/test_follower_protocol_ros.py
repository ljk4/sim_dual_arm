"""Real ROS/CDR and service evidence for the bounded-follower protocol.

On a host without a sourced ROS installation the module is skipped.  The
backend test starts the production node with ``backend_follower.yaml`` and the
real MuJoCo scene, then drives three follower blocks through ``SubmitFollower``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

try:
    import rclpy
    from rclpy.serialization import deserialize_message, serialize_message
    from robot_api_msgs.msg import FollowerReference as FollowerReferenceMessage
    from robot_api_msgs.srv import GetStreamAnchor, QueryStreamCommand, SubmitFollower
    from robot_contracts.action_chunks import ChunkAnchor
    from robot_contracts.models import ContractError, RunContext, Target
    from robot_contracts.profiles import load_profile
    from robot_contracts.stream_request import (
        REQUEST_STREAM_FIRST,
        REQUEST_STREAM_NEXT,
        StreamRequest,
        Trajectory,
    )
    from robot_contracts.trajectory.spline import ReferenceSample
    from robot_ros_support.conversions import context_message, time_ns
    from robot_ros_support.follower_conversions import (
        follower_reference_from_message,
        follower_reference_message,
        reference_segment_messages,
        reference_segments_from_messages,
    )
    from robot_ros_support.stream_conversions import anchor_from_message, stream_command_message
    from robot_trajectory.follower_reference import validate
    from robot_trajectory.trajectory_generation import generate_follower_plan
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[2]
for source in (
    ROOT / "simulation/mujoco/src",
    ROOT / "shared/common/python/contracts/src",
    ROOT / "shared/common/python/ros_support/src",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.common.config.backend_config import load_backend_config  # noqa: E402
from robot_backend_sim.common.config.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.common.config.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.common.control.control_state import ChunkStatus  # noqa: E402
from robot_backend_sim.common.follower.follower_protocol import (  # noqa: E402
    follower_reference_from_chunk_status,
    prepare_follower_request,
    validation_limits,
)
from test_backend_ros import Stack, _profile_context, acquire  # noqa: E402

FOLLOWER_CONFIG = ROOT / "tests/fixtures/simulation/backend_follower.yaml"
BACKEND = load_backend_config(FOLLOWER_CONFIG)
CONTROL = load_control_config(BACKEND.control_path)
FOLLOWER = load_follower_config(BACKEND.follower_path)
PROFILE = load_profile(BACKEND.profile_path, BACKEND.schemas_path)
BASE = PROFILE.initial_positions
COUNT = len(BASE)
GRID_NS = 2_000_000


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    started = not rclpy.ok()
    if started:
        rclpy.init()
    yield
    if started and rclpy.ok():
        rclpy.shutdown()


def _context() -> RunContext:
    return RunContext(
        PROFILE.robot_id,
        PROFILE.digest,
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-follower-ros",
        control_epoch=1,
    )


def _targets() -> tuple[Target, ...]:
    """Exactly 60 original 20 Hz predictions, each with all 16 joint values."""
    targets = []
    right_joint1 = 8
    for index in range(1, 61):
        fraction = index / 60.0
        smooth = 3.0 * fraction * fraction - 2.0 * fraction * fraction * fraction
        positions = list(BASE)
        positions[right_joint1] = BASE[right_joint1] + 0.04 * smooth
        targets.append(Target(index * 50_000_000, tuple(positions)))
    return tuple(targets)


def _request(
    context: RunContext,
    lease: str,
    anchor: ChunkAnchor,
    *,
    kind: int,
    observation_id: int,
    final: bool,
    observation_ns: int,
) -> StreamRequest:
    return StreamRequest(
        trajectory=Trajectory(
            context,
            lease,
            "command-follower-ros",
            1,
            observation_id,
            observation_ns + 5_000_000_000,
            PROFILE.joint_names,
            _targets(),
        ),
        kind=kind,
        anchor=anchor,
        observation_time_ns=observation_ns,
        target_count=FOLLOWER.target_count,
        settle_at_end=final,
    )


def _submission(request: StreamRequest) -> SubmitFollower.Request:
    limits = validation_limits(CONTROL, FOLLOWER, duration_grid_ns=GRID_NS)
    plan = generate_follower_plan(request, PROFILE, limits)
    return SubmitFollower.Request(
        request=stream_command_message(request), segments=reference_segment_messages(plan)
    )


def _submit(stack: Stack, request: StreamRequest):
    return stack.call("submit_follower", _submission(request), timeout=20.0)


def _acquire_anchor(stack: Stack, context: RunContext, lease: str) -> ChunkAnchor:
    request = GetStreamAnchor.Request(
        context=context_message(context),
        lease_id=lease,
        has_start=False,
    )
    response = stack.call("get_stream_anchor", request)
    assert response.success, response.reason
    return anchor_from_message(response.anchor)


def _query(stack: Stack, context: RunContext, command_id: str):
    return stack.call(
        "query_stream_command",
        QueryStreamCommand.Request(
            context=context_message(context), command_id=command_id
        ),
    )


def test_follower_reference_really_serializes_and_roundtrips():
    anchor_start = 1_000_000_000
    anchor = ChunkAnchor(
        "follower-cdr",
        0,
        anchor_start,
        ReferenceSample(BASE, (0.0,) * COUNT, (0.0,) * COUNT),
        PROFILE.digest,
    )
    raw = _request(
        _context(),
        "lease-1",
        anchor,
        kind=REQUEST_STREAM_FIRST,
        observation_id=1,
        final=False,
        observation_ns=700_000_000,
    )
    submit = _submission(raw)
    decoded_submit = deserialize_message(serialize_message(submit), SubmitFollower.Request)
    assert decoded_submit.request.kind == 2
    plan = reference_segments_from_messages(decoded_submit.segments, PROFILE.joint_names)
    packet = prepare_follower_request(
        raw, plan, PROFILE, CONTROL, FOLLOWER, duration_grid_ns=GRID_NS
    )
    chunk = ChunkStatus(
        context=raw.command.context,
        command_id=raw.command.command_id,
        command_seq=raw.command.command_seq,
        chunk_seq=1,
        observation_id=raw.command.observation_id,
        phase="ACCEPTED",
        reason="PENDING",
        accepted_sim_ns=800_000_000,
        started_sim_ns=None,
        finished_sim_ns=None,
        accepted_wall_ns=1_800_000_000,
        started_wall_ns=None,
        finished_wall_ns=None,
        snapshot=None,
    )
    record = follower_reference_from_chunk_status(raw, packet, chunk, motion_seq=4)
    decoded = deserialize_message(
        serialize_message(follower_reference_message(record)), FollowerReferenceMessage
    )
    restored = follower_reference_from_message(decoded)
    assert restored == record
    assert restored.reference_digest == record.reference_digest
    assert restored.window.source_targets == raw.trajectory.targets
    assert restored.algorithm == record.algorithm
    validate(
        restored,
        PROFILE,
        validation_limits(CONTROL, FOLLOWER, duration_grid_ns=GRID_NS),
    )

    tampered = follower_reference_message(record)
    tampered.reference_digest = "0" * 64
    with pytest.raises(ContractError, match="FOLLOWER_DIGEST_MISMATCH"):
        follower_reference_from_message(deserialize_message(
            serialize_message(tampered), FollowerReferenceMessage
        ))

    response = SubmitFollower.Response(
        accepted=True,
        reason="ACCEPTED",
        has_execution=False,
        has_chunk=True,
        has_reference=True,
        reference=follower_reference_message(record),
    )
    decoded_response = deserialize_message(serialize_message(response), SubmitFollower.Response)
    assert decoded_response.has_reference
    assert (
        follower_reference_from_message(decoded_response.reference).reference_digest
        == record.reference_digest
    )

    tampered = follower_reference_message(record)
    tampered.reference_digest = "0" * 64

    with pytest.raises(ContractError) as caught:
        follower_reference_from_message(tampered)
    assert caught.value.reason == "FOLLOWER_DIGEST_MISMATCH"


class FollowerStack(Stack):
    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path, speed=1.0, config_path=FOLLOWER_CONFIG)
        self.follower_references: list[FollowerReferenceMessage] = []
        self.node.create_subscription(
            FollowerReferenceMessage,
            f"{self.namespace}/follower_references",
            self.follower_references.append,
            10,
        )
        self.clients["submit_follower"] = self.node.create_client(
            SubmitFollower, f"{self.namespace}/submit_follower"
        )


def test_three_follower_blocks_over_real_ros_services(tmp_path):
    stack = FollowerStack(tmp_path)
    try:
        stack.wait_ready()
        assert "bounded_target_follower" in stack.info.capabilities
        context = _profile_context(
            PROFILE, stack.info, episode="follower-ros-episode", control_epoch=1
        )
        lease = acquire(stack, context)
        stack.wait_for(lambda: bool(stack.states), timeout=10.0)

        first_anchor = _acquire_anchor(stack, context, lease)
        first_request = _request(
            context,
            lease,
            first_anchor,
            kind=REQUEST_STREAM_FIRST,
            observation_id=1,
            final=False,
            observation_ns=time_ns(stack.states[-1].stamp),
        )
        first = _submit(stack, first_request)
        assert first.accepted, first.reason
        assert first.has_reference and first.has_chunk and first.has_execution
        first_record = follower_reference_from_message(first.reference)
        assert first_record.chunk_seq == 1 and not first_record.is_final
        first_digest = first_record.reference_digest
        first_sim = first_record.accepted_sim_time_ns
        first_wall = first_record.accepted_wall_time_ns

        retry = _submit(stack, first_request)
        assert retry.accepted, retry.reason
        retry_record = follower_reference_from_message(retry.reference)
        assert retry_record.reference_digest == first_digest
        assert retry_record.accepted_sim_time_ns == first_sim
        assert retry_record.accepted_wall_time_ns == first_wall

        changed_curve = _submission(first_request)
        changed_curve.segments[0].c0[0] += 0.0001
        conflict = stack.call("submit_follower", changed_curve)
        assert not conflict.accepted
        assert conflict.reason == "COMMAND_ID_CONFLICT"

        changed_targets = list(first_request.trajectory.targets)
        changed_targets[-1] = Target(
            changed_targets[-1].time_ns,
            (changed_targets[-1].positions[0] + 1e-3, *changed_targets[-1].positions[1:]),
        )
        modified = StreamRequest(
            trajectory=Trajectory(
                first_request.trajectory.context,
                first_request.trajectory.lease_id,
                first_request.trajectory.command_id,
                first_request.trajectory.command_seq,
                first_request.trajectory.observation_id,
                first_request.trajectory.accept_before_ns,
                first_request.trajectory.joint_names,
                tuple(changed_targets),
            ),
            kind=first_request.kind,
            anchor=first_request.anchor,
            observation_time_ns=first_request.observation_time_ns,
            target_count=first_request.target_count,
            settle_at_end=False,
        )
        conflict = _submit(stack, modified)
        assert not conflict.accepted
        assert conflict.reason == "COMMAND_ID_CONFLICT"

        stack.wait_for(
            lambda: (
                (response := _query(stack, context, "command-follower-ros")).success
                and response.execution.has_started
            ),
            timeout=20.0,
        )
        second_anchor = _acquire_anchor(stack, context, lease)
        assert second_anchor.predecessor_seq == 1
        second_observation = _query(
            stack, context, "command-follower-ros"
        ).execution.measurement_sim_time_ns
        second_request = _request(
            context,
            lease,
            second_anchor,
            kind=REQUEST_STREAM_NEXT,
            observation_id=2,
            final=False,
            observation_ns=second_observation,
        )
        second = _submit(stack, second_request)
        assert second.accepted, second.reason
        second_record = follower_reference_from_message(second.reference)
        assert second_record.chunk_seq == 2 and not second_record.is_final
        second_retry = _submit(stack, second_request)
        assert second_retry.accepted, second_retry.reason
        second_retry_record = follower_reference_from_message(second_retry.reference)
        assert second_retry_record.reference_digest == second_record.reference_digest
        assert second_retry_record.accepted_sim_time_ns == second_record.accepted_sim_time_ns
        assert second_retry_record.accepted_wall_time_ns == second_record.accepted_wall_time_ns

        stack.wait_for(
            lambda: (
                (response := _query(stack, context, "command-follower-ros")).success
                and response.execution.chunk_seq == 2
            ),
            timeout=20.0,
        )
        third_anchor = _acquire_anchor(stack, context, lease)
        assert third_anchor.predecessor_seq == 2
        third_observation = _query(
            stack, context, "command-follower-ros"
        ).execution.measurement_sim_time_ns
        third_request = _request(
            context,
            lease,
            third_anchor,
            kind=REQUEST_STREAM_NEXT,
            observation_id=3,
            final=True,
            observation_ns=third_observation,
        )
        third = _submit(stack, third_request)
        assert third.accepted, third.reason
        third_record = follower_reference_from_message(third.reference)
        assert third_record.chunk_seq == 3 and third_record.is_final
        third_retry = _submit(stack, third_request)
        assert third_retry.accepted, third_retry.reason
        third_retry_record = follower_reference_from_message(third_retry.reference)
        assert third_retry_record.reference_digest == third_record.reference_digest
        assert third_retry_record.accepted_sim_time_ns == third_record.accepted_sim_time_ns
        assert third_retry_record.accepted_wall_time_ns == third_record.accepted_wall_time_ns

        terminal = stack.wait_for(
            lambda: (
                response
                if (response := _query(stack, context, "command-follower-ros")).success
                and response.execution.phase in ("COMPLETED", "CANCELED", "FAILED")
                else None
            ),
            timeout=40.0,
        )
        assert terminal.execution.phase == "COMPLETED", terminal.execution.reason
        records = {
            record.chunk_seq: record
            for record in (first_record, second_record, third_record)
        }
        assert set(records) == {1, 2, 3}
        assert len({record.reference_digest for record in records.values()}) == 3
        assert records[1].window.source_targets == first_request.trajectory.targets
        assert records[3].actual_terminal_positions != records[2].actual_terminal_positions
        limits = validation_limits(CONTROL, FOLLOWER, duration_grid_ns=GRID_NS)
        for record in records.values():
            assert len(record.request.trajectory.targets) == 60
            assert len(record.window.source_targets) == 60
            validate(record, PROFILE, limits)

        stack.wait_for(lambda: len(stack.follower_references) >= 3, timeout=10.0)
        published = [follower_reference_from_message(item) for item in stack.follower_references]
        assert {item.chunk_seq for item in published} == {1, 2, 3}
    finally:
        code = stack.close()
        assert code == 0, stack._output()
