"""Explicit conversions between wire messages and immutable domain values."""

from builtin_interfaces.msg import Duration, Time
from robot_api_msgs.msg import CommandStatus as CommandStatusMessage
from robot_api_msgs.msg import RunContext as ContextMessage
from robot_api_msgs.msg import TrajectoryCommand
from robot_contracts.models import (
    CommandStatus,
    ContractError,
    RunContext,
    Target,
    Trajectory,
)
from robot_contracts.status import validate_command_status
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def time_ns(value: Time | Duration) -> int:
    return value.sec * 1_000_000_000 + value.nanosec


def stamp(value: int) -> Time:
    seconds, nanoseconds = divmod(value, 1_000_000_000)
    return Time(sec=seconds, nanosec=nanoseconds)


def duration(value: int) -> Duration:
    seconds, nanoseconds = divmod(value, 1_000_000_000)
    return Duration(sec=seconds, nanosec=nanoseconds)


def context_from_message(value: ContextMessage) -> RunContext:
    return RunContext(
        value.robot_id,
        value.robot_profile_hash,
        value.backend_instance_id,
        value.time_domain_id,
        value.clock_epoch,
        value.episode_id,
        value.control_epoch,
    )


def context_message(value: RunContext) -> ContextMessage:
    return ContextMessage(
        robot_id=value.robot_id,
        robot_profile_hash=value.robot_profile_hash,
        backend_instance_id=value.backend_instance_id,
        time_domain_id=value.time_domain_id,
        clock_epoch=value.clock_epoch,
        episode_id=value.episode_id,
        control_epoch=value.control_epoch,
    )


def trajectory_message(value: Trajectory) -> JointTrajectory:
    result = JointTrajectory(joint_names=list(value.joint_names))
    result.points = [
        JointTrajectoryPoint(
            positions=list(point.positions), time_from_start=duration(point.time_ns)
        )
        for point in value.targets
    ]
    return result


def command_message(value: Trajectory) -> TrajectoryCommand:
    return TrajectoryCommand(
        context=context_message(value.context),
        lease_id=value.lease_id,
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        accept_before=stamp(value.accept_before_ns),
        trajectory=trajectory_message(value),
    )


def command_from_message(value: TrajectoryCommand) -> Trajectory:
    if time_ns(value.trajectory.header.stamp) != 0:
        raise ContractError("UNSUPPORTED_CAPABILITY", "Only immediate trajectory starts in stage 2")
    if any(
        point.velocities or point.accelerations or point.effort for point in value.trajectory.points
    ):
        raise ContractError("UNSUPPORTED_CAPABILITY", "Position targets only")
    return Trajectory(
        context_from_message(value.context),
        value.lease_id,
        value.command_id,
        value.command_seq,
        value.observation_id,
        time_ns(value.accept_before),
        tuple(value.trajectory.joint_names),
        tuple(
            Target(time_ns(point.time_from_start), tuple(point.positions))
            for point in value.trajectory.points
        ),
    )


def command_status_message(value: CommandStatus) -> CommandStatusMessage:
    """Serialize one immutable status snapshot; arrays stay in joint order."""
    validate_command_status(value)
    return CommandStatusMessage(
        context=context_message(value.context),
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        phase=value.phase,
        reason=value.reason,
        sample_time=stamp(value.sample_time_ns),
        accepted_at=stamp(value.accepted_at_ns),
        started_at=stamp(value.started_at_ns),
        elapsed=duration(value.elapsed_ns),
        execution_clock=value.execution_clock,
        interpolation_version=value.interpolation_version,
        joint_names=list(value.joint_names),
        initial_positions=list(value.initial_positions),
        reference_positions=list(value.reference_positions),
        measured_positions=list(value.measured_positions),
        tracking_error=list(value.tracking_error),
        progress=float(value.progress),
    )


def command_status_from_message(value: CommandStatusMessage) -> CommandStatus:
    """Deserialize and intrinsic-validate one status snapshot."""
    status = CommandStatus(
        context=context_from_message(value.context),
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        phase=value.phase,
        reason=value.reason,
        sample_time_ns=time_ns(value.sample_time),
        accepted_at_ns=time_ns(value.accepted_at),
        started_at_ns=time_ns(value.started_at),
        elapsed_ns=time_ns(value.elapsed),
        execution_clock=value.execution_clock,
        interpolation_version=value.interpolation_version,
        joint_names=tuple(value.joint_names),
        initial_positions=tuple(value.initial_positions),
        reference_positions=tuple(value.reference_positions),
        measured_positions=tuple(value.measured_positions),
        tracking_error=tuple(value.tracking_error),
        progress=float(value.progress),
    )
    validate_command_status(status)
    return status
