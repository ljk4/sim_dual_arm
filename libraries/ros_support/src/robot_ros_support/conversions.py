"""Explicit conversions between wire messages and immutable domain values."""

from builtin_interfaces.msg import Duration, Time
from robot_api_msgs.msg import RunContext as ContextMessage
from robot_contracts.models import ContractError, RunContext, Target, Trajectory
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


def trajectory_from_message(value: JointTrajectory) -> tuple[tuple[str, ...], tuple[Target, ...]]:
    """Decode a position-only target source, rejecting unsupported semantics."""
    if time_ns(value.header.stamp) != 0:
        raise ContractError(
            "UNSUPPORTED_CAPABILITY", "only immediate trajectory starts are supported"
        )
    if value.header.frame_id:
        raise ContractError("UNSUPPORTED_CAPABILITY", "trajectory frame_id must be empty")
    targets: list[Target] = []
    for point in value.points:
        if point.velocities or point.accelerations or point.effort:
            raise ContractError("UNSUPPORTED_CAPABILITY", "position targets only")
        targets.append(Target(time_ns(point.time_from_start), tuple(point.positions)))
    return tuple(value.joint_names), tuple(targets)
