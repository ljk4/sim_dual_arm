"""Immutable domain values shared by adapters and validation code."""

from dataclasses import dataclass


class ContractError(ValueError):
    """A machine-readable rejection, separate from its diagnostic detail."""

    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        super().__init__(f"{reason}: {detail}" if detail else reason)


@dataclass(frozen=True)
class RunContext:
    robot_id: str
    robot_profile_hash: str
    backend_instance_id: str
    time_domain_id: str
    clock_epoch: int
    episode_id: str = ""
    control_epoch: int = 0

    @property
    def generation(self) -> tuple[str, int]:
        return self.backend_instance_id, self.clock_epoch


@dataclass(frozen=True)
class JointSpec:
    name: str
    unit: str
    minimum: float
    maximum: float
    max_velocity: float
    initial: float


@dataclass(frozen=True)
class CameraSpec:
    name: str
    width: int
    height: int
    frame_id: str


@dataclass(frozen=True)
class RobotProfile:
    robot_id: str
    profile_id: str
    digest: str
    joints: tuple[JointSpec, ...]
    camera: CameraSpec

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    @property
    def initial_positions(self) -> tuple[float, ...]:
        return tuple(joint.initial for joint in self.joints)


@dataclass(frozen=True)
class Target:
    time_ns: int
    positions: tuple[float, ...]


@dataclass(frozen=True)
class Trajectory:
    context: RunContext
    lease_id: str
    command_id: str
    command_seq: int
    observation_id: int
    accept_before_ns: int
    joint_names: tuple[str, ...]
    targets: tuple[Target, ...]
