"""Pure, auditable records for one accepted bounded-follower reference.

A :class:`FollowerReference` is deliberately **not** an
:class:`~robot_contracts.native.AcceptedReference`.  A follower does not pass
through every original prediction point, so reusing the strict reference record
would make an unsafe claim.  This record keeps the complete raw
:class:`~robot_contracts.native.NativeRequest`, the explicit cubic curve, the
original anchor/window, the algorithm/sampler/normalization versions, the real
limits and terminal speeds, the acceptance times and a canonical SHA-256 digest.

The module uses only the Python standard library.  It never imports Ruckig,
NumPy, SciPy, MuJoCo, ROS or a backend implementation.  Syntax validation is
performed by the immutable constructor; :func:`validate` independently rebuilds
the accepted DSH-030 ``PreparedFollowerPlan`` from the retained fields using the
single shared speed/braking/deviation implementation.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral, Real

from .action_chunks import (
    INT64_MAX,
    MAX_TARGET_COUNT,
    UINT64_MAX,
    ActionWindow,
    ChunkAnchor,
)
from .follower import (
    FOLLOWER_ALGORITHM_VERSION,
    FollowerLimitFlags,
    PreparedFollowerPlan,
)
from .follower_math import normalize_follower_positions
from .follower_reference_digest import (
    DIGEST_HEX_LENGTH,
    FOLLOWER_REFERENCE_DIGEST_VERSION,
    anchor_payload,
    compute_follower_reference_digest,
    request_payload,
    segment_payload,
    window_payload,
)
from .models import ContractError, JointSpec, RobotProfile, RunContext
from .native import NativeRequest
from .native_reference import NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT
from .reference import MAX_HORIZON_NS, MAX_JOINTS, ReferencePlan, ReferenceSample

FOLLOWER_SAMPLER_VERSION = "follower-sampler-v1"
FOLLOWER_NORMALIZATION_VERSION = "hard-boundary-normalization-v1"
FOLLOWER_REFERENCE_KINDS = (NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT)

_DEVIATION_ATOL = 1e-12


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_REFERENCE", f"{location}: {message}")


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid(location, "must be an integer")
    result = int(value)
    if result < minimum or result > maximum:
        raise _invalid(location, f"must be in {minimum}..{maximum}")
    return result


def _optional_integer(value: object, location: str, maximum: int) -> int | None:
    if value is None:
        return None
    return _integer(value, location, 0, maximum)


def _finite(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise _invalid(location, "must be a finite real number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(location, "cannot be represented as a finite float") from error
    if not math.isfinite(result):
        raise _invalid(location, "must be finite")
    return result


def _vector(value: object, location: str, length: int) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if len(items) != length:
        raise _invalid(location, f"has {len(items)} values for {length} joints")
    return tuple(_finite(item, f"{location}[{index}]") for index, item in enumerate(items))


def _bools(value: object, location: str, length: int) -> tuple[bool, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of bools")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of bools") from error
    if len(items) != length:
        raise _invalid(location, f"has {len(items)} values for {length} joints")
    result: list[bool] = []
    for index, item in enumerate(items):
        if not isinstance(item, bool):
            raise _invalid(f"{location}[{index}]", "must be a bool")
        result.append(item)
    return tuple(result)


def _names(value: object, location: str, length: int) -> tuple[str, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of names")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of names") from error
    if len(items) != length:
        raise _invalid(location, f"has {len(items)} names for {length} joints")
    result: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, str) or not item:
            raise _invalid(f"{location}[{index}]", "must be a non-empty string")
        if item in result:
            raise _invalid(f"{location}[{index}]", "must be unique")
        result.append(item)
    return tuple(result)


@dataclass(frozen=True, slots=True)
class FollowerAppliedSample:
    """One raw coefficient sample and its auditable hard-boundary correction.

    ``applied_positions`` are exactly what the physical owner writes as the
    reference position; ``sampled_positions`` remain the untouched cubic values.
    ``corrections = sampled - applied`` per axis.  Velocities and accelerations
    are reported unchanged because position normalization never alters them.
    """

    sampled_positions: tuple[float, ...]
    applied_positions: tuple[float, ...]
    corrections: tuple[float, ...]
    sampled_velocities: tuple[float, ...]
    sampled_accelerations: tuple[float, ...]

    @property
    def has_correction(self) -> bool:
        return any(value != 0.0 for value in self.corrections)


@dataclass(frozen=True, slots=True)
class FollowerReference:
    """One accepted bounded-follower block with explicit, protected evidence.

    The constructor validates identity, dimensions, times and finite values and
    verifies a supplied canonical digest.  It does not rerun Ruckig or physics;
    :func:`validate` performs the independent DSH-030 plan reconstruction.
    """

    request: NativeRequest
    motion_seq: int
    accepted_sim_time_ns: int
    accepted_wall_time_ns: int
    joint_names: tuple[str, ...]
    plan: ReferencePlan
    anchor: ChunkAnchor
    window: ActionWindow
    max_accelerations: tuple[float, ...]
    max_target_deviation_limits: tuple[float, ...]
    raw_terminal_velocities: tuple[float, ...]
    restricted_terminal_velocities: tuple[float, ...]
    actual_terminal_positions: tuple[float, ...]
    actual_terminal_velocities: tuple[float, ...]
    actual_terminal_accelerations: tuple[float, ...]
    max_target_deviations: tuple[float, ...]
    is_final: bool
    requested_window_ns: int
    ruckig_duration_ns: int
    duration_grid_ns: int | None
    velocity_limited: tuple[bool, ...]
    acceleration_limited: tuple[bool, ...]
    boundary_preserved: tuple[bool, ...]
    algorithm_version: str = FOLLOWER_ALGORITHM_VERSION
    sampler_version: str = FOLLOWER_SAMPLER_VERSION
    normalization_version: str = FOLLOWER_NORMALIZATION_VERSION
    reference_digest: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.request, NativeRequest):
            raise _invalid("request", "must be a NativeRequest")
        if self.request.kind not in FOLLOWER_REFERENCE_KINDS:
            raise _invalid(
                "request.kind",
                "follower references require STREAM_FIRST or STREAM_NEXT",
            )
        if self.request.anchor is None or self.request.window is None:
            raise _invalid("request", "follower references require an anchor and window")
        if not isinstance(self.plan, ReferencePlan):
            raise _invalid("plan", "must be a ReferencePlan")
        if not isinstance(self.anchor, ChunkAnchor):
            raise _invalid("anchor", "must be a ChunkAnchor")
        if not isinstance(self.window, ActionWindow):
            raise _invalid("window", "must be an ActionWindow")
        if not isinstance(self.is_final, bool):
            raise _invalid("is_final", "must be a bool")
        if self.algorithm_version != FOLLOWER_ALGORITHM_VERSION:
            raise _invalid(
                "algorithm_version",
                f"unsupported algorithm version {self.algorithm_version!r}",
            )
        if self.sampler_version != FOLLOWER_SAMPLER_VERSION:
            raise _invalid(
                "sampler_version",
                f"unsupported sampler version {self.sampler_version!r}",
            )
        if self.normalization_version != FOLLOWER_NORMALIZATION_VERSION:
            raise _invalid(
                "normalization_version",
                f"unsupported normalization version {self.normalization_version!r}",
            )

        motion_seq = _integer(self.motion_seq, "motion_seq", 1, UINT64_MAX)
        accepted_sim = _integer(
            self.accepted_sim_time_ns, "accepted_sim_time_ns", 0, INT64_MAX
        )
        accepted_wall = _integer(
            self.accepted_wall_time_ns, "accepted_wall_time_ns", 0, INT64_MAX
        )
        joint_names = _names(self.joint_names, "joint_names", len(self.joint_names))
        if not 1 <= len(joint_names) <= MAX_JOINTS:
            raise _invalid("joint_names", f"must contain 1..{MAX_JOINTS} names")
        if self.plan.joint_names != joint_names:
            raise _invalid("joint_names", "must match the explicit plan")
        if tuple(self.request.trajectory.joint_names) != joint_names:
            raise _invalid("joint_names", "must match the raw request")
        for name in ("positions", "velocities", "accelerations"):
            _vector(
                getattr(self.anchor.reference, name),
                f"anchor.reference.{name}",
                len(joint_names),
            )
        if self.request.anchor != self.anchor:
            raise _invalid("anchor", "must equal the raw request anchor")
        if self.request.window != self.window:
            raise _invalid("window", "must equal the recomputed raw request window")
        if self.window.observation_id != self.request.trajectory.observation_id:
            raise _invalid("window.observation_id", "must equal the request observation_id")
        if self.window.target_count != self.request.target_count:
            raise _invalid("window.target_count", "must equal the request target_count")
        if self.window.handoff_time_ns != self.anchor.start_sim_time_ns:
            raise _invalid("window.handoff_time_ns", "must equal the anchor start time")
        if self.is_final != self.request.settle_at_end:
            raise _invalid(
                "is_final",
                "must equal the raw request settle_at_end final-block flag",
            )
        if accepted_sim < self.request.observation_time_ns:
            raise _invalid(
                "accepted_sim_time_ns",
                "must not be earlier than the source observation time",
            )
        if accepted_sim > self.anchor.start_sim_time_ns:
            raise _invalid(
                "accepted_sim_time_ns",
                "must not be later than the fixed hand-off/start time",
            )
        if accepted_sim > self.request.trajectory.accept_before_ns:
            raise _invalid(
                "accepted_sim_time_ns",
                "must not be later than the request accept_before robot time",
            )

        max_accelerations = _vector(
            self.max_accelerations, "max_accelerations", len(joint_names)
        )
        for index, value in enumerate(max_accelerations):
            if value <= 0.0:
                raise _invalid(f"max_accelerations[{index}]", "must be positive")
        deviation_limits = _vector(
            self.max_target_deviation_limits,
            "max_target_deviation_limits",
            len(joint_names),
        )
        for index, value in enumerate(deviation_limits):
            if value <= 0.0:
                raise _invalid(f"max_target_deviation_limits[{index}]", "must be positive")
        raw_velocities = _vector(
            self.raw_terminal_velocities, "raw_terminal_velocities", len(joint_names)
        )
        restricted_velocities = _vector(
            self.restricted_terminal_velocities,
            "restricted_terminal_velocities",
            len(joint_names),
        )
        actual_positions = _vector(
            self.actual_terminal_positions, "actual_terminal_positions", len(joint_names)
        )
        actual_velocities = _vector(
            self.actual_terminal_velocities, "actual_terminal_velocities", len(joint_names)
        )
        actual_accelerations = _vector(
            self.actual_terminal_accelerations,
            "actual_terminal_accelerations",
            len(joint_names),
        )
        deviations = _vector(
            self.max_target_deviations, "max_target_deviations", len(joint_names)
        )
        for index, value in enumerate(deviations):
            if value < 0.0:
                raise _invalid(f"max_target_deviations[{index}]", "must not be negative")
        requested = _integer(
            self.requested_window_ns, "requested_window_ns", 1, MAX_HORIZON_NS
        )
        if requested != self.window.end_time_ns:
            raise _invalid("requested_window_ns", "must equal the window end time")
        ruckig_duration = _integer(
            self.ruckig_duration_ns, "ruckig_duration_ns", 1, MAX_HORIZON_NS
        )
        if ruckig_duration < self.plan.duration_ns:
            raise _invalid("ruckig_duration_ns", "must cover the explicit plan duration")
        grid = _optional_integer(self.duration_grid_ns, "duration_grid_ns", MAX_HORIZON_NS)
        if grid == 0:
            raise _invalid("duration_grid_ns", "must be positive or absent")
        if grid is not None and self.plan.duration_ns % grid != 0:
            raise _invalid("duration_grid_ns", "plan duration must lie on the grid")
        if self.is_final:
            if self.plan.duration_ns < requested:
                raise _invalid("plan.duration_ns", "a final plan must cover the window")
        elif self.plan.duration_ns != requested:
            raise _invalid("plan.duration_ns", "a non-final plan must end at the hand-off")
        velocity_limited = _bools(
            self.velocity_limited, "velocity_limited", len(joint_names)
        )
        acceleration_limited = _bools(
            self.acceleration_limited, "acceleration_limited", len(joint_names)
        )
        boundary_preserved = _bools(
            self.boundary_preserved, "boundary_preserved", len(joint_names)
        )

        if self.reference_digest is not None:
            digest = self.reference_digest
            if (
                not isinstance(digest, str)
                or len(digest) != DIGEST_HEX_LENGTH
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise _invalid(
                    "reference_digest", "must be a lowercase SHA-256 hex digest"
                )

        object.__setattr__(self, "motion_seq", motion_seq)
        object.__setattr__(self, "accepted_sim_time_ns", accepted_sim)
        object.__setattr__(self, "accepted_wall_time_ns", accepted_wall)
        object.__setattr__(self, "joint_names", joint_names)
        object.__setattr__(self, "max_accelerations", max_accelerations)
        object.__setattr__(self, "max_target_deviation_limits", deviation_limits)
        object.__setattr__(self, "raw_terminal_velocities", raw_velocities)
        object.__setattr__(self, "restricted_terminal_velocities", restricted_velocities)
        object.__setattr__(self, "actual_terminal_positions", actual_positions)
        object.__setattr__(self, "actual_terminal_velocities", actual_velocities)
        object.__setattr__(self, "actual_terminal_accelerations", actual_accelerations)
        object.__setattr__(self, "max_target_deviations", deviations)
        object.__setattr__(self, "requested_window_ns", requested)
        object.__setattr__(self, "ruckig_duration_ns", ruckig_duration)
        object.__setattr__(self, "duration_grid_ns", grid)
        object.__setattr__(self, "velocity_limited", velocity_limited)
        object.__setattr__(self, "acceleration_limited", acceleration_limited)
        object.__setattr__(self, "boundary_preserved", boundary_preserved)

        expected = compute_follower_reference_digest(self)
        if self.reference_digest is None:
            object.__setattr__(self, "reference_digest", expected)
        elif self.reference_digest != expected:
            raise ContractError(
                "FOLLOWER_DIGEST_MISMATCH",
                "reference_digest does not cover the retained request/curve/times",
            )

    # ------------------------------------------------------------ identity
    @property
    def context(self) -> RunContext:
        return self.request.trajectory.context

    @property
    def command_id(self) -> str:
        return self.request.trajectory.command_id

    @property
    def command_seq(self) -> int:
        return self.request.trajectory.command_seq

    @property
    def observation_id(self) -> int:
        return self.request.trajectory.observation_id

    @property
    def kind(self) -> int:
        return self.request.kind

    @property
    def mode(self) -> str:
        return self.request.mode

    @property
    def stream_id(self) -> str:
        return self.anchor.stream_id

    @property
    def chunk_seq(self) -> int:
        return self.anchor.seq

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)

    @property
    def duration_ns(self) -> int:
        return self.plan.duration_ns

    @property
    def start_sim_time_ns(self) -> int:
        return self.anchor.start_sim_time_ns

    @property
    def end_sim_time_ns(self) -> int:
        return self.start_sim_time_ns + self.plan.duration_ns

    @property
    def extension_ns(self) -> int:
        return max(0, self.plan.duration_ns - self.requested_window_ns)

    @property
    def flags(self) -> FollowerLimitFlags:
        return FollowerLimitFlags(
            self.velocity_limited,
            self.acceleration_limited,
            self.boundary_preserved,
        )

    # ------------------------------------------------------------- sampling
    def sample_at(self, sim_time_ns: int) -> ReferenceSample:
        """Return the untouched coefficient sample (DSH-030 ``sample_at``)."""
        sample = self._plan_sample_at(sim_time_ns)
        return ReferenceSample(sample.positions, sample.velocities, sample.accelerations)

    def applied_sample_at(
        self,
        sim_time_ns: int,
        profile_or_joints: RobotProfile | Sequence[JointSpec],
    ) -> FollowerAppliedSample:
        """Reconstruct the physically applied reference and its corrections."""
        joints = _joints_from(profile_or_joints)
        sample = self.sample_at(sim_time_ns)
        normalized = normalize_follower_positions(sample.positions, joints)
        return FollowerAppliedSample(
            sampled_positions=normalized.sampled_positions,
            applied_positions=normalized.applied_positions,
            corrections=normalized.corrections,
            sampled_velocities=sample.velocities,
            sampled_accelerations=sample.accelerations,
        )

    def _plan_sample_at(self, sim_time_ns: int) -> ReferenceSample:
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, Integral):
            raise _invalid("sim_time_ns", "must be an integer")
        value = int(sim_time_ns)
        if value < self.start_sim_time_ns or value > self.end_sim_time_ns:
            raise ContractError(
                "REFERENCE_EXPIRED",
                f"sim_time_ns {value} is outside "
                f"[{self.start_sim_time_ns}, {self.end_sim_time_ns}]",
            )
        return self.plan.sample_interval(value - self.start_sim_time_ns)

    # ---------------------------------------------------------- construction
    @classmethod
    def from_prepared(
        cls,
        prepared: PreparedFollowerPlan,
        request: NativeRequest,
        *,
        motion_seq: int,
        accepted_sim_time_ns: int,
        accepted_wall_time_ns: int,
        joint_names: Sequence[str] | None = None,
    ) -> FollowerReference:
        """Capture one prepared follower plan as an immutable accepted record."""
        if not isinstance(prepared, PreparedFollowerPlan):
            raise _invalid("prepared", "must be a PreparedFollowerPlan")
        if not isinstance(request, NativeRequest):
            raise _invalid("request", "must be a NativeRequest")
        names = prepared.joint_names if joint_names is None else tuple(joint_names)
        return cls(
            request=request,
            motion_seq=motion_seq,
            accepted_sim_time_ns=accepted_sim_time_ns,
            accepted_wall_time_ns=accepted_wall_time_ns,
            joint_names=names,
            plan=prepared.plan,
            anchor=prepared.anchor,
            window=prepared.window,
            max_accelerations=prepared.max_accelerations,
            max_target_deviation_limits=prepared.max_target_deviation_limits,
            raw_terminal_velocities=prepared.raw_terminal_velocities,
            restricted_terminal_velocities=prepared.restricted_terminal_velocities,
            actual_terminal_positions=prepared.actual_terminal_positions,
            actual_terminal_velocities=prepared.actual_terminal_velocities,
            actual_terminal_accelerations=prepared.actual_terminal_accelerations,
            max_target_deviations=prepared.max_target_deviations,
            is_final=prepared.is_final,
            requested_window_ns=prepared.requested_window_ns,
            ruckig_duration_ns=prepared.ruckig_duration_ns,
            duration_grid_ns=prepared.duration_grid_ns,
            velocity_limited=prepared.flags.velocity_limited,
            acceleration_limited=prepared.flags.acceleration_limited,
            boundary_preserved=prepared.flags.boundary_preserved,
        )

    def to_document(self) -> dict[str, object]:
        """Return a complete fresh JSON-serialisable audit document."""
        return {
            "kind": "bounded_follower_reference",
            "digest_version": FOLLOWER_REFERENCE_DIGEST_VERSION,
            "request": request_payload(self.request),
            "motion_seq": self.motion_seq,
            "accepted_sim_time_ns": self.accepted_sim_time_ns,
            "accepted_wall_time_ns": self.accepted_wall_time_ns,
            "joint_names": list(self.joint_names),
            "anchor": anchor_payload(self.anchor),
            "window": window_payload(self.window),
            "plan": {
                "duration_ns": self.plan.duration_ns,
                "segments": [segment_payload(segment) for segment in self.plan.segments],
            },
            "max_accelerations": list(self.max_accelerations),
            "max_target_deviation_limits": list(self.max_target_deviation_limits),
            "raw_terminal_velocities": list(self.raw_terminal_velocities),
            "restricted_terminal_velocities": list(self.restricted_terminal_velocities),
            "actual_terminal_positions": list(self.actual_terminal_positions),
            "actual_terminal_velocities": list(self.actual_terminal_velocities),
            "actual_terminal_accelerations": list(self.actual_terminal_accelerations),
            "max_target_deviations": list(self.max_target_deviations),
            "is_final": self.is_final,
            "requested_window_ns": self.requested_window_ns,
            "ruckig_duration_ns": self.ruckig_duration_ns,
            "duration_grid_ns": self.duration_grid_ns,
            "extension_ns": self.extension_ns,
            "velocity_limited": list(self.velocity_limited),
            "acceleration_limited": list(self.acceleration_limited),
            "boundary_preserved": list(self.boundary_preserved),
            "algorithm_version": self.algorithm_version,
            "sampler_version": self.sampler_version,
            "normalization_version": self.normalization_version,
            "reference_digest": self.reference_digest,
        }


def _joints_from(
    profile_or_joints: RobotProfile | Sequence[JointSpec],
) -> tuple[JointSpec, ...]:
    if isinstance(profile_or_joints, RobotProfile):
        return tuple(profile_or_joints.joints)
    if isinstance(profile_or_joints, str | bytes):
        raise _invalid("joints", "must be a profile or an array of JointSpec values")
    try:
        joints = tuple(profile_or_joints)
    except TypeError as error:
        raise _invalid("joints", "must be a profile or an array of JointSpec values") from error
    if not joints or any(not isinstance(joint, JointSpec) for joint in joints):
        raise _invalid("joints", "must contain JointSpec values")
    return joints


def _same_vector(left: Sequence[float], right: Sequence[float]) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=_DEVIATION_ATOL)
        for a, b in zip(left, right, strict=True)
    )


@dataclass(frozen=True, slots=True)
class FollowerValidationLimits:
    """Explicit pure-standard-library constraints for one follower record.

    Backend/runtime callers construct this from their already validated
    configuration; contracts never import a backend config class.  Every field
    is required (``duration_grid_ns`` may be ``None`` for continuous planning),
    so validation cannot silently skip a missing attribute.
    """

    joint_names: tuple[str, ...]
    max_accelerations: tuple[float, ...]
    max_target_deviations: tuple[float, ...]
    target_count: int
    max_terminal_tail_duration_ns: int
    duration_grid_ns: int | None = None

    def __post_init__(self) -> None:
        names = _names(self.joint_names, "limits.joint_names", len(self.joint_names))
        if not 1 <= len(names) <= MAX_JOINTS:
            raise _invalid("limits.joint_names", f"must contain 1..{MAX_JOINTS} names")
        accelerations = _vector(
            self.max_accelerations, "limits.max_accelerations", len(names)
        )
        for index, value in enumerate(accelerations):
            if value <= 0.0:
                raise _invalid(f"limits.max_accelerations[{index}]", "must be positive")
        deviations = _vector(
            self.max_target_deviations, "limits.max_target_deviations", len(names)
        )
        for index, value in enumerate(deviations):
            if value <= 0.0:
                raise _invalid(f"limits.max_target_deviations[{index}]", "must be positive")
        if (
            isinstance(self.target_count, bool)
            or not isinstance(self.target_count, Integral)
            or not 2 <= int(self.target_count) <= MAX_TARGET_COUNT
        ):
            raise _invalid(
                "limits.target_count",
                f"must be an integer in 2..{MAX_TARGET_COUNT}",
            )
        tail = _integer(
            self.max_terminal_tail_duration_ns,
            "limits.max_terminal_tail_duration_ns",
            1,
            MAX_HORIZON_NS,
        )
        grid = _optional_integer(
            self.duration_grid_ns, "limits.duration_grid_ns", MAX_HORIZON_NS
        )
        if grid == 0:
            raise _invalid("limits.duration_grid_ns", "must be positive or absent")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "max_accelerations", accelerations)
        object.__setattr__(self, "max_target_deviations", deviations)
        object.__setattr__(self, "target_count", int(self.target_count))
        object.__setattr__(self, "max_terminal_tail_duration_ns", tail)
        object.__setattr__(self, "duration_grid_ns", grid)


def validate(
    record: FollowerReference,
    profile: RobotProfile,
    limits: FollowerValidationLimits,
) -> PreparedFollowerPlan:
    """Independently rebuild the DSH-030 plan and verify retained evidence.

    ``limits`` must be the concrete :class:`FollowerValidationLimits` produced
    from the control/follower configuration (or a runtime snapshot).  There is
    no optional duck-typed path: missing or mismatched limits are rejected.
    """
    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if not isinstance(limits, FollowerValidationLimits):
        raise _invalid("limits", "must be a FollowerValidationLimits")
    if limits.joint_names != tuple(profile.joint_names):
        raise ContractError("PROFILE_MISMATCH", "limits joint order differs from profile")
    if record.joint_names != limits.joint_names:
        raise ContractError("PROFILE_MISMATCH", "record joint order differs from limits")
    if record.anchor.robot_profile_hash != profile.digest:
        raise ContractError("PROFILE_MISMATCH", "record anchor profile digest differs")
    if not _same_vector(limits.max_accelerations, record.max_accelerations):
        raise ContractError(
            "FOLLOWER_LIMIT_MISMATCH",
            "record max_accelerations differ from the validation limits",
        )
    if not _same_vector(
        limits.max_target_deviations, record.max_target_deviation_limits
    ):
        raise ContractError(
            "FOLLOWER_LIMIT_MISMATCH",
            "record deviation limits differ from the validation limits",
        )
    if record.window.target_count != limits.target_count:
        raise ContractError(
            "FOLLOWER_LIMIT_MISMATCH",
            "record window.target_count differs from the validation limits",
        )
    if record.request.target_count != limits.target_count:
        raise ContractError(
            "FOLLOWER_LIMIT_MISMATCH",
            "raw request target_count differs from the validation limits",
        )
    if record.duration_grid_ns != limits.duration_grid_ns:
        raise ContractError(
            "FOLLOWER_GRID_MISMATCH",
            "record duration grid differs from the validation limits",
        )
    if (
        record.requested_window_ns > limits.max_terminal_tail_duration_ns
        or record.plan.duration_ns > limits.max_terminal_tail_duration_ns
    ):
        raise ContractError(
            "FOLLOWER_HORIZON_EXCEEDED",
            "record requested/executed horizon exceeds the configured terminal tail",
        )

    # Recheck the complete raw intent and acceptance time before rebuilding.
    if record.is_final != record.request.settle_at_end:
        raise _invalid("is_final", "differs from the raw request settle_at_end flag")
    if not (
        record.request.observation_time_ns
        <= record.accepted_sim_time_ns
        <= record.anchor.start_sim_time_ns
        and record.accepted_sim_time_ns <= record.request.trajectory.accept_before_ns
    ):
        raise _invalid(
            "accepted_sim_time_ns",
            "must follow the observation and precede both hand-off and admission deadline",
        )

    rebuilt = PreparedFollowerPlan(
        anchor=record.anchor,
        window=record.window,
        plan=record.plan,
        joints=tuple(profile.joints),
        max_accelerations=record.max_accelerations,
        max_target_deviation_limits=record.max_target_deviation_limits,
        raw_terminal_velocities=record.raw_terminal_velocities,
        restricted_terminal_velocities=record.restricted_terminal_velocities,
        is_final=record.is_final,
        requested_window_ns=record.requested_window_ns,
        ruckig_duration_ns=record.ruckig_duration_ns,
        duration_grid_ns=record.duration_grid_ns,
    )
    if rebuilt.plan != record.plan:
        raise _invalid("plan", "reconstructed plan differs")
    for name in (
        "actual_terminal_positions",
        "actual_terminal_velocities",
        "actual_terminal_accelerations",
        "max_target_deviations",
        "raw_terminal_velocities",
        "restricted_terminal_velocities",
    ):
        if not _same_vector(getattr(record, name), getattr(rebuilt, name)):
            raise _invalid(name, "differs from the independently rebuilt DSH-030 plan")
    if record.flags != rebuilt.flags:
        raise _invalid("flags", "limit flags differ from the independently rebuilt plan")
    if record.extension_ns != rebuilt.extension_ns:
        raise _invalid("extension_ns", "extended horizon differs from the rebuilt plan")
    return rebuilt


__all__ = [
    "DIGEST_HEX_LENGTH",
    "FOLLOWER_NORMALIZATION_VERSION",
    "FOLLOWER_REFERENCE_DIGEST_VERSION",
    "FOLLOWER_REFERENCE_KINDS",
    "FOLLOWER_SAMPLER_VERSION",
    "FollowerAppliedSample",
    "FollowerReference",
    "FollowerValidationLimits",
    "compute_follower_reference_digest",
    "validate",
]
