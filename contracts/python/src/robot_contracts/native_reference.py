"""Pure native request and auditable accepted-reference records.

This module uses only the Python standard library and the existing shared
contracts.  It never imports ROS, a backend, SciPy, NumPy or MuJoCo, and it
never reads a clock/scene.  The ROS transport and the native backend both use
these values so request identity, mode semantics and the canonical reference
digest have one implementation.

Mode conventions (times are integer nanoseconds):

``MOTION``
    Source target times are relative to the actual ``STARTED`` application
    time.  The caller supplies the last hold reference; the initial and final
    velocities are zero.  ``anchor``/``observation_time_ns``/``target_count``
    are absent/zero and ``settle_at_end`` is true.

``STREAM_FIRST``/``STREAM_NEXT``
    Source target times are relative to the source observation capture time.
    ``anchor.start_sim_time_ns`` is the fixed absolute hand-off time.  The
    first stream predecessor is zero; every successor has a positive
    predecessor.  ``target_count`` selects only points strictly after the
    hand-off, while the complete source (including skipped prefix and unused
    tail) is retained and validated.
"""

from __future__ import annotations

import hashlib
import json
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
from .control import (
    MAX_ID_LENGTH,
    ControlCommand,
    validate_context,
)
from .models import ContractError, JointSpec, RobotProfile, Target, Trajectory
from .reference import (
    MAX_HORIZON_NS,
    MAX_JOINTS,
    MAX_SEGMENTS,
    VELOCITY_ATOL,
    CubicSegment,
    ReferencePlan,
    ReferenceSample,
    validate_reference_inputs,
    verify_reference_plan,
)

NATIVE_MOTION = 1
NATIVE_STREAM_FIRST = 2
NATIVE_STREAM_NEXT = 3
NATIVE_KINDS = (NATIVE_MOTION, NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT)
NATIVE_KIND_NAMES = {
    NATIVE_MOTION: "MOTION",
    NATIVE_STREAM_FIRST: "STREAM_FIRST",
    NATIVE_STREAM_NEXT: "STREAM_NEXT",
}
INTERPOLATION_VERSION = "explicit_cubic_v1"
REFERENCE_DIGEST_VERSION = "native-reference-v1"
DIGEST_HEX_LENGTH = 64


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_NATIVE_REQUEST", f"{location}: {message}")


def _record_invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_REFERENCE_RECORD", f"{location}: {message}")


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


def _finite_tuple(
    value: object,
    location: str,
    *,
    maximum: int = MAX_JOINTS,
) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if not 1 <= len(items) <= maximum:
        raise _invalid(location, f"must contain 1..{maximum} values")
    return tuple(_finite(item, f"{location}[{index}]") for index, item in enumerate(items))


def _text(value: object, location: str, *, maximum: int = MAX_ID_LENGTH) -> str:
    if not isinstance(value, str):
        raise _invalid(location, "must be a string")
    if not value:
        raise _invalid(location, "must not be empty")
    if len(value) > maximum:
        raise _invalid(location, f"must be at most {maximum} characters")
    return value


def _names(value: object, location: str, *, maximum: int = MAX_JOINTS) -> tuple[str, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of names")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of names") from error
    if not 1 <= len(items) <= maximum:
        raise _invalid(location, f"must contain 1..{maximum} names")
    result: list[str] = []
    for index, item in enumerate(items):
        name = _text(item, f"{location}[{index}]")
        if name in result:
            raise _invalid(f"{location}[{index}]", "must be unique")
        result.append(name)
    return tuple(result)


def _normalise_trajectory(value: object) -> Trajectory:
    if not isinstance(value, Trajectory):
        raise _invalid("trajectory", "must be a Trajectory")
    validate_context(value.context)
    lease_id = _text(value.lease_id, "trajectory.lease_id")
    command_id = _text(value.command_id, "trajectory.command_id")
    command_seq = _integer(value.command_seq, "trajectory.command_seq", 1, UINT64_MAX)
    observation_id = _integer(value.observation_id, "trajectory.observation_id", 1, UINT64_MAX)
    accept_before_ns = _integer(
        value.accept_before_ns, "trajectory.accept_before_ns", 0, INT64_MAX
    )
    if isinstance(value.joint_names, str | bytes):
        raise _invalid("trajectory.joint_names", "must be an array of names")
    joint_names = _names(value.joint_names, "trajectory.joint_names")

    if isinstance(value.targets, str | bytes):
        raise _invalid("trajectory.targets", "must be an array of Target values")
    try:
        target_items = tuple(value.targets)
    except TypeError as error:
        raise _invalid("trajectory.targets", "must be an array of Target values") from error
    if not 1 <= len(target_items) <= MAX_SEGMENTS:
        raise _invalid("trajectory.targets", f"must contain 1..{MAX_SEGMENTS} targets")
    previous_time = 0
    targets: list[Target] = []
    for index, target in enumerate(target_items):
        if not isinstance(target, Target):
            raise _invalid(f"trajectory.targets[{index}]", "must be a Target")
        time_ns = _integer(
            target.time_ns,
            f"trajectory.targets[{index}].time_ns",
            1,
            MAX_HORIZON_NS,
        )
        if time_ns <= previous_time:
            raise _invalid(
                f"trajectory.targets[{index}].time_ns",
                "must be strictly increasing and after relative zero",
            )
        positions = _finite_tuple(
            target.positions, f"trajectory.targets[{index}].positions"
        )
        if len(positions) != len(joint_names):
            raise _invalid(
                f"trajectory.targets[{index}].positions",
                f"has {len(positions)} values for {len(joint_names)} joints",
            )
        targets.append(Target(time_ns, positions))
        previous_time = time_ns

    # Constructing the shared command validates identity length/ranges too.
    ControlCommand(
        value.context,
        lease_id,
        command_id,
        command_seq,
        observation_id,
        accept_before_ns,
    )
    return Trajectory(
        context=value.context,
        lease_id=lease_id,
        command_id=command_id,
        command_seq=command_seq,
        observation_id=observation_id,
        accept_before_ns=accept_before_ns,
        joint_names=joint_names,
        targets=tuple(targets),
    )


@dataclass(frozen=True, slots=True)
class NativeRequest:
    """One frozen native command request in exactly one of the three modes."""

    trajectory: Trajectory
    kind: int
    anchor: ChunkAnchor | None = None
    observation_time_ns: int = 0
    target_count: int = 0
    settle_at_end: bool = True
    final_velocities: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        trajectory = _normalise_trajectory(self.trajectory)
        if isinstance(self.kind, bool) or not isinstance(self.kind, Integral):
            raise _invalid("kind", "must be an integer")
        kind = int(self.kind)
        if kind not in NATIVE_KINDS:
            raise _invalid("kind", f"must be one of {NATIVE_KINDS}")
        if not isinstance(self.settle_at_end, bool):
            raise _invalid("settle_at_end", "must be a bool")
        observation_time_ns = _integer(
            self.observation_time_ns, "observation_time_ns", 0, INT64_MAX
        )

        joint_count = len(trajectory.joint_names)
        final_velocities: tuple[float, ...] | None = None
        if self.final_velocities is not None:
            final_velocities = _finite_tuple(
                self.final_velocities, "final_velocities"
            )
            if len(final_velocities) != joint_count:
                raise _invalid(
                    "final_velocities",
                    f"has {len(final_velocities)} values for {joint_count} joints",
                )
            if self.settle_at_end and any(
                abs(velocity) > VELOCITY_ATOL for velocity in final_velocities
            ):
                raise _invalid(
                    "final_velocities",
                    "must be zero when settle_at_end is true",
                )

        anchor = self.anchor
        if kind == NATIVE_MOTION:
            if anchor is not None:
                raise _invalid("anchor", "MOTION must not carry a ChunkAnchor")
            if observation_time_ns != 0:
                raise _invalid("observation_time_ns", "MOTION must use zero")
            target_count = _integer(self.target_count, "target_count", 0, 0)
            if self.settle_at_end is not True:
                raise _invalid("settle_at_end", "MOTION must settle at the end")
            if final_velocities is not None:
                raise _invalid("final_velocities", "MOTION always uses zero boundaries")
        else:
            if not isinstance(anchor, ChunkAnchor):
                raise _invalid("anchor", "stream modes require a ChunkAnchor")
            if anchor.robot_profile_hash != trajectory.context.robot_profile_hash:
                raise _invalid(
                    "anchor.robot_profile_hash",
                    "must equal the trajectory context robot_profile_hash",
                )
            if len(anchor.reference.positions) != joint_count:
                raise _invalid(
                    "anchor.reference.positions",
                    f"has {len(anchor.reference.positions)} values for {joint_count} joints",
                )
            if kind == NATIVE_STREAM_FIRST and anchor.predecessor_seq != 0:
                raise _invalid(
                    "anchor.predecessor_seq", "STREAM_FIRST must have predecessor_seq 0"
                )
            if kind == NATIVE_STREAM_NEXT and anchor.predecessor_seq <= 0:
                raise _invalid(
                    "anchor.predecessor_seq", "STREAM_NEXT must have a positive predecessor"
                )
            target_count = _integer(
                self.target_count, "target_count", 2, MAX_TARGET_COUNT
            )
            if target_count > len(trajectory.targets):
                raise _invalid(
                    "target_count", "must not exceed the retained source target count"
                )
            if observation_time_ns > anchor.start_sim_time_ns:
                raise _invalid(
                    "observation_time_ns",
                    "must not be later than the fixed hand-off/start time",
                )
            if observation_time_ns + trajectory.targets[-1].time_ns > INT64_MAX:
                raise _invalid(
                    "observation_time_ns",
                    "observation time plus relative target time exceeds int64",
                )
        object.__setattr__(self, "trajectory", trajectory)
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "target_count", target_count)
        object.__setattr__(self, "observation_time_ns", observation_time_ns)
        object.__setattr__(self, "final_velocities", final_velocities)

    @property
    def command(self) -> ControlCommand:
        return ControlCommand(
            self.trajectory.context,
            self.trajectory.lease_id,
            self.trajectory.command_id,
            self.trajectory.command_seq,
            self.trajectory.observation_id,
            self.trajectory.accept_before_ns,
        )

    @property
    def mode(self) -> str:
        return NATIVE_KIND_NAMES[self.kind]

    @property
    def window(self) -> ActionWindow | None:
        """Return the recomputed stream window; MOTION has none."""
        if self.kind == NATIVE_MOTION:
            return None
        assert self.anchor is not None
        return ActionWindow(
            self.trajectory.observation_id,
            self.observation_time_ns,
            self.anchor.start_sim_time_ns,
            self.trajectory.targets,
            self.target_count,
        )


def _joint_names(value: object, location: str) -> tuple[str, ...]:
    try:
        return _names(value, location)
    except ContractError as error:
        raise _record_invalid(location, str(error)) from error


def _record_vector(value: object, location: str) -> tuple[float, ...]:
    try:
        return _finite_tuple(value, location)
    except ContractError as error:
        raise _record_invalid(location, str(error)) from error


def _request_payload(request: NativeRequest) -> dict[str, object]:
    trajectory = request.trajectory
    context = trajectory.context
    anchor = request.anchor
    return {
        "kind": request.kind,
        "trajectory": {
            "context": {
                "robot_id": context.robot_id,
                "robot_profile_hash": context.robot_profile_hash,
                "backend_instance_id": context.backend_instance_id,
                "time_domain_id": context.time_domain_id,
                "clock_epoch": context.clock_epoch,
                "episode_id": context.episode_id,
                "control_epoch": context.control_epoch,
            },
            "lease_id": trajectory.lease_id,
            "command_id": trajectory.command_id,
            "command_seq": trajectory.command_seq,
            "observation_id": trajectory.observation_id,
            "accept_before_ns": trajectory.accept_before_ns,
            "joint_names": list(trajectory.joint_names),
            "targets": [
                {"time_ns": target.time_ns, "positions": list(target.positions)}
                for target in trajectory.targets
            ],
        },
        "anchor": None
        if anchor is None
        else {
            "stream_id": anchor.stream_id,
            "predecessor_seq": anchor.predecessor_seq,
            "start_sim_time_ns": anchor.start_sim_time_ns,
            "robot_profile_hash": anchor.robot_profile_hash,
            "positions": list(anchor.reference.positions),
            "velocities": list(anchor.reference.velocities),
            "accelerations": list(anchor.reference.accelerations),
        },
        "observation_time_ns": request.observation_time_ns,
        "target_count": request.target_count,
        "settle_at_end": request.settle_at_end,
        "final_velocities": None
        if request.final_velocities is None
        else list(request.final_velocities),
    }


def reference_digest_payload(record: AcceptedReference) -> dict[str, object]:
    """Return the versioned canonical document hashed by the record digest."""
    return {
        "digest_version": REFERENCE_DIGEST_VERSION,
        "request": _request_payload(record.request),
        "motion_seq": record.motion_seq,
        "accepted_sim_time_ns": record.accepted_sim_time_ns,
        "joint_names": list(record.joint_names),
        "segments": [
            {
                "duration_ns": segment.duration_ns,
                "c0": list(segment.c0),
                "c1": list(segment.c1),
                "c2": list(segment.c2),
                "c3": list(segment.c3),
            }
            for segment in record.plan.segments
        ],
        "initial_positions": list(record.initial_positions),
        "initial_velocities": list(record.initial_velocities),
        "final_velocities": list(record.final_velocities),
        "max_accelerations": list(record.max_accelerations),
        "selected_indices": list(record.selected_indices),
        "interpolation_version": record.interpolation_version,
    }


def compute_reference_digest(record: AcceptedReference) -> str:
    """SHA-256 of stable canonical JSON over every public reference field."""
    document = reference_digest_payload(record)
    try:
        encoded = json.dumps(
            document,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("ascii")
    except (TypeError, ValueError) as error:  # pragma: no cover - validated inputs
        raise _record_invalid("reference_digest", "record is not finite JSON") from error
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True, slots=True)
class AcceptedReference:
    """One independently auditable accepted reference record.

    ``reference_digest`` may be omitted during construction and is then filled
    from the explicit public fields.  A supplied value is verified, so a record
    deserialized with a stale digest is rejected rather than silently trusted.
    """

    request: NativeRequest
    motion_seq: int
    accepted_sim_time_ns: int
    joint_names: tuple[str, ...]
    plan: ReferencePlan
    initial_positions: tuple[float, ...]
    initial_velocities: tuple[float, ...]
    final_velocities: tuple[float, ...]
    max_accelerations: tuple[float, ...]
    selected_indices: tuple[int, ...]
    interpolation_version: str = INTERPOLATION_VERSION
    reference_digest: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.request, NativeRequest):
            raise _record_invalid("request", "must be a NativeRequest")
        motion_seq = _integer(self.motion_seq, "motion_seq", 1, UINT64_MAX)
        accepted_sim_time_ns = _integer(
            self.accepted_sim_time_ns, "accepted_sim_time_ns", 0, INT64_MAX
        )
        joint_names = _joint_names(self.joint_names, "joint_names")
        if not isinstance(self.plan, ReferencePlan):
            raise _record_invalid("plan", "must be a ReferencePlan")
        if self.plan.joint_names != joint_names:
            raise _record_invalid("plan.joint_names", "must match record joint_names")
        vectors = {
            name: _record_vector(getattr(self, name), name)
            for name in (
                "initial_positions",
                "initial_velocities",
                "final_velocities",
                "max_accelerations",
            )
        }
        for name, values in vectors.items():
            if len(values) != len(joint_names):
                raise _record_invalid(
                    name, f"has {len(values)} values for {len(joint_names)} joints"
                )
        if any(value < 0.0 for value in vectors["max_accelerations"]):
            raise _record_invalid("max_accelerations", "must not be negative")
        if isinstance(self.selected_indices, str | bytes):
            raise _record_invalid("selected_indices", "must be an array of integers")
        try:
            selected_items = tuple(self.selected_indices)
        except TypeError as error:
            raise _record_invalid(
                "selected_indices", "must be an array of integers"
            ) from error
        if len(selected_items) > MAX_SEGMENTS:
            raise _record_invalid(
                "selected_indices", f"must contain at most {MAX_SEGMENTS} values"
            )
        selected: list[int] = []
        previous = -1
        for index, value in enumerate(selected_items):
            item = _integer(value, f"selected_indices[{index}]", 0, MAX_SEGMENTS - 1)
            if item <= previous:
                raise _record_invalid(
                    "selected_indices", "must be strictly increasing and unique"
                )
            selected.append(item)
            previous = item
        if self.interpolation_version != INTERPOLATION_VERSION:
            raise _record_invalid(
                "interpolation_version", f"must be {INTERPOLATION_VERSION!r}"
            )
        if self.reference_digest is not None:
            digest = self.reference_digest
            if (
                not isinstance(digest, str)
                or len(digest) != DIGEST_HEX_LENGTH
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise _record_invalid(
                    "reference_digest", "must be a lowercase SHA-256 hex digest"
                )

        object.__setattr__(self, "motion_seq", motion_seq)
        object.__setattr__(self, "accepted_sim_time_ns", accepted_sim_time_ns)
        object.__setattr__(self, "joint_names", joint_names)
        for name, values in vectors.items():
            object.__setattr__(self, name, values)
        object.__setattr__(self, "selected_indices", tuple(selected))
        object.__setattr__(self, "interpolation_version", INTERPOLATION_VERSION)

        expected = compute_reference_digest(self)
        if self.reference_digest is None:
            object.__setattr__(self, "reference_digest", expected)
        elif self.reference_digest != expected:
            raise ContractError(
                "REFERENCE_DIGEST_MISMATCH",
                "reference_digest does not cover the retained request/plan/times",
            )


def _backward_final_velocities(window: ActionWindow) -> tuple[float, ...]:
    last, previous = window.targets[-1], window.targets[-2]
    delta_ns = last.time_ns - previous.time_ns
    if delta_ns <= 0:
        raise ContractError(
            "INVALID_REFERENCE_RECORD", "selected times are not strictly increasing"
        )
    delta_s = delta_ns / 1_000_000_000
    return tuple(
        (right - left) / delta_s
        for left, right in zip(previous.positions, last.positions, strict=True)
    )


def expected_final_velocities(request: NativeRequest) -> tuple[float, ...]:
    """Derive the SRID stream final velocity exactly as prepare_chunk does."""
    if request.kind == NATIVE_MOTION:
        return (0.0,) * len(request.trajectory.joint_names)
    assert request.anchor is not None
    if request.settle_at_end:
        return (0.0,) * len(request.trajectory.joint_names)
    if request.final_velocities is not None:
        return request.final_velocities
    window = request.window
    assert window is not None
    return _backward_final_velocities(window)


def _vector_close(left: Sequence[float], right: Sequence[float], *, atol: float) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=atol)
        for a, b in zip(left, right, strict=True)
    )


def validate_accepted_reference(
    record: AcceptedReference,
    profile: RobotProfile,
    max_accelerations: Sequence[float],
) -> None:
    """Independently verify one accepted reference against the profile.

    The complete retained source (including a non-executed prefix/tail) is
    checked for joint limits.  Stream selection is recomputed with the shared
    ``ActionWindow``; the plan is verified with the existing continuous
    position/velocity/acceleration extrema implementation.  Cross-chunk
    continuity is intentionally limited to position and velocity by the
    protocol hand-off, never asserted for acceleration here.
    """
    if not isinstance(record, AcceptedReference):
        raise _record_invalid("record", "must be an AcceptedReference")
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if tuple(record.joint_names) != tuple(profile.joint_names):
        raise ContractError(
            "PROFILE_MISMATCH", "record joint names/order differ from profile"
        )
    if record.request.trajectory.joint_names != record.joint_names:
        raise _record_invalid("request.trajectory.joint_names", "must match record")
    if record.request.trajectory.context.robot_profile_hash != profile.digest:
        raise ContractError("PROFILE_MISMATCH", "request profile digest differs")
    try:
        limits = tuple(
            _finite(value, f"max_accelerations[{index}]")
            for index, value in enumerate(max_accelerations)
        )
    except TypeError as error:
        raise _record_invalid("max_accelerations", "must be an array") from error
    except ContractError as error:
        raise _record_invalid("max_accelerations", str(error)) from error
    if len(limits) != len(record.joint_names):
        raise _record_invalid(
            "max_accelerations", "must match the record joint dimension"
        )
    if not _vector_close(limits, record.max_accelerations, atol=0.0):
        raise _record_invalid(
            "max_accelerations", "differs from the retained accepted limits"
        )

    # Recompute the canonical digest in case a caller used object.__setattr__.
    if record.reference_digest != compute_reference_digest(record):
        raise ContractError(
            "REFERENCE_DIGEST_MISMATCH", "accepted reference fields were modified"
        )

    request = record.request
    joints: tuple[JointSpec, ...] = tuple(profile.joints)
    if request.kind == NATIVE_MOTION:
        if record.selected_indices:
            raise _record_invalid(
                "selected_indices", "MOTION must not carry a stream selection"
            )
        if any(abs(value) > VELOCITY_ATOL for value in record.initial_velocities):
            raise _record_invalid("initial_velocities", "MOTION must start at zero")
        if any(abs(value) > VELOCITY_ATOL for value in record.final_velocities):
            raise _record_invalid("final_velocities", "MOTION must end at zero")
        # Full source limits (the motion source is the complete plan input).
        validate_reference_inputs(
            record.initial_positions,
            request.trajectory.targets,
            joints,
            limits,
            initial_velocities=(0.0,) * len(record.joint_names),
            final_velocities=(0.0,) * len(record.joint_names),
        )
        verify_reference_plan(
            record.plan,
            record.initial_positions,
            request.trajectory.targets,
            joints,
            limits,
            initial_velocities=(0.0,) * len(record.joint_names),
            final_velocities=(0.0,) * len(record.joint_names),
        )
        return

    assert request.anchor is not None
    window = request.window
    assert window is not None
    selected = tuple(record.selected_indices)
    if selected != tuple(window.selected_indices):
        raise ContractError(
            "REFERENCE_SELECTION_MISMATCH",
            "selected_indices differ from the source window recomputation",
        )
    if len(selected) != request.target_count:
        raise _record_invalid(
            "selected_indices", "must contain exactly target_count entries"
        )
    anchor = request.anchor
    if not _vector_close(
        record.initial_positions, anchor.reference.positions, atol=0.0
    ):
        raise _record_invalid(
            "initial_positions", "must equal the anchor reference positions"
        )
    if not _vector_close(
        record.initial_velocities, anchor.reference.velocities, atol=0.0
    ):
        raise _record_invalid(
            "initial_velocities", "must equal the anchor reference velocities"
        )
    expected_final = expected_final_velocities(request)
    if not _vector_close(record.final_velocities, expected_final, atol=VELOCITY_ATOL):
        raise _record_invalid(
            "final_velocities",
            "differ from the explicit or backward-difference/settle mode",
        )
    # Complete retained source, including skipped prefix and unused tail.
    validate_reference_inputs(
        record.initial_positions,
        request.trajectory.targets,
        joints,
        limits,
        initial_velocities=record.initial_velocities,
        final_velocities=record.final_velocities,
    )
    verify_reference_plan(
        record.plan,
        record.initial_positions,
        window.targets,
        joints,
        limits,
        initial_velocities=record.initial_velocities,
        final_velocities=record.final_velocities,
    )


def sample_accepted_reference(
    record: AcceptedReference,
    sim_time_ns: int,
    *,
    actual_started_sim_ns: int | None = None,
) -> ReferenceSample:
    """Rebuild the reference at an absolute application time.

    MOTION requires ``actual_started_sim_ns`` because acceptance is not the
    plan origin.  Its declared final mode is an endpoint hold (the executor
    keeps sampling the final target and waits for measured stability), so after
    the horizon it returns the final p with zero v/a; before the origin it is
    still ``REFERENCE_EXPIRED``.  STREAM takes the zero point from the fixed
    anchor.  A non-final stream outside its horizon raises
    ``REFERENCE_EXPIRED``; a final zero-velocity stream may explicitly hold its
    endpoint, never implicitly invent a zero-velocity continuation for an
    uncontinued chunk.
    """
    if not isinstance(record, AcceptedReference):
        raise _record_invalid("record", "must be an AcceptedReference")
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, Integral):
        raise _record_invalid("sim_time_ns", "must be an integer")
    sim_time = int(sim_time_ns)
    if sim_time < 0 or sim_time > INT64_MAX:
        raise _record_invalid("sim_time_ns", "must be a non-negative int64")

    request = record.request
    if request.kind == NATIVE_MOTION:
        if actual_started_sim_ns is None:
            raise _record_invalid(
                "actual_started_sim_ns",
                "MOTION reconstruction requires the actual started time",
            )
        start = _integer(
            actual_started_sim_ns, "actual_started_sim_ns", 0, INT64_MAX
        )
        # MOTION always declares settle_at_end=True and zero boundaries.  The
        # executor holds the last target after the horizon; this is the
        # declared final mode, not a rescue for an uncontinued stream.
        final_hold = True
    else:
        if actual_started_sim_ns is not None:
            raise _record_invalid(
                "actual_started_sim_ns",
                "STREAM reconstruction takes the start from its anchor",
            )
        assert request.anchor is not None
        start = request.anchor.start_sim_time_ns
        final_hold = request.settle_at_end

    elapsed = sim_time - start
    if elapsed < 0:
        raise ContractError(
            "REFERENCE_EXPIRED",
            f"sim_time_ns {sim_time} is before the plan origin {start}",
        )
    if elapsed <= record.plan.duration_ns:
        return record.plan.sample_interval(elapsed)
    if final_hold:
        terminal = record.plan.sample_interval(record.plan.duration_ns)
        if any(
            abs(value) > VELOCITY_ATOL
            for value in (*record.final_velocities, *terminal.velocities)
        ):
            raise ContractError(
                "REFERENCE_EXPIRED",
                "a declared final hold requires a zero-velocity endpoint",
            )
        zero = (0.0,) * len(terminal.positions)
        return ReferenceSample(terminal.positions, zero, zero)
    raise ContractError(
        "REFERENCE_EXPIRED",
        f"elapsed_ns {elapsed} is outside [0, {record.plan.duration_ns}]",
    )


def plan_from_segments(
    joint_names: Sequence[str],
    segments: Sequence[CubicSegment],
) -> ReferencePlan:
    """Explicit helper used by transports to rebuild the public plan."""
    return ReferencePlan(tuple(joint_names), tuple(segments))


__all__ = [
    "DIGEST_HEX_LENGTH",
    "INTERPOLATION_VERSION",
    "NATIVE_KINDS",
    "NATIVE_KIND_NAMES",
    "NATIVE_MOTION",
    "NATIVE_STREAM_FIRST",
    "NATIVE_STREAM_NEXT",
    "REFERENCE_DIGEST_VERSION",
    "AcceptedReference",
    "NativeRequest",
    "compute_reference_digest",
    "expected_final_velocities",
    "plan_from_segments",
    "reference_digest_payload",
    "sample_accepted_reference",
    "validate_accepted_reference",
]
