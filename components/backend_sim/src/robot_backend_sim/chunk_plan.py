"""Precompile one observation-anchored action chunk before the physical loop.

The chunk uses the shared :mod:`robot_contracts.reference` curve for sampling
and verification and only this module imports SciPy (through the existing
reference compiler).  It prepares math, not a physical state machine: no clock,
scene, thread, IO, network or logging is touched here.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral

from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import ContractError, JointSpec, RobotProfile
from robot_contracts.reference import (
    VELOCITY_ATOL,
    ReferencePlan,
    ReferenceSample,
    validate_reference_inputs,
    validate_velocity_boundaries,
    verify_reference_plan,
)

from .braking import plan_braking
from .motion_config import PandaControlConfig
from .reference_compiler import compile_reference_plan


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_PREPARED_CHUNK", f"{location}: {message}")


def _check_positive_accelerations(accelerations: Sequence[float]) -> None:
    for index, acceleration in enumerate(accelerations):
        if acceleration <= 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must be positive")


def _estimate_final_velocities(window: ActionWindow) -> tuple[float, ...]:
    """Backward difference of the selected last two points, in per-second units."""
    last, previous = window.targets[-1], window.targets[-2]
    delta_ns = last.time_ns - previous.time_ns
    if delta_ns <= 0:
        raise _invalid("source_targets", "selected times are not strictly increasing")
    delta_s = delta_ns / 1_000_000_000
    return tuple(
        (right - left) / delta_s
        for left, right in zip(previous.positions, last.positions, strict=True)
    )


@dataclass(frozen=True)
class PreparedChunk:
    """One fully verified, immutable chunk ready for a later physical owner.

    Derived values such as joint names, dimensions, duration and sequence are
    properties of ``plan``/``anchor``/``window`` and are not stored twice.
    """

    anchor: ChunkAnchor
    window: ActionWindow
    plan: ReferencePlan
    joints: tuple[JointSpec, ...]
    max_accelerations: tuple[float, ...]
    final_velocities: tuple[float, ...]
    settle_at_end: bool

    def __post_init__(self) -> None:
        if not isinstance(self.anchor, ChunkAnchor):
            raise _invalid("anchor", "must be a ChunkAnchor")
        if not isinstance(self.window, ActionWindow):
            raise _invalid("window", "must be an ActionWindow")
        if not isinstance(self.plan, ReferencePlan):
            raise _invalid("plan", "must be a ReferencePlan")
        if not isinstance(self.settle_at_end, bool):
            raise _invalid("settle_at_end", "must be a bool")

        # Reuse the shared input/range validator for the anchor's initial
        # boundary, the complete source window (including overlap prefix and
        # unused tail), joint metadata, accelerations and declared boundaries.
        # It does not fit the source as a curve; it is purely input validation.
        inputs = validate_reference_inputs(
            self.anchor.reference.positions,
            self.window.source_targets,
            self.joints,
            self.max_accelerations,
            initial_velocities=self.anchor.reference.velocities,
            final_velocities=self.final_velocities,
        )
        _check_positive_accelerations(inputs.max_accelerations)
        final_velocities = inputs.final_velocities
        if self.settle_at_end and any(
            abs(velocity) > VELOCITY_ATOL for velocity in final_velocities
        ):
            raise _invalid(
                "final_velocities",
                "must be zero when settle_at_end is True",
            )
        if self.anchor.start_sim_time_ns != self.window.handoff_time_ns:
            raise _invalid(
                "anchor.start_sim_time_ns",
                "must equal window.handoff_time_ns",
            )
        if self.plan.joint_names != inputs.joint_names:
            raise _invalid("plan.joint_names", "must match the prepared joint order")
        if self.plan.duration_ns != self.window.end_time_ns:
            raise _invalid(
                "plan.duration_ns",
                f"is {self.plan.duration_ns}ns but the selected window ends at "
                f"{self.window.end_time_ns}ns",
            )
        verify_reference_plan(
            self.plan,
            inputs.initial_positions,
            self.window.targets,
            inputs.joints,
            inputs.max_accelerations,
            initial_velocities=inputs.initial_velocities,
            final_velocities=final_velocities,
        )

        # A continuous chunk may outlive its successor.  Verify that the actual
        # plan endpoint can always be braked with the same joints/limits; the
        # braking plan is discarded (not stored as a second source of truth).
        end = self.plan.sample_interval(self.plan.duration_ns)
        plan_braking(
            end.positions,
            end.velocities,
            inputs.joints,
            inputs.max_accelerations,
        )

        object.__setattr__(self, "joints", inputs.joints)
        object.__setattr__(self, "max_accelerations", inputs.max_accelerations)
        object.__setattr__(self, "final_velocities", final_velocities)

    @property
    def seq(self) -> int:
        return self.anchor.seq

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.plan.joint_names

    @property
    def joint_count(self) -> int:
        return self.plan.joint_count

    @property
    def duration_ns(self) -> int:
        return self.plan.duration_ns

    @property
    def start_sim_time_ns(self) -> int:
        return self.anchor.start_sim_time_ns

    @property
    def end_sim_time_ns(self) -> int:
        return self.anchor.start_sim_time_ns + self.plan.duration_ns

    def sample_at(self, sim_time_ns: int) -> ReferenceSample:
        """Sample strictly inside the absolute chunk interval ``start..end``."""
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, Integral):
            raise _invalid("sim_time_ns", "must be an integer")
        sim_time = int(sim_time_ns)
        start = self.start_sim_time_ns
        end = self.end_sim_time_ns
        if sim_time < start or sim_time > end:
            raise ContractError(
                "REFERENCE_EXPIRED",
                f"sim_time_ns {sim_time} is outside [{start}, {end}]",
            )
        return self.plan.sample_interval(sim_time - start)


def prepare_chunk(
    anchor: ChunkAnchor,
    window: ActionWindow,
    profile: RobotProfile,
    config: PandaControlConfig,
    *,
    final_velocities: Sequence[float] | None = None,
    settle_at_end: bool = False,
) -> PreparedChunk:
    """Validate, select final velocity and compile one chunk exactly once."""
    if not isinstance(anchor, ChunkAnchor):
        raise _invalid("anchor", "must be a ChunkAnchor")
    if not isinstance(window, ActionWindow):
        raise _invalid("window", "must be an ActionWindow")
    if not isinstance(profile, RobotProfile):
        raise _invalid("profile", "must be a RobotProfile")
    if not isinstance(config, PandaControlConfig):
        raise _invalid("config", "must be a PandaControlConfig")
    if not isinstance(settle_at_end, bool):
        raise _invalid("settle_at_end", "must be a bool")
    if tuple(profile.joint_names) != tuple(config.joint_names):
        raise ContractError(
            "PROFILE_MISMATCH",
            "profile joint order differs from control config",
        )
    if anchor.robot_profile_hash != profile.digest:
        raise ContractError(
            "PROFILE_MISMATCH",
            "anchor robot_profile_hash differs from profile digest",
        )
    if anchor.start_sim_time_ns != window.handoff_time_ns:
        raise _invalid(
            "anchor.start_sim_time_ns",
            "must equal window.handoff_time_ns",
        )

    if settle_at_end:
        if final_velocities is not None:
            explicit = validate_velocity_boundaries(
                final_velocities,
                profile.joints,
                "final_velocities",
            )
            if any(abs(velocity) > VELOCITY_ATOL for velocity in explicit):
                raise _invalid(
                    "final_velocities",
                    "must be zero when settle_at_end is True",
                )
        final = (0.0,) * len(profile.joints)
    elif final_velocities is not None:
        final = validate_velocity_boundaries(
            final_velocities,
            profile.joints,
            "final_velocities",
        )
    else:
        final = validate_velocity_boundaries(
            _estimate_final_velocities(window),
            profile.joints,
            "final_velocities",
        )

    # Full-source validation before the one compile: a bad non-executed prefix
    # cannot be hidden by window slicing.  This normalizes joints/accelerations
    # and both velocity boundaries through the same rules as the shared plan.
    inputs = validate_reference_inputs(
        anchor.reference.positions,
        window.source_targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=anchor.reference.velocities,
        final_velocities=final,
    )
    _check_positive_accelerations(inputs.max_accelerations)

    plan = compile_reference_plan(
        inputs.initial_positions,
        window.targets,
        inputs.joints,
        inputs.max_accelerations,
        initial_velocities=inputs.initial_velocities,
        final_velocities=inputs.final_velocities,
    )
    return PreparedChunk(
        anchor=anchor,
        window=window,
        plan=plan,
        joints=inputs.joints,
        max_accelerations=inputs.max_accelerations,
        final_velocities=inputs.final_velocities,
        settle_at_end=settle_at_end,
    )


__all__ = ["PreparedChunk", "prepare_chunk"]
