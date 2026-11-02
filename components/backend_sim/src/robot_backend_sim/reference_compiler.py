"""Compile timed position waypoints into explicit cubic reference plans.

This adapter is the only place that uses SciPy. The shared contract stores the
resulting coefficients, so sampling and verification never refit or import the
native spline library.
"""

from __future__ import annotations

from collections.abc import Sequence

from robot_contracts.models import ContractError, JointSpec, Target
from robot_contracts.reference import (
    CubicSegment,
    ReferencePlan,
    validate_reference_inputs,
    verify_reference_plan,
)


def _seconds(time_ns: int) -> float:
    return time_ns / 1_000_000_000


def compile_reference_plan(
    initial_positions: Sequence[float],
    targets: Sequence[Target],
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    *,
    initial_velocities: Sequence[float] | None = None,
    final_velocities: Sequence[float] | None = None,
) -> ReferencePlan:
    """Compile original waypoints once with a SciPy ``CubicSpline``.

    The time axis is ``[0, *targets.time_ns] / 1e9`` seconds, with one row per
    time waypoint and one column per joint. ``axis=0`` therefore keeps the
    caller's joint order. SciPy stores descending-power coefficients ``c[k]``
    for ``(t - t_k)``; they are reversed into the shared ascending ``c0..c3``
    convention without transposing axes.

    ``initial_velocities``/``final_velocities`` are explicit first derivatives
    at the two ends. ``None`` means zero for that end; when both are ``None``
    the historical ``clamped`` boundary is used so old coefficients stay
    byte-for-byte compatible.  Explicit arrays are validated by the shared
    contract before SciPy is called and are never clipped.
    """
    inputs = validate_reference_inputs(
        initial_positions,
        targets,
        joints,
        max_accelerations,
        initial_velocities=initial_velocities,
        final_velocities=final_velocities,
    )
    times = (0.0, *(_seconds(time_ns) for time_ns in inputs.target_times_ns))
    for index, (left, right) in enumerate(zip(times, times[1:], strict=False)):
        if not right > left:
            raise ContractError(
                "INVALID_REFERENCE_PLAN",
                f"targets[{index}].time_ns collapses on the float seconds axis",
            )

    try:
        from scipy.interpolate import CubicSpline
    except ImportError as error:
        raise ContractError(
            "SCIPY_UNAVAILABLE",
            "SciPy is required to compile a reference plan",
        ) from error

    rows = [inputs.initial_positions, *inputs.target_positions]
    if initial_velocities is None and final_velocities is None:
        bc_type: object = "clamped"
    else:
        bc_type = (
            (1, inputs.initial_velocities),
            (1, inputs.final_velocities),
        )
    try:
        spline = CubicSpline(
            times,
            rows,
            axis=0,
            bc_type=bc_type,
            extrapolate=False,
        )
        coefficients = spline.c
    except Exception as error:
        raise ContractError(
            "REFERENCE_COMPILATION_FAILED",
            f"SciPy CubicSpline rejected the waypoints: {error}",
        ) from error

    try:
        expected_shape = (4, len(inputs.target_times_ns), len(inputs.joint_names))
        if tuple(coefficients.shape) != expected_shape:
            raise ContractError(
                "REFERENCE_COMPILATION_FAILED",
                f"SciPy coefficient shape {tuple(coefficients.shape)} != {expected_shape}",
            )
        segments = tuple(
            CubicSegment(
                duration_ns=(
                    inputs.target_times_ns[index]
                    - (inputs.target_times_ns[index - 1] if index else 0)
                ),
                c0=tuple(float(value) for value in coefficients[3, index, :]),
                c1=tuple(float(value) for value in coefficients[2, index, :]),
                c2=tuple(float(value) for value in coefficients[1, index, :]),
                c3=tuple(float(value) for value in coefficients[0, index, :]),
            )
            for index in range(len(inputs.target_times_ns))
        )
        plan = ReferencePlan(inputs.joint_names, segments)
    except ContractError:
        raise
    except Exception as error:
        raise ContractError(
            "REFERENCE_COMPILATION_FAILED",
            f"could not convert SciPy coefficients: {error}",
        ) from error

    verify_reference_plan(
        plan,
        inputs.initial_positions,
        tuple(
            Target(time_ns, positions)
            for time_ns, positions in zip(
                inputs.target_times_ns,
                inputs.target_positions,
                strict=True,
            )
        ),
        inputs.joints,
        inputs.max_accelerations,
        initial_velocities=inputs.initial_velocities,
        final_velocities=inputs.final_velocities,
    )
    return plan


__all__ = ["compile_reference_plan"]
