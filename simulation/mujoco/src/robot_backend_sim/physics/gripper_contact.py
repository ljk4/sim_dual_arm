"""Gripper contact evidence, completion decisions and bounded release tracking.

``GripperContactTracker`` is an explicit component owned by the motion
controller.  It reads contact evidence and per-finger state, evaluates the
accepted POSITION/CONTACT completion rules, classifies constrained axes and
owns one ``ContactUnloader`` per side.  It never writes targets or steps physics.
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_trajectory.sampling import sample_reference_interval

from robot_backend_sim.config.motion_config import PandaControlConfig
from robot_backend_sim.physics.contacts import (
    GripperCompletion,
    GripperContactEvidence,
    empty_contact_evidence,
    evaluate_gripper_completion,
    measure_contact_evidence,
    validated_panda_completions,
)
from robot_backend_sim.physics.panda_io import Measurement
from robot_backend_sim.physics.scene import Scene

__all__ = ["REFERENCE_VELOCITY_ATOL_M_S", "ContactUnloader", "GripperContactTracker"]

# A reference velocity this close to zero is numerical standstill, not a
# measured opening command.
REFERENCE_VELOCITY_ATOL_M_S = 1e-9
_GRIPPER_INDEX = {"left": 7, "right": 15}
_SIDES = ("left", "right")
_ARM_INDICES = tuple(index for index in range(16) if index not in (7, 15))


@dataclass(slots=True)
class ContactUnloader:
    """Never change a reference; only identify a mechanically blocked release."""

    object_name: str | None = None
    started_ns: int | None = None
    used: bool = False

    def reset(self) -> None:
        self.object_name = None
        self.started_ns = None
        self.used = False

    def update(
        self,
        *,
        now_ns: int,
        timeout_ns: int,
        reference_position: float,
        reference_velocity: float,
        measured_position: float,
        position_tolerance: float,
        closing_contact_ok: bool,
        contact: GripperContactEvidence,
    ) -> bool:
        if timeout_ns <= 0:
            return False
        if closing_contact_ok and reference_velocity <= REFERENCE_VELOCITY_ATOL_M_S:
            # A new genuinely closing command may arm another release. Merely
            # pausing an expired release cannot replenish its time budget.
            if not self.used or reference_velocity < -REFERENCE_VELOCITY_ATOL_M_S:
                self.object_name = contact.candidate_object
                self.started_ns = None
                self.used = False
        if reference_velocity <= REFERENCE_VELOCITY_ATOL_M_S or self.object_name is None:
            return False
        if not self.used:
            self.started_ns = now_ns
            self.used = True
        assert self.started_ns is not None
        same_contact = (
            not contact.ambiguous
            and contact.candidate_object == self.object_name
            and all(finger.count_for(self.object_name) > 0 for finger in contact.fingers)
        )
        return (
            same_contact
            and now_ns - self.started_ns < timeout_ns
            and reference_position < measured_position - position_tolerance
        )


class GripperContactTracker:
    """Own contact measurement, completion bounds and constrained-axis release."""

    def __init__(
        self,
        scene: Scene,
        binding: object,
        config: PandaControlConfig,
        *,
        contact_unload_timeout_ns: int = 0,
    ) -> None:
        self._scene = scene
        self._binding = binding
        self._config = config
        self._contact_unload_timeout_ns = int(contact_unload_timeout_ns)
        self._unloaders = {side: ContactUnloader() for side in _SIDES}
        self._unloading_axes: frozenset[int] = frozenset()

    @property
    def unloading_axes(self) -> frozenset[int]:
        return self._unloading_axes

    def reset(self) -> None:
        for unloader in self._unloaders.values():
            unloader.reset()
        self._unloading_axes = frozenset()

    def empty_contacts(self) -> tuple[GripperContactEvidence, ...]:
        return tuple(
            empty_contact_evidence(self._scene.data, self._binding, side)
            for side in _SIDES
        )

    def measure_contacts(self) -> tuple[GripperContactEvidence, ...]:
        return tuple(
            measure_contact_evidence(self._scene.model, self._scene.data, self._binding, side)
            for side in _SIDES
        )

    def _completion(
        self,
        side: str,
        measurement: Measurement,
        contacts: tuple[GripperContactEvidence, ...],
        initial: float,
        final: float,
    ) -> GripperCompletion:
        index = _GRIPPER_INDEX[side]
        contact = next(item for item in contacts if item.side == side)
        return evaluate_gripper_completion(
            side=side,
            initial_reference_m=initial,
            final_reference_m=final,
            target_position_m=final,
            measured_position_m=measurement.positions[index],
            measured_velocity_m_s=measurement.velocities[index],
            position_tolerance_m=self._config.position_tolerances[index],
            velocity_tolerance_m_s=self._config.velocity_tolerances[index],
            force_min_n=self._config.contact_force_min_n,
            finger_speed_max_m_s=self._config.finger_velocity_max_m_s,
            contact=contact,
        )

    def completions(
        self,
        measurement: Measurement,
        contacts: tuple[GripperContactEvidence, ...],
        initial_positions: tuple[float, ...],
        final_positions: tuple[float, ...],
    ) -> tuple[GripperCompletion, ...]:
        return validated_panda_completions(tuple(
            self._completion(
                side,
                measurement,
                contacts,
                initial_positions[_GRIPPER_INDEX[side]],
                final_positions[_GRIPPER_INDEX[side]],
            )
            for side in _SIDES
        ))

    def completion_bounds(
        self,
        initial_positions: tuple[float, ...] | None,
        final_targets: tuple[float, ...] | None,
        active_plan: object | None,
        hold_positions: tuple[float, ...],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Initial and final/target positions for gripper completion evidence."""
        if initial_positions is None:
            return hold_positions, hold_positions
        final = final_targets
        if final is None and active_plan is not None:
            final = sample_reference_interval(
                active_plan.plan, active_plan.plan.duration_ns
            ).positions
        return initial_positions, hold_positions if final is None else final

    def arm_stable(self, measurement: Measurement, reference: object) -> bool:
        return all(
            abs(measurement.positions[index] - reference.positions[index])  # type: ignore[attr-defined]
            <= self._config.position_tolerances[index]
            and abs(measurement.velocities[index]) <= self._config.velocity_tolerances[index]
            for index in _ARM_INDICES
        )

    def update_normal_stability(
        self,
        measurement: Measurement,
        reference: object,
        completions: tuple[GripperCompletion, ...],
        horizon_sim_ns: int,
        stable_since_ns: int | None,
        signature: tuple[tuple[str, str | None], ...] | None,
    ) -> tuple[bool, int | None, tuple[tuple[str, str | None], ...] | None, int]:
        """Return ``(settled, stable_since, signature, stable_for_ns)``."""
        overall = self.arm_stable(measurement, reference) and all(
            completion.mode != "NONE" for completion in completions
        )
        current = tuple(
            (
                completion.mode,
                completion.contact_object if completion.mode == "CONTACT" else None,
            )
            for completion in completions
        )
        if measurement.sim_time_ns < horizon_sim_ns:
            overall = False
        if not overall:
            return False, None, None, 0
        if stable_since_ns is None or current != signature:
            stable_since_ns = measurement.sim_time_ns
            signature = current
        stable_for = measurement.sim_time_ns - stable_since_ns
        return stable_for >= self._config.stable_duration_ns, stable_since_ns, signature, stable_for

    def hold_completions(
        self,
        measurement: Measurement,
        contacts: tuple[GripperContactEvidence, ...],
        hold_positions: tuple[float, ...],
    ) -> tuple[GripperCompletion, ...]:
        return validated_panda_completions(tuple(
            self._completion(
                side,
                measurement,
                contacts,
                hold_positions[_GRIPPER_INDEX[side]],
                hold_positions[_GRIPPER_INDEX[side]],
            )
            for side in _SIDES
        ))

    def constrained_axes(
        self,
        reference: object,
        measurement: Measurement,
        contacts: tuple[GripperContactEvidence, ...],
    ) -> frozenset[int]:
        """Exempt a loaded, slow same-object grip below its measured aperture.

        A rising reference still compresses the object while it is narrower
        than the measured opening. Its velocity alone cannot request release.
        Once the reference reaches that aperture, contact completion no longer
        applies and normal tracking (or bounded unloading) takes over.
        """
        constrained: set[int] = set()
        unloading: set[int] = set()
        for side in _SIDES:
            index = _GRIPPER_INDEX[side]
            target = reference.positions[index]  # type: ignore[attr-defined]
            contact = next(item for item in contacts if item.side == side)
            completion = evaluate_gripper_completion(
                side=side,
                initial_reference_m=target,
                final_reference_m=target,
                target_position_m=target,
                measured_position_m=measurement.positions[index],
                measured_velocity_m_s=measurement.velocities[index],
                position_tolerance_m=self._config.position_tolerances[index],
                velocity_tolerance_m_s=self._config.velocity_tolerances[index],
                force_min_n=self._config.contact_force_min_n,
                finger_speed_max_m_s=self._config.finger_velocity_max_m_s,
                contact=contact,
            )
            closing_contact = completion.contact_ok
            if closing_contact:
                constrained.add(index)
            if self._unloaders[side].update(
                now_ns=measurement.sim_time_ns,
                timeout_ns=self._contact_unload_timeout_ns,
                reference_position=target,
                reference_velocity=reference.velocities[index],  # type: ignore[attr-defined]
                measured_position=measurement.positions[index],
                position_tolerance=self._config.position_tolerances[index],
                closing_contact_ok=closing_contact,
                contact=contact,
            ):
                constrained.add(index)
                unloading.add(index)
        self._unloading_axes = frozenset(unloading)
        return frozenset(constrained)
