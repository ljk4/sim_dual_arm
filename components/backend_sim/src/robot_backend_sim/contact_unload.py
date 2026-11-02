"""Bound one evidence-backed release transition per gripper closing cycle."""

from dataclasses import dataclass

from .contacts import GripperContactEvidence


@dataclass(slots=True)
class ContactUnload:
    """Never change a reference; only identify a mechanically blocked release."""

    object_name: str | None = None
    started_ns: int | None = None
    used: bool = False

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
        if closing_contact_ok and reference_velocity <= 1e-9:
            # A new genuinely closing command may arm another release. Merely
            # pausing an expired release cannot replenish its time budget.
            if not self.used or reference_velocity < -1e-9:
                self.object_name = contact.candidate_object
                self.started_ns = None
                self.used = False
        if reference_velocity <= 1e-9 or self.object_name is None:
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
