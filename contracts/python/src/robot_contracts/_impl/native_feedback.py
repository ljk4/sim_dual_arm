"""Finger and gripper contact-evidence DTOs for native feedback."""

from __future__ import annotations

import math
from typing import Annotated, ClassVar, Literal, Self

from pydantic import BeforeValidator, Field, model_validator

from ..control import MAX_ID_LENGTH
from ..validation import (
    NonEmptyStr,
    StrictContractModel,
    StrictFloat,
    as_tuple,
    contract_error,
)

MAX_FINGER_OBJECTS = 8
MAX_GRIPPER_FINGERS = 2

class NativeFinger(StrictContractModel):
    """One finger's summed normal forces/counts for each allowed object."""

    _error_reason: ClassVar[str] = "INVALID_NATIVE_GRIPPER"

    side: Literal["left", "right"]
    finger_index: Annotated[int, Field(strict=True, ge=1, le=2)]
    body_name: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    joint_speed_m_s: Annotated[StrictFloat, Field(ge=0.0)]
    object_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                            Field(max_length=MAX_FINGER_OBJECTS)]
    normal_forces: Annotated[tuple[float, ...], BeforeValidator(as_tuple),
                             Field(max_length=MAX_FINGER_OBJECTS)]
    contact_counts: Annotated[tuple[int, ...], BeforeValidator(as_tuple),
                              Field(max_length=MAX_FINGER_OBJECTS)]

    @model_validator(mode="after")
    def _evidence(self) -> Self:
        if len(set(self.object_names)) != len(self.object_names):
            raise contract_error("object_names", "must be unique")
        for name in ("normal_forces", "contact_counts"):
            if len(getattr(self, name)) != len(self.object_names):
                raise contract_error(name, "must match object_names")
        if any(isinstance(value, bool) or not math.isfinite(value)
               for value in self.normal_forces):
            raise contract_error("normal_forces", "must be finite real numbers")
        if any(value < 0.0 for value in self.normal_forces):
            raise contract_error("normal_forces", "must not be negative")
        if any(isinstance(value, bool) or not isinstance(value, int)
               for value in self.contact_counts):
            raise contract_error("contact_counts", "must be integers")
        if any(value < 0 for value in self.contact_counts):
            raise contract_error("contact_counts", "must not be negative")
        return self


class NativeGripper(StrictContractModel):
    """One gripper's original target, measured state and per-finger evidence."""

    _error_reason: ClassVar[str] = "INVALID_NATIVE_GRIPPER"

    side: Literal["left", "right"]
    mode: Literal["NONE", "POSITION", "CONTACT"]
    position_ok: bool
    contact_ok: bool
    contact_object: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)] | None
    ambiguous: bool
    target_position_m: StrictFloat
    initial_reference_m: StrictFloat
    final_reference_m: StrictFloat
    measured_position_m: StrictFloat
    measured_velocity_m_s: StrictFloat
    target_error_m: StrictFloat
    fingers: Annotated[tuple[NativeFinger, ...], BeforeValidator(as_tuple),
                       Field(min_length=MAX_GRIPPER_FINGERS,
                             max_length=MAX_GRIPPER_FINGERS)]

    @model_validator(mode="after")
    def _evidence(self) -> Self:
        if self.mode == "POSITION" and not self.position_ok:
            raise contract_error("mode", "POSITION requires position_ok")
        if self.mode == "CONTACT":
            if not self.contact_ok:
                raise contract_error("mode", "CONTACT requires contact_ok")
            if self.contact_object is None:
                raise contract_error("mode", "CONTACT requires a contact object")
        if self.mode == "NONE" and (self.position_ok or self.contact_ok):
            raise contract_error("mode", "NONE cannot carry a successful flag")
        if self.contact_ok and self.contact_object is None:
            raise contract_error("contact_ok", "requires a contact object")
        if self.contact_ok and self.ambiguous:
            raise contract_error("contact_ok", "cannot be true when ambiguous")
        if not math.isclose(self.target_error_m,
                            self.target_position_m - self.measured_position_m,
                            rel_tol=1e-9, abs_tol=1e-12):
            raise contract_error("target_error_m", "must equal target - measured")
        if any(finger.side != self.side for finger in self.fingers):
            raise contract_error("fingers", "must match the gripper side")
        if tuple(finger.finger_index for finger in self.fingers) != (1, 2):
            raise contract_error("fingers", "must be ordered finger_index 1 then 2")
        return self
