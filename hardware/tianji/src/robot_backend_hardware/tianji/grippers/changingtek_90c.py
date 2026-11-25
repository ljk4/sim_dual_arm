"""Changingtek CTAG2F90C closed-loop stepper gripper driver.

The CTAG2F90C is a collaborative two-finger gripper (90mm stroke) that uses
the same closed-loop stepper Modbus RTU protocol as the CTAG2F120s. This
differs from the CTAG2F90D (see changingtek_90d.py), which exposes a
different feedback register layout. Only the default stroke and motion
parameters are pinned here; all communication is inherited.
"""

from __future__ import annotations

from robot_backend_hardware.tianji.grippers.changingtek_closed_loop import (
    ChangingtekClosedLoop,
)


class Changingtek90C(ChangingtekClosedLoop):
    """Driver for CTAG2F90C (closed-loop stepper, 90mm stroke)."""

    def __init__(
        self,
        name: str = "gripper_90c",
        port: str = "COM3",
        slave_id: int = 1,
        baudrate: int = 115200,
        timeout: float = 0.3,
        max_stroke_mm: float = 90.0,
        default_speed_pct: float = 100.0,
        default_force_pct: float = 60.0,
        default_accel_pct: float = 100.0,
        default_decel_pct: float = 100.0,
        state_key_prefix: str = "",
        invert_stroke: bool = False,
    ) -> None:
        super().__init__(
            name=name,
            port=port,
            slave_id=slave_id,
            baudrate=baudrate,
            timeout=timeout,
            max_stroke_mm=max_stroke_mm,
            default_speed_pct=default_speed_pct,
            default_force_pct=default_force_pct,
            default_accel_pct=default_accel_pct,
            default_decel_pct=default_decel_pct,
            state_key_prefix=state_key_prefix,
            invert_stroke=invert_stroke,
        )
