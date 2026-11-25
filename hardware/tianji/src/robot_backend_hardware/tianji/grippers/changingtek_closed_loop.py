"""Changingtek closed-loop stepper gripper driver (generic).

Implements the Modbus RTU (RS-485) protocol shared by the Changingtek
closed-loop stepper actuator family, e.g. CTAG2F90C, CTAG2F120s and the
CTPM/CTAG series listed in the "闭环步进" operation manual. All of these
share the same register map:

    - enable            0x0100
    - temp-zone motion  0x0102~0x0108 (pos_h, pos_l, speed, force, accel,
                                       decel, trigger)
    - status flags      0x0601 (torque reached), 0x0602 (position reached),
                        0x0604 (ready)
    - feedback          0x060B (speed), 0x060C (current),
                        0x060D~0x060E (position, unit 0.01mm)

Concrete models (Changingtek90C, Changingtek120S) only differ in their
default stroke / motion parameters and subclass this driver.
"""

from __future__ import annotations

import logging
import threading

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import minimalmodbus

from robot_backend_hardware.common.gripper import (
    GripperCommand,
    GripperState,
    TwoFingerGripper,
)

log = logging.getLogger(__name__)

# ─── Register map (holding registers, FC 0x03/0x06/0x10) ────────────────────

# Write registers
_REG_ENABLE = 0x0100
_REG_TMP_POS_H = 0x0102
_REG_TMP_POS_L = 0x0103
_REG_TMP_SPEED = 0x0104
_REG_TMP_FORCE = 0x0105
_REG_TMP_ACCEL = 0x0106
_REG_TMP_DECEL = 0x0107
_REG_TMP_TRIGGER = 0x0108

# Read registers (status / feedback)
_REG_TORQUE_REACHED = 0x0601
_REG_POS_REACHED = 0x0602
_REG_READY = 0x0604
_REG_SPEED_FB = 0x060B
_REG_CURRENT_FB = 0x060C
_REG_POS_FB_H = 0x060D
_REG_POS_FB_L = 0x060E
_REG_ALARM = 0x0612

# Position feedback / command unit: 1 unit = 0.01mm
_POS_UNITS_PER_MM = 100.0


class ChangingtekClosedLoop(TwoFingerGripper):
    """Generic driver for Changingtek closed-loop stepper actuators."""

    def __init__(
        self,
        name: str,
        port: str = "COM3",
        slave_id: int = 1,
        baudrate: int = 115200,
        timeout: float = 0.3,
        max_stroke_mm: float = 120.0,
        default_speed_pct: float = 100.0,   # 0~100, % of max speed
        default_force_pct: float = 60.0,    # 0~100, % of max torque/current
        default_accel_pct: float = 100.0,   # 0~100, % of max acceleration
        default_decel_pct: float = 100.0,   # 0~100, % of max deceleration
        state_key_prefix: str = "",
        invert_stroke: bool = False,
    ) -> None:
        super().__init__(
            name=name,
            max_stroke_mm=max_stroke_mm,
            default_speed_pct=default_speed_pct,
            default_force_pct=default_force_pct,
            state_key_prefix=state_key_prefix,
        )
        self._port = port
        self._slave_id = slave_id
        self._baudrate = baudrate
        self._timeout = timeout
        self._accel_pct = default_accel_pct
        self._decel_pct = default_decel_pct
        # When True, raw encoder direction is opposite to HAL opening: raw 0 =
        # closed, raw max = open. The command position (mm) is mirrored about
        # max_stroke_mm before writing, and feedback is mirrored back on read.
        self._invert_stroke = bool(invert_stroke)

        self._instrument: minimalmodbus.Instrument | None = None
        # Serial transaction lock. Under serviced I/O both get() and set() run
        # on the same poll thread (see service_step), so there is no concurrent
        # access on a single-instance bus. The lock is retained defensively for
        # future multi-threaded / shared-bus access; it never spans read+write.
        self._lock = threading.Lock()
        self._connected = False

    # ─── Lifecycle ───────────────────────────────────────────────────────

    def start(self) -> None:
        import minimalmodbus
        import serial

        try:
            inst = minimalmodbus.Instrument(self._port, self._slave_id)
            inst.serial.baudrate = self._baudrate
            inst.serial.bytesize = 8
            inst.serial.parity = serial.PARITY_NONE
            inst.serial.stopbits = 1
            inst.serial.timeout = self._timeout
            inst.mode = minimalmodbus.MODE_RTU
            # Half-duplex RS485: a set() write is immediately followed by a
            # get() read on the same poll thread. Stale write-response bytes
            # left in the buffer would be misparsed as the next read response,
            # freezing/garbling feedback. Clear buffers before each transaction.
            inst.clear_buffers_before_each_transaction = True

            if not inst.serial.is_open:
                inst.serial.open()

            self._instrument = inst
            self._connected = True
            self.enable()
            log.info("%s [%s] connected on %s", type(self).__name__, self.name, self._port)
        except Exception as e:
            self._connected = False
            log.warning("%s [%s] connection failed: %s", type(self).__name__, self.name, e)

    def stop(self) -> None:
        if self._instrument is not None:
            try:
                self.disable()
            except Exception:
                pass
            try:
                self._instrument.serial.close()
            except Exception:
                pass
            self._instrument = None
            self._connected = False
            log.info("%s [%s] stopped", type(self).__name__, self.name)

    def is_healthy(self) -> bool:
        return self._connected

    # ─── Low-level Modbus helpers ────────────────────────────────────────

    def _w1(self, addr: int, value: int) -> None:
        with self._lock:
            self._instrument.write_register(addr, value, functioncode=6)

    def _wn(self, addr: int, values: list[int]) -> None:
        with self._lock:
            self._instrument.write_registers(addr, values)

    def _r(self, addr: int, count: int = 1) -> list[int]:
        with self._lock:
            return self._instrument.read_registers(addr, count, functioncode=3)

    # ─── Enable / Disable ────────────────────────────────────────────────

    def enable(self) -> None:
        if self._instrument:
            self._w1(_REG_ENABLE, 1)

    def disable(self) -> None:
        if self._instrument:
            self._w1(_REG_ENABLE, 0)

    def read_alarm(self) -> int:
        """Read the alarm bitmask (0x0612). Returns 0 when disconnected."""
        if not self._connected:
            return 0
        try:
            return int(self._r(_REG_ALARM, 1)[0])
        except Exception as e:
            log.debug("%s alarm read error: %s", type(self).__name__, e)
            return 0

    # ─── get() ───────────────────────────────────────────────────────────

    def get(self) -> GripperState:
        if not self._connected:
            return GripperState(0.0, 0.0, 0.0, False, False, valid=False)

        try:
            pos_regs = self._r(_REG_POS_FB_H, 2)
            raw_pos = ((pos_regs[0] & 0xFFFF) << 16) | (pos_regs[1] & 0xFFFF)
            position_mm = raw_pos / _POS_UNITS_PER_MM
            if self._invert_stroke:
                position_mm = self.max_stroke_mm - position_mm

            speed = float(self._r(_REG_SPEED_FB, 1)[0])
            current = float(self._r(_REG_CURRENT_FB, 1)[0])

            status_regs = self._r(_REG_TORQUE_REACHED, 2)
            torque_reached = bool(status_regs[0])
            pos_reached = bool(status_regs[1])

            is_holding = torque_reached and not pos_reached
            is_moving = not (torque_reached or pos_reached)

            return GripperState(
                position_mm=position_mm,
                speed=speed,
                current=current,
                is_holding=is_holding,
                is_moving=is_moving,
            )
        except Exception as e:
            log.debug("%s poll error: %s", type(self).__name__, e)
            return GripperState(0.0, 0.0, 0.0, False, False, valid=False)

    # ─── set() ───────────────────────────────────────────────────────────

    def set(self, cmd: GripperCommand) -> None:
        if not self._connected:
            return

        position_mm = cmd.position_mm
        if self._invert_stroke:
            position_mm = self.max_stroke_mm - position_mm
        pos_units = int(position_mm * _POS_UNITS_PER_MM)
        pos_units = max(0, min(pos_units, int(self.max_stroke_mm * _POS_UNITS_PER_MM)))

        speed = int(cmd.speed_pct if cmd.speed_pct is not None else self.default_speed_pct)
        force = int(cmd.force_pct if cmd.force_pct is not None else self.default_force_pct)
        accel = int(self._accel_pct)
        decel = int(self._decel_pct)

        speed = max(0, min(speed, 100))
        force = max(0, min(force, 100))
        accel = max(0, min(accel, 100))
        decel = max(0, min(decel, 100))

        hi = (pos_units >> 16) & 0xFFFF
        lo = pos_units & 0xFFFF

        # Batch write 0x0102~0x0108 (7 registers) in one transaction
        values = [hi, lo, speed, force, accel, decel, 1]  # last = trigger
        try:
            self._wn(_REG_TMP_POS_H, values)
        except Exception as e:
            log.warning("%s set() failed: %s", type(self).__name__, e)
            self._connected = False
