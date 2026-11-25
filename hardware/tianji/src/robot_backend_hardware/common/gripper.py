"""Two-finger gripper base class.

Defines the unified interface for all two-finger gripper devices.
Concrete implementations (e.g. Changingtek 120S, 90D) inherit from this.
"""

from __future__ import annotations

import threading
from abc import abstractmethod
from dataclasses import dataclass

import numpy as np

from robot_backend_hardware.common.device import Actuator

# Device command/state interface uses meters; gripper drivers use mm internally.
_MM_PER_M = 1000.0

# Command deadband (mm). A high-rate client (e.g. VR teleop at 120 Hz) resends
# the same target every frame. Each set() re-writes trigger=1 and restarts the
# move from the current position, so the actuator keeps re-launching and never
# reaches the target. Only forward a target to the mailbox when it differs from
# the last accepted one by more than this deadband.
_COMMAND_DEADBAND_MM = 0.3


@dataclass
class GripperState:
    """Snapshot of gripper status returned by get()."""

    position_mm: float  # current jaw opening distance in mm
    speed: float  # instantaneous speed (raw feedback)
    current: float  # instantaneous current (reflects torque)
    is_holding: bool  # object detected between jaws
    is_moving: bool  # motion in progress
    valid: bool = True  # False when no measurement is available


@dataclass
class GripperCommand:
    """Command payload accepted by set()."""

    position_mm: float  # target jaw opening distance in mm
    speed_pct: float | None = None  # speed override 0~100%, None = use default
    force_pct: float | None = None  # force/torque limit 0~100%, None = use default


class TwoFingerGripper(Actuator):
    """Abstract base for two-finger parallel-jaw grippers.

    Subclasses implement the low-level hardware communication.
    This class bridges the gripper-specific get/set API to the HAL
    Actuator poll_state / send_cmd interface.
    """

    # Serial half-duplex bus: reads and writes share the poll thread.
    serviced_io = True

    def __init__(
        self,
        name: str,
        max_stroke_mm: float,
        default_speed_pct: float = 100.0,
        default_force_pct: float = 60.0,
        state_key_prefix: str = "",
    ) -> None:
        super().__init__(name)
        self.max_stroke_mm = max_stroke_mm
        self.default_speed_pct = default_speed_pct
        self.default_force_pct = default_force_pct
        self._prefix = state_key_prefix or name

        # Latest-wins command mailbox. send_cmd() only stores the target here;
        # the poll thread flushes it to hardware in service_step(). The lock
        # guards the in-memory handoff only, never the serial transaction.
        self._last_accepted_mm: float | None = None
        self._pending_cmd: GripperCommand | None = None
        self._cmd_lock = threading.Lock()

    # ─── Abstract gripper-specific interface ─────────────────────────────

    @abstractmethod
    def get(self) -> GripperState:
        """Read current gripper state from hardware."""

    @abstractmethod
    def set(self, cmd: GripperCommand) -> None:
        """Send motion command to gripper hardware."""

    @abstractmethod
    def enable(self) -> None:
        """Enable the gripper actuator (if supported by hardware)."""

    @abstractmethod
    def disable(self) -> None:
        """Disable the gripper actuator (if supported by hardware)."""

    # ─── Device actuator interface implementation ───────────────────────────

    def get_features(self) -> dict[str, dict[str, Any]]:
        p = self._prefix
        return {
            f"{p}_gripper_state_pos": {
                "dtype": "float64",
                "shape": (1,),
                "description": "Gripper opening in meters",
            },
            f"{p}_gripper_state_speed": {
                "dtype": "float64",
                "shape": (1,),
                "description": "Gripper speed feedback",
            },
            f"{p}_gripper_state_current": {
                "dtype": "float64",
                "shape": (1,),
                "description": "Gripper current feedback",
            },
            f"{p}_gripper_state_holding": {
                "dtype": "float64",
                "shape": (1,),
                "description": "1.0 if object held, else 0.0",
            },
            f"{p}_gripper_cmd_pos": {
                "dtype": "float64",
                "shape": (1,),
                "description": "Target opening in meters",
            },
            f"{p}_gripper_cmd_speed": {
                "dtype": "float64",
                "shape": (1,),
                "description": "Speed override 0~100%",
            },
            f"{p}_gripper_cmd_force": {
                "dtype": "float64",
                "shape": (1,),
                "description": "Force limit 0~100%",
            },
        }

    def poll_state(self) -> dict[str, np.ndarray]:
        state = self.get()
        if not state.valid:
            return {}
        p = self._prefix

        return {
            f"{p}_gripper_state_pos": np.array(
                [state.position_mm / _MM_PER_M], dtype=np.float64
            ),
            f"{p}_gripper_state_speed": np.array([state.speed], dtype=np.float64),
            f"{p}_gripper_state_current": np.array([state.current], dtype=np.float64),
            f"{p}_gripper_state_holding": np.array(
                [1.0 if state.is_holding else 0.0], dtype=np.float64
            ),
        }

    def send_cmd(self, cmd: dict[str, np.ndarray]) -> None:
        """Non-blocking: parse the command and store it in the mailbox.

        The actual serial write happens on the poll thread in service_step().
        This keeps the shared command pipeline free of half-duplex serial I/O.
        """
        p = self._prefix
        pos_key = f"{p}_gripper_cmd_pos"
        if pos_key not in cmd:
            return

        # Device command interface is in meters; the driver works in mm.
        position_mm = float(cmd[pos_key][0]) * _MM_PER_M
        position_mm = max(0.0, min(position_mm, self.max_stroke_mm))

        # Deadband: a high-rate client resends the same target every frame.
        # Skip resubmitting (which would re-trigger the move) unless the target
        # actually changed. This lets the actuator run to the target instead of
        # restarting every poll cycle.
        if (
            self._last_accepted_mm is not None
            and abs(position_mm - self._last_accepted_mm) <= _COMMAND_DEADBAND_MM
        ):
            return
        self._last_accepted_mm = position_mm

        speed_key = f"{p}_gripper_cmd_speed"
        force_key = f"{p}_gripper_cmd_force"

        speed_pct = float(cmd[speed_key][0]) if speed_key in cmd else None
        force_pct = float(cmd[force_key][0]) if force_key in cmd else None

        self._submit(GripperCommand(
            position_mm=position_mm,
            speed_pct=speed_pct,
            force_pct=force_pct,
        ))


    def service_step(self) -> dict[str, np.ndarray]:
        """One serviced-I/O cycle: flush pending command, then read state.

        Runs on the poll thread so the serial bus is accessed by a single
        thread. When the mailbox is empty only a read happens, so state
        reporting continues uninterrupted.
        """
        with self._cmd_lock:
            pending = self._pending_cmd
            self._pending_cmd = None
        if pending is not None:
            self.set(pending)
        return self.poll_state()

    def _submit(self, cmd: GripperCommand) -> None:
        """Store the latest target in the mailbox (latest-wins)."""
        with self._cmd_lock:
            self._pending_cmd = cmd
