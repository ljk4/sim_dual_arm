"""Lifecycle and I/O interfaces shared by concrete hardware devices."""
from abc import ABC, abstractmethod
from typing import Any

import numpy as np


class Device(ABC):
    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def start(self) -> None:
        """Connect to the physical device."""

    @abstractmethod
    def stop(self) -> None:
        """Stop the device and close owned resources."""

    @abstractmethod
    def is_healthy(self) -> bool:
        """Report whether device I/O is available."""

    @abstractmethod
    def get_features(self) -> dict[str, dict[str, Any]]:
        """Describe device-local command and state fields."""


class Actuator(Device):
    serviced_io = False
    supports_pose_motion = False

    @abstractmethod
    def poll_state(self) -> dict[str, np.ndarray]:
        """Read available measurements."""

    @abstractmethod
    def send_cmd(self, cmd: dict[str, np.ndarray]) -> None:
        """Submit device-local targets."""

    def service_step(self) -> dict[str, np.ndarray]:
        """Service one cycle; serial devices also flush pending writes here."""
        return self.poll_state()
