"""Load and inspect the TianJi device configuration without opening I/O."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Self

from pydantic import Field, ValidationError, model_validator
import yaml

from robot_backend_hardware.common.camera.capture import CameraConfig
from robot_backend_hardware.common.config import DeviceSettings


class ArmsSettings(DeviceSettings):
    host: str = Field(min_length=1)
    left_arm: str = "arm_left"
    right_arm: str = "arm_right"
    enable_on_start: bool = False
    command_vel: float = Field(default=0.3, gt=0, le=1)
    connect_timeout_s: float = Field(default=10.0, gt=0)
    auto_clear_fault: bool = True
    max_joint_step_deg: float = Field(default=1.0, ge=0)
    tracking_arm_retry_interval_s: float = Field(default=1.0, gt=0)
    tracking_arm_max_failures: int = Field(default=3, ge=1)
    tracking_settle_timeout_s: float = Field(default=3.0, gt=0)
    poll_hz: float = Field(default=120.0, gt=0)
    command_dump_dir: str | None = None

    @model_validator(mode="after")
    def distinct_arms(self) -> Self:
        if not self.left_arm or not self.right_arm or self.left_arm == self.right_arm:
            raise ValueError("left_arm and right_arm must name two distinct arms")
        return self


class GripperSettings(DeviceSettings):
    port: str = Field(min_length=1)
    usb_port: str = ""
    serial_number: str = ""
    slave_id: int = Field(default=1, ge=1, le=247)
    baudrate: int = Field(default=115200, gt=0)
    timeout: float = Field(default=0.3, gt=0)
    max_stroke_mm: float = Field(default=90.0, gt=0)
    default_speed_pct: float = Field(default=100.0, ge=0, le=100)
    default_force_pct: float = Field(default=60.0, ge=0, le=100)
    default_accel_pct: float = Field(default=100.0, ge=0, le=100)
    default_decel_pct: float = Field(default=100.0, ge=0, le=100)
    invert_stroke: bool = True


class GrippersSettings(DeviceSettings):
    left: GripperSettings
    right: GripperSettings


class TianJiSettings(DeviceSettings):
    arms: ArmsSettings
    grippers: GrippersSettings
    cameras: list[CameraConfig] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_devices(self) -> Self:
        if self.grippers.left.port == self.grippers.right.port:
            raise ValueError("left and right grippers require distinct serial ports")
        names = [camera.name for camera in self.cameras]
        if len(names) != len(set(names)):
            raise ValueError("camera names must be unique")
        identities = []
        for camera in self.cameras:
            identity = camera.serial_number if camera.backend == "realsense" else camera.device
            if not identity:
                raise ValueError(f"camera {camera.name!r} requires a serial_number or device")
            identities.append((camera.backend == "realsense", identity))
        if len(identities) != len(set(identities)):
            raise ValueError("each camera requires a distinct physical device")
        return self


def load_config(path: str | Path) -> TianJiSettings:
    with Path(path).open(encoding="utf-8") as stream:
        return TianJiSettings.model_validate(yaml.safe_load(stream))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = os.environ.get(
        "HARDWARE_CONFIG", str(Path(__file__).resolve().parents[3] / "configs/devices.yaml")
    )
    parser.add_argument("--config", type=Path, default=Path(default_config), help="device YAML")
    args = parser.parse_args()
    try:
        config = load_config(args.config)
    except (OSError, yaml.YAMLError, ValidationError) as error:
        parser.error(str(error))
    print(f"config: {args.config.resolve()}")
    print(yaml.safe_dump(config.model_dump(mode="json"), sort_keys=False), end="")


if __name__ == "__main__":
    main()
