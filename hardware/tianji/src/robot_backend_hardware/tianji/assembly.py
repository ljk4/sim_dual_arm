"""Construct TianJi device owners; construction never connects to hardware."""
from dataclasses import dataclass

from robot_backend_hardware.common.camera.capture import MultiCameraCapture
from robot_backend_hardware.tianji.arms import ApexRobotActuator
from robot_backend_hardware.tianji.config import TianJiSettings
from robot_backend_hardware.tianji.grippers.changingtek_90c import Changingtek90C
from robot_backend_hardware.tianji.grippers.serial_ports import resolve_serial_port


@dataclass(frozen=True)
class TianJiDevices:
    arms: ApexRobotActuator
    grippers: dict[str, Changingtek90C]
    cameras: MultiCameraCapture


def build_devices(config: TianJiSettings) -> TianJiDevices:
    grippers = {}
    for side in ("left", "right"):
        settings = getattr(config.grippers, side)
        options = settings.model_dump(exclude={"usb_port", "serial_number"})
        options["port"] = resolve_serial_port(
            settings.port, settings.serial_number, settings.usb_port
        )
        grippers[side] = Changingtek90C(
            name=f"gripper_{side}",
            state_key_prefix=f"{side}arm",
            **options,
        )
    return TianJiDevices(
        arms=ApexRobotActuator(**config.arms.model_dump()),
        grippers=grippers,
        cameras=MultiCameraCapture(config.cameras),
    )
