from pathlib import Path

from pydantic import ValidationError
import pytest
import yaml

from robot_backend_hardware.tianji.assembly import build_devices
from robot_backend_hardware.tianji.config import TianJiSettings, load_config


def device_document():
    path = Path(__file__).parents[1] / "configs/devices.yaml"
    document = yaml.safe_load(path.read_text())
    document["arms"]["host"] = "controller.invalid"
    for side in ("left", "right"):
        document["grippers"][side]["port"] = f"/dev/test-{side}"
        document["grippers"][side]["usb_port"] = f"test-{side}"
    for camera in document["cameras"]:
        if camera["backend"] == "realsense":
            camera["serial_number"] = f"test-{camera['name']}"
        else:
            camera["device"] = f"/dev/test-{camera['name']}"
    return document


def test_config_constructs_all_devices_without_opening_them(tmp_path):
    path = tmp_path / "devices.yaml"
    path.write_text(yaml.safe_dump(device_document()))
    config = load_config(path)
    devices = build_devices(config)

    assert devices.arms._client is None
    assert devices.arms._enable_on_start is False
    assert len(devices.cameras.camera_names) == 7
    assert len(devices.grippers) == 2
    assert all(not gripper.is_healthy() for gripper in devices.grippers.values())
    assert all(gripper._invert_stroke for gripper in devices.grippers.values())
    assert devices.grippers["left"]._prefix == "leftarm"
    assert devices.grippers["right"]._prefix == "rightarm"
    assert all(camera.width == 640 and camera.height == 480 and camera.fps == 30
               for camera in config.cameras)


def test_default_deployment_config_is_complete():
    path = Path(__file__).parents[1] / "configs/devices.yaml"
    config = load_config(path)
    assert config.arms.host
    assert config.grippers.left.usb_port and config.grippers.right.usb_port
    assert len(config.cameras) == 7


def test_missing_device_identity_is_rejected():
    document = device_document()
    document["arms"]["host"] = ""
    with pytest.raises(ValidationError):
        TianJiSettings.model_validate(document)


@pytest.mark.parametrize("mutation", [
    lambda data: data["arms"].update(poll_hz=0),
    lambda data: data["arms"].update(unknown_option=True),
    lambda data: data["arms"].update(right_arm=data["arms"]["left_arm"] if "left_arm" in data["arms"] else "arm_left"),
    lambda data: data["grippers"]["right"].update(port=data["grippers"]["left"]["port"]),
    lambda data: data["cameras"][1].update(name=data["cameras"][0]["name"]),
    lambda data: data["cameras"][1].update(serial_number=data["cameras"][0]["serial_number"]),
    lambda data: data["cameras"][0].update(fps=-1),
])
def test_rejects_invalid_or_ambiguous_configuration(mutation):
    document = device_document()
    mutation(document)
    with pytest.raises(ValidationError):
        TianJiSettings.model_validate(document)
