import numpy as np
import pytest

from robot_backend_hardware.common.gripper import GripperCommand
from robot_backend_hardware.tianji.grippers.changingtek_90c import Changingtek90C


class ModbusRegisters:
    def __init__(self):
        self.events = []

    def read_registers(self, address, count, functioncode):
        self.events.append(("read", address))
        if address == 0x060D:
            return [0, 3000]  # 30 mm raw travel; 60 mm opening after inversion.
        if address == 0x0601:
            return [0, 1]
        return [12] * count

    def write_registers(self, address, values):
        self.events.append(("write", address, values))


def connected_gripper():
    gripper = Changingtek90C(port="/dev/test-gripper", state_key_prefix="leftarm", invert_stroke=True)
    gripper._instrument = ModbusRegisters()
    gripper._connected = True
    return gripper


def test_feedback_inverts_travel_and_converts_mm_to_m():
    gripper = connected_gripper()
    assert gripper.poll_state()["leftarm_gripper_state_pos"][0] == pytest.approx(0.06)


def test_command_inverts_opening_and_writes_motion_registers():
    gripper = connected_gripper()
    gripper.set(GripperCommand(position_mm=60.0, speed_pct=40.0, force_pct=25.0))
    assert gripper._instrument.events == [("write", 0x0102, [0, 3000, 40, 25, 100, 100, 1])]


def test_serial_service_flushes_latest_command_before_reading():
    gripper = connected_gripper()
    for opening in (0.02, 0.06):
        gripper.send_cmd({"leftarm_gripper_cmd_pos": np.array([opening])})
    assert gripper._instrument.events == []
    state = gripper.service_step()
    assert gripper._instrument.events[0] == ("write", 0x0102, [0, 3000, 100, 60, 100, 100, 1])
    assert sum(event[0] == "write" for event in gripper._instrument.events) == 1
    assert state["leftarm_gripper_state_pos"][0] == pytest.approx(0.06)


def test_deadband_does_not_retrigger_same_motion():
    gripper = connected_gripper()
    for _ in range(3):
        gripper.send_cmd({"leftarm_gripper_cmd_pos": np.array([0.06])})
        gripper.service_step()
    assert sum(event[0] == "write" for event in gripper._instrument.events) == 1


def test_disconnected_gripper_has_no_measurements():
    gripper = Changingtek90C(port="/dev/test-gripper")
    assert gripper.poll_state() == {}


def test_failed_read_has_no_measurements(monkeypatch):
    gripper = connected_gripper()

    def fail(*args, **kwargs):
        raise OSError("serial read failed")

    monkeypatch.setattr(gripper._instrument, "read_registers", fail)
    assert gripper.poll_state() == {}
