from robot_backend_hardware.tianji.grippers import serial_ports


def test_usb_topology_takes_priority_over_existing_alias(monkeypatch):
    monkeypatch.setattr(serial_ports.glob, "glob", lambda pattern: ["/sys/class/tty/ttyUSB3"])
    monkeypatch.setattr(
        serial_ports.os.path, "realpath",
        lambda path: "/sys/devices/usb8/8-1/8-1.1/8-1.1:1.0/ttyUSB3",
    )
    monkeypatch.setattr(serial_ports.os.path, "exists", lambda path: True)
    assert serial_ports.resolve_serial_port("/dev/gripper_left", usb_port="8-1.1") == "/dev/ttyUSB3"


def test_existing_alias_is_used_when_usb_topology_is_missing(monkeypatch):
    monkeypatch.setattr(serial_ports.glob, "glob", lambda pattern: [])
    monkeypatch.setattr(serial_ports.os.path, "exists", lambda path: True)
    assert serial_ports.resolve_serial_port("/dev/gripper_left", usb_port="8-1.1") == "/dev/gripper_left"


def test_serial_number_is_used_after_usb_and_alias(monkeypatch):
    monkeypatch.setattr(serial_ports, "find_serial_port_by_usb_port", lambda port: None)
    monkeypatch.setattr(serial_ports.os.path, "exists", lambda path: False)
    monkeypatch.setattr(serial_ports, "find_serial_port_by_ftdi_serial", lambda serial: "/dev/ttyUSB4")
    assert serial_ports.resolve_serial_port("/dev/gripper_left", "test-serial", "8-1.1") == "/dev/ttyUSB4"


def test_absent_device_retains_configured_port(monkeypatch):
    monkeypatch.setattr(serial_ports.glob, "glob", lambda pattern: [])
    monkeypatch.setattr(serial_ports.os.path, "exists", lambda path: False)
    assert serial_ports.resolve_serial_port("/dev/gripper_left", usb_port="8-1.1") == "/dev/gripper_left"
