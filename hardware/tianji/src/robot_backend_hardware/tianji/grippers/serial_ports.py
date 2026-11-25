"""Resolve TianJi serial devices by USB topology before udev aliases."""
import glob
import os


def find_serial_port_by_ftdi_serial(serial_number: str) -> str | None:
    for tty in sorted(glob.glob("/sys/class/tty/ttyUSB*")):
        parent = os.path.realpath(tty + "/device")
        for _ in range(5):
            parent = os.path.dirname(parent)
            serial_path = os.path.join(parent, "serial")
            if os.path.exists(serial_path):
                with open(serial_path, encoding="utf-8") as stream:
                    if stream.read().strip() == serial_number:
                        return "/dev/" + os.path.basename(tty)
                break
    return None


def find_serial_port_by_usb_port(usb_port: str) -> str | None:
    for tty in sorted(glob.glob("/sys/class/tty/ttyUSB*")):
        path = os.path.realpath(tty + "/device")
        for part in path.split(os.sep):
            if ":" in part and "-" in part:
                if part.split(":")[0] == usb_port:
                    return "/dev/" + os.path.basename(tty)
                break
    return None


def resolve_serial_port(port: str, serial_number: str = "", usb_port: str = "") -> str:
    if usb_port:
        found = find_serial_port_by_usb_port(usb_port)
        if found:
            return found
    if port and os.path.exists(port):
        return port
    if serial_number:
        found = find_serial_port_by_ftdi_serial(serial_number)
        if found:
            return found
    return port
