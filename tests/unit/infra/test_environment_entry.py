"""The Compose files are directly usable without a host wrapper."""

from __future__ import annotations

import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
COMPOSES = {
    "simulation": ROOT / "simulation/mujoco/docker/docker-compose.yml",
    "runtime": ROOT / "runtime/ros2/docker/docker-compose.yml",
    "learning": ROOT / "learning/lerobot/docker/docker-compose.yml",
    "hardware": ROOT / "hardware/tianji/docker/docker-compose.yml",
}


def test_environment_entry_lists_hardware_without_starting_docker():
    result = subprocess.run(
        ["bash", str(ROOT / "env.sh"), "list"],
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0
    assert "hardware" in result.stdout.split()


def test_hardware_compose_exposes_usb_serial_and_camera_devices():
    document = yaml.safe_load(COMPOSES["hardware"].read_text())
    service = document["services"]["hardware"]
    assert "/dev:/dev" in service["volumes"]
    assert "/sys:/sys:ro" in service["volumes"]
    assert "/run/udev:/run/udev:ro" in service["volumes"]
    assert set(service["group_add"]) >= {"20", "44", "46"}
    assert "c 189:* rmw" in service["device_cgroup_rules"]
    assert "c 188:* rmw" in service["device_cgroup_rules"]
    assert "c 81:* rmw" in service["device_cgroup_rules"]
    assert (ROOT / "hardware/tianji/configs/devices.yaml").is_file()


def test_standalone_compose_files_use_host_network():
    assert not (ROOT / "docker-compose.yml").exists()
    assert not (ROOT / "scripts/env.sh").exists()
    for service_name, path in COMPOSES.items():
        document = yaml.safe_load(path.read_text())
        assert document["name"] == "sim-dual-arm"
        assert set(document["services"]) == {service_name}
        assert "networks" not in document
        service = document["services"][service_name]
        assert service["build"] == "."
        assert service["network_mode"] == "host"
        assert "networks" not in service
        assert "hostname" not in service
        assert "extra_hosts" not in service
        assert service["env_file"] == "../configs/container.env"
        assert "command" not in service and "entrypoint" not in service
        assert service["init"] is True
        assert "${" not in path.read_text()
        assert (path.parent / "Dockerfile").is_file()
        assert (path.parent.parent / "scripts/test.sh").is_file()
        assert not (path.parent.parent / "scripts/run.sh").exists()
    assert (COMPOSES["simulation"].parent / "../").resolve() == ROOT / "simulation/mujoco"

    fastdds = ET.fromstring((ROOT / "shared/common/tools/docker/fastdds.xml").read_text())
    ns = {"fast": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    descriptor = fastdds.find(".//fast:transport_descriptor", ns)
    assert descriptor is not None
    assert descriptor.findtext("fast:type", default="", namespaces=ns) == "UDPv4"
    addresses = [
        node.text
        for node in descriptor.findall("fast:interfaceWhiteList/fast:address", ns)
    ]
    assert addresses == ["127.0.0.1"]
    assert (
        fastdds.findtext(".//fast:rtps/fast:useBuiltinTransports", default="", namespaces=ns)
        == "false"
    )


def test_business_scripts_still_refuse_host_execution():
    scripts = [path.parent.parent / "scripts/test.sh" for path in COMPOSES.values()]
    scripts.extend(
        [
            ROOT / "simulation/mujoco/scripts/backend.sh",
            ROOT / "simulation/mujoco/scripts/inference_backend.sh",
            ROOT / "simulation/mujoco/scripts/teacher_backend.sh",
            ROOT / "simulation/mujoco/scripts/viewer.sh",
            ROOT / "runtime/ros2/scripts/record.sh",
        ]
    )
    for script in scripts:
        result = subprocess.run(
            ["bash", str(script), "--help"],
            cwd=ROOT,
            env={**os.environ, "PROJECT_CONTAINER": "0"},
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 2, script
        assert "must run inside a project container" in result.stderr


def test_parallel_entry_shows_help_on_host_without_starting_docker():
    result = subprocess.run(
        ["bash", str(ROOT / "record_parallel.sh"), "--help"],
        env={**os.environ, "PROJECT_CONTAINER": "0"},
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0
    assert "--workers" in result.stdout and "--count" in result.stdout


def test_parallel_entry_explains_host_requirement_inside_container():
    result = subprocess.run(
        ["bash", str(ROOT / "record_parallel.sh"), "--help"],
        env={**os.environ, "PROJECT_CONTAINER": "1"},
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 2
    assert "宿主机" in result.stderr
