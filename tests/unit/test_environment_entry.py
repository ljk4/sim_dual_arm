"""The three Compose files are directly usable without a host wrapper."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
COMPOSES = {
    "simulation": ROOT / "simulation/mujoco/docker/docker-compose.yml",
    "runtime": ROOT / "runtime/ros2/docker/docker-compose.yml",
    "learning": ROOT / "learning/lerobot/docker/docker-compose.yml",
}


def test_standalone_compose_files_share_only_required_networks():
    assert not (ROOT / "docker-compose.yml").exists()
    assert not (ROOT / "scripts/env.sh").exists()
    expected_networks = {
        "simulation": ["ros"],
        "runtime": ["ros", "learning"],
        "learning": ["learning"],
    }
    for service_name, path in COMPOSES.items():
        document = yaml.safe_load(path.read_text())
        assert document["name"] == "sim-dual-arm"
        assert set(document["services"]) == {service_name}
        service = document["services"][service_name]
        assert service["build"] == "."
        assert service["networks"] == expected_networks[service_name]
        assert service["env_file"] == "../configs/container.env"
        assert "command" not in service and "entrypoint" not in service
        assert service["init"] is True
        assert "${" not in path.read_text()
        assert (path.parent / "Dockerfile").is_file()
        assert (path.parent.parent / "scripts/run.sh").is_file()
    assert (COMPOSES["simulation"].parent / "../").resolve() == ROOT / "simulation/mujoco"


def test_business_scripts_still_refuse_host_execution():
    scripts = [path.parent.parent / "scripts/run.sh" for path in COMPOSES.values()]
    scripts.append(ROOT / "runtime/ros2/scripts/record.sh")
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
        ["bash", str(ROOT / "runtime/ros2/scripts/record_parallel.sh"), "--help"],
        env={**os.environ, "PROJECT_CONTAINER": "0"},
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0
    assert "--workers" in result.stdout and "--count" in result.stdout


def test_parallel_entry_explains_host_requirement_inside_container():
    result = subprocess.run(
        ["bash", str(ROOT / "runtime/ros2/scripts/record_parallel.sh"), "--help"],
        env={**os.environ, "PROJECT_CONTAINER": "1"},
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 2
    assert "宿主机" in result.stderr
