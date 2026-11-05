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
    for service_name, path in COMPOSES.items():
        script = path.parent.parent / "scripts/run.sh"
        result = subprocess.run(
            ["bash", str(script), "--help"],
            cwd=ROOT,
            env={**os.environ, "PROJECT_CONTAINER": "0"},
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 2, service_name
        assert "must run inside a project container" in result.stderr
