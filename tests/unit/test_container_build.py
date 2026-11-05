"""Deployment failure boundaries: SDK ownership/origin and image provenance."""

import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load_tool(name: str):
    path = ROOT / "infrastructure/scripts" / f"{name}.py"
    specification = importlib.util.spec_from_file_location(f"dsh052_{name}", path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


check_dockerfiles = _load_tool("check_dockerfiles")
policy_sdk = _load_tool("policy_sdk")


def sdk_fixture(tmp_path):
    project = tmp_path / "project"
    proto = project / "contracts/policy/v1/policy.proto"
    proto.parent.mkdir(parents=True)
    proto.write_text('syntax = "proto3";\n')
    sdk = tmp_path / "sdk"
    for relative in policy_sdk.REQUIRED_FILES:
        path = sdk / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("")
    (sdk / "source.sha256").write_text(hashlib.sha256(proto.read_bytes()).hexdigest())
    return project, sdk


def test_stale_sdk_rejected_before_import(tmp_path, monkeypatch):
    project, sdk = sdk_fixture(tmp_path)
    (sdk / "source.sha256").write_text("0" * 64)
    monkeypatch.setattr(
        policy_sdk.importlib, "import_module", lambda _: pytest.fail("imported stale SDK")
    )
    with pytest.raises(policy_sdk.SDKError, match="POLICY_SDK_STALE"):
        policy_sdk.verify_sdk(project, sdk)


def test_incomplete_sdk_rejected(tmp_path):
    project, sdk = sdk_fixture(tmp_path)
    (sdk / "dual_arm_policy/v1/policy_pb2.py").unlink()
    with pytest.raises(policy_sdk.SDKError, match="POLICY_SDK_INCOMPLETE"):
        policy_sdk.verify_sdk(project, sdk)


def test_installed_sdk_cannot_replace_mounted_sdk(tmp_path, monkeypatch):
    project, sdk = sdk_fixture(tmp_path)
    module = ModuleType(policy_sdk.MODULES[0])
    module.__file__ = str(tmp_path / "installed/policy_pb2.py")
    monkeypatch.setitem(sys.modules, policy_sdk.MODULES[0], module)
    monkeypatch.syspath_prepend(str(sdk))
    with pytest.raises(policy_sdk.SDKError, match="POLICY_SDK_FALLBACK"):
        policy_sdk.verify_sdk(project, sdk)


def test_unknown_output_is_preserved(tmp_path):
    output = tmp_path / "unrelated"
    output.mkdir()
    important = output / "user-data.txt"
    important.write_text("keep")
    with pytest.raises(policy_sdk.SDKError, match="UNOWNED_OUTPUT"):
        policy_sdk.prepare_output(tmp_path / "project", output)
    assert important.read_text() == "keep"
    assert sorted(path.name for path in output.iterdir()) == ["user-data.txt"]


def test_symlink_output_not_adopted(tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    target = tmp_path / "user-file"
    target.write_text("keep")
    (output / policy_sdk.OWNER_FILE).symlink_to(target)
    with pytest.raises(policy_sdk.SDKError, match="symlink"):
        policy_sdk.prepare_output(tmp_path / "project", output)
    assert target.read_text() == "keep"


def test_repository_ruckig_wheel_matches_provenance_and_unique_lock():
    docker = ROOT / "simulation/mujoco/docker"
    provenance = json.loads((docker / "wheels/provenance.json").read_text())
    wheel = docker / "wheels" / provenance["wheel_file"]
    assert wheel.stat().st_size == provenance["wheel_bytes"]
    assert hashlib.sha256(wheel.read_bytes()).hexdigest() == provenance["wheel_sha256"]
    lock = (docker / "requirements.lock").read_text()
    assert f"{provenance['package']}=={provenance['version']}" in lock
    assert provenance["wheel_sha256"] in lock
    assert "--find-links=/opt/dependencies" in lock
    assert not (docker / "sim-requirements.lock").exists()
    assert not (docker / "ruckig-requirements.lock").exists()
    assert not (docker / "sim-tools-requirements.txt").exists()
    assert not (docker / "ruckig-build-requirements.lock").exists()
    assert not (docker / "ruckig-source.json").exists()


def test_sim_dockerfile_uses_single_hash_lock_without_extra_tool_installations():
    text = (ROOT / "simulation/mujoco/docker/Dockerfile").read_text()
    assert "requirements-dev" not in text
    assert "sim-packages" not in text
    assert "ros-jazzy-rosbag2" not in text
    assert "requirements.lock" in text
    assert "sim-requirements.lock" not in text
    assert "sim-tools-requirements.txt" not in text
    assert "ruckig-requirements.lock" not in text
    assert "python3-colcon-common-extensions" not in text
    assert "pytest" not in text
    assert "ruff" not in text
    assert "setuptools" not in text
    assert "RUFF_CACHE_DIR" not in text
    assert "image_metadata" not in text
    assert "build-tools" not in text
    assert "versions.json" not in text
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" not in text


def test_sim_dockerfile_copies_fixed_wheel_once_and_installs_with_hashes():
    text = (ROOT / "simulation/mujoco/docker/Dockerfile").read_text()
    from_lines = [line for line in text.splitlines() if line.startswith("FROM ")]
    assert len(from_lines) == 1
    copy_lines = [line for line in text.splitlines() if line.startswith("COPY ")]
    assert len(copy_lines) == 1
    assert " AS " not in text
    assert "COPY --from" not in text
    assert "ruckig-builder" not in text
    assert "pip wheel" not in text
    assert "build-essential" not in text
    assert "python3-dev" not in text
    assert "ruckig-build-requirements.lock" not in text
    assert "ruckig-source.json" not in text
    assert "/opt/ruckig" not in text
    assert "provenance" not in text
    assert "ruckig-check" not in text
    assert "requirements.lock" in text
    assert "sim-requirements.lock" not in text
    assert (
        "wheels/ruckig-0.15.3-cp312-cp312-linux_x86_64.whl" in text
    )
    assert sum(line.startswith("RUN pip install ") for line in text.splitlines()) == 1
    assert (
        "pip install --no-cache-dir --require-hashes "
        "-r /opt/dependencies/requirements.lock" in text
    )
    assert "/opt/sim" not in text


@pytest.mark.parametrize(
    "instruction",
    [
        "RUN python - <<'PY'\nprint('hidden')\nPY\n",
        'RUN python -c "print(1)"\n',
        "COPY simulation /workspace/simulation\n",
        'COPY ["learning", "/workspace/learning"]\n',
        "COPY simulation/mujoco/scripts/run.sh /opt/run.sh\n",
        'COPY . /workspace\n',
        "FROM ${DEV_ROS_IMAGE} AS runtime\n",
        'ENTRYPOINT ["bash", "/workspace/simulation/mujoco/scripts/run.sh"]\n',
        'ENV PATH="/opt/venv/bin:${PATH}" PYTHONPATH=/workspace/src\n',
        'WORKDIR /workspace\n',
        'RUN if test -f /tmp/input; then do_work; fi\n',
    ],
)
def test_dockerfile_rejects_embedded_programs_and_application_copy(instruction):
    assert check_dockerfiles.check_text(instruction)


def test_dockerfile_allows_dependency_files_and_pinned_wheel_artifacts():
    instructions = """COPY requirements.lock \\
     wheels/ruckig.whl /opt/dependencies/
RUN pip install --no-cache-dir --require-hashes -r /opt/dependencies/requirements.lock
"""
    assert check_dockerfiles.check_text(instructions) == []


def test_repository_dockerfile_and_compose_boundaries_pass():
    import subprocess

    result = subprocess.run(
        [sys.executable, str(ROOT / "infrastructure/scripts/check_dockerfiles.py")],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Dockerfile/Compose boundaries passed" in result.stdout


def test_compose_checker_rejects_task_startup_healthcheck_and_socket():
    document = {
        "services": {
            "runtime": {
                "image": "sim-dual-arm/runtime:current",
                "build": {
                    "context": ".",
                    "dockerfile": "Dockerfile",
                    "target": "test",
                },
                "command": ["python3", "-m", "dual_arm_runtime.native_node"],
                "healthcheck": {"test": ["CMD", "true"]},
                "entrypoint": ["bash", "/workspace/runtime/ros2/scripts/run.sh"],
                "environment": {
                    "PROJECT_CONTAINER": "1",
                    "ROS_DOMAIN_ID": "42",
                },
                "volumes": [
                    {"type": "bind", "source": ".", "target": "/workspace", "read_only": True},
                    {"type": "bind", "source": ".build", "target": "/build"},
                    {"type": "bind", "source": "var", "target": "/data"},
                    {
                        "type": "bind",
                        "source": ".build/ros-dev",
                        "target": "/opt/ros-dev",
                        "read_only": True,
                    },
                    {
                        "type": "bind",
                        "source": ".build/sdk",
                        "target": "/opt/policy-sdk",
                        "read_only": True,
                    },
                    {
                        "type": "bind",
                        "source": "/var/run/docker.sock",
                        "target": "/var/run/docker.sock",
                    },
                ],
            }
        }
    }
    issues = check_dockerfiles.check_compose_document(document, name="test")
    joined = "\n".join(issues)
    assert "idle CMD" in joined
    assert "healthcheck" in joined
    assert "Docker socket" in joined
    assert "must not select a build target" in joined


def test_learning_compose_must_not_mount_ros_or_host_venv():
    document = {
        "services": {
            "learning": {
                "image": "sim-dual-arm/learning:current",
                "build": {
                    "context": ".",
                    "dockerfile": "Dockerfile",
                },
                "command": ["idle"],
                "entrypoint": ["bash", "/workspace/learning/lerobot/scripts/run.sh"],
                "environment": {
                    "PROJECT_CONTAINER": "1",
                    "ROS_DOMAIN_ID": "42",
                    "LEARNING_VENV": "/opt/host-learning-venv",
                },
                "volumes": [
                    {"type": "bind", "source": ".", "target": "/workspace", "read_only": True},
                    {"type": "bind", "source": ".build", "target": "/build"},
                    {"type": "bind", "source": "var", "target": "/data"},
                    {
                        "type": "bind",
                        "source": ".build/ros-dev",
                        "target": "/opt/ros-dev",
                        "read_only": True,
                    },
                    {
                        "type": "bind",
                        "source": ".build/sdk",
                        "target": "/opt/policy-sdk",
                        "read_only": True,
                    },
                    {
                        "type": "bind",
                        "source": "/host/venv",
                        "target": "/opt/host-learning-venv",
                        "read_only": True,
                    },
                ],
            }
        }
    }
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("must not mount /opt/ros-dev" in item for item in issues)
    assert any("must not define ROS_DOMAIN_ID" in item for item in issues)
    assert any("must not define LEARNING_VENV" in item for item in issues)
    assert any("must not mount a host virtualenv" in item for item in issues)


def test_compose_checker_rejects_variable_binds_and_requires_nested_writable_mounts():
    document = {
        "services": {
            "runtime": {
                "image": "sim-dual-arm/runtime:current",
                "build": {
                    "context": ".",
                    "dockerfile": "Dockerfile",
                },
                "command": ["idle"],
                "entrypoint": ["bash", "/workspace/runtime/ros2/scripts/run.sh"],
                "environment": {
                    "PROJECT_CONTAINER": "1",
                    "ROS_DOMAIN_ID": "42",
                },
                "gpus": "all",
                "networks": ["ros", "learning"],
                "volumes": [
                    {"type": "bind", "source": "${PROJECT_ROOT}", "target": "/workspace", "read_only": True},
                    {"type": "bind", "source": ".build", "target": "/build"},
                    {"type": "bind", "source": "var", "target": "/data"},
                    {
                        "type": "bind",
                        "source": "/sdk/ros",
                        "target": "/opt/ros-dev",
                        "read_only": True,
                    },
                    {
                        "type": "bind",
                        "source": "/sdk/policy",
                        "target": "/opt/policy-sdk",
                        "read_only": True,
                    },
                ],
            }
        },
        "networks": {
            "ros": {},
            "learning": {},
        },
    }
    joined = "\n".join(
        check_dockerfiles.check_compose_document(document, name="test")
    )
    assert "direct path" in joined
    assert "/workspace/var writable" in joined
    assert "/workspace/.build writable" in joined
    assert "must not require GPUs" in joined


def test_compose_checker_rejects_old_environment_entrypoint():
    document = {
        "services": {
            "runtime": {
                "image": "sim-dual-arm/runtime:current",
                "build": {
                    "context": ".",
                    "dockerfile": "Dockerfile",
                },
                "command": ["idle"],
                "entrypoint": [
                    "bash",
                    "/workspace/infrastructure/docker/entrypoints/environment.sh",
                ],
                "environment": {
                    "PROJECT_CONTAINER": "1",
                    "ROS_DOMAIN_ID": "42",
                },
                "volumes": [
                    {"type": "bind", "source": "/repo", "target": "/workspace", "read_only": True},
                    {"type": "bind", "source": "/build", "target": "/build"},
                    {"type": "bind", "source": "/data", "target": "/data"},
                    {
                        "type": "bind",
                        "source": "/sdk/ros",
                        "target": "/opt/ros-dev",
                        "read_only": True,
                    },
                    {
                        "type": "bind",
                        "source": "/sdk/policy",
                        "target": "/opt/policy-sdk",
                        "read_only": True,
                    },
                ],
            }
        }
    }
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("inherit the image's idle CMD" in item for item in issues)


def test_compose_checker_requires_simulation_scoped_mounts_init_and_dds():
    import yaml

    document = yaml.safe_load(
        (ROOT / "simulation/mujoco/docker/docker-compose.yml").read_text()
    )
    assert check_dockerfiles.check_compose_document(document, name="test") == []
    service = document["services"]["simulation"]
    service["entrypoint"] = ["bash", "/app/scripts/run.sh"]
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("inherit the image's idle CMD" in item for item in issues)
    service.pop("entrypoint")
    service["command"] = ["backend"]
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("inherit the image's idle CMD" in item for item in issues)
    service.pop("command")
    service.pop("gpus")
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("runtime: nvidia and gpus: all" in item for item in issues)
    service["gpus"] = "all"
    service["runtime"] = "runc"
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("runtime: nvidia and gpus: all" in item for item in issues)
    service["runtime"] = "nvidia"
    service["init"] = False
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("init: true" in item for item in issues)
    service["init"] = True
    service["volumes"].append(
        {"type": "bind", "source": "/third_party", "target": "/third_party", "read_only": True}
    )
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("must mount only" in item for item in issues)


def test_compose_checker_rejects_local_context_with_root_style_dockerfile():
    document = {
        "services": {
            "simulation": {
                "image": "sim-dual-arm/simulation:current",
                "build": {
                    "context": ".",
                    "dockerfile": "simulation/mujoco/docker/Dockerfile",
                },
            }
        }
    }
    issues = check_dockerfiles.check_compose_document(document, name="test")
    assert any("build must use the local Dockerfile" in item for item in issues)
