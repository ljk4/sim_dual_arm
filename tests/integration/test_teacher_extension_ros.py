"""Both Teacher adapters resolve the selected extension capability generically."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("robot_api_msgs")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from dual_arm_runtime.teacher_client import teacher_service_type  # noqa: E402
from robot_backend_sim.common.ros.teacher_extension import (  # noqa: E402
    register_teacher_service,
    teacher_extension_modules,
)
from robot_contracts.models import ContractError  # noqa: E402

CAPABILITIES = (
    "marvin_stack_api_msgs.teacher_chunk.v1",
    "marvin_handover_api_msgs.teacher_chunk.v1",
)


class _Node:
    def __init__(self) -> None:
        self.services: list[tuple[object, str]] = []

    def create_service(self, service_type, name, callback, callback_group):
        self.services.append((service_type, name))
        return object()


def _available() -> list[str]:
    available = []
    for capability in CAPABILITIES:
        package = capability.split(".", 1)[0]
        try:
            importlib.import_module(f"{package}.srv")
        except ImportError:
            continue
        available.append(capability)
    return available


def test_runtime_and_simulation_resolve_the_same_teacher_type():
    available = _available()
    if not available:
        pytest.skip("no scenario Teacher extension is built in this SDK")
    for capability in available:
        package = capability.split(".", 1)[0]
        messages = importlib.import_module(f"{package}.msg")
        services = importlib.import_module(f"{package}.srv")
        assert str(messages.TeacherCapability.ID) == capability
        assert teacher_service_type(capability) is services.GetTeacherChunk
        request = services.GetTeacherChunk.Request()
        # Both Teacher extensions align their target clock to the scheduled hand-off.
        assert hasattr(request, "handoff_time")
        capability_type, service_type = teacher_extension_modules(capability)
        assert capability_type is messages.TeacherCapability
        assert service_type is services.GetTeacherChunk
        node = _Node()
        resolved = register_teacher_service(
            node,
            capability=capability,
            root="/robot",
            callback=object(),
            callback_group=object(),
        )
        assert resolved == capability
        assert node.services == [(services.GetTeacherChunk, "/robot/get_task_chunk")]


def test_old_capability_and_unknown_package_are_rejected():
    with pytest.raises(ContractError) as caught:
        teacher_service_type("teacher_chunk")
    assert caught.value.reason == "UNSUPPORTED_TEACHER_CAPABILITY"

    with pytest.raises(ContractError) as caught:
        teacher_service_type("missing_extension_msgs.teacher_chunk.v1")
    assert caught.value.reason in {
        "TEACHER_EXTENSION_MISSING",
        "UNSUPPORTED_TEACHER_CAPABILITY",
    }

    sim_srv = importlib.import_module("sim_api_msgs.srv")
    assert not hasattr(sim_srv, "GetTeacherChunk")
