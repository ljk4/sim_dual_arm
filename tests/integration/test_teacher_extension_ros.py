"""Both Teacher adapters use the same generated extension type and capability."""

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
from panda_stack_api_msgs.msg import TeacherCapability  # noqa: E402
from panda_stack_api_msgs.srv import GetTeacherChunk  # noqa: E402
from robot_backend_sim.ros.teacher_extension import register_teacher_service  # noqa: E402
from robot_contracts.models import ContractError  # noqa: E402


class _Node:
    def __init__(self) -> None:
        self.services: list[tuple[object, str]] = []

    def create_service(self, service_type, name, callback, callback_group):
        self.services.append((service_type, name))
        return object()


def test_runtime_and_simulation_resolve_the_same_teacher_type():
    assert teacher_service_type(TeacherCapability.ID) is GetTeacherChunk
    node = _Node()
    capability = register_teacher_service(
        node, root="/robot", callback=object(), callback_group=object()
    )
    assert capability == TeacherCapability.ID
    assert node.services == [(GetTeacherChunk, "/robot/get_task_chunk")]


def test_old_capability_and_old_package_are_rejected():
    with pytest.raises(ContractError) as caught:
        teacher_service_type("teacher_chunk")
    assert caught.value.reason == "UNSUPPORTED_TEACHER_CAPABILITY"

    sim_srv = importlib.import_module("sim_api_msgs.srv")
    assert not hasattr(sim_srv, "GetTeacherChunk")
