"""The common runtime can load and create control clients without simulation types."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("grpc")

ROOT = Path(__file__).resolve().parents[2]


def test_common_runtime_does_not_load_sim_interfaces():
    script = '''
import importlib.abc
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
class RejectSim(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"sim_api_msgs", "panda_stack_api_msgs"}:
            raise AssertionError("optional interface loaded: " + fullname)
sys.meta_path.insert(0, RejectSim())
from dual_arm_runtime.robot_control_client import RobotControlClient
from dual_arm_runtime.runtime_config import load_runtime_configuration
from dual_arm_runtime.runtime_node import RuntimeNode
root = Path(sys.argv[1])
configuration = load_runtime_configuration(
    root / "shared/scenarios/panda_sim/stack/configs/runtime/teacher_collect.yaml", root
)
created = []
node = SimpleNamespace(create_client=lambda service, name, **kwargs: created.append(name))
RobotControlClient(node, configuration.recipe, configuration.profile, (1.0,) * 16,
                   threading.Event(), lambda error: None)
assert len(created) == 7
assert all(not name.endswith("reset_simulation") for name in created)
assert "sim_api_msgs" not in sys.modules
assert "panda_stack_api_msgs" not in sys.modules
'''
    environment = {**os.environ, "PYTHONPATH": ":".join(str(ROOT / path) for path in (
        "shared/common/python/contracts/src",
        "shared/common/python/trajectory/src",
        "shared/common/python/run_logging/src",
        "shared/common/python/ros_support/src",
        "runtime/ros2/ros/dual_arm_runtime"))
        + ":" + os.environ.get("PYTHONPATH", "")}
    result = subprocess.run([sys.executable, "-c", script, str(ROOT)], env=environment,
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
