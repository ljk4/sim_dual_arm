"""Pure audit rules consume immutable Python facts, without an SDK or ROS setup."""
from __future__ import annotations

import ast
import importlib
import json
import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from dual_arm_runtime.offline.audit.audit_stream import StreamAudit
from dual_arm_runtime.offline.audit.records import BackendInfo, CameraFrame, State
from robot_contracts.models import ContractError, RunContext
from robot_contracts.profiles import load_profile
from robot_contracts.trajectory.spline import ReferencePlan

ROOT = Path(__file__).resolve().parents[2]
RULES = ROOT / "runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/audit"
FORBIDDEN = {
    "rclpy",
    "robot_api_msgs",
    "sim_api_msgs",
    "rosbag2_py",
    "mcap_ros2",
    "robot_ros_support",
}


def _audit():
    path = ROOT / "shared/scenarios/marvin_sim/stack/profiles/robots/marvin_stack/profile.json"
    profile = load_profile(path, ROOT / "shared/common/interfaces/schemas")
    context = RunContext(profile.robot_id, profile.digest, "backend", "time", 0, "episode", 1)
    configuration = {
        "profile": json.loads(path.read_text()), "profile_hash": profile.digest,
        "run_context": context.model_dump(),
        "recipe": {"robot_namespace": "/robot", "motions": [],
                   "observation": {"max_age_ns": 1_000_000_000, "wall_arrival_timeout_s": 1.0},
                   "control_constraints": {"max_accelerations": [1.0] * 16}},
    }
    return StreamAudit("episode", configuration), context, profile


def test_rules_accept_pure_state_and_detect_capture_conflicts():
    audit, context, profile = _audit()
    info = BackendInfo(context, "mujoco", False, profile.joint_names, profile.camera_names,
                       tuple(audit.required_capabilities))
    audit.info(info)
    state = State(context, 1, 2, 2, profile.joint_names, profile.initial_positions, (0.0,) * 16)
    audit.state(state)
    audit.capture_payload("/robot/state", state, b"test_only capture")
    assert audit.states == {1: 2}
    with pytest.raises(ContractError, match="conflicting payloads"):
        audit.capture_payload("/robot/state", state, b"different test_only capture")
    with pytest.raises(ContractError, match="stamp is invalid"):
        audit.state(replace(state, state_seq=2, joints_stamp_ns=3))


def test_camera_validation_uses_decoded_metadata():
    audit, context, profile = _audit()
    camera = profile.cameras[0]
    frame = CameraFrame(context, 1, 2, camera.width, camera.height, "rgb8",
                        bytes(camera.width * camera.height * 3), camera.width * 3,
                        camera.frame_id, camera.width, camera.height, camera.frame_id,
                        camera.calibration_digest)
    audit.frame("/robot/cameras/global/frame", frame)
    assert audit.frames == {("global", 1): 2}
    with pytest.raises(ContractError, match="Camera RGB step differs"):
        audit.frame("/robot/cameras/global/frame", replace(frame, step=1))
    with pytest.raises(ContractError, match="CameraInfo frame_id differs"):
        audit.frame("/robot/cameras/global/frame", replace(frame, info_frame_id="wrong"))


def test_recording_audit_checks_backend_kind_for_requested_simulation_only():
    audit, context, profile = _audit()
    configuration = {**audit.configuration, "source": "hardware"}
    audit = StreamAudit("episode", configuration)
    info = BackendInfo(
        context, "hardware", False, profile.joint_names, profile.camera_names,
        tuple(audit.required_capabilities),
    )
    audit.info(info)
    reset_audit = StreamAudit("episode", {**configuration, "reset_used": True})
    with pytest.raises(ContractError, match="simulation features require"):
        reset_audit.info(info)


def test_rule_imports_are_closed_to_ros_dependencies():
    modules = ["dual_arm_runtime.offline.audit." + p.stem for p in RULES.glob("*.py")]
    for path in RULES.glob("*.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom):
                assert (node.module or "").split(".")[0] not in FORBIDDEN
            elif isinstance(node, ast.Import):
                assert not {alias.name.split(".")[0] for alias in node.names} & FORBIDDEN
    # A separate interpreter rejects attempted transitive imports, even in a ROS container.
    script = '''
import importlib, importlib.abc, sys
blocked = set(sys.argv[1].split(","))
class Reject(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in blocked:
            raise AssertionError("forbidden dependency: " + fullname)
sys.meta_path.insert(0, Reject())
for name in sys.argv[2:]:
    importlib.import_module(name)
'''
    environment = {**os.environ, "PYTHONPATH": ":".join(str(ROOT / path) for path in (
        "shared/common/python/contracts/src",
        "shared/common/python/trajectory/src",
        "shared/common/python/run_logging/src",
        "runtime/ros2/ros/dual_arm_runtime"))}
    result = subprocess.run([sys.executable, "-c", script, ",".join(FORBIDDEN), *modules],
                            env=environment, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr


def test_contract_data_cannot_import_trajectory_algorithms():
    for path in (ROOT / "shared/common/python/contracts/src").rglob("*.py"):
        assert "robot_trajectory" not in path.read_text(), path
    assert not hasattr(ReferencePlan, "sample")
    assert importlib.import_module("robot_contracts.trajectory")


def test_simulation_core_keeps_ros_at_the_adapter_boundary():
    base = ROOT / "simulation/mujoco/src/robot_backend_sim"
    for path in base.rglob("*.py"):
        if "ros" in path.relative_to(base).parts or path.name in {"ros_conv.py", "viewer.py"}:
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom):
                assert (node.module or "").split(".")[0] not in FORBIDDEN, path
            elif isinstance(node, ast.Import):
                assert not {alias.name.split(".")[0] for alias in node.names} & FORBIDDEN, path
