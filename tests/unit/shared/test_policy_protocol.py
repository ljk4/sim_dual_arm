from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]


def test_source_proto_has_fixed_observation_time_and_policy_metadata():
    text = (ROOT / "shared/common/interfaces/policy/policy.proto").read_text()
    assert "time_from_observation_ns = 1;" in text
    assert "time_from_start_ns" not in text
    assert "uint32 chunk_size = 14;" in text
    assert "bool test_only = 15;" in text
    assert "action_period_ns = 10;" in text
    # Robot recipes (joint order, camera names, chunk counts) belong to the
    # scenario documentation, not to the common wire comments.
    for name in (
        "left_joint1",
        "left_joint7",
        "left_gripper_width",
        "right_joint1",
        "right_joint7",
        "right_gripper_width",
    ):
        assert name not in text
    scenario = (ROOT / "shared/scenarios/marvin/mujoco/stack/README.md").read_text()
    for name in (
        "left_joint1",
        "left_joint7",
        "left_gripper_width",
        "right_joint1",
        "right_joint7",
        "right_gripper_width",
    ):
        assert name in scenario
    assert "60 点 × 16 维" in scenario
    for name in ("global", "left_wrist", "right_wrist"):
        assert f"`{name}`" in scenario


def test_generated_sdk_matches_the_contract_when_available():
    pb2 = pytest.importorskip("dual_arm_policy.policy_pb2")
    assert pb2.DESCRIPTOR.package == "bimanual.policy"
    target = pb2.JointTarget.DESCRIPTOR.fields_by_name
    assert target["time_from_observation_ns"].number == 1
    assert "time_from_start_ns" not in target
    info = pb2.PolicyInfo.DESCRIPTOR.fields_by_name
    assert len(info) == 13
    assert "stateful" not in info
    assert info["chunk_size"].number == 14
    assert info["test_only"].number == 15
    assert info["action_period_ns"].number == 10
