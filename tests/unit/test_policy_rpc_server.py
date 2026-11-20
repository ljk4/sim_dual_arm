from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytest.importorskip("grpc")
pb2 = pytest.importorskip("dual_arm_policy.policy_pb2")
grpc = pytest.importorskip("grpc")
pb2_grpc = pytest.importorskip("dual_arm_policy.policy_pb2_grpc")
from dual_arm_learning.policy_server import PolicyServer  # noqa: E402
from fake_provider import FakeProvider, load_fake_config  # noqa: E402


def test_real_grpc_server_uses_the_generated_descriptor():
    config, settings = load_fake_config(ROOT / "tests/fakes/policy/fake_policy.yaml")
    config = config.model_copy(update={"port": 0})
    server = PolicyServer(config, FakeProvider(config, settings))
    port = server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    try:
        stub = pb2_grpc.PolicyServiceStub(channel)
        info = stub.GetPolicyInfo(
            pb2.GetPolicyInfoRequest(model_bundle_digest=config.model_bundle_digest),
            timeout=2.0,
        )
        assert info.command_space == "joint_position"
        assert info.chunk_size == 60
        assert info.action_period_ns == 50_000_000
        assert info.test_only is True
        assert tuple(info.action_joint_names) == config.action_joint_names
        assert pb2.DESCRIPTOR.package == "bimanual.policy"
    finally:
        server.stop(0.2)
        channel.close()
