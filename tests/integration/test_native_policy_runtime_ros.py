"""Actual policy RPC, ROS, physics and MCAP; including delayed and stale replies."""

from __future__ import annotations

import json
import os
import signal
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("dual_arm_policy.v1.policy_pb2")
from dual_arm_learning.policy_server import PolicyServer  # noqa: E402
from dual_arm_runtime.audit import inspect_episode  # noqa: E402
from fake_provider import FakeProvider, load_fake_config  # noqa: E402
from robot_run_logging import EventLogger  # noqa: E402
from test_native_runtime_ros import Processes, _runtime_json, ros_context  # noqa: E402,F401

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "fault", ["none", "wrong_epoch", "disconnect", "late", "midstream_disconnect"]
)
def test_policy_rpc_native_runtime_and_faults(tmp_path, fault):
    config, settings = load_fake_config(ROOT / "learning/lerobot/configs/fake_policy.yaml")
    config = replace(config, port=0)
    settings = (
        replace(settings, stop_after_predictions=2)
        if fault == "midstream_disconnect"
        else replace(settings, fault=fault)
    )
    log = EventLogger(
        tmp_path / "policy-log",
        run_id="policy-integration",
        component="policy",
        settings=config.logging,
    )
    provider = FakeProvider(config, settings)
    server = PolicyServer(config, provider, log)
    port = server.start()
    policy = yaml.safe_load(
        (ROOT / "runtime/ros2/configs/native_policy.yaml").read_text()
    )["policy"]
    policy["endpoint"] = f"127.0.0.1:{port}"
    if fault == "late":
        policy["predict_deadline_s"] = 0.1
    processes = Processes(
        tmp_path,
        native_overrides={"policy": policy},
        backend_source=ROOT / "tests/fixtures/simulation/backend_follower.yaml",
        runtime_source=ROOT / "runtime/ros2/configs/native_policy.yaml",
    )
    try:
        runtime = processes.start_runtime(once=True)
        code = runtime.wait(timeout=40)
        episode, _ = _runtime_json(tmp_path / "runtime.log")
        if fault == "none":
            assert code == 0, processes._output()
            assert episode["runtime"]["outcome"] == "completed"
            facts = inspect_episode(Path(episode["directory"]))
            assert facts["policy_chunks"] == facts["applied_chunks"] == 8
            assert facts["raw_prediction_points"] == 480
            assert facts["task"]["outcome"] == "not_evaluated"
            assert provider.predictions == 8
        else:
            assert code != 0, processes._output()
            assert episode["runtime"]["outcome"] == "failed"
            assert episode["task"]["outcome"] == "not_evaluated"
    finally:
        processes.stop()
        server.stop()
        log.close()
    events = [
        json.loads(line)
        for p in (tmp_path / "policy-log").glob("events*.jsonl")
        for line in p.read_text().splitlines()
    ]
    assert [e["event_seq"] for e in events] == list(range(1, len(events) + 1))
    if fault == "none":
        predictions = [
            e
            for e in events
            if e["event"] == "policy.rpc.finished" and e["fields"]["method"] == "Predict"
        ]
        assert len(predictions) == 8
    else:
        assert any(e["event"] == "policy.rpc.rejected" for e in events)


@pytest.mark.parametrize("mode", ["end", "signal"])
def test_policy_stream_stops_with_real_terminal_feedback(tmp_path, mode):
    from robot_api_msgs.srv import EndEpisode, StartEpisode
    from test_native_runtime_ros import RuntimeClient

    config, settings = load_fake_config(ROOT / "learning/lerobot/configs/fake_policy.yaml")
    config = replace(config, port=0)
    server = PolicyServer(config, FakeProvider(config, settings))
    port = server.start()
    policy = yaml.safe_load(
        (ROOT / "runtime/ros2/configs/native_policy.yaml").read_text()
    )["policy"]
    policy.update(endpoint=f"127.0.0.1:{port}", max_chunks=32)
    processes = Processes(
        tmp_path,
        native_overrides={"policy": policy},
        backend_source=ROOT / "tests/fixtures/simulation/backend_follower.yaml",
        runtime_source=ROOT / "runtime/ros2/configs/native_policy.yaml",
    )
    client = RuntimeClient()
    try:
        runtime = processes.start_runtime(once=False)
        client.attach(runtime, processes._output)
        client.wait_phase({"IDLE"}, timeout=20)
        response = client.call(client.start_client, StartEpisode.Request(task_id="policy-stop"))
        assert response.success, response.reason
        active = client.wait_phase({"ACTIVE", "FAULT"}, timeout=20)
        assert active.phase == "ACTIVE", processes._output()
        client.spin(0.7)
        if mode == "end":
            stopped = client.call(client.end_client, EndEpisode.Request(reason="TEST_END"))
            assert stopped.success, stopped.reason
            client.wait_phase({"IDLE", "FAULT"}, timeout=20)
        if runtime.poll() is None:
            os.killpg(runtime.pid, signal.SIGTERM)
        assert runtime.wait(timeout=25) in (0, 143), processes._output()
        episode, summary = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == ("stopped" if mode == "end" else "interrupted"), (
            processes._output()
        )
        assert episode["recording"]["complete"], processes._output()
        report = inspect_episode(Path(summary["record_directory"]))
        assert report["task"]["outcome"] == "not_evaluated"
        assert report["commands_without_terminal"] == 0
    finally:
        client.close()
        processes.stop()
        server.stop()


def test_first_definitive_follower_rejection_does_not_await_execution(tmp_path):
    config, settings = load_fake_config(ROOT / "learning/lerobot/configs/fake_policy.yaml")
    config = replace(config, port=0)
    server = PolicyServer(config, FakeProvider(config, settings))
    port = server.start()
    follower = yaml.safe_load((ROOT / "simulation/mujoco/configs/control/panda_follower.yaml").read_text())
    follower["max_target_deviations"] = [1e-6] * 16
    follower_path = tmp_path / "reject-follower.yaml"
    follower_path.write_text(yaml.safe_dump(follower))
    backend = yaml.safe_load((ROOT / "tests/fixtures/simulation/backend_follower.yaml").read_text())
    backend["robot"]["follower"] = str(follower_path)
    backend_path = tmp_path / "reject-backend-source.yaml"
    backend_path.write_text(yaml.safe_dump(backend))
    policy = yaml.safe_load(
        (ROOT / "runtime/ros2/configs/native_policy.yaml").read_text()
    )["policy"]
    policy["endpoint"] = f"127.0.0.1:{port}"
    processes = Processes(
        tmp_path,
        native_overrides={"policy": policy},
        backend_source=backend_path,
        runtime_source=ROOT / "runtime/ros2/configs/native_policy.yaml",
    )
    try:
        process = processes.start_runtime(once=True)
        assert process.wait(timeout=25) != 0
        episode, _ = _runtime_json(tmp_path / "runtime.log")
        assert episode["runtime"]["outcome"] == "failed"
        assert "TERMINAL_WAIT_ERROR" not in episode["runtime"]["reason"]
        assert episode["runtime"]["reason"].startswith("FOLLOWER_DEVIATION_EXCEEDED")
        assert episode["recording"]["complete"]
    finally:
        processes.stop()
        server.stop()
