import json
from pathlib import Path

from dual_arm_learning.policy_types import ProviderContext, ProviderJointSample, ProviderObservation
from fake_provider import FakeProvider, FakeSettings, load_fake_config

ROOT = Path(__file__).resolve().parents[2]


def test_fake_provider_replays_only_applied_labels(tmp_path):
    labels = {
        "labels": [
            {
                "observation_id": 1,
                "observation_sim_time_ns": 1_000_000_000,
                "label_sim_time_ns": 1_050_000_000,
                "applied_positions": [0.04] * 16,
                "measured_positions": [0.0] * 16,
            }
        ]
    }
    path = tmp_path / "labels.json"
    path.write_text(json.dumps(labels), encoding="utf-8")
    config, settings = load_fake_config(ROOT / "configs/learning/replay_policy.yaml")
    provider = FakeProvider(
        config,
        FakeSettings(
            replay_labels=str(path),
            latency_s=0.0,
        ),
    )
    context = ProviderContext(
        config.robot_profile_hash,
        config.robot_profile_hash,
        "backend",
        "sim",
        1,
        "episode",
        1,
    )
    provider.open("session", context, "open", b"{}", b"{}", b"{}")
    observation = ProviderObservation(
        observation_id=1,
        reference_robot_time_ns=1_000_000_000,
        joints=ProviderJointSample(
            state_seq=1,
            captured_at_robot_time_ns=1_000_000_000,
            joint_names=config.joint_names,
            positions=(0.0,) * 16,
            velocities=(0.0,) * 16,
        ),
        cameras=(),
        task_id=config.task_id,
        task_text=config.task_text,
    )
    prediction = provider.predict("session", "request", context, observation)
    assert len(prediction.targets) == config.chunk_size
    assert prediction.targets[0].positions == (0.04,) * 16
    assert prediction.targets[0].positions != observation.joints.positions


def test_replay_uses_relative_time_across_independent_episode_clocks(tmp_path):
    from dataclasses import replace

    path = tmp_path / "labels.json"
    path.write_text(
        json.dumps(
            {
                "labels": [
                    {
                        "label_sim_time_ns": 1_050_000_000 + index * 50_000_000,
                        "applied_positions": [0.01 * (index + 1)] * 16,
                    }
                    for index in range(3)
                ]
            }
        )
    )
    config, _ = load_fake_config(ROOT / "configs/learning/replay_policy.yaml")
    provider = FakeProvider(config, FakeSettings(replay_labels=str(path), latency_s=0.0))
    context = ProviderContext("robot", config.robot_profile_hash, "backend", "sim", 1, "ep", 1)
    provider.open("session", context, "open", b"{}", b"{}", b"{}")
    observed = ProviderObservation(
        observation_id=1,
        reference_robot_time_ns=5_000_000_000,
        joints=ProviderJointSample(1, 5_000_000_000, config.joint_names, (0.0,) * 16, (0.0,) * 16),
        cameras=(),
        task_id=config.task_id,
        task_text=config.task_text,
    )
    first = provider.predict("session", "first", context, observed)
    assert [target.positions[0] for target in first.targets[:3]] == [0.01, 0.02, 0.03]
    second = provider.predict(
        "session",
        "second",
        context,
        replace(observed, observation_id=2, reference_robot_time_ns=5_050_000_000),
    )
    assert second.targets[0].positions[0] == 0.02
    provider.reset("session", "reset", context)
    reset = provider.predict(
        "session",
        "reset-first",
        context,
        replace(observed, observation_id=3, reference_robot_time_ns=20_000_000_000),
    )
    assert reset.targets[0].positions[0] == 0.01
