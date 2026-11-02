import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/run"))

import stack_once  # noqa: E402
from dual_arm_learning.act_contract import ActContract, load_act_settings  # noqa: E402


def contract(settings) -> ActContract:
    profile = json.loads(settings.profile.read_text())
    joints = profile["joints"]
    return ActContract(
        bundle_root=Path("/tmp/bundle"),
        bundle_digest="a" * 64,
        model_sha256="b" * 64,
        state_dict_sha256="b" * 64,
        config_sha256="c" * 64,
        processor_digest="d" * 64,
        stats_sha256="e" * 64,
        profile_file_sha256="f" * 64,
        profile_semantic_sha256="1" * 64,
        observation_schema_hash="2" * 64,
        action_schema_hash="3" * 64,
        joint_names=tuple(item["name"] for item in joints),
        joint_lower_bounds=tuple(item["min"] for item in joints),
        joint_upper_bounds=tuple(item["max"] for item in joints),
        camera_names=("global", "left_wrist", "right_wrist"),
        camera_width=320,
        camera_height=240,
        chunk_size=60,
        state_dim=16,
        action_period_ns=50_000_000,
        action_lead_ns=50_000_000,
        fps=20,
        checkpoint_step=10,
        checkpoint_mode="full",
        dataset_seed=20261102,
        train_episodes=45,
        validation_episodes=5,
        train_global_indices=tuple(
            index for index in range(50) if index not in {9, 19, 29, 39, 49}
        ),
        validation_global_indices=(9, 19, 29, 39, 49),
        synthetic=False,
        profile=profile,
        observation={},
        action={},
    )


def test_act_generation_uses_bundle_digests_and_independent_overlay(tmp_path):
    settings = load_act_settings(ROOT / "configs/learning/act_policy.yaml")
    settings = __import__("dataclasses").replace(
        settings,
        bundle=Path("/tmp/bundle"),
        profile=ROOT / "profiles/robots/panda_stack_learning_v1/profile.json",
        observation_schema=ROOT / "profiles/observations/panda_rgb_joint_v1.json",
        action_schema=ROOT / "profiles/actions/panda_joint_position_v1.json",
    )
    generated_root = tmp_path / "generated"
    runtime_path, backend_path, act_path, source = stack_once.generate_inputs(
        ROOT / "configs/runtime/native_task_act.yaml",
        seed=20000,
        run_id="act-unit",
        settings=settings,
        contract=contract(settings),
        act_template=ROOT / "configs/learning/act_policy.yaml",
        generated_root=generated_root,
    )
    runtime = yaml.safe_load(runtime_path.read_text())
    backend = yaml.safe_load(backend_path.read_text())
    act = yaml.safe_load(act_path.read_text())
    assert source == "policy"
    assert runtime["policy"]["endpoint"] == "act-policy:50051"
    assert runtime["policy"]["model_bundle_digest"] == "a" * 64
    assert runtime["policy"]["processor_digest"] == "d" * 64
    assert runtime["policy"]["allow_test_policy"] is False
    assert runtime["policy"]["expected_test_only"] is False
    assert runtime["reset"] == {"enabled": True, "seed": 20000}
    assert backend["run"]["seed"] == 20000
    assert backend["robot"]["task_mode"] == "inference"
    assert act["model"]["bundle"] == "/opt/act-bundle"
    assert act["model"]["profile"].startswith("/workspace/")
    snapshot = json.loads((generated_root / "act-unit/act_contract.json").read_text())
    assert snapshot["contract"]["bundle_digest"] == "a" * 64
    assert snapshot["generated"]["sha256"]


def test_campaign_mode_alignment_rejects_act_reserved_fake_or_missing_bundle():
    base = {
        "campaign_mode": "act",
        "campaign_kind": "rollout",
        "act_mode": False,
        "source": "policy",
        "replay_labels": None,
        "policy_mode": "auto",
    }
    assert stack_once.campaign_alignment_error(**base) is not None
    assert (
        stack_once.campaign_alignment_error(**{**base, "act_mode": True, "policy_mode": "fake"})
        is not None
    )
    assert stack_once.campaign_alignment_error(**{**base, "act_mode": True}) is None
    assert (
        stack_once.campaign_alignment_error(
            **{
                **base,
                "campaign_mode": "teacher",
                "act_mode": True,
            }
        )
        is not None
    )
    assert (
        stack_once.campaign_alignment_error(
            **{
                **base,
                "campaign_mode": "teacher",
                "campaign_kind": "label_replay",
                "replay_labels": Path("/tmp/labels.json"),
                "act_mode": False,
                "policy_mode": "fake",
            }
        )
        is None
    )


def test_public_stack_entry_bootstraps_its_own_paths_without_pythonpath():
    import os
    import subprocess
    import sys

    environment = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools/run/stack_once.py"), "--help"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
    assert "--bundle" in result.stdout


def test_act_runtime_has_its_own_readonly_policy_sdk_mount():
    document = yaml.safe_load((ROOT / "deploy/compose/native_act_policy.yaml").read_text())
    volumes = document["services"]["native-runtime"]["volumes"]
    assert any(item["target"] == "/opt/policy-sdk" and item["read_only"] for item in volumes)
