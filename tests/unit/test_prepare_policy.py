from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning/lerobot/scripts"))
import prepare_policy  # noqa: E402
from dual_arm_learning.policy_config import PolicyConfigError  # noqa: E402


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_prepare_policy_uses_real_verified_bundle_and_new_output(tmp_path):
    pytest.importorskip("torch")
    from act_fixtures import make_act_bundle

    bundle = make_act_bundle(tmp_path / "bundle", synthetic=True)
    output = tmp_path / "prepared"
    manifest = prepare_policy.prepare_policy(
        template=ROOT / "learning/lerobot/configs/policy_temporal.yaml",
        bundle=bundle,
        run_id="policy-1",
        output=output,
        require_formal=False,
        project_root=ROOT,
    )
    binding = json.loads((output / "policy_binding.json").read_text())
    contract = json.loads((output / "act_contract.json").read_text())
    assert binding["kind"] == "policy-binding"
    assert binding["endpoint"] == "learning:50051"
    assert binding["model"]["bundle_digest"] == contract["contract"]["bundle_digest"]
    assert binding["model"]["processor_digest"] == contract["contract"]["processor_digest"]
    assert binding["model"]["synthetic"] is True
    assert binding["profile"]["sha256"] == _sha(
        ROOT / "profiles/robots/panda_stack_learning_v1/profile.json"
    )
    assert binding["observation_schema"]["sha256"] == _sha(
        ROOT / "profiles/observations/panda_rgb_joint_v1.json"
    )
    assert manifest["outputs"]["policy_binding_sha256"] == _sha(
        output / "policy_binding.json"
    )
    policy = (output / "policy.yaml").read_text()
    assert "FROM_VERIFIED" not in policy
    assert "/workspace/profiles/robots/panda_stack_learning_v1/profile.json" in policy
    with pytest.raises(prepare_policy.PrepareError, match="refusing to overwrite"):
        prepare_policy.prepare_policy(
            template=ROOT / "learning/lerobot/configs/policy_temporal.yaml",
            bundle=bundle,
            run_id="policy-2",
            output=output,
            require_formal=False,
        )


def test_prepare_policy_rejects_missing_or_synthetic_formal_bundle(tmp_path):
    missing = tmp_path / "missing"
    with pytest.raises(PolicyConfigError, match="bundle directory is missing"):
        prepare_policy.prepare_policy(
            template=ROOT / "learning/lerobot/configs/policy_temporal.yaml",
            bundle=missing,
            run_id="bad",
            output=tmp_path / "out",
            require_formal=False,
        )
    assert not (tmp_path / "out").exists()

    pytest.importorskip("torch")
    from act_fixtures import make_act_bundle

    bundle = make_act_bundle(tmp_path / "bundle", synthetic=True)
    with pytest.raises(PolicyConfigError, match="synthetic"):
        prepare_policy.prepare_policy(
            template=ROOT / "learning/lerobot/configs/policy_temporal.yaml",
            bundle=bundle,
            run_id="formal",
            output=tmp_path / "formal-out",
            require_formal=True,
        )
    assert not (tmp_path / "formal-out").exists()
