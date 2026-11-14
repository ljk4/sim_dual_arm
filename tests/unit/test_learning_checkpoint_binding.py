"""Checkpoint consumers must bind effective inputs and accepted data."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_learning import bundle
from dual_arm_learning.checkpoints import (
    CHECKPOINT_FORMAT,
    CheckpointError,
    training_binding,
    validate_checkpoint_binding,
)
from dual_arm_learning.config import load_training_config
from model_helpers import replace

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_stack"


def test_effective_overrides_and_dataset_changes_cannot_reuse_checkpoint(tmp_path):
    config = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    (tmp_path / "manifest.json").write_text(json.dumps({"dataset": "accepted-A"}))
    config = replace(config, data_root=tmp_path)
    checkpoint = {"training_binding": training_binding(config)}
    validate_checkpoint_binding(checkpoint, config)
    changed_mode = replace(config, mode="full")
    assert config.sha256 == changed_mode.sha256
    with pytest.raises(CheckpointError, match="effective configuration"):
        validate_checkpoint_binding(checkpoint, changed_mode)
    (tmp_path / "manifest.json").write_text(json.dumps({"dataset": "accepted-B"}))
    with pytest.raises(CheckpointError, match="accepted dataset"):
        validate_checkpoint_binding(checkpoint, config)


@pytest.mark.parametrize("change", ["mode", "dataset"])
def test_bundle_export_checks_binding_without_a_sample_filter(tmp_path, monkeypatch, change):
    torch = pytest.importorskip("torch")
    config = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    data_root = tmp_path / "dataset"
    data_root.mkdir()
    manifest = data_root / "manifest.json"
    manifest.write_text('{"dataset": "accepted-A"}')
    config = replace(config, mode="full", data_root=data_root)
    assert config.sample_filter is None
    checkpoint = {
        "format": CHECKPOINT_FORMAT,
        "config_sha256": config.sha256,
        "model": {"weight": torch.ones(1)},
        "training_binding": training_binding(config),
    }
    run = tmp_path / "run"
    run.mkdir()
    torch.save(checkpoint, run / "latest.pt")
    monkeypatch.setattr(
        bundle,
        "verify_dataset_manifest",
        lambda *_args, **_kwargs: SimpleNamespace(synthetic=False),
    )
    bundle._export_inputs(config, run, "latest.pt", False)
    if change == "mode":
        changed = replace(config, mode="overfit")
    else:
        manifest.write_text('{"dataset": "accepted-B"}')
        changed = config
    assert changed.sha256 == config.sha256
    with pytest.raises(CheckpointError, match="effective configuration or accepted dataset"):
        bundle._export_inputs(changed, run, "latest.pt", False)


def test_comment_only_yaml_edit_preserves_semantic_checkpoint_binding(tmp_path, monkeypatch):
    torch = pytest.importorskip("torch")
    source_yaml = SCENARIO / "configs/learning/act_training.yaml"
    data_root = tmp_path / "dataset"
    data_root.mkdir()
    (data_root / "manifest.json").write_text('{"dataset": "accepted-A"}')
    original = replace(load_training_config(source_yaml, ROOT), mode="full", data_root=data_root)
    edited_yaml = tmp_path / "act_training.yaml"
    edited_yaml.write_text("# comment update\n" + source_yaml.read_text(encoding="utf-8"))
    edited = replace(
        load_training_config(edited_yaml, ROOT),
        mode="full",
        data_root=data_root,
        source=source_yaml,
    )
    assert edited_yaml.read_bytes() != source_yaml.read_bytes()
    checkpoint = {
        "format": CHECKPOINT_FORMAT,
        "config_sha256": "0" * 64,
        "model": {"weight": torch.ones(1)},
        "training_binding": training_binding(original),
    }
    run = tmp_path / "run"
    run.mkdir()
    torch.save(checkpoint, run / "latest.pt")
    monkeypatch.setattr(
        bundle,
        "verify_dataset_manifest",
        lambda *_args, **_kwargs: SimpleNamespace(synthetic=False),
    )
    bundle._export_inputs(edited, run, "latest.pt", False)
