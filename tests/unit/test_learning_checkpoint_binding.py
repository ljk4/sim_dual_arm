"""Resume must bind resolved CLI inputs and accepted data, not only YAML text."""

import json
from dataclasses import replace
from pathlib import Path

import pytest
from dual_arm_learning.checkpoints import (
    CheckpointError,
    training_binding,
    validate_checkpoint_binding,
)
from dual_arm_learning.config import load_training_config

ROOT = Path(__file__).resolve().parents[2]


def test_effective_overrides_and_dataset_changes_cannot_reuse_checkpoint(tmp_path):
    config = load_training_config(ROOT / "learning/lerobot/configs/act_training.yaml", ROOT)
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
