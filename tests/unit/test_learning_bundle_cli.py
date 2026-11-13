"""The bundle command must honor the same effective inputs as training."""

import json
from pathlib import Path

from dual_arm_learning import cli

ROOT = Path(__file__).resolve().parents[2]


def test_bundle_honors_mode_and_data_overrides_and_preserves_absolute_run(monkeypatch, capsys):
    run = Path("/data/panda_stack/models/runs/full-20261110T102358Z-10629")

    def export(**kwargs):
        config = kwargs["config"]
        assert config.mode == "full"
        assert config.mode_spec.steps == 5000
        assert config.data_root == ROOT / "test-data"
        assert kwargs["run_dir"] == run
        assert kwargs["output_dir"] == ROOT / "test-bundle"
        assert kwargs["checkpoint_name"] == "best.pt"
        return {"bundle_digest": "test", "files": {}, "checkpoint": {"step": 5000},
                "synthetic": False}

    monkeypatch.setattr(cli, "export_bundle", export)
    assert cli.main([
        "--project-root", str(ROOT), "bundle", "--config",
        "shared/scenarios/panda_stack/configs/learning/act_training.yaml",
        "--mode", "full", "--run", str(run),
        "--output", "test-bundle", "--data-root", "test-data",
    ]) == 0
    assert json.loads(capsys.readouterr().out)["checkpoint_step"] == 5000
