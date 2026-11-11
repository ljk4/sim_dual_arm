"""Public ACT train/resume/bundle lifecycle on explicitly synthetic CPU data."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

pytestmark = pytest.mark.skipif(
    os.environ.get("DSH035_HEAVY") != "1", reason="synthetic ACT integration"
)
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/unit"))
from learning_fixtures import make_select  # noqa: E402


def test_public_act_training_resume_and_bundle_use_real_model(tmp_path):
    torch = pytest.importorskip("torch")
    from dual_arm_learning.config import load_dataset_config
    from dual_arm_learning.dataset import convert
    from torchvision.models import resnet18

    dataset_config = load_dataset_config(ROOT / "learning/lerobot/configs/act_dataset.yaml", ROOT)
    from model_helpers import replace

    dataset_config = replace(dataset_config, output_root=tmp_path / "data")
    select = make_select(tmp_path / "sources", dataset_config)
    convert(select_path=select, config=dataset_config, synthetic=True)
    document = yaml.safe_load((ROOT / "learning/lerobot/configs/act_training.yaml").read_text())
    document["device"] = "cpu"
    document["data"]["root"] = str(tmp_path / "data")
    document["cache_root"] = str(tmp_path / "cache")
    document["model"].update(
        dim_model=32,
        n_heads=4,
        dim_feedforward=64,
        n_encoder_layers=1,
        n_decoder_layers=1,
        n_vae_encoder_layers=1,
    )
    # Synthetic lifecycle checks must not depend on downloaded production weights.
    weights = tmp_path / "synthetic-resnet18.pth"
    torch.save(resnet18(weights=None).state_dict(), weights)
    document["model"]["weights"]["path"] = str(weights)
    document["training"].update(
        batch_size=2, log_every_steps=1, val_every_steps=1, save_every_steps=1, val_batches=1
    )
    document["modes"]["smoke"].update(steps=3, validation=True, reserve_seconds=120.0)
    document["budget"].update(
        ledger=str(tmp_path / "budget.jsonl"), lock=str(tmp_path / "budget.lock")
    )
    document["checkpoint"]["root"] = str(tmp_path / "runs")
    document["logging"]["root"] = str(tmp_path / "runs")
    config = tmp_path / "training.yaml"
    config.write_text(yaml.safe_dump(document))

    def cli(*arguments, operator=False):
        result = subprocess.run(
            ["bash", str(ROOT / "learning/lerobot/scripts/run.sh"),
             *([] if operator else ["cli"]), *arguments],
            capture_output=True,
            text=True,
            timeout=180,
        )
        assert result.returncode == 0, result.stdout + "\n" + result.stderr
        return result if operator else json.loads(result.stdout)

    log = tmp_path / "train-console.log"
    first = cli("train", "--config", str(config), "--max-steps", "2", "--allow-synthetic",
                "--log-file", str(log), operator=True)
    run, = (tmp_path / "runs").iterdir()
    assert f"run_dir: {run}" in first.stdout
    assert "2/2 step 100.0%" in first.stderr
    assert "loss=" in first.stderr
    assert "Traceback" not in log.read_text()
    assert len(first.stderr.splitlines()) < 20
    resumed = cli("train", "--config", str(config), "--resume-run", str(run), "--allow-synthetic")
    assert resumed["steps"] == 3
    budget = cli("budget-status", "--config", str(config))
    assert len(budget["runs"][run.name]["segments"]) == 2
    terminals = [
        json.loads(line)
        for line in (tmp_path / "budget.jsonl").read_text().splitlines()
        if json.loads(line)["event"] == "FINISHED"
    ]
    assert len(terminals) == 2
    assert budget["used_seconds"] == pytest.approx(sum(t["elapsed_seconds"] for t in terminals))
    metrics = [json.loads(line) for line in (run / "metrics.jsonl").read_text().splitlines()]
    assert all(row["kl_loss"] > 0 for row in metrics if row["phase"] == "train")
    assert all(row["grad_norm"] > 0 for row in metrics if row["phase"] == "train")
    assert len(list(run.glob("events-*"))) == 2
    bundle = cli(
        "bundle",
        "--config",
        str(config),
        "--run",
        str(run),
        "--output",
        str(tmp_path / "bundle"),
        "--checkpoint",
        "best.pt",
        "--allow-synthetic",
    )
    assert bundle["synthetic"] is True
    cli("verify-bundle", "--bundle", str(tmp_path / "bundle"))
    assert not list((tmp_path / "bundle").rglob("*.pt"))
