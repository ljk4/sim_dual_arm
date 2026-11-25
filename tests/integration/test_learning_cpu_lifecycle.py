"""Public ACT training lifecycle on explicitly synthetic CPU data."""

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
SCENARIO = ROOT / "shared/scenarios/marvin_sim/stack"
sys.path.insert(0, str(ROOT / "tests/unit/fixtures"))
from learning_fixtures import make_select  # noqa: E402


def test_public_act_training_writes_a_self_contained_model(tmp_path):
    torch = pytest.importorskip("torch")
    from dual_arm_learning.training.bundle import load_model, verify_model_dir
    from dual_arm_learning.config import load_dataset_config
    from dual_arm_learning.dataset.dataset import convert
    from model_helpers import replace
    from torchvision.models import resnet18

    dataset_document = yaml.safe_load(
        (SCENARIO / "configs/learning/act_dataset.yaml").read_text()
    )
    dataset_document["validation_percent"] = 50
    dataset_path = tmp_path / "dataset.yaml"
    dataset_path.write_text(yaml.safe_dump(dataset_document))
    dataset_config = load_dataset_config(dataset_path, ROOT)
    dataset_config = replace(
        dataset_config, output_root=tmp_path / "data", cache_root=tmp_path / "cache"
    )
    select = make_select(tmp_path / "sources", dataset_config, episodes=2)
    convert(select_path=select, config=dataset_config, synthetic=True)
    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["device"] = "cpu"
    document["data"]["config"] = str(dataset_path)
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
    weights = tmp_path / "synthetic-resnet18.pth"
    torch.save(resnet18(weights=None).state_dict(), weights)
    document["model"]["weights"]["path"] = str(weights)
    document["training"].update(batch_size=2, log_every_steps=1, val_every_steps=1, val_batches=1)
    document["modes"]["smoke"].update(steps=3, validation=True)
    document["checkpoint"]["root"] = str(tmp_path / "models")
    document["logging"]["root"] = str(tmp_path / "logs")
    config = tmp_path / "training.yaml"
    config.write_text(yaml.safe_dump(document))

    log = tmp_path / "train-console.log"
    result = subprocess.run(
        [
            "bash",
            str(ROOT / "learning/lerobot/scripts/train.sh"),
            "--config",
            str(config),
            "--max-steps",
            "2",
            "--allow-synthetic",
            "--log-file",
            str(log),
        ],
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    (model_dir,) = (tmp_path / "models").iterdir()
    assert model_dir.name.startswith("act_")
    verify_model_dir(model_dir)
    assert sorted(path.name for path in model_dir.iterdir()) == [
        "model.safetensors",
        "training_args.yaml",
    ]
    assert "Traceback" not in log.read_text()
    assert f"model_dir: {model_dir}" in result.stdout

    args = yaml.safe_load((model_dir / "training_args.yaml").read_text())
    assert args["mode"] == "smoke"
    assert args["steps_completed"] == 2
    handle = load_model(model_dir, device="cpu")
    assert handle.observation_schema["state_dim"] == 16
    assert handle.action_schema["chunk_size"] == 60

    log_root = tmp_path / "logs" / model_dir.name
    metrics = [json.loads(line) for line in (log_root / "metrics.jsonl").read_text().splitlines()]
    assert all(row["kl_loss"] > 0 for row in metrics if row["phase"] == "train")
    assert all(row["grad_norm"] > 0 for row in metrics if row["phase"] == "train")
    report = json.loads((log_root / "training_report.json").read_text())
    assert report["dataset"]["training_global_indices"] == [0]
    assert report["dataset"]["validation_global_indices"] == [1]
