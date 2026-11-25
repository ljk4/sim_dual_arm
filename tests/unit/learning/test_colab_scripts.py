"""Unit tests for Colab workflow helpers, loss plotting, rclone auth, and path resolution."""

from __future__ import annotations

import base64
import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from docs.colab.scripts import probe_colab_training  # noqa: E402
from docs.colab.scripts.colab_workflow import compute_scaled_steps  # noqa: E402
from docs.colab.scripts.loss_plot import load_metrics, render_loss_plot  # noqa: E402
from docs.colab.scripts.rclone_auth import decode_rclone_conf  # noqa: E402


@pytest.mark.parametrize(
    ("base_steps", "batch_size", "expected"),
    [
        (20000, 32, 20000),
        (20000, 64, 10000),
        (20000, 48, 14000),
        (20000, 128, 5000),
        (20000, 256, 3000),
        (500, 256, 1000),
    ],
)
def test_compute_scaled_steps_ceiling_to_1000(base_steps: int, batch_size: int, expected: int) -> None:
    assert compute_scaled_steps(base_steps, batch_size) == expected


def test_compute_scaled_steps_disabled() -> None:
    assert compute_scaled_steps(20000, 64, auto_scale=False) == 20000


def test_render_loss_plot_creates_valid_png(tmp_path: Path) -> None:
    metrics_path = tmp_path / "metrics.jsonl"
    out_png = tmp_path / "loss_curve.png"

    assert render_loss_plot(metrics_path, out_png) is None

    rows = [
        {"step": 10, "phase": "train", "loss": 1.20, "l1_loss": 0.90, "kl_loss": 0.030},
        {"step": 500, "phase": "train", "loss": 0.65, "l1_loss": 0.50, "kl_loss": 0.015},
        {"step": 1000, "phase": "train", "loss": 0.42, "l1_loss": 0.34, "kl_loss": 0.008},
        {"step": 1000, "phase": "validation", "val_l1": 0.38},
        {"step": 1500, "phase": "train", "loss": 0.31, "l1_loss": 0.25, "kl_loss": 0.006},
        {"step": 2000, "phase": "train", "loss": 0.24, "l1_loss": 0.19, "kl_loss": 0.005},
        {"step": 2000, "phase": "validation", "val_l1": 0.22},
    ]
    metrics_path.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")

    train_rows, val_rows = load_metrics(metrics_path)
    assert len(train_rows) == 5
    assert len(val_rows) == 2

    rendered = render_loss_plot(metrics_path, out_png, title="marvin/mujoco/handover (full)")
    assert rendered == out_png
    assert out_png.is_file()
    assert not out_png.with_suffix(".tmp.png").exists()
    assert out_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_decode_rclone_conf_accepts_ini_and_base64() -> None:
    sample_ini = "[gdrive]\ntype = drive\nscope = drive\n"
    assert decode_rclone_conf(sample_ini, "gdrive").startswith("[gdrive]\n")

    encoded = base64.b64encode(sample_ini.encode("utf-8")).decode("ascii")
    with_spaces = f"  {encoded[:12]}\n{encoded[12:]}  "
    assert decode_rclone_conf(with_spaces, "gdrive") == sample_ini

    with pytest.raises(ValueError, match="\\[other\\]"):
        decode_rclone_conf(encoded, "other")

    with pytest.raises(ValueError, match="有效的 base64"):
        decode_rclone_conf("not-valid-b64!!!", "gdrive")


def test_probe_default_project_root_points_to_repo_root(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("PROJECT_ROOT", raising=False)
    assert probe_colab_training._default_project_root() == REPO_ROOT
