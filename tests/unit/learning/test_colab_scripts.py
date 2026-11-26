"""Unit tests for Colab workflow helpers, loss plotting, rclone auth, and path resolution."""

from __future__ import annotations

import ast
import base64
from datetime import datetime, timezone
import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from docs.colab.scripts import colab_workflow, drive_sync, probe_colab_training  # noqa: E402
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


def test_upload_and_download_raw_dataset_packs_only_selected_exports(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = tmp_path / "repo"
    task_dir = repo / "output" / "marvin" / "mujoco" / "stack"
    raw_dir = task_dir / "datasets" / "raw"
    remote_dir = tmp_path / "drive" / "marvin" / "mujoco" / "stack"

    for idx in range(2):
        attempt = raw_dir / f"formal-0000{idx}"
        (attempt / "export" / "rgb" / "0").mkdir(parents=True)
        (attempt / "export" / "labels.json").write_text('{"labels": []}', encoding="utf-8")
        (attempt / "export" / "rgb" / "0" / "global.png").write_bytes(b"png")
        (attempt / "raw").mkdir(parents=True)
        (attempt / "raw" / "episode.mcap").write_bytes(b"mcap-data")

    pilot = raw_dir / "pilot-99999"
    (pilot / "export").mkdir(parents=True)
    (pilot / "export" / "labels.json").write_text("{}", encoding="utf-8")

    select_doc = {
        "status": "SELECTED",
        "qualified": 2,
        "selected": [
            {
                "attempt_id": "formal:0",
                "seed": 0,
                "attempt_dir": "/data/marvin/mujoco/stack/datasets/raw/formal-00000",
            },
            {
                "attempt_id": "formal:1",
                "seed": 1,
                "attempt_dir": "/data/marvin/mujoco/stack/datasets/raw/formal-00001",
            },
        ],
    }
    (raw_dir / "formal-select.json").write_text(json.dumps(select_doc), encoding="utf-8")

    ctx = drive_sync.SyncContext(
        repo_root=repo,
        robot="marvin",
        backend="mujoco",
        task="stack",
        rclone_remote="gdrive:sim_dual_arm",
        dataset_archive="lerobot.tar",
        local_task_dir=task_dir,
        remote_task_dir="gdrive:sim_dual_arm/marvin/mujoco/stack",
        raw_archive="raw_exports.tar",
    )

    def _map_remote(spec: str) -> Path:
        prefix = "gdrive:sim_dual_arm/marvin/mujoco/stack/"
        assert spec.startswith(prefix)
        return remote_dir / spec.removeprefix(prefix)

    def _fake_rclone(args: list[str], *, capture: bool = False) -> subprocess.CompletedProcess[str]:
        op = args[0]
        if op == "mkdir":
            _map_remote(args[1]).mkdir(parents=True, exist_ok=True)
            return subprocess.CompletedProcess(args, 0, stdout="", stderr="")
        if op == "copyto":
            src, dst = args[1], args[2]
            if src.startswith("gdrive:"):
                shutil.copy2(_map_remote(src), dst)
            else:
                target = _map_remote(dst)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, target)
            return subprocess.CompletedProcess(args, 0, stdout="", stderr="")
        if op == "lsl":
            target = _map_remote(args[1])
            return subprocess.CompletedProcess(args, 0, stdout=f"{target.stat().st_size} {target.name}\n", stderr="")
        raise AssertionError(f"unexpected rclone op: {args}")

    monkeypatch.setattr(drive_sync, "_run_rclone", _fake_rclone)
    monkeypatch.setattr(
        drive_sync,
        "_remote_files",
        lambda remote: {p.name for p in _map_remote(remote).iterdir() if p.is_file()}
        if _map_remote(remote).is_dir()
        else set(),
    )

    drive_sync.upload_raw_dataset(ctx)
    archive = remote_dir / "datasets" / "raw_exports.tar"
    assert archive.is_file()

    unpacked = tmp_path / "colab_raw"
    drive_sync.download_raw_dataset(ctx, target_dir=unpacked)
    assert (unpacked / "formal-select.json").is_file()
    assert (unpacked / "formal-00000" / "export" / "labels.json").is_file()
    assert (unpacked / "formal-00001" / "export" / "rgb" / "0" / "global.png").is_file()
    assert not (unpacked / "formal-00000" / "raw").exists()
    assert not (unpacked / "pilot-99999").exists()


def test_prepare_dataset_auto_routing_by_modtimes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[str] = []

    monkeypatch.setattr(
        colab_workflow,
        "convert_dataset",
        lambda **kwargs: calls.append("convert") or (tmp_path / "converted"),
    )
    monkeypatch.setattr(
        colab_workflow,
        "fetch_dataset",
        lambda **kwargs: calls.append("fetch") or (tmp_path / "fetched"),
    )

    t_old = datetime(2026, 11, 26, 10, 0, tzinfo=timezone.utc)
    t_new = datetime(2026, 11, 26, 12, 0, tzinfo=timezone.utc)

    monkeypatch.setattr(
        colab_workflow,
        "remote_file_modtimes",
        lambda _remote: {"raw_exports.tar": t_new, "lerobot.tar": t_old},
    )
    colab_workflow.prepare_dataset(
        work_dir=tmp_path,
        rclone_remote="gdrive:sim_dual_arm",
        robot="marvin",
        backend="mujoco",
        task="stack",
    )

    monkeypatch.setattr(
        colab_workflow,
        "remote_file_modtimes",
        lambda _remote: {"raw_exports.tar": t_old, "lerobot.tar": t_new},
    )
    colab_workflow.prepare_dataset(
        work_dir=tmp_path,
        rclone_remote="gdrive:sim_dual_arm",
        robot="marvin",
        backend="mujoco",
        task="stack",
    )

    monkeypatch.setattr(
        colab_workflow,
        "remote_file_modtimes",
        lambda _remote: {"raw_exports.tar": t_old},
    )
    colab_workflow.prepare_dataset(
        work_dir=tmp_path,
        rclone_remote="gdrive:sim_dual_arm",
        robot="marvin",
        backend="mujoco",
        task="stack",
    )

    assert calls == ["convert", "fetch", "convert"]


def test_colab_notebooks_and_setup_script_syntax() -> None:
    for rel in ("docs/colab/train_act_colab.ipynb", "docs/colab/sync_drive_rclone.ipynb"):
        nb = json.loads((REPO_ROOT / rel).read_text(encoding="utf-8"))
        for cell in nb["cells"]:
            if cell.get("cell_type") == "code":
                ast.parse("".join(cell.get("source", [])), filename=rel)

    res = subprocess.run(
        ["bash", "-n", str(REPO_ROOT / "docs/colab/scripts/setup_colab_learning.sh")],
        check=False,
        capture_output=True,
        text=True,
    )
    assert res.returncode == 0, res.stderr
