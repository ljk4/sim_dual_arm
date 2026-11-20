"""One collection attempt writes a flat, duplicate-free artefact tree."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import yaml
from dual_arm_runtime.offline import collect_attempt


def _campaign(tmp_path: Path) -> Path:
    path = tmp_path / "campaign.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "export": {"max_labels": 700},
                "quality": {"sidecar": "export/quality.json"},
                "paths": {"ledger": str(tmp_path / "logs/collection/attempts.jsonl")},
            }
        ),
        encoding="utf-8",
    )
    return path


def _runtime_recipe(tmp_path: Path) -> Path:
    path = tmp_path / "runtime.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "reset": {"enabled": True, "seed": 0},
                "action_source": "teacher",
            }
        ),
        encoding="utf-8",
    )
    return path


def test_collect_attempt_flattens_raw_logs_and_export_without_copies(
    tmp_path, monkeypatch
):
    attempt = tmp_path / "formal-10000-run"
    attempt.mkdir()
    campaign = _campaign(tmp_path)
    recipe = _runtime_recipe(tmp_path)
    commands: list[list[str]] = []

    def fake_run(command, check):
        commands.append(list(command))
        del check
        module = command[command.index("-m") + 1]
        if module == "dual_arm_runtime.runtime_node":
            raw = Path(command[command.index("--episode-dir") + 1])
            raw.mkdir(parents=True)
            (raw / "manifest.json").write_text(
                json.dumps({"recording": {"complete": True}}), encoding="utf-8"
            )
            (raw / "task_result.json").write_text(
                json.dumps({"result": {"outcome": "succeeded"}}), encoding="utf-8"
            )
            (raw / "recording").mkdir()
        else:
            out = Path(command[command.index("--out") + 1])
            out.mkdir(parents=True)
            (out / "labels.json").write_text("{}", encoding="utf-8")
            (out / "quality.json").write_text(
                json.dumps({"qualified": True}), encoding="utf-8"
            )
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(collect_attempt, "check_ros_sdk", lambda *a, **k: 0)
    monkeypatch.setattr(collect_attempt.subprocess, "run", fake_run)
    monkeypatch.setenv("CAMPAIGN_ATTEMPT_DIR", str(attempt))
    monkeypatch.setenv("CAMPAIGN_SEED", "10000")
    monkeypatch.setenv("CAMPAIGN_RUN_ID", "run-1")
    monkeypatch.setenv("CAMPAIGN_EPISODE_ID", "formal-10000-run")

    assert collect_attempt.main(
        [
            "--campaign-config", str(campaign),
            "--runtime-config", str(recipe),
            "--backend-log", str(tmp_path),
        ]
    ) == 0

    logs = tmp_path / "logs/collection/attempts" / attempt.name
    assert (logs / "runtime.yaml").is_file()
    assert (attempt / "raw" / "manifest.json").is_file()
    assert (attempt / "raw" / "task_result.json").is_file()
    assert (attempt / "export" / "quality.json").is_file()
    assert not (attempt / "stack").exists()
    assert not (attempt / "logs").exists()
    assert not (attempt / "runtime.yaml").exists()
    assert not (attempt / "manifest.json").exists()
    assert not (attempt / "task_result.json").exists()
    assert not (attempt / "quality.json").exists()
    assert not (attempt / "launch_result.json").exists()

    runtime_command, export_command = commands
    assert Path(runtime_command[runtime_command.index("--episode-dir") + 1]) == (
        attempt / "raw"
    )
    assert Path(runtime_command[runtime_command.index("--runs-output") + 1]) == logs
    assert Path(export_command[export_command.index("--episode-dir") + 1]) == (
        attempt / "raw"
    )
    assert Path(export_command[export_command.index("--out") + 1]) == attempt / "export"


def test_collect_attempt_forwards_worker_namespace(tmp_path, monkeypatch):
    attempt = tmp_path / "formal-10000-run"
    attempt.mkdir()
    campaign = _campaign(tmp_path)
    commands: list[list[str]] = []

    def fake_run(command, check):
        commands.append(list(command))
        del check
        module = command[command.index("-m") + 1]
        if module == "dual_arm_runtime.runtime_node":
            raw = Path(command[command.index("--episode-dir") + 1])
            raw.mkdir(parents=True)
            (raw / "manifest.json").write_text(
                json.dumps({"recording": {"complete": True}}), encoding="utf-8"
            )
            (raw / "task_result.json").write_text(
                json.dumps({"result": {"outcome": "succeeded"}}), encoding="utf-8"
            )
            (raw / "recording").mkdir()
        else:
            out = Path(command[command.index("--out") + 1])
            out.mkdir(parents=True)
            (out / "labels.json").write_text("{}", encoding="utf-8")
            (out / "quality.json").write_text(
                json.dumps({"qualified": True}), encoding="utf-8"
            )
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(collect_attempt, "check_ros_sdk", lambda *a, **k: 0)
    monkeypatch.setattr(collect_attempt.subprocess, "run", fake_run)
    monkeypatch.setenv("CAMPAIGN_ATTEMPT_DIR", str(attempt))
    monkeypatch.setenv("CAMPAIGN_SEED", "10000")
    monkeypatch.setenv("CAMPAIGN_RUN_ID", "run-1")
    monkeypatch.setenv("CAMPAIGN_EPISODE_ID", "formal-10000-run")

    assert collect_attempt.main(
        [
            "--campaign-config", str(campaign),
            "--robot", "synthetic_sim",
            "--task", "stack",
            "--shared-root", str(tmp_path),
            "--robot-namespace", "/record_pool_1",
            "--backend-log", str(tmp_path),
        ]
    ) == 0

    runtime_command = commands[0]
    assert runtime_command[runtime_command.index("--robot") + 1] == "synthetic_sim"
    assert runtime_command[runtime_command.index("--task") + 1] == "stack"
    assert runtime_command[runtime_command.index("--robot-namespace") + 1] == "/record_pool_1"
    assert runtime_command[runtime_command.index("--reset-seed") + 1] == "10000"
    logs = tmp_path / "logs/collection/attempts" / attempt.name
    assert not (attempt / "runtime.yaml").exists()
    assert not (logs / "runtime.yaml").exists()
