"""Batch recording uses real campaign reservations and quality classification."""

import json
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml
from dual_arm_runtime.offline.collection import record


@pytest.fixture
def recording(tmp_path, monkeypatch):
    config = {
        "budget": {
            "formal_seed_first": 10000, "formal_seed_last": 10099,
            "formal_max_attempts": 10, "formal_target_qualified": 5,
            "max_raw_bytes": 100_000, "reserve_bytes": 1_000,
            "estimated_attempt_bytes": 1_000, "min_free_bytes": 0,
        },
        "paths": {
            "attempts_root": "out", "ledger": "out/attempts.jsonl",
            "lock": "out/.lock", "run_lock": "out/.run.lock",
            "formal_select": "out/formal-select.json",
            "required_paths": [], "sdk_marker": "",
        },
        "launch": {"command": [], "timeout_s": 10.0},
        "quality": {"sidecar": "export/quality.json"},
    }
    (tmp_path / "campaign.yaml").write_text(yaml.safe_dump(config))
    (tmp_path / "runtime.yaml").write_text("{}\n")
    (tmp_path / "backend-log").mkdir()
    sdk_check = Mock(return_value=0)
    monkeypatch.setattr(record, "check_ros_sdk", sdk_check)
    collector = tmp_path / "fake_collector.py"
    collector.write_text(
        '''import json
import os
import sys
from pathlib import Path

states = json.loads(sys.argv[1])
state = states[int(os.environ["CAMPAIGN_SEED"]) - 10000]
directory = Path(os.environ["CAMPAIGN_ATTEMPT_DIR"])
raw = directory / "raw"
raw.mkdir()
export = directory / "export"
export.mkdir()
runtime = "failed" if state == "runtime_failure" else "stopped"
task = "failed" if state == "task_failure" else "succeeded"
(raw / "manifest.json").write_text(json.dumps({
    "recording": {"complete": state != "recording_failure"},
    "runtime": {"outcome": runtime, "reason": "TEST"},
}))
(raw / "task_result.json").write_text(json.dumps({"result": {"outcome": task}}))
quality_ok = state != "quality_failure"
(export / "quality.json").write_text(json.dumps({
    "qualified": quality_ok,
    "visibility_qualified": quality_ok,
    "trajectory_qualified": True,
    "failure_reasons": [] if quality_ok else ["global target pixels missing"],
}))
(export / "labels.json").write_text(json.dumps({
    "summary": {"rejected": {"no_started_block": 2}},
}))
raise SystemExit(130 if state == "interrupted" else 1 if state == "runtime_failure" else 0)
'''
    )

    def run(states, *extra):
        monkeypatch.setattr(
            record, "_collector_command",
            lambda args: [sys.executable, str(collector), json.dumps(states)],
        )
        return record.main(
            [
                "--project-root",
                str(tmp_path),
                "--config",
                "campaign.yaml",
                "--runtime-config",
                "runtime.yaml",
                "--backend-log",
                "backend-log",
                *extra,
            ]
        )

    return tmp_path, config, sdk_check, run


def _summary(capsys):
    last_line = capsys.readouterr().out.splitlines()[-1]
    if last_line.startswith("运行摘要: "):
        return json.loads(Path(last_line.split("运行摘要: ", 1)[1]).read_text())
    return json.loads(last_line)


def _seeds(root):
    records = [json.loads(line) for line in (root / "out/attempts.jsonl").read_text().splitlines()]
    return [item["seed"] for item in records if item["event"] == "RESERVED"]


def test_records_requested_qualified_count_and_resets_failure_streak(recording, capsys):
    root, _, _, run = recording
    states = ["quality_failure", "task_failure", "qualified",
              "recording_failure", "runtime_failure", "qualified", "qualified"]

    assert run(states, "--count", "2") == 0

    summary = _summary(capsys)
    assert summary["qualified"] == 2
    assert summary["attempted"] == 6
    assert summary["consecutive_failures"] == 0
    assert summary["reason"] == "target_reached"
    assert _seeds(root) == list(range(10000, 10006))


def test_default_stops_after_three_consecutive_failures(recording, capsys):
    root, _, _, run = recording
    assert run(["runtime_failure", "task_failure", "quality_failure", "qualified"]) == 1

    summary = _summary(capsys)
    assert summary["reason"] == "consecutive_failures"
    assert summary["attempted"] == summary["consecutive_failures"] == 3
    assert summary["qualified"] == 0
    assert _seeds(root) == [10000, 10001, 10002]


def test_failed_export_keeps_diagnostics_in_ledger_after_cleanup(recording):
    root, _, _, run = recording
    assert run(["quality_failure"], "--max-consecutive-failures", "1") == 1

    terminal = json.loads((root / "out/attempts.jsonl").read_text().splitlines()[-1])
    assert not Path(terminal["attempt_dir"]).exists()
    assert terminal["diagnostics"]["quality_failure_reasons"] == [
        "global target pixels missing"
    ]
    assert terminal["diagnostics"]["visibility_qualified"] is False
    assert terminal["diagnostics"]["trajectory_qualified"] is True
    assert terminal["diagnostics"]["label_rejections"] == {"no_started_block": 2}


def test_configured_consecutive_failure_limit(recording, capsys):
    root, _, _, run = recording
    assert run(["task_failure", "quality_failure", "qualified"],
               "--count", "2", "--max-consecutive-failures", "2") == 1

    assert _summary(capsys)["attempted"] == 2
    assert _seeds(root) == [10000, 10001]


def test_default_count_is_one_new_qualified_episode_per_invocation(recording, capsys):
    root, _, _, run = recording
    for _ in range(2):
        assert run(["qualified", "qualified", "qualified"]) == 0
        summary = _summary(capsys)
        assert summary["qualified"] == summary["attempted"] == summary["requested"] == 1
    assert _seeds(root) == [10000, 10001]


def test_campaign_budget_stops_batch_without_reserving_extra_seed(recording, capsys):
    root, config, _, run = recording
    config["budget"]["formal_max_attempts"] = 2
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))

    assert run(["quality_failure"] * 3) == 2

    summary = _summary(capsys)
    assert summary["reason"] == "campaign_stopped"
    assert summary["attempted"] == 2
    assert _seeds(root) == [10000, 10001]


def test_campaign_global_qualified_target_still_limits_batch(recording, capsys):
    root, config, _, run = recording
    config["budget"]["formal_target_qualified"] = 1
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))

    assert run(["qualified", "qualified"], "--count", "2") == 2

    summary = _summary(capsys)
    assert summary["reason"] == "campaign_stopped"
    assert summary["qualified"] == summary["attempted"] == 1
    assert _seeds(root) == [10000]


def test_interrupt_does_not_start_next_attempt(recording, capsys):
    root, _, _, run = recording
    assert run(["interrupted", "qualified"], "--count", "2") == 130
    assert _summary(capsys)["reason"] == "interrupted"
    assert _seeds(root) == [10000]


def test_formal_target_reached_publishes_formal_select_atomically(recording, capsys):
    root, config, _, run = recording
    config["budget"]["formal_target_qualified"] = 2
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))

    assert run(["qualified", "qualified"], "--count", "2") == 0

    destination = root / "out" / "formal-select.json"
    assert _summary(capsys)["formal_select"] == str(destination)
    document = json.loads(destination.read_text())
    assert document["status"] == "SELECTED"
    assert document["target"] == document["qualified"] == 2
    assert [item["seed"] for item in document["selected"]] == [10000, 10001]
    assert not list(destination.parent.glob("*.tmp"))

    assert record.main(["status", "--project-root", str(root), "--config", "campaign.yaml"]) == 0
    assert "SELECTED（2 条）" in capsys.readouterr().out


@pytest.mark.parametrize(
    "arguments", [["--count", "0"], ["--count", "-1"], ["--count", "abc"],
                  ["--max-consecutive-failures", "0"], ["--set", "reset.seed=9"]],
)
def test_invalid_options_do_not_reserve_or_check_sdk(recording, arguments):
    root, _, sdk_check, run = recording
    with pytest.raises(SystemExit) as error:
        run([], *arguments)
    assert error.value.code == 2
    assert not (root / "out/attempts.jsonl").exists()
    sdk_check.assert_not_called()


def test_sdk_failure_does_not_reserve_attempt(recording):
    root, _, sdk_check, run = recording
    sdk_check.return_value = 2
    assert run(["qualified"]) == 2
    assert not (root / "out/attempts.jsonl").exists()


def test_missing_backend_log_does_not_reserve_attempt(recording):
    root, _, sdk_check, run = recording
    (root / "backend-log").rmdir()
    assert run(["qualified"]) == 2
    assert not (root / "out/attempts.jsonl").exists()
    sdk_check.assert_not_called()
