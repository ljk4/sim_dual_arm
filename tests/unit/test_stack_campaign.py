import json
import sys

import pytest
import yaml
from dual_arm_runtime.offline import campaign


def _project(tmp_path):
    config = {
        "schema_version": 1,
        "budget": {
            "pilot_seed_first": 1000,
            "pilot_seed_last": 1009,
            "pilot_max_attempts": 2,
            "pilot_teacher_seed_last": 1007,
            "pilot_label_replay_max": 1,
            "formal_seed_first": 10000,
            "formal_seed_last": 10099,
            "formal_max_attempts": 100,
            "formal_target_qualified": 50,
            "formal_validation_qualified_indices": [9, 19, 29, 39, 49],
            "test_seed_first": 20000,
            "test_seed_last": 20019,
            "test_max_attempts": 20,
            "max_raw_bytes": 100_000,
            "reserve_bytes": 1_000,
            "estimated_attempt_bytes": 1_000,
            "min_free_bytes": 0,
        },
        "paths": {
            "attempts_root": "build/campaign",
            "ledger": "build/campaign/attempts.jsonl",
            "lock": "build/campaign/.lock",
            "run_lock": "build/campaign/.run.lock",
            "formal_select": "build/campaign/formal-select.json",
            "required_paths": [],
            "sdk_marker": "",
        },
        "launch": {"command": [], "timeout_s": 30.0},
        "quality": {"sidecar": "export/quality.json", "max_blackout_s": 0.35},
    }
    root = tmp_path / "project"
    experiments = root / "shared/scenarios/marvin_sim/stack/configs/runtime/experiments"
    experiments.mkdir(parents=True)
    (experiments / "stack_campaign.yaml").write_text(
        yaml.safe_dump(config), encoding="utf-8"
    )
    return root


def _campaign(root):
    return (
        root / "shared/scenarios/marvin_sim/stack/configs/runtime/experiments/stack_campaign.yaml"
    )


def _run(root, run_id):
    return campaign.main(
        [
            "--project-root",
            str(root),
            "--config",
            str(_campaign(root)),
            "run",
            "--purpose",
            "pilot",
            "--run-id",
            run_id,
            "--runtime-config",
            "shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml",
            "--",
            sys.executable,
            "-c",
            "print('ok')",
        ]
    )


def test_campaign_reserves_sequential_seeds_and_counts_failures(tmp_path):
    root = _project(tmp_path)
    assert _run(root, "one") == 0
    assert _run(root, "two") == 0
    records = [
        json.loads(line)
        for line in (root / "build/campaign/attempts.jsonl").read_text().splitlines()
    ]
    seeds = [item["seed"] for item in records if item["event"] == "RESERVED"]
    assert seeds == [1000, 1001]
    assert campaign.main(
            ["--project-root", str(root), "--config", str(_campaign(root)), "status"]
        ) == 0
    assert _run(root, "three") == 2


def test_campaign_status_exposes_formal_target(tmp_path, capsys):
    root = _project(tmp_path)
    assert campaign.main(
            ["--project-root", str(root), "--config", str(_campaign(root)), "status"]
        ) == 0
    assert json.loads(capsys.readouterr().out)["formal_target_qualified"] == 50


def test_reserved_without_finish_still_consumes_budget(tmp_path):
    root = _project(tmp_path)
    ledger = root / "build/campaign/attempts.jsonl"
    ledger.parent.mkdir(parents=True)
    ledger.write_text(
        json.dumps(
            {
                "event": "RESERVED",
                "attempt_id": "pilot:1000:crashed",
                "purpose": "pilot",
                "kind": "rollout",
                "seed": 1000,
                "raw_bytes": 0,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    assert _run(root, "next") == 0
    records = [json.loads(line) for line in ledger.read_text().splitlines()]
    seeds = [item["seed"] for item in records if item["event"] == "RESERVED"]
    assert seeds == [1000, 1001]


def test_label_replay_consumes_a_pilot_attempt_and_needs_used_seed(tmp_path):
    root = _project(tmp_path)
    assert _run(root, "one") == 0
    assert (
        campaign.main(
            [
                "--project-root",
                str(root),
                "--config",
                str(_campaign(root)),
                "run",
                "--purpose",
                "pilot",
                "--kind",
                "label_replay",
                "--seed",
                "1000",
                "--run-id",
                "replay",
                "--runtime-config",
                "shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml",
                "--",
                sys.executable,
                "-c",
                "print('ok')",
                "runtime/ros2/scripts/test.sh",
            ]
        )
        == 0
    )
    records = [
        json.loads(line)
        for line in (root / "build/campaign/attempts.jsonl").read_text().splitlines()
    ]
    reserved = [item for item in records if item["event"] == "RESERVED"]
    assert [item["kind"] for item in reserved] == ["rollout", "label_replay"]
    assert _run(root, "third") == 2


def test_formal_is_teacher_only_and_test_is_act_only(tmp_path):
    root = _project(tmp_path)
    base = [
        "--project-root",
        str(root),
        "--config",
        str(_campaign(root)),
        "run",
        "--kind",
        "rollout",
        "--run-id",
        "guard",
        "--runtime-config",
        "shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml",
    ]
    assert (
        campaign.main(
            [*base, "--purpose", "formal", "--mode", "act", "--", sys.executable, "-c", "pass"]
        )
        == 2
    )
    assert (
        campaign.main(
            [*base, "--purpose", "test", "--mode", "teacher", "--", sys.executable, "-c", "pass"]
        )
        == 2
    )


def test_formal_stops_after_fifty_qualified(tmp_path):
    root = _project(tmp_path)
    ledger = root / "build/campaign/attempts.jsonl"
    ledger.parent.mkdir(parents=True)
    lines = []
    for index in range(50):
        seed = 10000 + index
        attempt_id = f"formal:{seed}:old"
        lines.append(
            json.dumps(
                {
                    "event": "RESERVED",
                    "attempt_id": attempt_id,
                    "purpose": "formal",
                    "kind": "rollout",
                    "mode": "teacher",
                    "seed": seed,
                    "raw_bytes": 0,
                }
            )
        )
        lines.append(
            json.dumps(
                {
                    "event": "FINISHED",
                    "attempt_id": attempt_id,
                    "qualified": True,
                    "raw_bytes": 0,
                }
            )
        )
    ledger.write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert (
        campaign.main(
            [
                "--project-root",
                str(root),
                "--config",
                str(_campaign(root)),
                "run",
                "--purpose",
                "formal",
                "--mode",
                "teacher",
                "--run-id",
                "after-target",
                "--runtime-config",
                "shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml",
                "--",
                sys.executable,
                "-c",
                "print('never')",
            ]
        )
        == 2
    )


def test_stale_run_lock_is_recovered_without_resetting_reserved_attempt(tmp_path):
    root = _project(tmp_path)
    lock = root / "build/campaign/.run.lock"
    lock.parent.mkdir(parents=True)
    lock.write_text("99999999", encoding="utf-8")
    assert _run(root, "recovered") == 0
    assert not lock.exists()


def test_teacher_and_replay_cannot_consume_reserved_act_attempts(tmp_path):
    import pytest

    root = _project(tmp_path)
    campaign_path = (
        root
        / "shared/scenarios/marvin_sim/stack/configs/runtime/experiments/stack_campaign.yaml"
    )
    config = yaml.safe_load(campaign_path.read_text())
    config["budget"].update(pilot_max_attempts=10, pilot_act_reserved_attempts=2)
    records = [
        {
            "event": "RESERVED",
            "attempt_id": str(index),
            "purpose": "pilot",
            "kind": "rollout",
            "mode": "teacher",
            "seed": 1000 + index,
        }
        for index in range(8)
    ]
    with pytest.raises(campaign.CampaignError, match="preserve"):
        campaign._allocate(config, records, "pilot", "rollout", "teacher", None, root)
    seed, mode = campaign._allocate(config, records, "pilot", "rollout", "act", None, root)
    assert (seed, mode) == (1008, "act")


def test_select_writes_generated_manifest_only_after_success(tmp_path):
    root = _project(tmp_path)
    config_path = (
        root
        / "shared/scenarios/marvin_sim/stack/configs/runtime/experiments/stack_campaign.yaml"
    )
    config = yaml.safe_load(config_path.read_text())
    config["budget"].update(
        formal_target_qualified=6,
        formal_validation_qualified_indices=[4, 5],
    )
    config["formal_split_description"] = "four train, two validate"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    out = root / "build/campaign/SELECTED.json"

    # Insufficient qualified attempts leave no manifest behind.
    assert campaign.main(
        [
            "--project-root",
            str(root),
            "--config",
            str(_campaign(root)),
            "select",
            "--output",
            str(out),
        ]
    ) == 1
    assert not out.exists()

    ledger = root / "build/campaign/attempts.jsonl"
    ledger.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for index in range(6):
        attempt_id = f"formal:{10000 + index}:run"
        lines.append(
            json.dumps(
                {
                    "event": "RESERVED",
                    "attempt_id": attempt_id,
                    "purpose": "formal",
                    "kind": "rollout",
                    "mode": "teacher",
                    "seed": 10000 + index,
                    "attempt_dir": str(root / f"build/campaign/attempts/{index}"),
                    "raw_bytes": 0,
                }
            )
        )
        lines.append(
            json.dumps(
                {
                    "event": "FINISHED",
                    "attempt_id": attempt_id,
                    "qualified": True,
                    "raw_bytes": 0,
                }
            )
        )
    ledger.write_text("\n".join(lines) + "\n", encoding="utf-8")

    assert campaign.main(
        [
            "--project-root",
            str(root),
            "--config",
            str(_campaign(root)),
            "select",
            "--output",
            str(out),
        ]
    ) == 0
    document = json.loads(out.read_text())
    assert document["status"] == "SELECTED"
    assert document["rule"] == "four train, two validate"
    assert len(document["training"]) == 4
    assert [item["seed"] for item in document["validation"]] == [10004, 10005]

    # A later run atomically refreshes the same selection manifest.
    assert campaign.main(
        [
            "--project-root",
            str(root),
            "--config",
            str(_campaign(root)),
            "select",
            "--output",
            str(out),
        ]
    ) == 0
    assert json.loads(out.read_text())["status"] == "SELECTED"
    assert not list(out.parent.glob("*.tmp"))


def _terminal_project(tmp_path):
    root = _project(tmp_path)
    attempt = root / "build/campaign/pilot-03102-record-42"
    episode = attempt / "raw"
    episode.mkdir(parents=True)
    (episode / "manifest.json").write_text(
        json.dumps(
            {
                "recording": {"complete": True, "failure_reason": ""},
                "runtime": {"outcome": "interrupted", "reason": "SIGTERM"},
                "started_at_ns": 2_000_000_000,
                "ended_at_ns": 42_000_000_000,
            }
        ),
        encoding="utf-8",
    )
    (episode / "task_result.json").write_text(
        json.dumps(
            {
                "result": {
                    "outcome": "failed",
                    "reason": "TASK_FAILED: CONTROL_FAILED: RUNTIME_SHUTDOWN",
                }
            }
        ),
        encoding="utf-8",
    )
    ledger = root / "build/campaign/attempts.jsonl"
    attempt_id = "pilot:3102:record-42"
    ledger.write_text(
        "\n".join(
            json.dumps(record)
            for record in (
                {
                    "event": "RESERVED",
                    "attempt_id": attempt_id,
                    "purpose": "pilot",
                    "kind": "rollout",
                    "mode": "teacher",
                    "seed": 3102,
                    "run_id": "record-42",
                    "attempt_dir": str(attempt),
                    "reserved_at_ns": 1_000_000_000,
                    "budget": {"purpose_attempt_limit": 3},
                },
                {
                    "event": "INTERRUPTED",
                    "attempt_id": attempt_id,
                    "seed": 3102,
                    "purpose": "pilot",
                    "kind": "rollout",
                    "mode": "teacher",
                    "exit_code": 130,
                    "wall_s": 0.0,
                    "failure": "interrupted",
                    "raw_bytes": 0,
                    "attempt_dir": str(attempt),
                    "finished_at_ns": 43_000_000_000,
                    "recording_complete": False,
                    "runtime_outcome": "",
                    "runtime_reason": "",
                    "task_outcome": "",
                    "quality_qualified": False,
                    "qualified": False,
                },
            )
        )
        + "\n",
        encoding="utf-8",
    )
    return root, attempt_id, ledger


def test_classify_reads_recorded_episode_without_root_copies(tmp_path):
    root, _attempt_id, _ledger = _terminal_project(tmp_path)
    classification = campaign.classify(
        root / "build/campaign/pilot-03102-record-42", "export/quality.json"
    )
    assert classification == {
        "recording_complete": True,
        "runtime_outcome": "interrupted",
        "runtime_reason": "SIGTERM",
        "task_outcome": "failed",
        "quality_qualified": False,
        "qualified": False,
    }


def test_interrupted_terminal_backfill_preserves_history_and_fills_real_fields(tmp_path):
    root, attempt_id, ledger = _terminal_project(tmp_path)
    config = (
        root
        / "shared/scenarios/marvin_sim/stack/configs/runtime/experiments/stack_campaign.yaml"
    )
    assert (
        campaign.main(
            [
                "--project-root",
                str(root),
                "--config",
                str(config),
                "backfill-terminal",
                "--attempt-id",
                attempt_id,
            ]
        )
        == 0
    )
    records = [json.loads(line) for line in ledger.read_text().splitlines()]
    assert [record["event"] for record in records] == ["RESERVED", "INTERRUPTED", "INTERRUPTED"]
    assert records[0]["budget"] == {"purpose_attempt_limit": 3}
    assert records[1]["wall_s"] == 0.0
    terminal = records[-1]
    assert terminal["attempt_id"] == attempt_id
    assert terminal["seed"] == 3102
    assert terminal["purpose"] == "pilot"
    assert terminal["exit_code"] == 130
    assert terminal["wall_s"] == 41.0
    assert terminal["recording_complete"] is True
    assert terminal["runtime_outcome"] == "interrupted"
    assert terminal["runtime_reason"] == "SIGTERM"
    assert terminal["task_outcome"] == "failed"
    assert terminal["qualified"] is False
    assert terminal["finished_at_ns"] == 43_000_000_000
    assert terminal["backfilled_from"]

    # A second append is refused once the true fields are present.
    assert (
        campaign.main(
            [
                "--project-root",
                str(root),
                "--config",
                str(config),
                "backfill-terminal",
                "--attempt-id",
                attempt_id,
            ]
        )
        == 2
    )
    assert len(ledger.read_text().splitlines()) == 3


def test_interrupt_waits_for_the_whole_launch_group_and_keeps_wall_time(tmp_path):
    import os
    import signal
    import sys
    import threading

    from dual_arm_runtime.offline.campaign_execution import (
        Attempt,
        LaunchInterrupted,
        run_command,
    )

    marker = tmp_path / "child-finished"
    attempt = Attempt(
        attempt_id="pilot:1:interrupt",
        purpose="pilot",
        kind="rollout",
        mode="teacher",
        seed=1,
        run_id="interrupt",
        episode_id="interrupt",
        directory=tmp_path,
        config="",
        code={},
        budget={},
        reserved_at_ns=0,
    )
    child = (
        "import signal,time;"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN);"
        f"time.sleep(1.0);open({str(marker)!r},'w').write('ok')"
    )
    command = [
        sys.executable,
        "-c",
        "import subprocess,sys,time;"
        f"subprocess.Popen([sys.executable,'-c',{child!r}]);"
        "time.sleep(30)",
    ]
    timer = threading.Timer(0.3, lambda: os.kill(os.getpid(), signal.SIGINT))
    timer.start()
    try:
        with pytest.raises(LaunchInterrupted) as interrupted:
            run_command(
                command,
                attempt,
                tmp_path,
                30.0,
                0,
                tmp_path,
                stop_grace_s=5.0,
            )
    finally:
        timer.cancel()
    assert interrupted.value.code == 130
    assert interrupted.value.failure == "interrupted"
    assert interrupted.value.wall_s >= 1.0
    assert marker.is_file()
