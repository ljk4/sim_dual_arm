import json
import sys
from pathlib import Path

import yaml

_TOOLS = Path(__file__).resolve().parents[2] / "runtime/ros2/scripts"
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import campaign  # noqa: E402


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
            "attempts_root": ".build/campaign",
            "ledger": ".build/campaign/attempts.jsonl",
            "lock": ".build/campaign/.lock",
            "run_lock": ".build/campaign/.run.lock",
            "required_paths": [],
            "sdk_marker": "",
        },
        "launch": {"command": [], "timeout_s": 30.0},
        "quality": {"sidecar": "quality.json", "max_blackout_s": 0.35},
    }
    root = tmp_path / "project"
    (root / "runtime/ros2/configs/experiments").mkdir(parents=True)
    (root / "runtime/ros2/configs/experiments/stack_campaign.yaml").write_text(
        yaml.safe_dump(config), encoding="utf-8"
    )
    return root


def _run(root, run_id):
    return campaign.main(
        [
            "--project-root",
            str(root),
            "run",
            "--purpose",
            "pilot",
            "--run-id",
            run_id,
            "--runtime-config",
            "runtime/ros2/configs/teacher_collect.yaml",
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
        for line in (root / ".build/campaign/attempts.jsonl").read_text().splitlines()
    ]
    seeds = [item["seed"] for item in records if item["event"] == "RESERVED"]
    assert seeds == [1000, 1001]
    assert campaign.main(["--project-root", str(root), "status"]) == 0
    assert _run(root, "three") == 2


def test_reserved_without_finish_still_consumes_budget(tmp_path):
    root = _project(tmp_path)
    ledger = root / ".build/campaign/attempts.jsonl"
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
                "runtime/ros2/configs/teacher_collect.yaml",
                "--",
                sys.executable,
                "-c",
                "print('ok')",
                "runtime/ros2/scripts/run.sh",
            ]
        )
        == 0
    )
    records = [
        json.loads(line)
        for line in (root / ".build/campaign/attempts.jsonl").read_text().splitlines()
    ]
    reserved = [item for item in records if item["event"] == "RESERVED"]
    assert [item["kind"] for item in reserved] == ["rollout", "label_replay"]
    assert _run(root, "third") == 2


def test_formal_is_teacher_only_and_test_is_act_only(tmp_path):
    root = _project(tmp_path)
    base = [
        "--project-root",
        str(root),
        "run",
        "--kind",
        "rollout",
        "--run-id",
        "guard",
        "--runtime-config",
        "runtime/ros2/configs/teacher_collect.yaml",
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
    ledger = root / ".build/campaign/attempts.jsonl"
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
                "run",
                "--purpose",
                "formal",
                "--mode",
                "teacher",
                "--run-id",
                "after-target",
                "--runtime-config",
                "runtime/ros2/configs/teacher_collect.yaml",
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
    lock = root / ".build/campaign/.run.lock"
    lock.parent.mkdir(parents=True)
    lock.write_text("99999999", encoding="utf-8")
    assert _run(root, "recovered") == 0
    assert not lock.exists()


def test_teacher_and_replay_cannot_consume_reserved_act_attempts(tmp_path):
    import pytest

    root = _project(tmp_path)
    config = yaml.safe_load(
        (root / "runtime/ros2/configs/experiments/stack_campaign.yaml").read_text()
    )
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
