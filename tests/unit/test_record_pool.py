"""Concurrent recordings share one ledger, stop exactly, and cancel child groups."""

import json
import os
import sys
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.offline import record, record_pool
from dual_arm_runtime.offline.campaign_ledger import CampaignError  # noqa: E402
from test_record import recording as recording


@pytest.fixture
def pool(recording, monkeypatch):  # noqa: F811 - imported pytest fixture
    root, config, _, _ = recording
    workers = []
    for index in range(2):
        directory = root / f"worker-{index}"
        directory.mkdir()
        (directory / "runtime.yaml").write_text("{}\n")
        (directory / "log").mkdir()
        workers.append({"id": f"worker-{index}", "domain_id": 70 + index,
                        "namespace": f"/worker_{index}",
                        "runtime_config": str(directory / "runtime.yaml"),
                        "backend_log": str(directory / "log")})
    manifest = root / "workers.json"
    manifest.write_text(json.dumps({"workers": workers}))
    monkeypatch.setattr(record_pool, "check_ros_sdk", lambda *args, **kwargs: 0)
    script = root / "parallel_fake.py"
    script.write_text('''import json
import os
import sys
import time
from pathlib import Path

state, delay = json.loads(sys.argv[1])[int(os.environ["CAMPAIGN_SEED"]) - 10000]
directory = Path(os.environ["CAMPAIGN_ATTEMPT_DIR"])
metadata = {"pid": os.getpid(), "domain_id": os.environ["ROS_DOMAIN_ID"],
            "runtime_config": os.environ["CAMPAIGN_RUNTIME_CONFIG"], "start": time.time(),
            "display": os.environ.get("DISPLAY")}
(directory / "worker.json").write_text(json.dumps(metadata))
time.sleep(delay)
raw = directory / "raw"
raw.mkdir()
export = directory / "export"
export.mkdir()
(raw / "manifest.json").write_text(json.dumps({
    "recording": {"complete": True}, "runtime": {"outcome": "stopped", "reason": "TEST"}}))
(raw / "task_result.json").write_text(json.dumps({"result": {"outcome": "succeeded"}}))
(export / "quality.json").write_text(json.dumps({
    "qualified": state == "qualified",
    "failure_reasons": ["camera blackout"] if state == "failed" else [],
}))
(export / "labels.json").write_text(json.dumps({
    "summary": {"rejected": {"brake_replaced": 3}},
}))
metadata["end"] = time.time()
(directory / "worker.json").write_text(json.dumps(metadata))
raise SystemExit(130 if state == "interrupted" else 0)
''')

    def run(states, *arguments):
        monkeypatch.setattr(
            record, "_collector_command",
            lambda args: [sys.executable, str(script), json.dumps(states)],
        )
        return record.main(
            [
                "--project-root",
                str(root),
                "--config",
                "campaign.yaml",
                "--runtime-config",
                "runtime.yaml",
                "--worker-manifest",
                str(manifest),
                *arguments,
            ]
        )

    return root, config, manifest, run


def _records(root):
    return [json.loads(line) for line in (root / "out/attempts.jsonl").read_text().splitlines()]


def _summary(root):
    return json.loads((root / "summary.json").read_text())


def test_workers_overlap_and_share_sequential_seeds_without_overshooting(pool):
    root, config, _, run = pool
    config["budget"]["formal_target_qualified"] = 2
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))
    assert run([["qualified", 0.3]] * 3, "--count", "2") == 0
    records = _records(root)
    reserved = [item for item in records if item["event"] == "RESERVED"]
    assert [item["seed"] for item in reserved] == [10000, 10001]
    facts = [json.loads((Path(item["attempt_dir"]) / "worker.json").read_text())
             for item in reserved]
    assert max(item["start"] for item in facts) < min(item["end"] for item in facts)
    assert {item["domain_id"] for item in facts} == {"70", "71"}
    assert len({item["runtime_config"] for item in facts}) == 2
    assert all(item["display"] is None for item in facts)
    assert _summary(root)["qualified"] == _summary(root)["attempted"] == 2
    document = json.loads(Path(_summary(root)["formal_select"]).read_text())
    assert document["status"] == "SELECTED"
    assert [item["seed"] for item in document["selected"]] == [10000, 10001]


def test_freed_worker_is_reused_until_target(pool):
    root, _, _, run = pool
    assert run([["qualified", 0.1], ["qualified", 0.6], ["qualified", 0.1]],
               "--count", "3") == 0
    running = [item for item in _records(root) if item["event"] == "RUNNING"]
    assert [item["worker"] for item in running] == ["worker-0", "worker-1", "worker-0"]
    assert _summary(root)["qualified"] == 3


def test_single_remaining_slot_does_not_start_extra_worker(pool):
    root, _, _, run = pool
    assert run([["qualified", 0.1], ["qualified", 0.1]]) == 0
    assert _summary(root)["attempted"] == 1


def test_failure_threshold_cancels_running_attempt_and_preserves_reservation(pool):
    root, _, _, run = pool
    assert run([["failed", 0.1], ["qualified", 10]], "--count", "2",
               "--max-consecutive-failures", "1") == 1
    summary = _summary(root)
    assert summary["reason"] == "consecutive_failures"
    assert summary["attempted"] == 2 and summary["cancelled"] == 1
    records = _records(root)
    assert len([item for item in records if item["event"] == "RESERVED"]) == 2
    assert len([item for item in records if item["event"] == "INTERRUPTED"]) == 1
    failed = next(item for item in records if item["event"] == "FINISHED")
    assert failed["diagnostics"]["quality_failure_reasons"] == ["camera blackout"]
    assert failed["diagnostics"]["label_rejections"] == {"brake_replaced": 3}
    assert not Path(failed["attempt_dir"]).exists()
    for path in (root / "out").glob("*/worker.json"):
        with pytest.raises(ProcessLookupError):
            os.kill(json.loads(path.read_text())["pid"], 0)


def test_budget_exhaustion_drains_already_reserved_attempts(pool):
    root, config, _, run = pool
    config["budget"]["formal_max_attempts"] = 2
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))
    assert run([["failed", 0.1], ["qualified", 0.4]], "--count", "3") == 2
    summary = _summary(root)
    assert summary["reason"] == "campaign_stopped"
    assert summary["qualified"] == 1 and summary["attempted"] == 2
    assert summary["cancelled"] == 0


def test_global_qualified_target_includes_inflight_slots(pool):
    root, config, _, run = pool
    config["budget"]["formal_target_qualified"] = 1
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))
    assert run([["qualified", 0.1]] * 2, "--count", "2") == 2
    assert _summary(root)["qualified"] == _summary(root)["attempted"] == 1


def test_interrupt_cancels_other_workers(pool):
    root, _, _, run = pool
    assert run([["interrupted", 0.1], ["qualified", 10]], "--count", "2") == 130
    assert _summary(root)["reason"] == "interrupted"
    assert _summary(root)["cancelled"] == 1


def test_attempt_timeout_stops_process_group(pool):
    root, config, _, run = pool
    config["launch"]["timeout_s"] = 0.15
    (root / "campaign.yaml").write_text(yaml.safe_dump(config))
    assert run([["qualified", 10]] * 2, "--count", "2",
               "--max-consecutive-failures", "1") == 1
    assert _summary(root)["reason"] == "consecutive_failures"
    assert _summary(root)["cancelled"] == 1
    assert any(item.get("exit_code") == 124 for item in _records(root))


def test_duplicate_worker_domains_are_rejected_before_reservation(pool):
    root, _, manifest, run = pool
    document = json.loads(manifest.read_text())
    document["workers"][1]["domain_id"] = document["workers"][0]["domain_id"]
    manifest.write_text(json.dumps(document))
    with pytest.raises(CampaignError, match="distinct domain_id"):
        record_pool.load_worker_plan(manifest)
    assert run([["qualified", 0.1]] * 2) == 2
    assert not (root / "out/attempts.jsonl").exists()


def _existing_qualified(root, count):
    output = root / "out"
    output.mkdir(exist_ok=True)
    records = []
    for index in range(count):
        attempt_id = f"formal:{10000 + index}:existing"
        records.extend([
            {"event": "RESERVED", "attempt_id": attempt_id, "run_id": f"existing-{index}",
             "purpose": "formal", "kind": "rollout", "mode": "teacher", "seed": 10000 + index,
             "attempt_dir": str(output / f"existing-{index}")},
            {"event": "FINISHED", "attempt_id": attempt_id, "qualified": True, "raw_bytes": 0},
        ])
    (output / "attempts.jsonl").write_text("".join(json.dumps(item) + "\n" for item in records))


def test_total_target_rechecks_ledger_and_only_collects_difference(pool):
    root, _, _, run = pool
    _existing_qualified(root, 1)
    assert run([["qualified", 0.1]] * 4, "--count", "3", "--target-total", "3") == 0
    summary = _summary(root)
    assert summary["requested"] == summary["qualified"] == summary["attempted"] == 2
    assert summary["target_total"] == summary["campaign_qualified"] == 3
    reserved = [item for item in _records(root) if item["event"] == "RESERVED"]
    assert [item["seed"] for item in reserved] == [10000, 10001, 10002]


def test_total_target_already_reached_reserves_nothing(pool):
    root, _, _, run = pool
    _existing_qualified(root, 2)
    assert run([], "--target-total", "2") == 0
    summary = _summary(root)
    assert summary["requested"] == summary["qualified"] == summary["attempted"] == 0
    assert summary["campaign_qualified"] == 2
    assert len([item for item in _records(root) if item["event"] == "RESERVED"]) == 2


def test_total_target_respects_campaign_cap_before_reserving(pool):
    root, _, _, run = pool
    assert run([], "--target-total", "6") == 2
    assert not (root / "out/attempts.jsonl").exists()
    assert not (root / "out/.run.lock").exists()


def test_formal_total_target_excludes_pilot_trial(pool):
    root, _, _, run = pool
    _existing_qualified(root, 0)
    pilot = [
        {"event": "RESERVED", "attempt_id": "pilot:1000:trial", "run_id": "trial",
         "purpose": "pilot", "kind": "rollout", "mode": "teacher", "seed": 1000},
        {"event": "FINISHED", "attempt_id": "pilot:1000:trial", "qualified": True, "raw_bytes": 0},
    ]
    with (root / "out/attempts.jsonl").open("a") as ledger:
        ledger.write("\n".join(json.dumps(item) for item in pilot) + "\n")
    assert run([["qualified", 0.1]] * 3, "--target-total", "2") == 0
    summary = _summary(root)
    assert summary["campaign_qualified"] == summary["qualified"] == summary["attempted"] == 2
    formal = [item for item in _records(root)
              if item["event"] == "RESERVED" and item["purpose"] == "formal"]
    assert [item["seed"] for item in formal] == [10000, 10001]
