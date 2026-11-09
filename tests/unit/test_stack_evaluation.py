import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "runtime/ros2/scripts"))
import stack_campaign  # noqa: E402
import stack_evaluate  # noqa: E402


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + "\n", encoding="utf-8")


def campaign(tmp_path: Path) -> tuple[Path, Path]:
    root = tmp_path / "project"
    config = {
        "schema_version": 1,
        "budget": {
            "pilot_seed_first": 1000,
            "pilot_seed_last": 1009,
            "pilot_max_attempts": 10,
            "pilot_act_reserved_attempts": 2,
            "pilot_teacher_seed_last": 1007,
            "pilot_label_replay_max": 2,
            "formal_seed_first": 10000,
            "formal_seed_last": 10099,
            "formal_max_attempts": 100,
            "formal_target_qualified": 50,
            "formal_validation_qualified_indices": [9, 19, 29, 39, 49],
            "test_seed_first": 20000,
            "test_seed_last": 20019,
            "test_max_attempts": 20,
            "evaluation_max_attempts": 20,
            "max_raw_bytes": 100000000,
            "reserve_bytes": 0,
            "estimated_attempt_bytes": 1,
            "min_free_bytes": 1,
        },
        "paths": {
            "attempts_root": "var/attempts",
            "ledger": "var/attempts.jsonl",
            "lock": "var/.ledger.lock",
            "run_lock": "var/.run.lock",
        },
        "quality": {"sidecar": "quality.json"},
        "launch": {"command": [], "timeout_s": 10.0},
    }
    config_path = root / "runtime/ros2/configs/experiments/stack_campaign.yaml"
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return root, config_path


def make_attempt(root: Path) -> Path:
    attempt = root / "var/attempts/evaluation-20000-run"
    export = attempt / "stack/run/export"
    _write(
        export / "labels.json",
        {
            "labels": [
                {
                    "chunk_seq": 1,
                    "command_id": "cmd",
                    "observation_sim_time_ns": 1_000_000_000,
                    "started_sim_time_ns": 1_024_000_000,
                    "label_sim_time_ns": 1_050_000_000,
                    "observation_period_ns": 50_000_000,
                    "raw_chunk_seq": 1,
                    "raw_source_observation_time_ns": 1_000_000_000,
                    "rgb": {},
                },
                {
                    "chunk_seq": 1,
                    "command_id": "cmd",
                    "observation_sim_time_ns": 1_050_000_000,
                    "started_sim_time_ns": 1_024_000_000,
                    "label_sim_time_ns": 1_100_000_000,
                    "observation_period_ns": 50_000_000,
                    "raw_chunk_seq": 1,
                    "raw_source_observation_time_ns": 1_000_000_000,
                    "rgb": {},
                },
            ]
        },
    )
    _write(
        attempt / "manifest.json",
        {
            "started_at_ns": 1_000_000_000,
            "ended_at_ns": 3_000_000_000,
            "runtime": {"outcome": "stopped", "reason": "TASK_SUCCEEDED"},
            "recording": {"complete": True},
            "configuration": {"action_source": "policy"},
        },
    )
    _write(
        export / "raw_predictions.json",
        {
            "predictions": [
                {
                    "chunk_seq": 1,
                    "source_observation_time_ns": 1_000_000_000,
                    "times_from_observation_ns": [
                        50_000_000 + index * 50_000_000 for index in range(60)
                    ],
                    "positions": [[0.0] * 16 for _ in range(60)],
                }
            ]
        },
    )
    _write(
        attempt / "task_result.json",
        {
            "schema_version": 1,
            "evaluated": True,
            "test_only": False,
            "action_source": "policy",
            "result": {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"},
            "task": {
                "state": "SUCCEEDED",
                "reason": "TASK_SUCCEEDED",
                "task_start_sim_time_ns": 1_000_000_000,
                "completed_sim_time_ns": 3_000_000_000,
                "sample_sim_time_ns": 3_000_000_000,
                "stable_elapsed_s": 2.0,
                "task_config_digest": "d" * 64,
                "plan_digest": "e" * 64,
                "conditions_passed": True,
            },
        },
    )
    _write(attempt / "quality.json", {"qualified": True, "failure_reasons": []})
    _write(
        attempt / "launch_result.json",
        {
            "export_directory": str(export),
            "policy_mode": "act",
            "launcher_exit_code": 0,
            "exit_code": 0,
            "act": {
                "bundle_digest": "a" * 64,
                "processor_digest": "b" * 64,
                "profile_semantic_sha256": "c" * 64,
                "observation_schema_hash": "f" * 64,
                "action_schema_hash": "1" * 64,
                "model_sha256": "2" * 64,
                "synthetic": False,
                "checkpoint_mode": "full",
                "checkpoint_step": 5000,
            },
            "act_metrics": {
                "predictions": 2,
                "latency_ms": {"p50": 100.0, "p95": 180.0, "max": 180.0},
                "vram_peak_mib": 512.0,
            },
        },
    )
    events = attempt / "stack/run/act-policy-log/events.000001.jsonl"
    events.parent.mkdir(parents=True)
    events.write_text(
        "\n".join(
            json.dumps(event)
            for event in (
                {
                    "event": "act.model.ready",
                    "fields": {
                        "ready": True,
                        "bundle_digest": "a" * 64,
                        "processor_digest": "b" * 64,
                        "profile_semantic_sha256": "c" * 64,
                        "observation_schema_hash": "f" * 64,
                        "action_schema_hash": "1" * 64,
                        "model_sha256": "2" * 64,
                        "synthetic": False,
                        "checkpoint_mode": "full",
                    },
                },
                {
                    "event": "act.predict.finished",
                    "fields": {
                        "latency_ns": 100_000_000,
                        "vram_peak_mib": 512.0,
                        "chunk_size": 60,
                    },
                },
                {
                    "event": "act.predict.finished",
                    "fields": {
                        "latency_ns": 180_000_000,
                        "vram_peak_mib": 512.0,
                        "chunk_size": 60,
                    },
                },
            )
        )
        + "\n"
    )
    return attempt


def test_evaluation_allocation_shares_the_fixed_test_seed_pool(tmp_path):
    root, config_path = campaign(tmp_path)
    config = stack_campaign._load(config_path)
    first, _ = stack_campaign._allocate(
        config, [], "evaluation", "rollout", "act", None, root / "var/attempts"
    )
    assert first == 20000
    record = {
        "event": "RESERVED",
        "attempt_id": "evaluation:20000:run",
        "purpose": "evaluation",
        "seed": 20000,
        "attempt_dir": str(root / "var/attempts/evaluation-20000-run"),
    }
    next_eval, _ = stack_campaign._allocate(
        config, [record], "evaluation", "rollout", "act", None, root / "var/attempts"
    )
    assert next_eval == 20001
    next_test, _ = stack_campaign._allocate(
        config, [record], "test", "rollout", "act", None, root / "var/attempts"
    )
    assert next_test == 20001


def test_evaluation_report_counts_missing_seeds_and_actual_metrics(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    ledger = root / "var/attempts.jsonl"
    _write(
        ledger,
        {
            "event": "RESERVED",
            "attempt_id": "evaluation:20000:run",
            "purpose": "evaluation",
            "kind": "rollout",
            "mode": "act",
            "seed": 20000,
            "attempt_dir": str(attempt),
        },
    )
    with ledger.open("a", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                {
                    "event": "FINISHED",
                    "attempt_id": "evaluation:20000:run",
                    "seed": 20000,
                    "purpose": "evaluation",
                    "mode": "act",
                    "kind": "rollout",
                    "exit_code": 0,
                    "wall_s": 2.0,
                    "failure": "",
                }
            )
            + "\n"
        )
    document = stack_evaluate.build_report(
        project_root=root,
        config_path=config_path,
        purposes=("evaluation",),
        output=tmp_path / "report",
        generate_media=False,
    )
    assert document["summary"]["denominator"] == 20
    assert document["summary"]["reserved"] == 1
    assert document["summary"]["terminal"] == 1
    assert document["summary"]["closed_loop"] == 1
    assert document["summary"]["success"] == 1
    assert document["attempts"][0]["seed"] == 20000
    assert document["attempts"][0]["first_started_sim_time_ns"] == 1_024_000_000
    assert document["attempts"][0]["first_reference_applied_sim_time_ns"] == 1_024_000_000
    assert document["attempts"][0]["first_predicted_target_sim_time_ns"] == 1_050_000_000
    assert document["attempts"][0]["first_exported_label_sim_time_ns"] == 1_050_000_000
    assert document["attempts"][0]["task_duration_sim_seconds"] == 2.0
    assert document["attempts"][0]["execution_span_seconds"] == 0.1
    assert document["attempts"][0]["task_success"] is True
    assert document["attempts"][0]["success"] is True
    assert document["attempts"][0]["run_success"] is True
    assert document["attempts"][0]["command_blocks"] == 1
    assert document["attempts"][0]["prediction_p50_ms"] == 100.0
    assert document["attempts"][0]["prediction_p95_ms"] == 180.0
    assert document["attempts"][0]["vram_peak_mib"] == 512.0
    assert document["attempts"][1]["failure_type"] == "MISSING_RESERVATION"
    assert (tmp_path / "report/report.md").is_file()



def _report(root: Path, config_path: Path, name: str) -> dict:
    return stack_evaluate.build_report(
        project_root=root,
        config_path=config_path,
        purposes=("evaluation",),
        output=root / name,
        generate_media=False,
    )


def _reserve(root: Path, attempt: Path) -> None:
    ledger = root / "var/attempts.jsonl"
    _write(
        ledger,
        {
            "event": "RESERVED",
            "attempt_id": "evaluation:20000:run",
            "purpose": "evaluation",
            "kind": "rollout",
            "mode": "act",
            "seed": 20000,
            "attempt_dir": str(attempt),
        },
    )
    with ledger.open("a", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                {
                    "event": "FINISHED",
                    "attempt_id": "evaluation:20000:run",
                    "seed": 20000,
                    "purpose": "evaluation",
                    "mode": "act",
                    "kind": "rollout",
                    "exit_code": 0,
                    "wall_s": 2.0,
                    "failure": "",
                }
            )
            + "\n"
        )


def test_fake_policy_labels_without_act_evidence_never_count(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    launch = json.loads((attempt / "launch_result.json").read_text())
    launch.pop("act", None)
    launch.pop("act_metrics", None)
    launch["policy_mode"] = "fake"
    (attempt / "launch_result.json").write_text(json.dumps(launch) + "\n", encoding="utf-8")
    for path in (attempt / "stack").glob("*/act-policy-log"):
        for child in path.iterdir():
            child.unlink()
        path.rmdir()
    _reserve(root, attempt)
    report = _report(root, config_path, "fake-report")
    row = report["attempts"][0]
    assert row["act_executed"] is False
    assert row["closed_loop"] is False
    assert row["success"] is False
    assert row["failure_type"] == "ACT_EVIDENCE_INCOMPLETE"
    assert report["summary"]["act_executed"] == 0
    assert report["summary"]["success"] == 0


def test_quality_failure_is_separate_from_physical_task_success(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    (attempt / "quality.json").write_text(
        json.dumps({"qualified": False, "failure_reasons": ["visibility"]}) + "\n",
        encoding="utf-8",
    )
    _reserve(root, attempt)
    report = _report(root, config_path, "quality-report")
    row = report["attempts"][0]
    assert row["task_success"] is True
    assert row["success"] is True
    assert row["quality_qualified"] is False
    assert row["run_success"] is False
    assert row["failure_type"] == "QUALITY_REJECTED"
    assert report["summary"]["success"] == 1
    assert report["summary"]["run_success"] == 0


def test_outcome_string_without_physical_task_facts_is_not_success(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    (attempt / "task_result.json").write_text(
        json.dumps({"result": {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"}}) + "\n",
        encoding="utf-8",
    )
    _reserve(root, attempt)
    report = _report(root, config_path, "no-task-report")
    row = report["attempts"][0]
    assert row["act_executed"] is True
    assert row["closed_loop"] is True
    assert row["task_success"] is False
    assert row["success"] is False
    assert row["run_success"] is False
    assert row["failure_type"] == "TASK_NOT_EVALUATED"


def test_time_resolution_separates_started_and_20hz_exported_label(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    export = Path(json.loads((attempt / "launch_result.json").read_text())["export_directory"])
    (export / "raw_predictions.json").unlink()
    _reserve(root, attempt)
    report = _report(root, config_path, "time-report")
    row = report["attempts"][0]
    assert row["first_started_sim_time_ns"] == 1_024_000_000
    assert row["first_reference_applied_sim_time_ns"] == 1_024_000_000
    assert row.get("first_predicted_target_sim_time_ns") is None
    assert row["first_exported_label_sim_time_ns"] == 1_050_000_000
    assert row["first_exported_label_exact"] is False
    assert row["task_duration_sim_seconds"] == 2.0
    assert row["execution_span_seconds"] == 0.1


def test_replay_record_with_act_like_files_is_never_act_execution(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    config = stack_campaign._load(config_path)
    row = stack_evaluate._metric_for_seed(
        root,
        config,
        20000,
        {
            "attempt_dir": str(attempt),
            "attempt_id": "pilot:1000:replay",
            "mode": "teacher",
            "kind": "label_replay",
            "seed": 20000,
        },
        {"event": "FINISHED", "mode": "teacher", "exit_code": 0},
    )
    assert row["act_executed"] is False
    assert row["closed_loop"] is False
    assert row["success"] is False
    assert row["failure_type"] == "ACT_EVIDENCE_INCOMPLETE"


def test_later_control_failure_does_not_erase_real_act_execution(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    path = attempt / "launch_result.json"
    launch = json.loads(path.read_text())
    launch["exit_code"] = 1
    launch["launcher_exit_code"] = 1
    _write(path, launch)
    path = attempt / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["runtime"] = {"outcome": "failed", "reason": "FOLLOWER_DEVIATION_EXCEEDED"}
    _write(path, manifest)
    path = attempt / "task_result.json"
    task = json.loads(path.read_text())
    task["result"] = {"outcome": "failed", "reason": "CONTROL_FAILED"}
    _write(path, task)
    _reserve(root, attempt)
    row = _report(root, config_path, "control-failure-report")["attempts"][0]
    assert row["act_executed"] is True
    assert row["closed_loop"] is True
    assert row["success"] is False
    assert row["run_success"] is False


def _write_backend_events(attempt: Path, *events: dict) -> Path:
    path = attempt / "stack/run/run/log/events.000001.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8"
    )
    return path


def _make_control_failure(attempt: Path) -> None:
    path = attempt / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["runtime"] = {"outcome": "failed", "reason": "FOLLOWER_DEVIATION_EXCEEDED"}
    _write(path, manifest)
    path = attempt / "task_result.json"
    task = json.loads(path.read_text())
    task["result"] = {"outcome": "failed", "reason": "CONTROL_FAILED"}
    _write(path, task)


def test_backend_rejection_detail_is_machine_readable_and_preferred(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    _write_backend_events(
        attempt,
        {
            "schema_version": 1,
            "component": "backend",
            "instance_id": "backend-1",
            "event_seq": 10,
            "event": "backend.submit_follower",
            "reason": "FOLLOWER_DEVIATION_EXCEEDED",
            "message": "rejected",
            "command_id": "cmd-7",
            "fields": {"mode": "FOLLOWER", "has_reference": False},
        },
        {
            "schema_version": 1,
            "component": "backend",
            "instance_id": "backend-1",
            "event_seq": 11,
            "event": "backend.submit_follower.rejected",
            "reason": "FOLLOWER_DEVIATION_EXCEEDED",
            "message": (
                "axis 'right_joint7' maximum target deviation 0.1504 "
                "exceeds configured 0.15"
            ),
            "command_id": "cmd-7",
            "request_id": "request-7",
            "fields": {},
        },
    )
    _reserve(root, attempt)
    report = _report(root, config_path, "backend-rejection-report")
    row = report["attempts"][0]
    assert report["summary"]["denominator"] == 20
    assert report["summary"]["act_executed"] == 1
    assert report["summary"]["closed_loop"] == 1
    assert report["summary"]["success"] == 0
    assert row["act_executed"] is True
    assert row["closed_loop"] is True
    assert row["success"] is False
    assert row["failure_type"] == "TASK_FAILED"
    assert len(row["controller_rejections"]) == 2
    first, detailed = row["controller_rejections"]
    assert first["reason"] == "FOLLOWER_DEVIATION_EXCEEDED"
    assert first["message"] == "rejected"
    assert first["command_id"] == "cmd-7"
    assert first["event_seq"] == 10
    assert first["path"].endswith("events.000001.jsonl")
    assert detailed["message"].startswith("axis 'right_joint7'")
    assert detailed["command_id"] == "cmd-7"
    assert detailed["request_id"] == "request-7"
    assert row["failure_detail"].startswith("FOLLOWER_DEVIATION_EXCEEDED: axis")
    assert "right_joint7" in row["failure_detail"]
    assert "0.1504" in row["failure_detail"]
    assert "0.15" in row["failure_detail"]


def test_missing_or_corrupt_backend_log_keeps_report_semantics(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    _reserve(root, attempt)
    missing = _report(root, config_path, "missing-backend-report")["attempts"][0]
    assert missing["controller_rejections"] == []
    assert missing["failure_detail"] == "FOLLOWER_DEVIATION_EXCEEDED"
    assert missing["act_executed"] is True
    assert missing["closed_loop"] is True
    assert missing["success"] is False
    assert missing["failure_type"] == "TASK_FAILED"

    path = attempt / "stack/run/run/log/events.000001.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('not json\n{"event": "backend.submit_follower.rejected"\n', encoding="utf-8")
    corrupt = _report(root, config_path, "corrupt-backend-report")["attempts"][0]
    assert corrupt["controller_rejections"] == []
    assert corrupt["failure_detail"] == "FOLLOWER_DEVIATION_EXCEEDED"
    assert corrupt["act_executed"] is True
    assert corrupt["closed_loop"] is True
    assert corrupt["success"] is False
    assert corrupt["failure_type"] == "TASK_FAILED"


def test_runtime_timing_failure_is_not_hidden_by_launcher_exit(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    manifest = json.loads((attempt / "manifest.json").read_text())
    manifest["runtime"]["reason"] = "CHUNK_LATE: follower submit rejected"
    _write(attempt / "manifest.json", manifest)
    launch = json.loads((attempt / "launch_result.json").read_text())
    launch["launcher_exit_code"] = launch["exit_code"] = 1
    _write(attempt / "launch_result.json", launch)
    _reserve(root, attempt)
    row = _report(root, config_path, "late-report")["attempts"][0]
    assert row["failure_detail"] == "CHUNK_LATE: follower submit rejected"
    assert row["act_executed"] is True
    assert row["success"] is False


def test_recorded_task_failure_is_complete_evidence(tmp_path):
    root, config_path = campaign(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    _reserve(root, attempt)
    row = _report(root, config_path, "failed-task-evidence-report")["attempts"][0]
    assert row["task_outcome"] == "failed"
    assert row["success"] is False
    assert "TASK_EVIDENCE_MISSING:outcome" not in row["issues"]

    path = attempt / "task_result.json"
    task = json.loads(path.read_text())
    del task["result"]["outcome"]
    _write(path, task)
    missing = _report(root, config_path, "missing-task-outcome-report")["attempts"][0]
    assert missing["success"] is False
    assert "TASK_EVIDENCE_MISSING:outcome" in missing["issues"]
