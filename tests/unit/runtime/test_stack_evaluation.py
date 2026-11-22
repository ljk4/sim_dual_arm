import json
from pathlib import Path

import pytest
import yaml
from collection_fixtures import ROBOT, SCENARIO_ID, TASK, build_collection_tree

ROOT = Path(__file__).resolve().parents[3]
from dual_arm_runtime.offline.collection import campaign  # noqa: E402
from dual_arm_runtime.offline.evaluation import evaluate, evaluation_metrics  # noqa: E402


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + "\n", encoding="utf-8")


def campaign_config(tmp_path: Path) -> tuple[Path, Path]:
    root = tmp_path / "project"
    config = {
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
            "formal_select": "var/datasets/formal-select.json",
        },
        "quality": {"sidecar": "quality.json"},
        "launch": {"command": [], "timeout_s": 10.0},
    }
    config_path = (
        root
        / "shared/scenarios/marvin_sim/stack/configs/runtime/experiments/stack_campaign.yaml"
    )
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return root, config_path


def make_attempt(root: Path) -> Path:
    attempt = root / "var/attempts/evaluation-20000-run"
    export = attempt / "export"
    _write(
        export / "labels.json",
        {
            "labels": [
                {
                    "chunk_seq": 1,
                    "command_id": "cmd",
                    "observation_backend_time_ns": 1_000_000_000,
                    "started_backend_time_ns": 1_024_000_000,
                    "label_backend_time_ns": 1_050_000_000,
                    "observation_period_ns": 50_000_000,
                    "raw_chunk_seq": 1,
                    "raw_source_observation_time_ns": 1_000_000_000,
                    "rgb": {},
                },
                {
                    "chunk_seq": 1,
                    "command_id": "cmd",
                    "observation_backend_time_ns": 1_050_000_000,
                    "started_backend_time_ns": 1_024_000_000,
                    "label_backend_time_ns": 1_100_000_000,
                    "observation_period_ns": 50_000_000,
                    "raw_chunk_seq": 1,
                    "raw_source_observation_time_ns": 1_000_000_000,
                    "rgb": {},
                },
            ]
        },
    )
    _write(
        attempt / "raw" / "manifest.json",
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
        attempt / "raw" / "task_result.json",
        {
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
    _write(export / "quality.json", {"qualified": True, "failure_reasons": []})
    events = attempt / "act-policy-log/events.000001.jsonl"
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
    root, config_path = campaign_config(tmp_path)
    config = campaign._load(config_path)
    first, _ = campaign._allocate(
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
    next_eval, _ = campaign._allocate(
        config, [record], "evaluation", "rollout", "act", None, root / "var/attempts"
    )
    assert next_eval == 20001
    next_test, _ = campaign._allocate(
        config, [record], "test", "rollout", "act", None, root / "var/attempts"
    )
    assert next_test == 20001


def test_evaluation_report_counts_missing_seeds_and_actual_metrics(tmp_path):
    root, config_path = campaign_config(tmp_path)
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
    document = evaluate.build_report(
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
    assert document["attempts"][0]["first_started_backend_time_ns"] == 1_024_000_000
    assert document["attempts"][0]["first_reference_applied_backend_time_ns"] == 1_024_000_000
    assert document["attempts"][0]["first_predicted_target_backend_time_ns"] == 1_050_000_000
    assert document["attempts"][0]["first_exported_label_backend_time_ns"] == 1_050_000_000
    assert document["attempts"][0]["task_duration_sim_seconds"] == 2.0
    assert document["attempts"][0]["execution_span_seconds"] == 0.1
    assert document["attempts"][0]["task_success"] is True
    assert document["attempts"][0]["success"] is True
    assert document["attempts"][0]["run_success"] is True
    assert document["attempts"][0]["command_blocks"] == 1
    assert document["attempts"][0]["act_executed"] is True
    assert document["attempts"][0]["prediction_p50_ms"] is None
    assert document["attempts"][0]["prediction_p95_ms"] is None
    assert document["attempts"][0]["vram_peak_mib"] is None
    assert document["attempts"][1]["failure_type"] == "MISSING_RESERVATION"
    assert (tmp_path / "report/report.md").is_file()



def _report(root: Path, config_path: Path, name: str) -> dict:
    return evaluate.build_report(
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
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    (attempt / "export" / "raw_predictions.json").unlink()
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
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    (attempt / "export" / "quality.json").write_text(
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
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    (attempt / "raw" / "task_result.json").write_text(
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


def _task_document(attempt: Path) -> dict:
    return json.loads((attempt / "raw" / "task_result.json").read_text())


def _write_task_document(attempt: Path, document: dict) -> None:
    _write(attempt / "raw" / "task_result.json", document)


@pytest.mark.parametrize("task_type", ["handover", "stack"])
def test_policy_success_with_one_second_window_is_offline_success(tmp_path, task_type):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    document = _task_document(attempt)
    document["task"].update(
        task_type=task_type,
        stable_elapsed_s=1.0,
        first_satisfied_sim_time_ns=2_000_000_000,
        completed_sim_time_ns=3_000_000_000,
        sample_sim_time_ns=3_000_000_000,
    )
    _write_task_document(attempt, document)
    _reserve(root, attempt)
    row = _report(root, config_path, f"one-second-{task_type}")["attempts"][0]
    assert row["task_success"] is True
    assert row["success"] is True
    assert row["run_success"] is True


@pytest.mark.parametrize(
    "mutate",
    [
        lambda document: document["task"].update(state="RUNNING"),
        lambda document: document["task"].update(conditions_passed=False),
        lambda document: document["task"].pop("stable_elapsed_s"),
        lambda document: document["task"].update(stable_elapsed_s=0.0),
        lambda document: document.update(evaluated=False),
        lambda document: document.update(test_only=True),
        lambda document: document.update(action_source="teacher"),
        lambda document: document.update(
            result={"outcome": "failed", "reason": "TASK_FAILED"}
        ),
    ],
    ids=[
        "running-state",
        "conditions-false",
        "no-stable-window",
        "zero-stable-window",
        "not-evaluated",
        "test-only",
        "teacher",
        "failed-result",
    ],
)
def test_one_second_success_still_requires_the_complete_frozen_fact(tmp_path, mutate):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    document = _task_document(attempt)
    document["task"].update(stable_elapsed_s=1.0)
    mutate(document)
    _write_task_document(attempt, document)
    _reserve(root, attempt)
    row = _report(root, config_path, "incomplete-fact")["attempts"][0]
    assert row["task_success"] is False
    assert row["success"] is False


def test_time_resolution_separates_started_and_20hz_exported_label(tmp_path):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    export = attempt / "export"
    (export / "raw_predictions.json").unlink()
    _reserve(root, attempt)
    report = _report(root, config_path, "time-report")
    row = report["attempts"][0]
    assert row["first_started_backend_time_ns"] == 1_024_000_000
    assert row["first_reference_applied_backend_time_ns"] == 1_024_000_000
    assert row.get("first_predicted_target_backend_time_ns") is None
    assert row["first_exported_label_backend_time_ns"] == 1_050_000_000
    assert row["first_exported_label_exact"] is False
    assert row["task_duration_sim_seconds"] == 2.0
    assert row["execution_span_seconds"] == 0.1


def test_replay_record_with_act_like_files_is_never_act_execution(tmp_path):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    config = campaign._load(config_path)
    row = evaluation_metrics.metric_for_seed(
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
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    path = attempt / "raw" / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["runtime"] = {"outcome": "failed", "reason": "FOLLOWER_DEVIATION_EXCEEDED"}
    _write(path, manifest)
    path = attempt / "raw" / "task_result.json"
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
    path = attempt / "logs/run/logs/runtime/instance/events.000001.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8"
    )
    return path


def _make_control_failure(attempt: Path) -> None:
    path = attempt / "raw" / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["runtime"] = {"outcome": "failed", "reason": "FOLLOWER_DEVIATION_EXCEEDED"}
    _write(path, manifest)
    path = attempt / "raw" / "task_result.json"
    task = json.loads(path.read_text())
    task["result"] = {"outcome": "failed", "reason": "CONTROL_FAILED"}
    _write(path, task)


def test_backend_rejection_detail_is_machine_readable_and_preferred(tmp_path):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
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
    assert "controller_rejections" not in row
    assert row["failure_detail"] == "FOLLOWER_DEVIATION_EXCEEDED"


def test_missing_or_corrupt_backend_log_keeps_report_semantics(tmp_path):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    _reserve(root, attempt)
    missing = _report(root, config_path, "missing-backend-report")["attempts"][0]
    assert "controller_rejections" not in missing
    assert missing["failure_detail"] == "FOLLOWER_DEVIATION_EXCEEDED"
    assert missing["act_executed"] is True
    assert missing["closed_loop"] is True
    assert missing["success"] is False
    assert missing["failure_type"] == "TASK_FAILED"

    path = attempt / "logs/run/logs/runtime/instance/events.000001.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('not json\n{"event": "backend.submit_follower.rejected"\n', encoding="utf-8")
    corrupt = _report(root, config_path, "corrupt-backend-report")["attempts"][0]
    assert "controller_rejections" not in corrupt
    assert corrupt["failure_detail"] == "FOLLOWER_DEVIATION_EXCEEDED"
    assert corrupt["act_executed"] is True
    assert corrupt["closed_loop"] is True
    assert corrupt["success"] is False
    assert corrupt["failure_type"] == "TASK_FAILED"


def test_runtime_timing_failure_is_reported(tmp_path):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    manifest = json.loads((attempt / "raw" / "manifest.json").read_text())
    manifest["runtime"]["reason"] = "CHUNK_LATE: follower submit rejected"
    _write(attempt / "raw" / "manifest.json", manifest)
    _reserve(root, attempt)
    row = _report(root, config_path, "late-report")["attempts"][0]
    assert row["failure_detail"] == "CHUNK_LATE: follower submit rejected"
    assert row["act_executed"] is True
    assert row["success"] is False


def test_recorded_task_failure_is_complete_evidence(tmp_path):
    root, config_path = campaign_config(tmp_path)
    attempt = make_attempt(root)
    _make_control_failure(attempt)
    _reserve(root, attempt)
    row = _report(root, config_path, "failed-task-evidence-report")["attempts"][0]
    assert row["task_outcome"] == "failed"
    assert row["success"] is False
    assert "TASK_EVIDENCE_MISSING:outcome" not in row["issues"]

    path = attempt / "raw" / "task_result.json"
    task = json.loads(path.read_text())
    del task["result"]["outcome"]
    _write(path, task)
    missing = _report(root, config_path, "missing-task-outcome-report")["attempts"][0]
    assert missing["success"] is False
    assert "TASK_EVIDENCE_MISSING:outcome" in missing["issues"]


def test_evaluate_arguments_require_exactly_one_config_source():
    with pytest.raises(SystemExit):
        evaluate.parse_arguments([])
    with pytest.raises(SystemExit):
        evaluate.parse_arguments(["--config", "a.yaml", "--robot", ROBOT, "--task", TASK])
    with pytest.raises(SystemExit):
        evaluate.parse_arguments(["--shared-root", "/shared", "--config", "a.yaml"])


def test_evaluate_defaults_to_generic_output_directory():
    args = evaluate.parse_arguments(["--config", "campaign.yaml"])
    assert args.output == Path("var/_direct/debug/evaluation")


def test_evaluate_scenario_resolves_the_campaign_recipe(tmp_path, monkeypatch):
    tree = build_collection_tree(tmp_path)
    seen = {}

    def fake_build_report(**kwargs):
        seen.update(kwargs)
        return {"summary": {}}

    monkeypatch.setattr(evaluate, "build_report", fake_build_report)
    code = evaluate.main(
        [
            "--project-root", str(tree.repo),
            "--robot", ROBOT,
            "--task", TASK,
            "--shared-root", str(tree.shared),
            "--no-media",
        ]
    )
    assert code == 0
    assert seen["config_path"] == tree.campaign_recipe.resolve()
    assert seen["output"] == (
        tree.repo / "var" / SCENARIO_ID / "debug/evaluation"
    ).resolve()
