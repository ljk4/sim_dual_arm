"""Read-only per-seed metric reconstruction from structured campaign artifacts."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def digest(value: object) -> str:
    return value if isinstance(value, str) and _SHA256.fullmatch(value) else ""


def _json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _find_export(attempt: Path) -> Path | None:
    export = attempt / "export"
    return export if (export / "labels.json").is_file() else None


def _labels(export: Path | None) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
    if export is None:
        return None, []
    document = _json(export / "labels.json")
    if document is None:
        return None, []
    rows = document.get("labels")
    if not isinstance(rows, list):
        return document, []
    return document, [row for row in rows if isinstance(row, dict)]


def _base_metric(
    seed: int, record: dict[str, Any] | None, terminal: dict[str, Any] | None
) -> dict[str, Any]:
    return {
        "seed": seed,
        "reserved": record is not None,
        "terminal": terminal is not None,
        "attempt_dir": "" if record is None else str(record.get("attempt_dir", "")),
        "attempt_id": "" if record is None else str(record.get("attempt_id", "")),
        "record_mode": "" if record is None else str(record.get("mode", "")),
        "record_kind": "" if record is None else str(record.get("kind", "")),
        "task_outcome": "",
        "task_action_source": "",
        "runtime_outcome": "",
        "runtime_reason": "",
        "recording_complete": False,
        "quality_qualified": False,
        "launch_ok": False,
        "task_evaluated": False,
        "task_test_only": None,
        "task_success": False,
        "act_executed": False,
        "closed_loop": False,
        "success": False,
        "run_success": False,
        "act_evidence": {},
        "task_evidence": {},
        "first_started_backend_time_ns": None,
        "first_reference_applied_backend_time_ns": None,
        "first_exported_label_backend_time_ns": None,
        "last_exported_label_backend_time_ns": None,
        "first_predicted_target_backend_time_ns": None,
        "label_period_ns": None,
        "first_exported_label_exact": False,
        "command_blocks": 0,
        "raw_prediction_count": 0,
        "prediction_count": 0,
        "prediction_p50_ms": None,
        "prediction_p95_ms": None,
        "prediction_max_ms": None,
        "vram_peak_mib": None,
        "task_duration_sim_seconds": None,
        "execution_span_seconds": None,
        "sim_seconds": None,
        "wall_seconds": None,
        "failure_type": "",
        "failure_detail": "",
        "recording_path": "",
        "preview_video": "",
        "preview_keyframes": [],
        "manifest": "",
        "export": "",
        "issues": [],
    }


def _apply_terminal(
    metric: dict[str, Any], record: dict[str, Any], terminal: dict[str, Any]
) -> None:
    metric["terminal_event"] = str(terminal.get("event", ""))
    metric["wall_seconds"] = terminal.get("wall_s")
    metric["failure_detail"] = str(terminal.get("failure", ""))
    metric["launch_ok"] = bool(
        terminal.get("event") == "FINISHED" and terminal.get("exit_code", 0) == 0
    )
    terminal_mode = terminal.get("mode")
    if terminal_mode is not None and str(terminal_mode) != str(record.get("mode", "")):
        metric["issues"].append("TERMINAL_MODE_MISMATCH")


def _apply_manifest(metric: dict[str, Any], manifest: dict[str, Any] | None) -> None:
    if manifest is None:
        return
    runtime = manifest.get("runtime")
    if isinstance(runtime, dict):
        metric["runtime_outcome"] = str(runtime.get("outcome", ""))
        metric["runtime_reason"] = str(runtime.get("reason", ""))
    recording = manifest.get("recording")
    metric["recording_complete"] = bool(
        isinstance(recording, dict) and recording.get("complete")
    )
    configuration = manifest.get("configuration")
    if isinstance(configuration, dict):
        metric["action_source"] = str(configuration.get("action_source", ""))
    started = manifest.get("started_at_ns")
    ended = manifest.get("ended_at_ns")
    if isinstance(started, int) and isinstance(ended, int) and ended >= started:
        metric["wall_seconds"] = round((ended - started) / 1e9, 6)


def _apply_task(metric: dict[str, Any], task: dict[str, Any] | None) -> None:
    if task is None:
        return
    metric["task_evaluated"] = task.get("evaluated") is True
    metric["task_test_only"] = task.get("test_only")
    metric["task_action_source"] = str(task.get("action_source", ""))
    task_result = task.get("result")
    if isinstance(task_result, dict):
        metric["task_outcome"] = str(task_result.get("outcome", ""))
        metric["task_reason"] = str(task_result.get("reason", ""))
    fact = task.get("task")
    if not isinstance(fact, dict):
        metric["issues"].append("TASK_FACT_MISSING")
        return
    metric["task_evidence"] = dict(fact)
    start_ns = fact.get("task_start_sim_time_ns")
    completed_ns = fact.get("completed_sim_time_ns")
    sample_ns = fact.get("sample_sim_time_ns")
    stable_elapsed = fact.get("stable_elapsed_s")
    end_ns = completed_ns if isinstance(completed_ns, int) and completed_ns > 0 else sample_ns
    if (
        isinstance(start_ns, int)
        and start_ns > 0
        and isinstance(end_ns, int)
        and end_ns >= start_ns
    ):
        metric["task_duration_sim_seconds"] = round((end_ns - start_ns) / 1e9, 6)
    metric["task_success"] = bool(
        metric["task_evaluated"]
        and metric["task_test_only"] is False
        and metric["task_action_source"] == "policy"
        and metric["task_outcome"] == "succeeded"
        and metric["task_reason"] == "TASK_SUCCEEDED"
        and fact.get("state") == "SUCCEEDED"
        and fact.get("conditions_passed") is True
        and isinstance(start_ns, int)
        and start_ns > 0
        and isinstance(completed_ns, int)
        and completed_ns >= start_ns
        and isinstance(sample_ns, int)
        and sample_ns >= completed_ns
        and isinstance(stable_elapsed, int | float)
        and not isinstance(stable_elapsed, bool)
        and stable_elapsed > 0.0
        and bool(fact.get("task_config_digest"))
        and bool(fact.get("plan_digest"))
    )


def _apply_quality(metric: dict[str, Any], quality: dict[str, Any] | None) -> None:
    if quality is None:
        return
    metric["quality_qualified"] = quality.get("qualified") is True
    failures = quality.get("failure_reasons")
    if isinstance(failures, list) and failures:
        metric["quality_failures"] = [str(item) for item in failures]


def _apply_export(
    metric: dict[str, Any],
    export: Path | None,
    labels_document: dict[str, Any] | None,
    labels: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    raw_predictions: list[dict[str, Any]] = []
    if export is not None:
        raw_document = _json(export / "raw_predictions.json")
        rows = raw_document.get("predictions") if isinstance(raw_document, dict) else None
        if isinstance(rows, list):
            raw_predictions = [row for row in rows if isinstance(row, dict)]
        metric["raw_prediction_count"] = len(raw_predictions)
        metric["prediction_count"] = len(raw_predictions)
    if labels:
        _apply_label_times(metric, labels, raw_predictions)
    elif labels_document is not None:
        metric["issues"].append("LABELS_EMPTY")
    return raw_predictions


def _apply_label_times(
    metric: dict[str, Any], labels: list[dict[str, Any]], raw_predictions: list[dict[str, Any]]
) -> None:
    started = [
        int(row["started_backend_time_ns"])
        for row in labels
        if isinstance(row.get("started_backend_time_ns"), int)
    ]
    exported = [
        int(row["label_backend_time_ns"])
        for row in labels
        if isinstance(row.get("label_backend_time_ns"), int)
    ]
    observations = [
        int(row["observation_backend_time_ns"])
        for row in labels
        if isinstance(row.get("observation_backend_time_ns"), int)
    ]
    metric["first_started_backend_time_ns"] = min(started) if started else None
    metric["first_reference_applied_backend_time_ns"] = min(started) if started else None
    metric["first_exported_label_backend_time_ns"] = min(exported) if exported else None
    metric["last_exported_label_backend_time_ns"] = max(exported) if exported else None
    periods = [
        int(row["observation_period_ns"])
        for row in labels
        if isinstance(row.get("observation_period_ns"), int)
    ]
    metric["label_period_ns"] = min(periods) if periods else None
    metric["first_exported_label_exact"] = False
    if exported and observations:
        metric["execution_span_seconds"] = round((max(exported) - min(observations)) / 1e9, 6)
        metric["sim_seconds"] = metric["execution_span_seconds"]
    blocks = {
        (row.get("chunk_seq"), row.get("command_id"))
        for row in labels
        if row.get("chunk_seq") is not None and row.get("command_id")
    }
    metric["command_blocks"] = len(blocks)
    if raw_predictions:
        first_prediction = min(
            raw_predictions,
            key=lambda row: int(row.get("source_observation_time_ns", 2**63 - 1)),
        )
        offsets = first_prediction.get("times_from_observation_ns")
        source_ns = first_prediction.get("source_observation_time_ns")
        if isinstance(offsets, list) and offsets and isinstance(source_ns, int):
            metric["first_predicted_target_backend_time_ns"] = source_ns + int(
                min(int(value) for value in offsets)
            )
    metric["closed_loop"] = bool(
        metric["command_blocks"] > 0
        and metric["first_started_backend_time_ns"] is not None
        and metric["raw_prediction_count"] > 0
        and metric["recording_complete"]
    )


def _apply_act_evidence(metric: dict[str, Any]) -> None:
    evidence = {
        "record_mode_act": metric["record_mode"] == "act",
        "record_kind_rollout": metric["record_kind"] == "rollout",
        "launch_action_source_policy": metric.get("action_source") == "policy",
        "raw_predictions": metric["raw_prediction_count"] > 0,
        "applied_blocks": metric["command_blocks"] > 0,
        "started_time": metric["first_started_backend_time_ns"] is not None,
    }
    metric["act_evidence"] = evidence
    metric["act_executed"] = bool(all(evidence.values()))
    metric["closed_loop"] = bool(metric["closed_loop"] and metric["act_executed"])
    if not metric["act_executed"]:
        missing = sorted(key for key, value in evidence.items() if not value)
        metric["issues"].extend(f"ACT_EVIDENCE_MISSING:{key}" for key in missing)


def _apply_task_issues(metric: dict[str, Any]) -> None:
    if metric["task_success"]:
        return
    missing_task = []
    if not metric["task_evaluated"]:
        missing_task.append("evaluated")
    if metric["task_test_only"] is not False:
        missing_task.append("test_only")
    if metric["task_action_source"] != "policy":
        missing_task.append("action_source")
    if not metric["task_outcome"]:
        missing_task.append("outcome")
    if not metric["task_evidence"]:
        missing_task.append("task_fact")
    metric["issues"].extend(f"TASK_EVIDENCE_MISSING:{key}" for key in missing_task)


def _success_flags(metric: dict[str, Any]) -> None:
    metric["success"] = bool(
        metric["act_executed"] and metric["closed_loop"] and metric["task_success"]
    )
    metric["run_success"] = bool(
        metric["success"]
        and metric["runtime_outcome"] in {"stopped", "completed"}
        and metric["recording_complete"]
        and metric["quality_qualified"]
        and metric["launch_ok"]
    )


def _failure_flags(metric: dict[str, Any]) -> None:
    if not metric["act_executed"]:
        metric["failure_type"] = "ACT_EVIDENCE_INCOMPLETE"
    elif not metric["closed_loop"]:
        metric["failure_type"] = "NO_APPLIED_ACT_BLOCK"
    elif not metric["task_success"]:
        metric["failure_type"] = "TASK_FAILED" if metric["task_evidence"] else "TASK_NOT_EVALUATED"
    elif metric["runtime_outcome"] not in {"stopped", "completed"}:
        metric["failure_type"] = "RUNTIME_NOT_STOPPED"
    elif not metric["recording_complete"]:
        metric["failure_type"] = "RECORDING_INCOMPLETE"
    elif not metric["quality_qualified"]:
        metric["failure_type"] = "QUALITY_REJECTED"
    elif not metric["launch_ok"]:
        metric["failure_type"] = "LAUNCH_FAILED"
    else:
        metric["failure_type"] = ""
    if metric["runtime_reason"] and metric["runtime_outcome"] not in {"stopped", "completed"}:
        metric["failure_detail"] = metric["runtime_reason"]
    if not metric["failure_detail"]:
        metric["failure_detail"] = (
            metric.get("runtime_reason")
            or metric.get("task_reason")
            or ", ".join(metric["issues"][:3])
            or ""
        )


def metric_for_seed(
    project_root: Path,
    config: dict[str, Any],
    seed: int,
    record: dict[str, Any] | None,
    terminal: dict[str, Any] | None,
) -> dict[str, Any]:
    del project_root, config
    metric = _base_metric(seed, record, terminal)
    if record is None:
        metric["failure_type"] = "MISSING_RESERVATION"
        metric["failure_detail"] = "seed was not reserved in the evaluation ledger"
        return metric
    attempt = Path(str(record.get("attempt_dir", "")))
    if not attempt.is_dir():
        metric["failure_type"] = "MISSING_ATTEMPT_DIRECTORY"
        metric["failure_detail"] = str(attempt)
        return metric
    manifest = _json(attempt / "raw" / "manifest.json")
    task = _json(attempt / "raw" / "task_result.json")
    export = _find_export(attempt)
    quality = _json(export / "quality.json") if export is not None else None
    labels_document, labels = _labels(export)
    metric["manifest"] = str(attempt / "raw" / "manifest.json") if manifest else ""
    metric["export"] = str(export) if export else ""
    metric["recording_path"] = str(attempt)
    if terminal is not None:
        _apply_terminal(metric, record, terminal)
    _apply_manifest(metric, manifest)
    _apply_task(metric, task)
    _apply_quality(metric, quality)
    _apply_export(metric, export, labels_document, labels)
    _apply_act_evidence(metric)
    _apply_task_issues(metric)
    _success_flags(metric)
    _failure_flags(metric)
    return metric


__all__ = [
    "digest",
    "metric_for_seed",
]
