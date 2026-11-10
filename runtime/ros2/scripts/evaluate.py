#!/usr/bin/env python3
"""Read-only ACT evaluation report and three-view evidence builder.

The ledger is never modified: this tool reads the append-only attempt records,
per-attempt manifests/exports and ACT policy event logs, then writes a separate
report directory.  Every fixed test seed is listed, including missing, rejected,
timed-out and failed attempts.  A missing prediction/application record is
reported as an incomplete closed loop, never as an executed ACT rollout.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "runtime/ros2/scripts"))
import campaign  # noqa: E402

CAMERA_ORDER = ("global", "left_wrist", "right_wrist")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_AXIS_NAME = re.compile(r"\b(?:left|right)_(?:joint[1-7]|gripper_width)\b")
_NUMBER = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
_BACKEND_REJECT_EVENTS = frozenset(
    {"backend.submit.rejected", "backend.submit_follower.rejected"}
)
_BACKEND_SUBMIT_EVENTS = frozenset({"backend.submit", "backend.submit_follower"})


def _digest(value: object) -> str:
    return value if isinstance(value, str) and _SHA256.fullmatch(value) else ""


def _json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _events(directory: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    if not directory.is_dir():
        return result
    for path in sorted(directory.glob("events.*.jsonl")):
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            text = line.strip()
            if not text.startswith("{"):
                continue
            try:
                event = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                result.append(event)
    return result


def _backend_rejection_events(attempt: Path) -> list[tuple[Path, dict[str, Any]]]:
    """Return only the backend's own structured submit-rejection events.

    Backend logs live under the attempt's backend run directory, but the exact
    run-id nesting is deployment-owned.  Searching the attempt for jsonl event
    files and then filtering by logger component/event name keeps this reader
    independent of that nesting.  Unreadable files and malformed lines are
    ignored: a damaged log must not prevent the evaluation report from being
    produced, and no axis is inferred without an explicit backend message.
    """
    result: list[tuple[Path, dict[str, Any]]] = []
    seen: set[Path] = set()
    for path in sorted(attempt.rglob("events.*.jsonl")):
        try:
            resolved = path.resolve()
        except OSError:
            resolved = path
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            text = line.strip()
            if not text.startswith("{"):
                continue
            try:
                event = json.loads(text)
            except json.JSONDecodeError:
                continue
            if not isinstance(event, dict):
                continue
            name = str(event.get("event") or "")
            if (
                event.get("component") != "backend"
                and not name.startswith("backend.")
            ):
                continue
            if name in _BACKEND_REJECT_EVENTS:
                result.append((path, event))
            elif name in _BACKEND_SUBMIT_EVENTS and str(
                event.get("message") or ""
            ).lower() == "rejected":
                result.append((path, event))
    return result


def _controller_rejections(attempt: Path) -> list[dict[str, Any]]:
    """Project backend rejections to a bounded, machine-readable record list."""
    result: list[dict[str, Any]] = []
    for path, event in _backend_rejection_events(attempt):
        fields = event.get("fields")
        if not isinstance(fields, dict):
            fields = {}
        record: dict[str, Any] = {
            "reason": str(event.get("reason") or ""),
            "message": str(event.get("message") or ""),
            "event": str(event.get("event") or ""),
            "path": str(path),
        }
        for key in ("command_id", "request_id"):
            value = event.get(key) or fields.get(key)
            if isinstance(value, str) and value:
                record[key] = value
        for key in ("event_seq", "sim_time_ns"):
            value = event.get(key)
            if isinstance(value, int) and not isinstance(value, bool):
                record[key] = value
        for key in ("component", "instance_id"):
            value = event.get(key)
            if isinstance(value, str) and value:
                record[key] = value
        result.append(record)
    return result


def _backend_failure_detail(rejections: list[dict[str, Any]]) -> str:
    """Prefer an axis/numeric backend reason over the generic submit error."""
    if not rejections:
        return ""

    def score(record: dict[str, Any]) -> int:
        detail = f"{record.get('reason', '')} {record.get('message', '')}"
        value = 0
        if _AXIS_NAME.search(detail):
            value += 4
        if "axis" in detail.lower():
            value += 2
        if _NUMBER.search(detail):
            value += 1
        return value

    best = max(rejections, key=score)
    if score(best) < 2:
        return ""
    reason = str(best.get("reason") or "")
    message = str(best.get("message") or "")
    if reason and message and reason not in message:
        return f"{reason}: {message}"
    return message or reason


def _prediction_metrics(directory: Path) -> dict[str, Any]:
    events = _events(directory)
    latencies: list[int] = []
    vram: list[float] = []
    rejects: list[str] = []
    ready_fields: dict[str, Any] = {}
    finished_fields: list[dict[str, Any]] = []
    for event in events:
        name = event.get("event")
        fields = event.get("fields")
        if not isinstance(fields, dict):
            fields = {}
        if name == "act.model.ready":
            ready_fields = dict(fields)
        if name == "act.predict.finished":
            finished_fields.append(dict(fields))
            value = fields.get("latency_ns")
            if isinstance(value, int) and value >= 0:
                latencies.append(value)
            value = fields.get("vram_peak_mib")
            if isinstance(value, int | float) and value >= 0:
                vram.append(float(value))
        elif name in {"act.predict.rejected", "policy.rpc.rejected"}:
            reason = str(fields.get("reason") or "")
            if reason:
                rejects.append(reason)

    def percentile(fraction: float) -> float | None:
        if not latencies:
            return None
        ordered = sorted(latencies)
        position = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
        return round(ordered[position] / 1_000_000, 3)

    return {
        "predictions": len(latencies),
        "rejected": len(rejects),
        "reject_reasons": sorted(set(rejects)),
        "p50_ms": percentile(0.50),
        "p95_ms": percentile(0.95),
        "max_ms": round(max(latencies) / 1_000_000, 3) if latencies else None,
        "mean_ms": round(statistics.fmean(latencies) / 1_000_000, 3) if latencies else None,
        "vram_peak_mib": max(vram) if vram else None,
        "ready": any(
            event.get("event") in {"act.model.ready", "policy.ready"} for event in events
        ),
        "ready_fields": ready_fields,
        "finished_fields": finished_fields,
    }


def _find_export(attempt: Path, launch: dict[str, Any] | None) -> Path | None:
    if isinstance(launch, dict):
        value = launch.get("export_directory")
        if isinstance(value, str) and Path(value).is_dir():
            return Path(value)
    candidates = sorted(attempt.glob("stack/*/export"))
    return candidates[0] if len(candidates) == 1 else None


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


def _metric_for_seed(
    project_root: Path,
    config: dict[str, Any],
    seed: int,
    record: dict[str, Any] | None,
    terminal: dict[str, Any] | None,
) -> dict[str, Any]:
    del project_root, config
    result: dict[str, Any] = {
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
        "first_started_sim_time_ns": None,
        "first_reference_applied_sim_time_ns": None,
        "first_exported_label_sim_time_ns": None,
        "last_exported_label_sim_time_ns": None,
        "label_period_ns": None,
        "first_exported_label_exact": False,
        "command_blocks": 0,
        "raw_prediction_count": 0,
        "prediction_count": 0,
        "prediction_launch_count": 0,
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
        "controller_rejections": [],
        "recording_path": "",
        "preview_video": "",
        "preview_keyframes": [],
        "manifest": "",
        "export": "",
        "issues": [],
    }
    if record is None:
        result["failure_type"] = "MISSING_RESERVATION"
        result["failure_detail"] = "seed was not reserved in the evaluation ledger"
        return result
    attempt = Path(str(record.get("attempt_dir", "")))
    if not attempt.is_dir():
        result["failure_type"] = "MISSING_ATTEMPT_DIRECTORY"
        result["failure_detail"] = str(attempt)
        return result
    result["controller_rejections"] = _controller_rejections(attempt)
    launch = _json(attempt / "launch_result.json")
    manifest = _json(attempt / "manifest.json")
    task = _json(attempt / "task_result.json")
    quality = _json(attempt / "quality.json")
    export = _find_export(attempt, launch)
    labels_document, labels = _labels(export)
    result["manifest"] = str(attempt / "manifest.json") if manifest else ""
    result["export"] = str(export) if export else ""
    result["recording_path"] = str(attempt)
    if terminal is not None:
        result["terminal_event"] = str(terminal.get("event", ""))
        result["wall_seconds"] = terminal.get("wall_s")
        result["failure_detail"] = str(terminal.get("failure", ""))
        terminal_mode = terminal.get("mode")
        if terminal_mode is not None and str(terminal_mode) != str(record.get("mode", "")):
            result["issues"].append("TERMINAL_MODE_MISMATCH")

    launch_act: dict[str, Any] = {}
    if launch is not None:
        value = launch.get("act")
        if isinstance(value, dict):
            launch_act = value
        result["launch_policy_mode"] = str(launch.get("policy_mode", ""))
        result["launch_exit_code"] = launch.get("exit_code")
        result["launcher_exit_code"] = launch.get("launcher_exit_code")
        result["launch_ok"] = bool(
            launch.get("exit_code") == 0 and launch.get("launcher_exit_code") == 0
        )
        result["bundle_digest"] = _digest(launch_act.get("bundle_digest")) or None
        result["processor_digest"] = _digest(launch_act.get("processor_digest")) or None
        result["profile_semantic_sha256"] = (
            _digest(launch_act.get("profile_semantic_sha256")) or None
        )
        result["observation_schema_hash"] = (
            _digest(launch_act.get("observation_schema_hash")) or None
        )
        result["action_schema_hash"] = _digest(launch_act.get("action_schema_hash")) or None
        result["checkpoint_step"] = launch_act.get("checkpoint_step")
        metrics = launch.get("act_metrics")
        if isinstance(metrics, dict):
            latency = metrics.get("latency_ms")
            if isinstance(latency, dict):
                result["prediction_p50_ms"] = latency.get("p50")
                result["prediction_p95_ms"] = latency.get("p95")
                result["prediction_max_ms"] = latency.get("max")
            value = metrics.get("predictions")
            if isinstance(value, int) and value >= 0:
                result["prediction_launch_count"] = value
            result["vram_peak_mib"] = metrics.get("vram_peak_mib")

    policy_log_metrics: dict[str, Any] = {}
    for candidate in sorted((attempt / "stack").glob("*/act-policy-log")):
        policy_log_metrics = _prediction_metrics(candidate)
        if (
            policy_log_metrics["predictions"]
            or policy_log_metrics["rejected"]
            or policy_log_metrics["ready_fields"]
        ):
            break
    if policy_log_metrics:
        result["prediction_count"] = int(policy_log_metrics.get("predictions", 0) or 0)
        result["prediction_p50_ms"] = policy_log_metrics.get("p50_ms")
        result["prediction_p95_ms"] = policy_log_metrics.get("p95_ms")
        result["prediction_max_ms"] = policy_log_metrics.get("max_ms")
        result["vram_peak_mib"] = policy_log_metrics.get("vram_peak_mib")
        result["reject_reasons"] = policy_log_metrics.get("reject_reasons", [])
        result["ready"] = policy_log_metrics.get("ready", False)
        if (
            result["prediction_launch_count"]
            and result["prediction_count"] != result["prediction_launch_count"]
        ):
            result["issues"].append("PREDICTION_COUNT_MISMATCH")

    if manifest is not None:
        runtime = manifest.get("runtime")
        if isinstance(runtime, dict):
            result["runtime_outcome"] = str(runtime.get("outcome", ""))
            result["runtime_reason"] = str(runtime.get("reason", ""))
        recording = manifest.get("recording")
        result["recording_complete"] = bool(
            isinstance(recording, dict) and recording.get("complete")
        )
        configuration = manifest.get("configuration")
        if isinstance(configuration, dict):
            result["action_source"] = str(configuration.get("action_source", ""))
        started = manifest.get("started_at_ns")
        ended = manifest.get("ended_at_ns")
        if isinstance(started, int) and isinstance(ended, int) and ended >= started:
            result["wall_seconds"] = round((ended - started) / 1e9, 6)

    if task is not None:
        result["task_evaluated"] = task.get("evaluated") is True
        result["task_test_only"] = task.get("test_only")
        result["task_action_source"] = str(task.get("action_source", ""))
        task_result = task.get("result")
        if isinstance(task_result, dict):
            result["task_outcome"] = str(task_result.get("outcome", ""))
            result["task_reason"] = str(task_result.get("reason", ""))
        fact = task.get("task")
        if isinstance(fact, dict):
            result["task_evidence"] = dict(fact)
            start_ns = fact.get("task_start_sim_time_ns")
            completed_ns = fact.get("completed_sim_time_ns")
            sample_ns = fact.get("sample_sim_time_ns")
            end_ns = (
                completed_ns
                if isinstance(completed_ns, int) and completed_ns > 0
                else sample_ns
            )
            if (
                isinstance(start_ns, int)
                and start_ns > 0
                and isinstance(end_ns, int)
                and end_ns >= start_ns
            ):
                result["task_duration_sim_seconds"] = round((end_ns - start_ns) / 1e9, 6)
            result["task_success"] = bool(
                result["task_evaluated"]
                and result["task_test_only"] is False
                and result["task_action_source"] == "policy"
                and result["task_outcome"] == "succeeded"
                and result["task_reason"] == "TASK_SUCCEEDED"
                and fact.get("state") == "SUCCEEDED"
                and fact.get("conditions_passed") is True
                and isinstance(start_ns, int)
                and start_ns > 0
                and isinstance(completed_ns, int)
                and completed_ns >= start_ns
                and isinstance(sample_ns, int)
                and sample_ns >= completed_ns
                and isinstance(fact.get("stable_elapsed_s"), int | float)
                and float(fact["stable_elapsed_s"]) >= 2.0
                and bool(fact.get("task_config_digest"))
                and bool(fact.get("plan_digest"))
            )
        else:
            result["issues"].append("TASK_FACT_MISSING")

    if quality is not None:
        result["quality_qualified"] = quality.get("qualified") is True
        failures = quality.get("failure_reasons")
        if isinstance(failures, list) and failures:
            result["quality_failures"] = [str(item) for item in failures]

    raw_predictions: list[dict[str, Any]] = []
    if export is not None:
        raw_document = _json(export / "raw_predictions.json")
        rows = raw_document.get("predictions") if isinstance(raw_document, dict) else None
        if isinstance(rows, list):
            raw_predictions = [row for row in rows if isinstance(row, dict)]
        result["raw_prediction_count"] = len(raw_predictions)

    if labels:
        started = [
            int(row["started_sim_time_ns"])
            for row in labels
            if isinstance(row.get("started_sim_time_ns"), int)
        ]
        exported = [
            int(row["label_sim_time_ns"])
            for row in labels
            if isinstance(row.get("label_sim_time_ns"), int)
        ]
        observations = [
            int(row["observation_sim_time_ns"])
            for row in labels
            if isinstance(row.get("observation_sim_time_ns"), int)
        ]
        result["first_started_sim_time_ns"] = min(started) if started else None
        # STARTED is the audited physical chunk boundary. Raw predictions can
        # precede that boundary and never prove that their first target ran.
        result["first_reference_applied_sim_time_ns"] = min(started) if started else None
        result["first_exported_label_sim_time_ns"] = min(exported) if exported else None
        result["last_exported_label_sim_time_ns"] = max(exported) if exported else None
        periods = [
            int(row["observation_period_ns"])
            for row in labels
            if isinstance(row.get("observation_period_ns"), int)
        ]
        result["label_period_ns"] = min(periods) if periods else None
        result["first_exported_label_exact"] = False
        if exported and observations:
            result["execution_span_seconds"] = round(
                (max(exported) - min(observations)) / 1e9, 6
            )
            result["sim_seconds"] = result["execution_span_seconds"]
        blocks = {
            (row.get("chunk_seq"), row.get("command_id"))
            for row in labels
            if row.get("chunk_seq") is not None and row.get("command_id")
        }
        result["command_blocks"] = len(blocks)
        if raw_predictions:
            first_prediction = min(
                raw_predictions,
                key=lambda row: int(row.get("source_observation_time_ns", 2**63 - 1)),
            )
            offsets = first_prediction.get("times_from_observation_ns")
            source_ns = first_prediction.get("source_observation_time_ns")
            if isinstance(offsets, list) and offsets and isinstance(source_ns, int):
                result["first_predicted_target_sim_time_ns"] = source_ns + int(
                    min(int(value) for value in offsets)
                )
        result["closed_loop"] = bool(
            result["command_blocks"] > 0
            and result["first_started_sim_time_ns"] is not None
            and result["raw_prediction_count"] > 0
            and result["recording_complete"]
        )
    elif labels_document is not None:
        result["issues"].append("LABELS_EMPTY")

    evidence = {
        "record_mode_act": record.get("mode") == "act",
        "record_kind_rollout": record.get("kind") == "rollout",
        "launch_policy_mode_act": result.get("launch_policy_mode") == "act",
        "launch_action_source_policy": result.get("action_source") == "policy",
        "launch_act_summary": bool(launch_act),
        "bundle_digest": bool(result.get("bundle_digest")),
        "processor_digest": bool(result.get("processor_digest")),
        "profile_semantic_sha256": bool(result.get("profile_semantic_sha256")),
        "observation_schema_hash": bool(result.get("observation_schema_hash")),
        "action_schema_hash": bool(result.get("action_schema_hash")),
        "bundle_not_synthetic": launch_act.get("synthetic") is False,
        "checkpoint_full": launch_act.get("checkpoint_mode") == "full",
        "model_sha256": bool(_digest(launch_act.get("model_sha256"))),
        "policy_ready": bool(result.get("ready")),
        "successful_predictions": result["prediction_count"] > 0,
        "raw_predictions": result["raw_prediction_count"] > 0,
        "applied_blocks": result["command_blocks"] > 0,
        "started_time": result["first_started_sim_time_ns"] is not None,
    }
    ready_fields = policy_log_metrics.get("ready_fields") if policy_log_metrics else {}
    if not isinstance(ready_fields, dict):
        ready_fields = {}
    evidence["log_bundle_digest_matches"] = bool(
        result.get("bundle_digest")
        and _digest(ready_fields.get("bundle_digest"))
        and ready_fields.get("bundle_digest") == result.get("bundle_digest")
    )
    evidence["log_processor_digest_matches"] = bool(
        result.get("processor_digest")
        and _digest(ready_fields.get("processor_digest"))
        and ready_fields.get("processor_digest") == result.get("processor_digest")
    )
    evidence["log_profile_matches"] = bool(
        result.get("profile_semantic_sha256")
        and ready_fields.get("profile_semantic_sha256")
        == result.get("profile_semantic_sha256")
    )
    evidence["log_model_sha_matches"] = bool(
        _digest(launch_act.get("model_sha256"))
        and ready_fields.get("model_sha256") == launch_act.get("model_sha256")
    )
    evidence["log_synthetic_false"] = ready_fields.get("synthetic") is False
    evidence["log_checkpoint_full"] = ready_fields.get("checkpoint_mode") == "full"
    evidence["log_observation_schema_matches"] = bool(
        result.get("observation_schema_hash")
        and ready_fields.get("observation_schema_hash") == result.get("observation_schema_hash")
    )
    evidence["log_action_schema_matches"] = bool(
        result.get("action_schema_hash")
        and ready_fields.get("action_schema_hash") == result.get("action_schema_hash")
    )
    finished_fields = (
        policy_log_metrics.get("finished_fields") if policy_log_metrics else []
    )
    evidence["prediction_chunks_60"] = bool(
        isinstance(finished_fields, list)
        and finished_fields
        and all(
            fields.get("chunk_size") == 60
            for fields in finished_fields
            if isinstance(fields, dict)
        )
    )
    result["act_evidence"] = evidence
    result["act_executed"] = bool(all(evidence.values()))
    result["closed_loop"] = bool(result["closed_loop"] and result["act_executed"])
    if not result["act_executed"]:
        missing = sorted(key for key, value in evidence.items() if not value)
        result["issues"].extend(f"ACT_EVIDENCE_MISSING:{key}" for key in missing)

    if not result["task_success"]:
        missing_task = []
        if not result["task_evaluated"]:
            missing_task.append("evaluated")
        if result["task_test_only"] is not False:
            missing_task.append("test_only")
        if result["task_action_source"] != "policy":
            missing_task.append("action_source")
        if not result["task_outcome"]:
            missing_task.append("outcome")
        if not result["task_evidence"]:
            missing_task.append("task_fact")
        result["issues"].extend(f"TASK_EVIDENCE_MISSING:{key}" for key in missing_task)

    result["success"] = bool(
        result["act_executed"] and result["closed_loop"] and result["task_success"]
    )
    result["run_success"] = bool(
        result["success"]
        and result["runtime_outcome"] in {"stopped", "completed"}
        and result["recording_complete"]
        and result["quality_qualified"]
        and result["launch_ok"]
    )

    if not result["act_executed"]:
        result["failure_type"] = "ACT_EVIDENCE_INCOMPLETE"
    elif not result["closed_loop"]:
        result["failure_type"] = "NO_APPLIED_ACT_BLOCK"
    elif not result["task_success"]:
        if result["task_evidence"]:
            result["failure_type"] = "TASK_FAILED"
        else:
            result["failure_type"] = "TASK_NOT_EVALUATED"
    elif result["runtime_outcome"] not in {"stopped", "completed"}:
        result["failure_type"] = "RUNTIME_NOT_STOPPED"
    elif not result["recording_complete"]:
        result["failure_type"] = "RECORDING_INCOMPLETE"
    elif not result["quality_qualified"]:
        result["failure_type"] = "QUALITY_REJECTED"
    elif not result["launch_ok"]:
        result["failure_type"] = "LAUNCH_FAILED"
    else:
        result["failure_type"] = ""
    backend_detail = _backend_failure_detail(result["controller_rejections"])
    if backend_detail and not result["success"]:
        result["failure_detail"] = backend_detail
    elif result["runtime_reason"] and result["runtime_outcome"] not in {"stopped", "completed"}:
        # A controller failure is more actionable than the launcher's exit 1.
        result["failure_detail"] = result["runtime_reason"]
    if not result["failure_detail"]:
        result["failure_detail"] = (
            result.get("runtime_reason")
            or result.get("task_reason")
            or ", ".join(result.get("reject_reasons", []))
            or ", ".join(result["issues"][:3])
            or ""
        )
    return result


def _frame_rows(export: Path, limit: int = 120) -> list[tuple[int, list[Path]]]:
    index = _json(export / "frame_index.json")
    labels = _json(export / "labels.json")
    if index is None or labels is None:
        return []
    frame_index = index.get("labels")
    rows = labels.get("labels")
    if not isinstance(frame_index, dict) or not isinstance(rows, list):
        return []
    camera_order = index.get("camera_order")
    if camera_order != list(CAMERA_ORDER):
        return []
    result: list[tuple[int, list[Path]]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        sequences = row.get("frame_seqs")
        label_time = row.get("label_sim_time_ns")
        if not isinstance(sequences, list) or len(sequences) != 3:
            continue
        if not isinstance(label_time, int):
            continue
        paths: list[Path] = []
        for camera_index, camera in enumerate(CAMERA_ORDER):
            entry = frame_index.get(str(sequences[camera_index]))
            if not isinstance(entry, dict):
                paths = []
                break
            record = entry.get(camera)
            if not isinstance(record, dict) or not isinstance(record.get("path"), str):
                paths = []
                break
            paths.append(export / record["path"])
        if len(paths) == 3 and all(path.is_file() for path in paths):
            result.append((label_time, paths))
    if len(result) > limit:
        step = max(1, math.ceil(len(result) / limit))
        result = result[::step][:limit]
    return result


def _montage(paths: list[Path], caption: str) -> Image.Image | None:
    try:
        images = [Image.open(path).convert("RGB").resize((320, 240)) for path in paths]
    except OSError:
        return None
    width = 320 * len(images)
    canvas = Image.new("RGB", (width, 270), (0, 0, 0))
    for index, image in enumerate(images):
        canvas.paste(image, (index * 320, 30))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 8), caption, fill=(255, 255, 255))
    for index, camera in enumerate(CAMERA_ORDER[: len(images)]):
        draw.text((index * 320 + 8, 250), camera, fill=(255, 255, 255))
    return canvas


def _triptych(paths: list[Path], out: Path, caption: str) -> bool:
    canvas = _montage(paths, caption)
    if canvas is None:
        return False
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out)
    return True


def _write_video(export: Path, out: Path, max_frames: int = 80) -> str:
    rows = _frame_rows(export, limit=max_frames)
    if not rows:
        return ""
    out.parent.mkdir(parents=True, exist_ok=True)
    if shutil.which("ffmpeg"):
        temporary = Path(tempfile.mkdtemp(prefix="act-eval-"))
        try:
            for index, (_label_time, paths) in enumerate(rows):
                _triptych(paths, temporary / f"{index:06d}.png", f"frame {index}")
            command = [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                "8",
                "-i",
                str(temporary / "%06d.png"),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(out),
            ]
            completed = subprocess.run(command, capture_output=True, timeout=60)
            if completed.returncode == 0 and out.is_file() and out.stat().st_size > 0:
                return str(out)
        except (OSError, subprocess.SubprocessError):
            pass
        finally:
            shutil.rmtree(temporary, ignore_errors=True)
    images = []
    for label_time, paths in rows:
        canvas = _montage(paths, f"t={label_time} ns")
        if canvas is not None:
            images.append(canvas)
    if not images:
        return ""
    gif = out.with_suffix(".gif")
    images[0].save(
        gif, save_all=True, append_images=images[1:], duration=125, loop=0
    )
    return str(gif)


def _keyframes(export: Path, out_dir: Path, prefix: str) -> list[str]:
    rows = _frame_rows(export, limit=1000)
    if not rows:
        return []
    positions = sorted({0, len(rows) // 2, len(rows) - 1})
    result = []
    for index in positions:
        label_time, paths = rows[index]
        out = out_dir / f"{prefix}-{index:04d}.png"
        if _triptych(paths, out, f"{prefix} t={label_time} ns"):
            result.append(str(out))
    return result


def _media_for(metric: dict[str, Any], output: Path) -> None:
    export = Path(metric["export"]) if metric["export"] else None
    if export is None or not export.is_dir():
        return
    preview = output / "preview"
    prefix = f"seed-{metric['seed']:05d}"
    video = _write_video(export, preview / f"{prefix}-three-view.mp4")
    metric["preview_video"] = video
    metric["preview_keyframes"] = _keyframes(export, preview, prefix)


def _markdown(metrics: list[dict[str, Any]], summary: dict[str, Any]) -> str:
    lines = [
        "# ACT evaluation report",
        "",
        "Read-only reconstruction from the append-only campaign ledger.  All fixed "
        "test seeds are counted; missing/rejected/timeout rows are not removed.",
        "",
        "`first label` is the 20 Hz exported label sampling time, not a 2 ms physical "
        "application instant; `first started` is the recorded block start.",
        "",
        f"- seeds: `{summary['seed_first']}..{summary['seed_last']}`",
        f"- denominator: {summary['denominator']}",
        f"- reservations: {summary['reserved']}",
        f"- terminal attempts: {summary['terminal']}",
        f"- credible ACT executions: {summary['act_executed']}",
        f"- closed loop: {summary['closed_loop']}",
        f"- physical task success: {summary['task_success']}",
        f"- physical success (ACT + task): {summary['success']}",
        f"- full run success (also runtime/recording/quality/launch): {summary['run_success']}",
        f"- quality qualified: {summary['quality_qualified']}",
        f"- ACT evidence incomplete: {summary['act_evidence_incomplete']}",
        f"- task evidence incomplete: {summary['task_evidence_incomplete']}",
        f"- errors/issues: {summary['issues']}",
        "",
        "| seed | attempt | task | task eval | ACT | closed | runtime | rec | launch | "
        "quality | physical success | run success | first started (s) | first label (s) | "
        "blocks | task sim (s) | exec span (s) | wall (s) | P50/P95/max (ms) | "
        "VRAM (MiB) | failure | evidence |",
        "| ---: | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
        "---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- | --- | --- |",
    ]
    for metric in metrics:
        first = metric["first_started_sim_time_ns"]
        first_s = "" if first is None else f"{first / 1e9:.3f}"
        label = metric["first_exported_label_sim_time_ns"]
        label_s = "" if label is None else f"{label / 1e9:.3f}"
        latency = "/".join(
            "" if metric.get(key) is None else str(metric.get(key))
            for key in ("prediction_p50_ms", "prediction_p95_ms", "prediction_max_ms")
        )
        evidence = metric.get("preview_video") or metric.get("recording_path") or ""
        lines.append(
            "| {seed} | {attempt} | {task} | {eval} | {act} | {closed} | {runtime} | "
            "{rec} | {launch} | {quality} | {physical} | {run} | {first} | {label} | "
            "{blocks} | {task_sim} | {exec} | {wall} | {latency} | {vram} | "
            "{failure} | {evidence} |".format(
                seed=metric["seed"],
                attempt=metric.get("attempt_id") or "-",
                task=metric.get("task_outcome") or "-",
                eval="yes" if metric.get("task_evaluated") else "no",
                act="yes" if metric.get("act_executed") else "no",
                closed="yes" if metric.get("closed_loop") else "no",
                runtime=metric.get("runtime_outcome") or "-",
                rec="yes" if metric.get("recording_complete") else "no",
                launch="yes" if metric.get("launch_ok") else "no",
                quality="yes" if metric.get("quality_qualified") else "no",
                physical="yes" if metric.get("success") else "no",
                run="yes" if metric.get("run_success") else "no",
                first=first_s or "-",
                label=label_s or "-",
                blocks=metric.get("command_blocks", 0),
                task_sim=""
                if metric.get("task_duration_sim_seconds") is None
                else metric["task_duration_sim_seconds"],
                exec=""
                if metric.get("execution_span_seconds") is None
                else metric["execution_span_seconds"],
                wall="" if metric.get("wall_seconds") is None else metric["wall_seconds"],
                latency=latency or "-",
                vram="" if metric.get("vram_peak_mib") is None else metric["vram_peak_mib"],
                failure=metric.get("failure_type") or "ok",
                evidence=evidence,
            )
        )
    return "\n".join(lines) + "\n"


def build_report(
    *,
    project_root: Path,
    config_path: Path,
    purposes: tuple[str, ...] = ("test", "evaluation"),
    output: Path,
    generate_media: bool = True,
) -> dict[str, Any]:
    config = campaign._load(config_path)
    paths = config["paths"]
    ledger = campaign.Ledger(
        project_root / paths["ledger"], project_root / paths["lock"]
    )
    with ledger.locked():
        records = ledger.records()
    terminal = campaign._finished(records)
    seed_first = int(config["budget"]["test_seed_first"])
    seed_last = int(config["budget"]["test_seed_last"])
    by_seed: dict[int, dict[str, Any]] = {}
    duplicates: list[int] = []
    for purpose in purposes:
        for record in campaign._attempts(records, purpose):
            seed = int(record.get("seed", -1))
            if seed in by_seed:
                duplicates.append(seed)
                continue
            by_seed[seed] = record
    metrics = [
        _metric_for_seed(
            project_root,
            config,
            seed,
            by_seed.get(seed),
            terminal.get(str(by_seed[seed]["attempt_id"])) if seed in by_seed else None,
        )
        for seed in range(seed_first, seed_last + 1)
    ]
    if generate_media:
        for metric in metrics:
            if metric.get("closed_loop"):
                _media_for(metric, output)
            elif metric.get("export"):
                metric["preview_keyframes"] = _keyframes(
                    Path(metric["export"]), output / "preview", f"seed-{metric['seed']:05d}"
                )
    model_digests = sorted(
        {str(item["bundle_digest"]) for item in metrics if item.get("bundle_digest")}
    )
    checkpoint_steps = sorted(
        {int(item["checkpoint_step"]) for item in metrics if item.get("checkpoint_step")}
    )
    issues = {
        item["failure_type"]
        for item in metrics
        if item["failure_type"] in {"MISSING_RESERVATION", "MISSING_ATTEMPT_DIRECTORY"}
    }
    if len(model_digests) > 1:
        issues.add("MULTIPLE_MODEL_DIGESTS")
    if len(checkpoint_steps) > 1:
        issues.add("MULTIPLE_CHECKPOINT_STEPS")
    summary = {
        "seed_first": seed_first,
        "seed_last": seed_last,
        "denominator": len(metrics),
        "reserved": sum(1 for item in metrics if item["reserved"]),
        "terminal": sum(1 for item in metrics if item["terminal"]),
        "act_executed": sum(1 for item in metrics if item["act_executed"]),
        "closed_loop": sum(1 for item in metrics if item["closed_loop"]),
        "task_success": sum(1 for item in metrics if item.get("task_success")),
        "success": sum(1 for item in metrics if item.get("success")),
        "run_success": sum(1 for item in metrics if item.get("run_success")),
        "quality_qualified": sum(1 for item in metrics if item.get("quality_qualified")),
        "act_evidence_incomplete": sum(1 for item in metrics if not item["act_executed"]),
        "task_evidence_incomplete": sum(
            1 for item in metrics if not item.get("task_evaluated") or not item.get("task_evidence")
        ),
        "duplicate_seeds": sorted(set(duplicates)),
        "model_digests": model_digests,
        "checkpoint_steps": checkpoint_steps,
        "issues": sorted(issues),
    }
    document = {
        "schema_version": 1,
        "ledger": str(project_root / paths["ledger"]),
        "purposes": list(purposes),
        "summary": summary,
        "attempts": metrics,
        "notes": [
            "fixed test seeds are never replaced or re-run to select a better result",
            "a missing prediction/application record is reported as an incomplete closed loop",
            "physical success requires credible ACT execution plus independent physical task facts",
            "quality is reported separately and does not by itself define physical task success",
            "previews are generated from recorded RGB only and never fed back to policy/training",
        ],
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "evaluation.json").write_text(
        json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output / "report.md").write_text(_markdown(metrics, summary), encoding="utf-8")
    return document


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("runtime/ros2/configs/experiments/campaign.yaml"),
    )
    parser.add_argument("--purpose", choices=("all", "test", "evaluation"), default="all")
    parser.add_argument(
        "--output", type=Path, default=Path(".build/reports/dsh-036/evaluation")
    )
    parser.add_argument("--no-media", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    try:
        document = build_report(
            project_root=args.project_root.resolve(),
            config_path=(
                args.config if args.config.is_absolute() else args.project_root / args.config
            ),
            purposes=(
                ("test", "evaluation") if args.purpose == "all" else (args.purpose,)
            ),
            output=args.output.resolve(),
            generate_media=not args.no_media,
        )
    except Exception as error:  # noqa: BLE001 - report failure must be visible
        print(f"evaluate: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(document["summary"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
