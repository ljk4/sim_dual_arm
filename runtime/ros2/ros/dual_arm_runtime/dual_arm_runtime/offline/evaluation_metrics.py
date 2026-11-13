"""Read-only per-seed metric reconstruction from campaign evidence."""

from __future__ import annotations

import json
import re
import statistics
from pathlib import Path
from typing import Any

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
# Fallback when a recording carries no profile snapshot: any profile-like axis
# token, not only the current scenario's left/right joint names.
_AXIS_NAME = re.compile(r"\b[A-Za-z][A-Za-z0-9]*_(?:joint\d+|gripper\w*)\b")
_NUMBER = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
_BACKEND_REJECT_EVENTS = frozenset(
    {"backend.submit.rejected", "backend.submit_follower.rejected"}
)
_BACKEND_SUBMIT_EVENTS = frozenset({"backend.submit", "backend.submit_follower"})


def digest(value: object) -> str:
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
            if event.get("component") != "backend" and not name.startswith("backend."):
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


def _axis_names(manifest: dict[str, Any] | None) -> tuple[str, ...]:
    """Read the recorded robot's joint names, when the snapshot carries them."""
    if not isinstance(manifest, dict):
        return ()
    configuration = manifest.get("configuration")
    if not isinstance(configuration, dict):
        return ()
    profile = configuration.get("profile")
    if not isinstance(profile, dict):
        return ()
    joints = profile.get("joints")
    if not isinstance(joints, list):
        return ()
    return tuple(
        str(joint["name"])
        for joint in joints
        if isinstance(joint, dict) and joint.get("name")
    )


def _backend_failure_detail(
    rejections: list[dict[str, Any]], axis_names: tuple[str, ...] = ()
) -> str:
    """Prefer an axis/numeric backend reason over the generic submit error."""
    if not rejections:
        return ""

    def mentions_axis(detail: str) -> bool:
        if axis_names:
            return any(
                re.search(rf"\b{re.escape(name)}\b", detail) is not None
                for name in axis_names
            )
        return _AXIS_NAME.search(detail) is not None

    def score(record: dict[str, Any]) -> int:
        detail = f"{record.get('reason', '')} {record.get('message', '')}"
        value = 0
        if mentions_axis(detail):
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
        "first_started_sim_time_ns": None,
        "first_reference_applied_sim_time_ns": None,
        "first_exported_label_sim_time_ns": None,
        "last_exported_label_sim_time_ns": None,
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
        "controller_rejections": [],
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


def _apply_policy_log(metric: dict[str, Any], attempt: Path) -> dict[str, Any]:
    policy_log_metrics: dict[str, Any] = {}
    candidate = attempt / "act-policy-log"
    if candidate.is_dir():
        policy_log_metrics = _prediction_metrics(candidate)
    if not policy_log_metrics:
        return policy_log_metrics
    ready_fields = policy_log_metrics.get("ready_fields")
    if not isinstance(ready_fields, dict):
        ready_fields = {}
    metric["bundle_digest"] = digest(ready_fields.get("bundle_digest")) or None
    metric["processor_digest"] = digest(ready_fields.get("processor_digest")) or None
    metric["profile_semantic_sha256"] = (
        digest(ready_fields.get("profile_semantic_sha256")) or None
    )
    metric["observation_schema_hash"] = (
        digest(ready_fields.get("observation_schema_hash")) or None
    )
    metric["action_schema_hash"] = digest(ready_fields.get("action_schema_hash")) or None
    metric["checkpoint_step"] = ready_fields.get("checkpoint_step")
    metric["prediction_count"] = int(policy_log_metrics.get("predictions", 0) or 0)
    metric["prediction_p50_ms"] = policy_log_metrics.get("p50_ms")
    metric["prediction_p95_ms"] = policy_log_metrics.get("p95_ms")
    metric["prediction_max_ms"] = policy_log_metrics.get("max_ms")
    metric["vram_peak_mib"] = policy_log_metrics.get("vram_peak_mib")
    metric["reject_reasons"] = policy_log_metrics.get("reject_reasons", [])
    metric["ready"] = policy_log_metrics.get("ready", False)
    return policy_log_metrics


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
        and isinstance(fact.get("stable_elapsed_s"), int | float)
        and float(fact["stable_elapsed_s"]) >= 2.0
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
    if labels:
        _apply_label_times(metric, labels, raw_predictions)
    elif labels_document is not None:
        metric["issues"].append("LABELS_EMPTY")
    return raw_predictions


def _apply_label_times(
    metric: dict[str, Any], labels: list[dict[str, Any]], raw_predictions: list[dict[str, Any]]
) -> None:
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
    metric["first_started_sim_time_ns"] = min(started) if started else None
    # STARTED is the audited physical chunk boundary. Raw predictions can
    # precede that boundary and never prove that their first target ran.
    metric["first_reference_applied_sim_time_ns"] = min(started) if started else None
    metric["first_exported_label_sim_time_ns"] = min(exported) if exported else None
    metric["last_exported_label_sim_time_ns"] = max(exported) if exported else None
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
            metric["first_predicted_target_sim_time_ns"] = source_ns + int(
                min(int(value) for value in offsets)
            )
    metric["closed_loop"] = bool(
        metric["command_blocks"] > 0
        and metric["first_started_sim_time_ns"] is not None
        and metric["raw_prediction_count"] > 0
        and metric["recording_complete"]
    )


def _apply_act_evidence(
    metric: dict[str, Any], policy_log_metrics: dict[str, Any]
) -> None:
    ready_fields = policy_log_metrics.get("ready_fields") if policy_log_metrics else {}
    if not isinstance(ready_fields, dict):
        ready_fields = {}
    finished_fields = policy_log_metrics.get("finished_fields") if policy_log_metrics else []
    if not isinstance(finished_fields, list):
        finished_fields = []
    evidence = {
        "record_mode_act": metric["record_mode"] == "act",
        "record_kind_rollout": metric["record_kind"] == "rollout",
        "launch_action_source_policy": metric.get("action_source") == "policy",
        "policy_ready": bool(metric.get("ready")),
        "bundle_digest": bool(metric.get("bundle_digest")),
        "processor_digest": bool(metric.get("processor_digest")),
        "profile_semantic_sha256": bool(metric.get("profile_semantic_sha256")),
        "observation_schema_hash": bool(metric.get("observation_schema_hash")),
        "action_schema_hash": bool(metric.get("action_schema_hash")),
        "bundle_not_synthetic": ready_fields.get("synthetic") is False,
        "checkpoint_full": ready_fields.get("checkpoint_mode") == "full",
        "model_sha256": bool(digest(ready_fields.get("model_sha256"))),
        "successful_predictions": metric["prediction_count"] > 0,
        "raw_predictions": metric["raw_prediction_count"] > 0,
        "applied_blocks": metric["command_blocks"] > 0,
        "started_time": metric["first_started_sim_time_ns"] is not None,
        "prediction_chunks_60": bool(
            finished_fields
            and all(
                fields.get("chunk_size") == 60
                for fields in finished_fields
                if isinstance(fields, dict)
            )
        ),
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


def _failure_flags(
    metric: dict[str, Any], axis_names: tuple[str, ...] = ()
) -> None:
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
    backend_detail = _backend_failure_detail(metric["controller_rejections"], axis_names)
    if backend_detail and not metric["success"]:
        metric["failure_detail"] = backend_detail
    elif metric["runtime_reason"] and metric["runtime_outcome"] not in {"stopped", "completed"}:
        # A controller failure is more actionable than the launcher's exit 1.
        metric["failure_detail"] = metric["runtime_reason"]
    if not metric["failure_detail"]:
        metric["failure_detail"] = (
            metric.get("runtime_reason")
            or metric.get("task_reason")
            or ", ".join(metric.get("reject_reasons", []))
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
    metric["controller_rejections"] = _controller_rejections(attempt)
    manifest = _json(attempt / "raw" / "manifest.json")
    axis_names = _axis_names(manifest)
    task = _json(attempt / "raw" / "task_result.json")
    export = _find_export(attempt)
    quality = _json(export / "quality.json") if export is not None else None
    labels_document, labels = _labels(export)
    metric["manifest"] = str(attempt / "raw" / "manifest.json") if manifest else ""
    metric["export"] = str(export) if export else ""
    metric["recording_path"] = str(attempt)
    if terminal is not None:
        _apply_terminal(metric, record, terminal)
    policy_log_metrics = _apply_policy_log(metric, attempt)
    _apply_manifest(metric, manifest)
    _apply_task(metric, task)
    _apply_quality(metric, quality)
    _apply_export(metric, export, labels_document, labels)
    _apply_act_evidence(metric, policy_log_metrics)
    _apply_task_issues(metric)
    _success_flags(metric)
    _failure_flags(metric, axis_names)
    return metric


__all__ = [
    "digest",
    "metric_for_seed",
]
