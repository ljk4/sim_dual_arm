"""Report assembly and publication for the offline kinematics check."""

from __future__ import annotations

import json
import os
import platform
import sys
import time
from pathlib import Path

from robot_backend_sim.common.runtime.diagnostics import utc_now, write_json_atomic
from robot_backend_sim.tools.check_kinematics_common import (
    ArtifactBudget,
    KinematicsCheckFailure,
    _emit,
)

COMPONENT = "kinematics_check"
KIND = "kinematics"


def _serialized_size(document: object) -> int:
    # Must match write_json_atomic's actual encoding byte for byte.
    return len((json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n").encode())

def _minimal_report(report: dict[str, object], reason: str, message: str) -> dict[str, object]:
    return {
        "kind": KIND,
        "component": COMPONENT,
        "run_id": report.get("run_id"),
        "instance_id": report.get("instance_id"),
        "status": "failed",
        "exit_code": 1,
        "reason": reason,
        "error": {"type": "KinematicsCheckFailure", "message": message[:2000]},
        "logs_closed": report.get("logs_closed", False),
        "report_budget_exceeded": True,
        "started_at_utc": report.get("started_at_utc"),
        "finished_at_utc": report.get("finished_at_utc"),
        "elapsed_s": report.get("elapsed_s"),
    }

def _new_report(arguments: object, instance_id: str) -> dict[str, object]:
    return {
        "kind": KIND,
        "component": COMPONENT,
        "run_id": arguments.run_id,
        "instance_id": instance_id,
        "status": "failed",
        "exit_code": 1,
        "reason": "not_started",
        "started_at_utc": utc_now(),
        "finished_at_utc": None,
        "elapsed_s": None,
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "environment": {
            "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
            "PYOPENGL_PLATFORM": os.environ.get("PYOPENGL_PLATFORM"),
        },
        "configs": None,
        "dependencies": None,
        "source": None,
        "scene": None,
        "collision_pairs": None,
        "layouts": [],
        "summary": None,
        "failures": [],
        "artifact": None,
        "artifacts": None,
        "log_directory": None,
        "logs_closed": False,
        "renderer_close_error": None,
        "logger_close_error": None,
        "error": None,
        "limitations": [
            "Linear joint-space sampling is discrete, not a continuous collision proof.",
            "The gripper stays open and both objects stay at their initial poses; "
            "lift/place are empty-hand key-pose previews, not object transport or stacking.",
            "IK dt is a solver step, not a controller velocity or execution period.",
        ],
    }

def _close_resources(
    logger: object | None,
    renderer: object | None,
    report: dict[str, object],
    failure: KinematicsCheckFailure | None,
) -> KinematicsCheckFailure | None:
    if logger is not None:
        try:
            _emit(
                logger,
                "finalizing",
                "kinematics check finalizing resources",
                {"failed": failure is not None},
            )
        except KinematicsCheckFailure as error:
            if failure is None:
                failure = error
    if renderer is not None:
        try:
            renderer.close()
        except Exception as error:
            report["renderer_close_error"] = (
                f"{getattr(error, 'reason', 'renderer_close_failed')}: {error}"
            )
            if failure is None:
                failure = KinematicsCheckFailure(
                    "renderer_close_failed", report["renderer_close_error"]
                )
    try:
        if logger is not None:
            if failure is None:
                _emit(
                    logger,
                    "checks_finished",
                    "kinematics diagnostics and artifact writes finished; closing resources",
                )
            else:
                _emit(
                    logger,
                    "kinematics_check_failed",
                    "kinematics key-pose check failed",
                    {"reason": failure.reason},
                )
            logger.check_health()
    except Exception as error:
        if failure is None:
            failure = KinematicsCheckFailure(
                "event_log_failed", f"logger finalization: {type(error).__name__}: {error}"
            )
    log_close_error: str | None = None
    if logger is not None:
        try:
            logger.close()
        except Exception as error:
            log_close_error = f"{type(error).__name__}: {error}"
            if failure is None:
                failure = KinematicsCheckFailure(
                    "logger_close_failed", f"event logger close failed: {log_close_error}"
                )
    report["logs_closed"] = logger is not None and log_close_error is None
    report["logger_close_error"] = log_close_error
    return failure

def _summarize_results(results: list[dict[str, object]]) -> dict[str, object]:
    all_paths = [
        path
        for result in results
        for path in result["paths"]  # type: ignore[index]
    ]
    clearance_entries = []
    for result in results:
        initial = result["initial_clearance"]
        if initial is not None:
            clearance_entries.append(
                {
                    "layout": result["layout_id"],
                    "segment": "initial",
                    "minimum_clearance": initial["minimum_clearance"],
                }
            )
        for path in result["paths"]:
            clearance_entries.append(
                {
                    "layout": result["layout_id"],
                    "segment": path["segment"],
                    "minimum_clearance": path["minimum_clearance"],
                }
            )
    minimum = min(
        clearance_entries,
        key=lambda entry: entry["minimum_clearance"]["distance_m"],
        default=None,
    )
    return {
        "layout_count": len(results),
        "layouts_passed": sum(1 for result in results if result["passed"]),
        "layouts_failed": sum(1 for result in results if not result["passed"]),
        "ik_targets": sum(len(result["ik"]) for result in results),
        "ik_targets_converged": sum(
            1 for result in results for target in result["ik"] if target["converged"]
        ),
        "segments": len(all_paths),
        "path_samples": sum(path["sample_count"] for path in all_paths),
        "minimum_clearance": minimum,
        "phases_with_vision": sum(len(result["vision"]) for result in results),
    }

def _publish_report(
    run_dir: Path,
    report: dict[str, object],
    failure: KinematicsCheckFailure | None,
    budget: ArtifactBudget | None,
    *,
    started: float,
    max_report_bytes: int,
) -> int:
    report["elapsed_s"] = round(time.monotonic() - started, 3)
    report["finished_at_utc"] = utc_now()
    if failure is None:
        report["status"] = "succeeded"
        report["exit_code"] = 0
        report["reason"] = None
    else:
        report["status"] = "failed"
        report["exit_code"] = 1
        report["reason"] = failure.reason
        report["error"] = {
            "type": type(failure).__name__,
            "message": failure.message[:2000],
        }
        recorded = report.get("failures")
        if not isinstance(recorded, list):
            recorded = []
        if not any(
            isinstance(item, dict)
            and item.get("reason") == failure.reason
            and item.get("message") == failure.message[:2000]
            for item in recorded
        ):
            recorded.append(
                {
                    "layout": None,
                    "phase": None,
                    "reason": failure.reason,
                    "message": failure.message[:2000],
                }
            )
        report["failures"] = recorded
    if budget is not None:
        report["artifacts"] = report.get("artifacts") or {
            "png_count": budget.count,
            "png_bytes": budget.total_bytes,
            "png_limit_bytes": budget.limit_bytes,
        }
    try:
        size = _serialized_size(report)
        if size > max_report_bytes:
            reason = failure.reason if failure is not None else "report_budget_exceeded"
            message = (
                failure.message
                if failure is not None
                else f"report needs {size} bytes, limit {max_report_bytes}"
            )
            report = _minimal_report(report, reason, message)
            if failure is None:
                failure = KinematicsCheckFailure("report_budget_exceeded", message)
            report["reason"] = failure.reason
            report["error"] = {
                "type": type(failure).__name__,
                "message": failure.message[:2000],
            }
    except (TypeError, ValueError) as error:
        report = _minimal_report(
            report,
            "report_serialization_failed",
            f"{type(error).__name__}: {error}",
        )
        failure = failure or KinematicsCheckFailure("report_serialization_failed", str(error))
    report_path = run_dir / "report.json"
    try:
        write_json_atomic(report_path, report)
    except Exception as error:
        print(f"error: report_write_failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(f"report: {report_path}")
    return int(report["exit_code"])

