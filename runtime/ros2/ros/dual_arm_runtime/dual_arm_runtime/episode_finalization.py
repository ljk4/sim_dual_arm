"""Episode cleanup, recording finalization and task-fact freezing."""

from __future__ import annotations

import json
from pathlib import Path

from robot_contracts.episodes import RuntimeOutcome, RuntimeResult, TaskOutcome
from robot_run_logging import EventContext, Scalar

from dual_arm_runtime.recording_queue import RecordingStats
from dual_arm_runtime.task_result import (
    task_fact_document,
    task_result_from_fact,
)


def stat_fields(stats: RecordingStats | None) -> dict[str, Scalar]:
    if stats is None:
        return {}
    return {
        "recording_accepted": stats.accepted,
        "recording_written": stats.written,
        "recording_pending_items": stats.pending_items,
        "recording_pending_bytes": stats.pending_bytes,
        "recording_peak_items": stats.peak_items,
        "recording_peak_bytes": stats.peak_bytes,
        "recording_failure_reason": stats.failure_reason or None,
    }


def _cleanup(runner: object, episode_id: str, clock_epoch: int | None) -> None:
    try:
        runner._set_status("ENDING", episode_id)
    except Exception as error:
        runner._faults.append(f"STATUS_ERROR: {error}")
    if runner._managed_runtime is not None:
        cleanup_reason = runner._managed_runtime.reason
    else:
        cleanup_reason = "EPISODE_END" if runner._end_requested else "RUNTIME_SHUTDOWN"
    try:
        runner._control.cleanup(cleanup_reason)
    except Exception as error:
        runner._faults.append(f"CLEANUP_ERROR: {error}")
    if not runner._sent_command_ids:
        return
    try:
        runner._io.wait_stop_settled(
            tuple(runner._sent_command_ids),
            runner._configuration.recipe.control.stop_timeout_s,
        )
    except Exception as error:
        runner._faults.append(f"TERMINAL_WAIT_ERROR: {error}")
        runner._emit_best_effort(
            "terminal_wait_failed",
            level="ERROR",
            reason="TERMINAL_WAIT_FAILED",
            context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
            fields={"command_count": len(runner._sent_command_ids)},
            exception=error,
        )


def _write_task_fact(
    runner: object, path: Path, manifest: dict, episode_id: str, clock_epoch: int | None
) -> None:
    try:
        document = task_fact_document(
            runner._frozen_task_fact,
            runner._task_result,
            action_source=runner._configuration.recipe.action_source,
            test_only=bool(manifest.get("test_only", False)),
        )
        temporary = path / "task_result.json.tmp"
        temporary.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
        temporary.replace(path / "task_result.json")
    except Exception as error:  # noqa: BLE001 - sidecar must not fake manifest truth
        runner._emit_best_effort(
            "task.result.write_failed",
            level="ERROR",
            reason="TASK_RESULT_WRITE_FAILED",
            context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
            exception=error,
        )


def _recorder_stats(recorder: object) -> RecordingStats | None:
    try:
        return recorder.stats()
    except Exception:
        return None


def _recorded_completed(
    runner: object,
    recorder: object,
    path: Path,
    episode_id: str,
    clock_epoch: int | None,
) -> tuple[bool, str, RecordingStats | None]:
    with runner._recorder_lock:
        runner._record_directory = str(path)
    manifest = json.loads((path / "manifest.json").read_text())
    _write_task_fact(runner, path, manifest, episode_id, clock_epoch)
    recording_complete = bool(manifest["recording"]["complete"])
    recording_failure = str(manifest["recording"]["failure_reason"])
    if not recording_complete:
        runner._faults.merge(recording_failure)
    manifest_runtime = RuntimeResult(
        RuntimeOutcome(manifest["runtime"]["outcome"]),
        manifest["runtime"]["reason"],
    )
    if manifest_runtime.outcome is RuntimeOutcome.FAILED:
        runner._faults.merge(manifest_runtime.reason)
    stats = _recorder_stats(recorder)
    runner._emit_best_effort(
        "recording.completed" if recording_complete else "recording.failed",
        level="INFO" if recording_complete else "ERROR",
        reason="RECORDING_COMPLETED" if recording_complete else "RECORDING_INCOMPLETE",
        context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
        fields={
            "path": str(path),
            "recording_complete": recording_complete,
            "recording_failure_reason": recording_failure or None,
            **stat_fields(stats),
        },
    )
    return recording_complete, recording_failure, stats


def _finish_recorder(
    runner: object, episode_id: str, clock_epoch: int | None
) -> tuple[bool | None, str, RecordingStats | None]:
    with runner._recorder_lock:
        recorder = runner._recorder
        runner._recorder = None
    if recorder is None:
        return None, "", None
    runtime = runner.current_runtime_result()
    try:
        path = recorder.finish(
            runtime,
            current_runtime=runner.current_runtime_result,
            task_result=runner._task_result,
        )
        return _recorded_completed(runner, recorder, path, episode_id, clock_epoch)
    except Exception as error:
        recording_failure = str(error)
        stats = _recorder_stats(recorder)
        runner._faults.append(f"RECORDING_ERROR: {error}")
        runner._emit_best_effort(
            "recording.failed",
            level="ERROR",
            reason="RECORDING_FAILED",
            context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
            fields={
                "path": runner._current_path() or None,
                "recording_complete": False,
                "recording_failure_reason": recording_failure or None,
                **stat_fields(stats),
            },
            exception=error,
        )
        return False, recording_failure, stats


def _emit_finished(
    runner: object,
    episode_id: str,
    clock_epoch: int | None,
    primary: str,
    recording_complete: bool | None,
    recording_failure: str,
    recording_stats: RecordingStats | None,
) -> RuntimeResult:
    runtime = runner.current_runtime_result()
    runner._emit_best_effort(
        "episode.finished",
        level="ERROR"
        if runtime.outcome is RuntimeOutcome.FAILED
        else "WARNING"
        if runtime.outcome in {RuntimeOutcome.INTERRUPTED, RuntimeOutcome.STOPPED}
        else "INFO",
        reason=f"EPISODE_{runtime.outcome.value.upper()}",
        context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
        fields={
            "task_id": runner._task_id,
            "record_directory": runner._current_path() or None,
            "recording_complete": recording_complete,
            "recording_failure_reason": recording_failure or None,
            "runtime_outcome": runtime.outcome.value,
            "runtime_reason": runtime.reason or None,
            "task_outcome": runner._task_result.outcome.value,
            "primary_reason": primary or None,
            **stat_fields(recording_stats),
        },
    )
    return runtime


def _final_status(runner: object, episode_id: str, runtime: RuntimeResult) -> None:
    try:
        runner._set_status(
            "FAULT" if runtime.outcome is RuntimeOutcome.FAILED else "IDLE",
            episode_id,
            runner.failure_reason if runtime.outcome is RuntimeOutcome.FAILED else "",
        )
    except Exception as error:
        runner._faults.append(f"STATUS_ERROR: {error}")


def _print_summary(
    runner: object,
    episode_id: str,
    recording_complete: bool | None,
    recording_failure: str,
) -> None:
    runtime = runner.current_runtime_result()
    print(
        json.dumps(
            {
                "episode_id": episode_id,
                "directory": runner.record_directory,
                "recording": {
                    "complete": recording_complete,
                    "failure_reason": recording_failure or None,
                },
                "runtime": {
                    "outcome": runtime.outcome.value,
                    "reason": runtime.reason or None,
                },
                "task": {
                    "outcome": runner._task_result.outcome.value,
                    "reason": runner._task_result.reason or None,
                },
            }
        ),
        flush=True,
    )


def finish(runner: object, episode_id: str) -> None:
    runner._finishing.set()
    clock_epoch = runner._current_clock_epoch()
    primary = runner.failure_reason
    _cleanup(runner, episode_id, clock_epoch)
    freeze_task_fact_after_failure(runner, episode_id, clock_epoch)
    recording_complete, recording_failure, recording_stats = _finish_recorder(
        runner, episode_id, clock_epoch
    )
    with runner._recorder_lock:
        runner._run_context = None
    runtime = _emit_finished(
        runner,
        episode_id,
        clock_epoch,
        primary,
        recording_complete,
        recording_failure,
        recording_stats,
    )
    _final_status(runner, episode_id, runtime)
    _print_summary(runner, episode_id, recording_complete, recording_failure)


def freeze_task_fact_after_failure(
    runner: object, episode_id: str, clock_epoch: int | None
) -> None:
    """Best-effort, bounded task truth for a non-managed runtime failure.

    A managed task end already froze its own fact.  This path only covers an
    unexpected runtime failure after Start, so a failed episode can still
    carry the independent judge dimension instead of silently dropping it.
    """
    recipe = runner._configuration.recipe
    stream = recipe.policy if recipe.action_source == "policy" else recipe.teacher
    if stream is None or stream.termination_mode != "task":
        return
    if runner._task_result.outcome is not TaskOutcome.NOT_EVALUATED:
        return
    context = runner.context
    client = runner._io.task_client()
    if context is None or client is None:
        return
    try:
        fact = client.state(context, 0)
    except Exception as error:  # noqa: BLE001 - preserve the primary runtime failure
        runner._emit_best_effort(
            "task.state_unavailable",
            level="WARNING",
            reason="TASK_STATE_UNAVAILABLE",
            context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
            exception=error,
        )
        fact = None
    if fact is not None:
        runner._frozen_task_fact = fact
        runner._io.record_task_state(fact)
        judged = task_result_from_fact(fact)
        if judged.outcome is not TaskOutcome.NOT_EVALUATED:
            runner._task_result = judged


__all__ = ["finish", "freeze_task_fact_after_failure", "stat_fields"]
