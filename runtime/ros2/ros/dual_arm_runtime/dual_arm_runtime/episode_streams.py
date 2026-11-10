"""Policy and teacher stream execution extracted from the lifecycle runner."""

from __future__ import annotations

from robot_contracts.episodes import RuntimeOutcome, RuntimeResult
from robot_contracts.models import ContractError, RunContext
from robot_run_logging import EventContext

from dual_arm_runtime.runtime_errors import FirstFollowerRejected
from dual_arm_runtime.task_result import task_result_for_runtime


def run_policy(runner: object, episode_id: str, actual_context: RunContext, task_id: str) -> None:
    from dual_arm_runtime.chunk_stream import PolicyStreamRunner, follower_limits
    from dual_arm_runtime.policy_client import PolicyClient

    policy = runner._configuration.recipe.policy
    if policy is None:
        raise ContractError("POLICY_CONFIGURATION", "policy settings are absent")
    snapshot = runner._configuration.snapshot
    client = None
    try:
        client = PolicyClient(
            policy,
            runner._configuration.profile,
            snapshot["profile"],
            snapshot["observation"],
            snapshot["action"],
            log=runner._log,
        )
        client.connect()
        session_id = client.open(actual_context, f"{episode_id}-open")
        runner._emit_best_effort(
            "policy.session.ready",
            context=EventContext(episode_id=episode_id, clock_epoch=actual_context.clock_epoch),
            fields={
                "session_id": session_id,
                "test_only": policy.expected_test_only,
                "action_source": runner._configuration.recipe.action_source,
                "chunk_size": policy.chunk_size,
                "action_period_ns": policy.action_period_ns,
                "max_chunks": policy.max_chunks,
                "target_count": runner._configuration.recipe.follower.target_count,
                "termination_mode": policy.termination_mode,
            },
        )
        raw_limits = runner._configuration.control_document.get("max_accelerations")
        if not isinstance(raw_limits, list):
            raise ContractError("SCHEMA_MISMATCH", "control max_accelerations are absent")
        monitor = None
        if policy.termination_mode == "task":
            monitor = runner._io.task_client()
            if monitor is None:
                raise ContractError(
                    "TASK_MONITOR_MISSING", "task-terminated policy has no task client"
                )
        stream = PolicyStreamRunner(
            runner._configuration,
            actual_context,
            episode_id,
            task_id,
            runner._io,
            runner._control,
            client,
            follower_limits(runner._configuration, raw_limits),
            stop_requested=runner._stop_requested,
            log=runner._log,
            task_monitor=monitor,
        )
        runner._sent_command_ids.append(stream.command_id)
        try:
            stream.run()
        except FirstFollowerRejected:
            if stream.chunks == 0:
                runner._sent_command_ids.remove(stream.command_id)
            raise
        finally:
            _close_policy_client(runner, client, episode_id, actual_context)
        if policy.termination_mode == "task":
            finish_managed_task(runner, stream, actual_context, policy.task_sim_budget_ns)
    finally:
        if client is not None:
            try:
                client.shutdown()
            except Exception:
                pass


def _close_policy_client(
    runner: object, client: object, episode_id: str, actual_context: RunContext
) -> None:
    try:
        client.close()
    except Exception as error:  # noqa: BLE001 - preserve the primary failure
        runner._emit_best_effort(
            "policy.session.close_failed",
            level="ERROR",
            reason="POLICY_CLOSE_FAILED",
            context=EventContext(
                episode_id=episode_id,
                clock_epoch=actual_context.clock_epoch,
            ),
            exception=error,
        )


def run_teacher(runner: object, episode_id: str, actual_context: RunContext, task_id: str) -> None:
    from dual_arm_runtime.chunk_stream import follower_limits
    from dual_arm_runtime.teacher_source import TeacherStreamRunner, TeacherTargetSource

    teacher = runner._configuration.recipe.teacher
    if teacher is None:
        raise ContractError("TEACHER_CONFIGURATION", "teacher settings are absent")
    client = runner._io.task_client()
    if client is None:
        raise ContractError("TEACHER_DISABLED", "runtime has no stack-task client")
    raw_limits = runner._configuration.control_document.get("max_accelerations")
    if not isinstance(raw_limits, list):
        raise ContractError("SCHEMA_MISMATCH", "control max_accelerations are absent")
    source = TeacherTargetSource(
        client,
        teacher,
        runner._configuration.profile,
        target_count=runner._configuration.recipe.follower.target_count,
        task_seed=runner._configuration.recipe.reset.seed,
        log=runner._log,
    )
    stream = TeacherStreamRunner(
        runner._configuration,
        actual_context,
        episode_id,
        task_id,
        runner._io,
        runner._control,
        source,
        follower_limits(runner._configuration, raw_limits),
        stop_requested=runner._stop_requested,
        log=runner._log,
    )
    runner._sent_command_ids.append(stream.command_id)
    try:
        stream.run()
    except FirstFollowerRejected:
        if stream.chunks == 0:
            runner._sent_command_ids.remove(stream.command_id)
        raise
    finally:
        source.finish()
    finish_managed_task(runner, stream, actual_context, teacher.task_sim_budget_ns)


def _task_ending(runner: object, context: RunContext, budget_ns: int) -> None:
    runner._emit_best_effort(
        "task.terminated",
        reason=runner._managed_runtime.reason,
        context=EventContext(episode_id=context.episode_id, clock_epoch=context.clock_epoch),
        fields={
            "task_outcome": runner._task_result.outcome.value,
            "task_reason": runner._task_result.reason,
            "task_state": (
                None if runner._frozen_task_fact is None else str(runner._frozen_task_fact.phase)
            ),
            "task_state_reason": (
                None
                if runner._frozen_task_fact is None
                else str(runner._frozen_task_fact.failure_reason)
            ),
            "task_epoch": (
                None
                if runner._frozen_task_fact is None
                else int(runner._frozen_task_fact.task_epoch)
            ),
            "budget_ns": budget_ns,
            "command_id": runner._last_stream_command_id,
        },
    )


def finish_managed_task(
    runner: object, stream: object, context: RunContext, budget_ns: int
) -> None:
    """Stop once, freeze the judge fact, then classify a normal task ending."""
    runtime_reason, _task_reason, fact = stream.task_end
    if not runtime_reason:
        raise ContractError(
            "TASK_NOT_TERMINAL", "managed stream ended without a frozen task fact"
        )
    runner._managed_runtime = RuntimeResult(RuntimeOutcome.STOPPED, runtime_reason)
    try:
        runner._control.stop_episode(runtime_reason)
    except ContractError as error:
        # A failed stop is a runtime failure, not a normal task ending.
        runner._managed_runtime = None
        raise ContractError("STOP_FAILED", str(error)) from error
    if int(getattr(stream, "chunks", 0)) > 0:
        try:
            runner._io.wait_terminal(
                stream.command_id,
                runner._configuration.recipe.control.stop_timeout_s,
                runner._stop_requested,
            )
        except Exception as error:  # noqa: BLE001 - terminal truth is mandatory
            runner._managed_runtime = None
            raise ContractError("TERMINAL_WAIT_FAILED", str(error)) from error
    client = runner._io.task_client()
    if client is not None:
        try:
            frozen = client.state(context, 0)
        except Exception as error:  # noqa: BLE001 - keep the stop fact, fail honestly
            runner._managed_runtime = None
            raise ContractError("TASK_STATE_QUERY_FAILED", str(error)) from error
        if frozen is not None:
            fact = frozen
            runner._io.record_task_state(fact)
    frozen_state = str(getattr(fact, "phase", ""))
    if frozen_state == "SUCCEEDED":
        runtime_reason = "TASK_SUCCEEDED"
        runner._managed_runtime = RuntimeResult(RuntimeOutcome.STOPPED, runtime_reason)
    elif runtime_reason == "TASK_SUCCEEDED":
        runner._managed_runtime = None
        raise ContractError(
            "TASK_STATE_CONFLICT", "runtime observed success but the frozen judge did not"
        )
    runner._frozen_task_fact = fact
    runner._task_result = task_result_for_runtime(runtime_reason, fact)
    runner._last_stream_command_id = stream.command_id
    _task_ending(runner, context, budget_ns)


__all__ = ["finish_managed_task", "run_policy", "run_teacher"]
