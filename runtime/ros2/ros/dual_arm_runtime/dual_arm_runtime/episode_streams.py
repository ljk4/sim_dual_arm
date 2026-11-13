"""Policy and teacher stream execution extracted from the lifecycle runner."""

from __future__ import annotations

from robot_contracts.episodes import RuntimeOutcome, RuntimeResult
from robot_contracts.models import ContractError, RunContext
from robot_run_logging import EventContext

from dual_arm_runtime.episode_runner_io import EpisodeLifecycle
from dual_arm_runtime.offline.audit.task_result import task_result_for_runtime
from dual_arm_runtime.offline.mcap_reader import task_fact_from_message
from dual_arm_runtime.runtime_errors import FirstFollowerRejected


def run_policy(
    runner: EpisodeLifecycle, episode_id: str, actual_context: RunContext, task_id: str
) -> None:
    from dual_arm_runtime.chunk_stream import PolicyStreamRunner, follower_limits
    from dual_arm_runtime.policy_client import PolicyClient

    policy = runner.configuration.recipe.policy
    if policy is None:
        raise ContractError("POLICY_CONFIGURATION", "policy settings are absent")
    snapshot = runner.configuration.snapshot
    client = None
    try:
        client = PolicyClient(
            policy,
            runner.configuration.profile,
            snapshot["profile"],
            snapshot["observation"],
            snapshot["action"],
            log=runner.log,
        )
        client.connect()
        session_id = client.open(actual_context, f"{episode_id}-open")
        runner.emit(
            "policy.session.ready",
            context=EventContext(episode_id=episode_id, clock_epoch=actual_context.clock_epoch),
            fields={
                "session_id": session_id,
                "test_only": policy.expected_test_only,
                "action_source": runner.configuration.recipe.action_source,
                "chunk_size": policy.chunk_size,
                "action_period_ns": policy.action_period_ns,
                "max_chunks": policy.max_chunks,
                "target_count": runner.configuration.recipe.follower.target_count,
                "termination_mode": policy.termination_mode,
            },
        )
        raw_limits = runner.configuration.control_document.get("max_accelerations")
        if not isinstance(raw_limits, list):
            raise ContractError("SCHEMA_MISMATCH", "control max_accelerations are absent")
        monitor = None
        if policy.termination_mode == "task":
            monitor = runner.io.task_client()
            if monitor is None:
                raise ContractError(
                    "TASK_MONITOR_MISSING", "task-terminated policy has no task client"
                )
        stream = PolicyStreamRunner(
            runner.configuration,
            actual_context,
            episode_id,
            task_id,
            runner.io,
            runner.control,
            client,
            follower_limits(runner.configuration, raw_limits),
            stop_requested=runner.stop_requested,
            log=runner.log,
            task_monitor=monitor,
        )
        runner.register_command(stream.command_id)
        try:
            stream.run()
        except FirstFollowerRejected:
            if stream.chunks == 0:
                runner.unregister_command(stream.command_id)
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
    runner: EpisodeLifecycle, client: object, episode_id: str, actual_context: RunContext
) -> None:
    try:
        client.close()
    except Exception as error:  # noqa: BLE001 - preserve the primary failure
        runner.emit(
            "policy.session.close_failed",
            level="ERROR",
            reason="POLICY_CLOSE_FAILED",
            context=EventContext(
                episode_id=episode_id,
                clock_epoch=actual_context.clock_epoch,
            ),
            exception=error,
        )


def run_teacher(
    runner: EpisodeLifecycle, episode_id: str, actual_context: RunContext, task_id: str
) -> None:
    from dual_arm_runtime.chunk_stream import follower_limits
    from dual_arm_runtime.teacher_source import TeacherStreamRunner, TeacherTargetSource

    teacher = runner.configuration.recipe.teacher
    if teacher is None:
        raise ContractError("TEACHER_CONFIGURATION", "teacher settings are absent")
    client = runner.io.task_client()
    if client is None:
        raise ContractError("TEACHER_DISABLED", "runtime has no task client")
    raw_limits = runner.configuration.control_document.get("max_accelerations")
    if not isinstance(raw_limits, list):
        raise ContractError("SCHEMA_MISMATCH", "control max_accelerations are absent")
    source = TeacherTargetSource(
        client,
        teacher,
        runner.configuration.profile,
        target_count=runner.configuration.recipe.follower.target_count,
        task_seed=runner.configuration.recipe.reset.seed,
        log=runner.log,
    )
    stream = TeacherStreamRunner(
        runner.configuration,
        actual_context,
        episode_id,
        task_id,
        runner.io,
        runner.control,
        source,
        follower_limits(runner.configuration, raw_limits),
        stop_requested=runner.stop_requested,
        log=runner.log,
    )
    runner.register_command(stream.command_id)
    try:
        stream.run()
    except FirstFollowerRejected:
        if stream.chunks == 0:
            runner.unregister_command(stream.command_id)
        raise
    finally:
        source.finish()
    finish_managed_task(runner, stream, actual_context, teacher.task_sim_budget_ns)


def _task_ending(runner: EpisodeLifecycle, context: RunContext, budget_ns: int) -> None:
    runner.emit(
        "task.terminated",
        reason=runner.managed_runtime.reason,
        context=EventContext(episode_id=context.episode_id, clock_epoch=context.clock_epoch),
        fields={
            "task_outcome": runner.task_result.outcome.value,
            "task_reason": runner.task_result.reason,
            "task_state": (
                None if runner.frozen_task_fact is None else str(runner.frozen_task_fact.phase)
            ),
            "task_state_reason": (
                None
                if runner.frozen_task_fact is None
                else str(runner.frozen_task_fact.failure_reason)
            ),
            "task_epoch": (
                None
                if runner.frozen_task_fact is None
                else int(runner.frozen_task_fact.task_epoch)
            ),
            "budget_ns": budget_ns,
            "command_id": runner.last_stream_command_id,
        },
    )


def finish_managed_task(
    runner: EpisodeLifecycle, stream: object, context: RunContext, budget_ns: int
) -> None:
    """Stop once, freeze the judge fact, then classify a normal task ending."""
    runtime_reason, _task_reason, fact = stream.task_end
    if not runtime_reason:
        raise ContractError(
            "TASK_NOT_TERMINAL", "managed stream ended without a frozen task fact"
        )
    runner.managed_runtime = RuntimeResult(RuntimeOutcome.STOPPED, runtime_reason)
    try:
        runner.control.stop_episode(runtime_reason)
    except ContractError as error:
        # A failed stop is a runtime failure, not a normal task ending.
        runner.managed_runtime = None
        raise ContractError("STOP_FAILED", str(error)) from error
    if int(getattr(stream, "chunks", 0)) > 0:
        try:
            runner.io.wait_terminal(
                stream.command_id,
                runner.configuration.recipe.control.stop_timeout_s,
                runner.stop_requested,
            )
        except Exception as error:  # noqa: BLE001 - terminal truth is mandatory
            runner.managed_runtime = None
            raise ContractError("TERMINAL_WAIT_FAILED", str(error)) from error
    client = runner.io.task_client()
    if client is not None:
        try:
            frozen = client.state(context, 0)
        except Exception as error:  # noqa: BLE001 - keep the stop fact, fail honestly
            runner.managed_runtime = None
            raise ContractError("TASK_STATE_QUERY_FAILED", str(error)) from error
        if frozen is not None:
            fact = frozen
            runner.io.record_task_state(fact)
    frozen_state = str(getattr(fact, "phase", ""))
    if frozen_state == "SUCCEEDED":
        runtime_reason = "TASK_SUCCEEDED"
        runner.managed_runtime = RuntimeResult(RuntimeOutcome.STOPPED, runtime_reason)
    elif runtime_reason == "TASK_SUCCEEDED":
        runner.managed_runtime = None
        raise ContractError(
            "TASK_STATE_CONFLICT", "runtime observed success but the frozen judge did not"
        )
    runner.frozen_task_fact = fact
    runner.task_result = task_result_for_runtime(
        runtime_reason, task_fact_from_message(fact)
    )
    runner.last_stream_command_id = stream.command_id
    _task_ending(runner, context, budget_ns)


__all__ = ["finish_managed_task", "run_policy", "run_teacher"]
