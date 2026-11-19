"""Follower-specific MCAP checks layered on the common stream audit."""

from __future__ import annotations

import math

from robot_contracts.models import ContractError
from robot_contracts.stream_request import REQUEST_MOTION
from robot_contracts.trajectory.follower_reference import (
    FollowerReference,
    FollowerValidationLimits,
)
from robot_trajectory.follower_reference import applied_follower_sample, sample_follower_reference
from robot_trajectory.follower_reference import validate as validate_follower_reference

from dual_arm_runtime.offline.audit.audit_common import record_error, vectors_close
from dual_arm_runtime.offline.audit.episode_audit import _Audit
from dual_arm_runtime.offline.audit.policy_audit_rules import require_completed_chunks


class _FollowerAudit(_Audit):
    def __init__(self, episode_id: str, configuration: dict) -> None:
        super().__init__(episode_id, configuration)
        recipe = configuration.get("recipe")
        if not isinstance(recipe, dict):
            raise record_error("follower recipe is missing")
        action_source = str(recipe.get("action_source", "policy"))
        section = "teacher" if action_source == "teacher" else "policy"
        policy = recipe.get(section)
        follower = recipe.get("follower")
        if not isinstance(policy, dict) or not isinstance(follower, dict):
            raise record_error(
                f"{section}/follower configuration is missing from the recording"
            )
        self.action_source = action_source
        self.termination_mode = str(policy.get("termination_mode", "finite_chunks"))
        if self.termination_mode not in {"finite_chunks", "task"}:
            raise record_error("unknown stream termination mode")
        control = recipe.get("control_constraints")
        if not isinstance(control, dict):
            raise record_error("control constraints are missing from the recording")
        accelerations = control.get("max_accelerations")
        deviations = follower.get("max_target_deviations")
        if not isinstance(accelerations, list) or not isinstance(deviations, list):
            raise record_error("follower limits are incomplete")
        if len(accelerations) != len(self.joint_names) or len(deviations) != len(self.joint_names):
            raise record_error("follower limits do not match the profile joints")
        self.policy = policy
        self.follower = follower
        self.limits = FollowerValidationLimits(
            joint_names=tuple(self.joint_names),
            max_accelerations=tuple(float(value) for value in accelerations),
            max_target_deviations=tuple(float(value) for value in deviations),
            target_count=int(follower["target_count"]),
            max_terminal_tail_duration_ns=int(follower["max_terminal_tail_duration_ns"]),
            duration_grid_ns=int(follower["duration_grid_ns"]),
        )
        self.applied_chunks: set[tuple[str, int]] = set()
        self.final_references = 0

    def reference(self, message: object) -> None:
        record = message
        if not self.context_value(record.request.trajectory.context):
            return
        command_id = record.request.trajectory.command_id
        request = self.commands.get(command_id)
        if request is None:
            raise record_error(f"follower reference precedes its command: {command_id}")
        if request.kind == REQUEST_MOTION:
            raise record_error("MOTION command cannot carry a follower reference")
        anchor = record.request.anchor
        if anchor is None:
            raise record_error("follower reference has no anchor")
        key = anchor.predecessor_seq
        if self.chunk_requests.get((command_id, key)) != record.request:
            raise record_error(
                f"follower reference differs from its raw chunk request: {command_id}/{key}"
            )
        try:
            validate_follower_reference(record, self._profile(), self.limits)
        except ContractError as error:
            raise record_error(f"follower reference validation failed: {error}") from error
        by_key = self.references.setdefault(command_id, {})
        existing = by_key.get(key)
        if existing is not None:
            if existing.reference_digest != record.reference_digest:
                raise record_error(
                    f"same follower chunk has conflicting references: {command_id}/{key}"
                )
            self.reference_duplicates += 1
            return
        by_key[key] = record
        if record.is_final:
            self.final_references += 1

    def reference_sample(
        self,
        record: object,
        execution: object,
        actual_started_sim_ns: int | None,
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], int]:
        del actual_started_sim_ns
        if not isinstance(record, FollowerReference):
            raise record_error("follower sample record has the wrong type")
        if execution.reference_kind != "CHUNK":  # type: ignore[attr-defined]
            raise record_error("follower applied sample is not a chunk reference")
        application = execution.reference_application_sim_time_ns  # type: ignore[attr-defined]
        try:
            raw = sample_follower_reference(record, application)
            applied = applied_follower_sample(record, application, self._profile())
        except ContractError as error:
            raise record_error(f"applied follower sample failed: {error}") from error
        corrections = tuple(
            sampled - actual
            for sampled, actual in zip(
                applied.sampled_positions, applied.applied_positions, strict=True
            )
        )
        if not vectors_close(corrections, applied.corrections, 1e-12):
            raise record_error("applied follower correction differs")
        if not vectors_close(raw.positions, applied.sampled_positions, 1e-12):
            raise record_error("raw coefficient sample differs from the applied record")
        if not vectors_close(raw.velocities, applied.sampled_velocities, 1e-12):
            raise record_error("raw coefficient velocity differs")
        if not vectors_close(raw.accelerations, applied.sampled_accelerations, 1e-9):
            raise record_error("raw coefficient acceleration differs")
        self.applied_chunks.add((record.command_id, record.chunk_seq))
        return (
            applied.applied_positions,
            applied.sampled_velocities,
            applied.sampled_accelerations,
            record.start_sim_time_ns,
        )

    def finish(self) -> dict:
        result = super().finish()
        policy = self.policy
        configured_chunks = int(policy["max_chunks"])
        chunk_size = int(policy["chunk_size"])
        period_ns = int(policy["action_period_ns"])
        raw_points = 0
        for command_id, by_key in self.references.items():
            keys = sorted(by_key)
            if keys != list(range(len(keys))):
                raise record_error("follower chunk sequence has a gap")
            for index, key in enumerate(keys):
                record = by_key[key]
                if record.chunk_seq != key + 1:
                    raise record_error("follower chunk sequence differs from its raw anchor")
                targets = record.request.trajectory.targets
                if len(targets) != chunk_size:
                    raise record_error(
                        f"raw follower source is not {chunk_size} points: {command_id}/{key}"
                    )
                for target_index, target in enumerate(targets):
                    if target.time_ns != (target_index + 1) * period_ns:
                        raise record_error("raw follower source is not on the 50 ms grid")
                    if len(target.positions) != len(self.joint_names):
                        raise record_error(
                            f"raw follower source width {len(target.positions)} != {len(self.joint_names)}"
                        )
                    if not all(math.isfinite(value) for value in target.positions):
                        raise record_error("raw follower source has a non-finite position")
                if record.window.source_targets != targets:
                    raise record_error("raw follower window does not retain every source point")
                expected_final = index == len(keys) - 1
                if record.is_final and not expected_final:
                    raise record_error("a follower chunk other than the last is final")
                if expected_final and not record.is_final:
                    # A legal abort can leave a non-final last accepted block.
                    if algorithm_completed(result):
                        raise record_error("completed follower stream lacks a final block")
                raw_points += len(targets)
        policy_chunks = sum(len(value) for value in self.references.values())
        require_completed_chunks(
            completed=algorithm_completed(result),
            policy_chunks=policy_chunks,
            configured_chunks=configured_chunks,
            strict=self.termination_mode == "finite_chunks",
        )
        if (
            algorithm_completed(result)
            and self.termination_mode == "finite_chunks"
            and not self.final_references
        ):
            raise record_error("completed follower references contain no final block")
        applied = 0
        for command_id, by_key in self.references.items():
            for key in by_key:
                if (command_id, key + 1) in self.applied_chunks:
                    applied += 1
        if (
            self.termination_mode == "finite_chunks"
            and algorithm_completed(result)
            and applied != sum(len(value) for value in self.references.values())
        ):
            raise record_error("completed follower stream has an unapplied accepted reference")
        result.update(
            {
                "stream_format": "v1",
                "policy_format": "continuous-bounded-follower",
                "configured_chunks": configured_chunks,
                "policy_chunks": policy_chunks,
                "raw_prediction_points": raw_points,
                "applied_chunks": applied,
                "final_chunks": self.final_references,
                "chunk_size": chunk_size,
                "action_period_ns": period_ns,
                "configured_target_count": self.limits.target_count,
                "action_source": self.action_source,
                "termination_mode": self.termination_mode,
                "test_only": bool(policy.get("expected_test_only", False)),
            }
        )
        return result


def algorithm_completed(result: dict) -> bool:
    """A completed command is the only case that must explain every reference."""
    return bool(result.get("completed_commands")) and not bool(
        result.get("commands_without_terminal")
    )


__all__ = ["_FollowerAudit", "algorithm_completed"]
