from dual_arm_runtime.offline.audit.records import TaskFact
from dual_arm_runtime.offline.audit.task_result import task_fact_document, task_result_for_runtime
from robot_contracts.episodes import TaskOutcome


def _fact(phase, reason):
    return TaskFact(
        phase=phase,
        failure_reason=reason,
        task_type="stack",
        task_epoch=3,
        state_seq=9,
        sample_time_ns=1_000_000_002,
        task_config_digest="config",
        plan_digest="plan",
        metrics=(
            {
                "task_start_sim_time_ns": 1_000_000_000,
                "first_satisfied_sim_time_ns": 2_000_000_000,
                "completed_sim_time_ns": 2_500_000_000,
                "stable_elapsed_s": 2.0,
                "conditions_passed": True,
                "conditions_bitmask": 16383,
                "source_target_normal_force_n": 1.0,
                "target_table_normal_force_n": 1.0,
                "finger_contact_count": 0,
                "right_gripper_opening_m": 0.08,
                "right_tcp_distance_m": 0.1,
            }
        ),
    )


def test_frozen_task_success_and_timeout_are_independent_dimensions():
    success = task_result_for_runtime("TASK_SUCCEEDED", _fact("SUCCEEDED", "SUCCESS"))
    assert success.outcome is TaskOutcome.SUCCEEDED
    timed = task_result_for_runtime(
        "TASK_TIMEOUT", _fact("FAILED", "TIMEOUT: task exceeded")
    )
    assert timed.outcome is TaskOutcome.TIMED_OUT
    gate = task_result_for_runtime("TASK_FAILED", _fact("RUNNING", ""))
    assert gate.outcome is TaskOutcome.NOT_EVALUATED
    no_fact = task_result_for_runtime("TASK_SUCCEEDED", None)
    assert no_fact.outcome is TaskOutcome.NOT_EVALUATED


def test_sidecar_stays_small_and_has_no_images_or_masks():
    document = task_fact_document(
        _fact("SUCCEEDED", "SUCCESS"),
        task_result_for_runtime("TASK_SUCCEEDED", _fact("SUCCEEDED", "SUCCESS")),
        action_source="teacher",
        test_only=False,
    )
    assert document["task"]["state"] == "SUCCEEDED"
    assert document["result"]["outcome"] == "succeeded"
    assert "images" not in document and "segmentation" not in document
