"""Batch evaluation requires task and recording evidence, not process success."""

import json

import pytest
from dual_arm_runtime.batch_policy import _episode_succeeded, _parse_summary


def test_episode_summary_is_selected_before_the_final_process_summary():
    episode = {
        "episode_id": "evaluation-1",
        "task": {"outcome": "failed", "reason": "TASK_TIMEOUT"},
        "runtime": {"outcome": "stopped"},
        "recording": {"complete": True},
    }
    stdout = "\n".join(
        [
            "Runtime configuration",
            json.dumps(episode),
            json.dumps({"process_outcome": "succeeded", "exit_code": 0}),
        ]
    )
    summary = _parse_summary(stdout)
    assert summary == episode
    assert not _episode_succeeded(0, summary)


@pytest.mark.parametrize(
    ("exit_code", "task", "runtime", "recording", "expected"),
    [
        (0, "succeeded", "stopped", True, True),
        (0, "succeeded", "completed", True, True),
        (0, "failed", "stopped", True, False),
        (0, "succeeded", "failed", True, False),
        (0, "succeeded", "stopped", False, False),
        (1, "succeeded", "stopped", True, False),
    ],
)
def test_task_success_requires_a_clean_runtime_and_complete_recording(
    exit_code, task, runtime, recording, expected
):
    summary = {
        "task": {"outcome": task},
        "runtime": {"outcome": runtime},
        "recording": {"complete": recording},
    }
    assert _episode_succeeded(exit_code, summary) is expected


def test_process_success_without_task_evidence_is_not_a_success():
    stdout = json.dumps({"process_outcome": "succeeded", "exit_code": 0})
    assert _parse_summary(stdout) is None
    assert not _episode_succeeded(0, {})
