from pathlib import Path

import numpy as np
import pytest
from dual_arm_learning.chunk_admission import admit_action_chunk, save_prediction

NAMES = tuple(
    f"joint{i}" if i not in (7, 15) else ("left_gripper_width" if i == 7 else "right_gripper_width")
    for i in range(16)
)
LOWER = [-3.0] * 16
UPPER = [3.0] * 16
for index in (7, 15):
    LOWER[index], UPPER[index] = 0.0, 0.08


def test_only_grippers_are_admitted_and_raw_is_preserved(tmp_path: Path):
    raw = np.zeros((60, 16), dtype=np.float32)
    raw[:, 7], raw[:, 15] = 0.08001, -0.0292
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, 0.03)
    assert result.accepted
    assert np.array_equal(result.positions[:, :7], raw[:, :7])
    assert np.all(result.positions[:, 7] == 0.08) and np.all(result.positions[:, 15] == 0.0)
    path, size = save_prediction(
        tmp_path, "session:request", raw, result, bytes_used=0, max_bytes=100000
    )
    with np.load(path, allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["raw_act"], raw)
        np.testing.assert_array_equal(stored["admitted"], result.positions)
        assert set(stored.files) == {
            "raw_act",
            "admitted",
            "correction",
            "accepted",
            "reason",
        }
    assert size > 0
    raw[:, 15] = -0.0301
    assert not admit_action_chunk(raw, NAMES, LOWER, UPPER, 0.03).accepted
    raw[:, 15] = 0.0
    raw[:, 0] = 3.1
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, 0.03)
    assert not result.accepted and result.positions[0, 0] == raw[0, 0]
    with pytest.raises(ValueError, match="byte budget"):
        save_prediction(tmp_path, "too-big", raw, result, bytes_used=0, max_bytes=1)


def test_optional_temporal_arrays_are_saved_in_the_same_artifact(tmp_path: Path):
    raw = np.zeros((60, 16), dtype=np.float32)
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, 0.03)
    execution = np.full((60, 16), 0.25, dtype=np.float64)
    counts = np.arange(60, dtype=np.int64) % 4 + 1
    path, _ = save_prediction(
        tmp_path,
        "session:temporal",
        raw,
        result,
        bytes_used=0,
        max_bytes=100000,
        execution_targets=execution,
        ensemble_counts=counts,
        history_observation_ids=np.asarray([1, 2], dtype=np.int64),
        history_observation_time_ns=np.asarray([10, 20], dtype=np.int64),
    )
    with np.load(path, allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["raw_act"], raw)
        np.testing.assert_array_equal(stored["admitted"], result.positions)
        np.testing.assert_array_equal(stored["execution_targets"], execution)
        np.testing.assert_array_equal(stored["ensemble_counts"], counts)
        np.testing.assert_array_equal(stored["history_observation_ids"], [1, 2])
        np.testing.assert_array_equal(stored["history_observation_time_ns"], [10, 20])
