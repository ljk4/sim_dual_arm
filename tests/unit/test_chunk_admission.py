from pathlib import Path

import numpy as np
import pytest
from dual_arm_learning.chunk_admission import admit_action_chunk, save_prediction

NAMES = (
    "left_joint1", "left_joint2", "left_joint3", "left_joint4",
    "left_joint5", "left_joint6", "left_joint7", "left_gripper_width",
    "right_joint1", "right_joint2", "right_joint3", "right_joint4",
    "right_joint5", "right_joint6", "right_joint7", "right_gripper_width",
)
LOWER = [-3.0] * 7 + [0.0] + [-3.0] * 7 + [0.0]
UPPER = [3.0] * 7 + [0.08] + [3.0] * 7 + [0.08]
GRIPPER_QUOTA = {"left_gripper_width": 0.005, "right_gripper_width": 0.005}


def test_configured_axes_are_projected_onto_both_bounds_and_raw_is_preserved():
    raw = np.zeros((60, 16), dtype=np.float32)
    raw[0, 6], raw[1, 6] = 3.004, -3.004
    raw[2, 7], raw[3, 7] = 0.081, -0.001
    corrections = {"left_joint7": 0.01} | GRIPPER_QUOTA
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections)

    assert result.accepted and result.reason == ""
    assert result.positions[0, 6] == 3.0 and result.positions[1, 6] == -3.0
    assert result.positions[2, 7] == 0.08 and result.positions[3, 7] == 0.0
    assert result.correction[0, 6] == pytest.approx(-0.004, abs=1e-6)
    assert result.correction[1, 6] == pytest.approx(0.004, abs=1e-6)
    assert result.correction[2, 7] == pytest.approx(-0.001, abs=1e-6)
    assert result.correction[3, 7] == pytest.approx(0.001, abs=1e-6)
    # Every other axis keeps the exact raw value and an exactly zero correction.
    untouched = [index for index in range(16) if index not in (6, 7)]
    assert np.array_equal(result.positions[:, untouched], raw[:, untouched])
    assert not result.correction[:, untouched].any()


def test_unlisted_and_zero_quota_axes_still_reject_the_whole_chunk():
    corrections = {"left_joint7": 0.01, "right_gripper_width": 0.0}

    raw = np.zeros((60, 16), dtype=np.float32)
    raw[-1, 0] = 3.1  # The last, normally unselected point is still covered.
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections)
    assert not result.accepted
    assert result.reason == "left_joint1: prediction outside hard limits"
    assert result.positions[-1, 0] == raw[-1, 0]

    raw = np.zeros((60, 16), dtype=np.float32)
    raw[30, 15] = -1e-6  # An explicitly zero quota is not a free pass.
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections)
    assert not result.accepted
    assert result.reason == "right_gripper_width: prediction outside hard limits"


def test_over_quota_rejects_with_the_original_raw_retained():
    corrections = {"left_joint7": 0.01} | GRIPPER_QUOTA

    raw = np.zeros((60, 16), dtype=np.float32)
    raw[:, 6] = 3.02
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections)
    assert not result.accepted
    assert result.reason.startswith("left_joint7: admission correction 0.02 exceeds 0.01")
    assert result.positions[0, 6] == 3.0
    assert result.correction[0, 6] == pytest.approx(-0.02)

    raw = np.zeros((60, 16), dtype=np.float32)
    raw[59, 7] = 0.086
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections)
    assert not result.accepted
    assert result.reason.startswith("left_gripper_width: admission correction 0.006")
    assert result.positions[59, 7] == 0.08


def test_quota_units_are_per_axis_not_a_single_shared_threshold():
    corrections = {"left_joint7": 0.01, "left_gripper_width": 0.005}

    raw = np.zeros((60, 16), dtype=np.float32)
    raw[:, 6] = 3.006  # 0.006 rad is inside the 0.01 rad arm quota.
    assert admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections).accepted

    raw = np.zeros((60, 16), dtype=np.float32)
    raw[:, 7] = 0.086  # 0.006 m is outside the 0.005 m gripper quota.
    assert not admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections).accepted


def test_all_in_range_output_is_identical_for_any_quota():
    raw = np.linspace(-1.0, 1.0, 60 * 16, dtype=np.float32).reshape(60, 16)
    raw[:, 7] = raw[:, 15] = 0.04
    for corrections in ({}, {"left_joint7": 0.01} | GRIPPER_QUOTA):
        result = admit_action_chunk(raw, NAMES, LOWER, UPPER, corrections)
        assert result.accepted
        assert np.array_equal(result.positions, raw)
        assert not result.correction.any()


def test_empty_mapping_is_a_strict_full_chunk_bounds_check():
    raw = np.zeros((60, 16), dtype=np.float32)
    assert admit_action_chunk(raw, NAMES, LOWER, UPPER, {}).accepted

    raw[59, 6] = 3.0001
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, {})
    assert not result.accepted
    assert result.reason == "left_joint7: prediction outside hard limits"


def test_admission_evidence_reuses_raw_admitted_and_correction_fields(tmp_path: Path):
    raw = np.zeros((60, 16), dtype=np.float32)
    raw[:, 7], raw[:, 15] = 0.08001, -0.0029
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, GRIPPER_QUOTA)
    assert result.accepted
    path, size = save_prediction(
        tmp_path, "session:request", raw, result, bytes_used=0, max_bytes=100000
    )
    with np.load(path, allow_pickle=False) as stored:
        np.testing.assert_array_equal(stored["raw_act"], raw)
        np.testing.assert_array_equal(stored["admitted"], result.positions)
        np.testing.assert_array_equal(stored["correction"], result.correction)
        assert bool(stored["accepted"]) and str(stored["reason"]) == ""
        assert set(stored.files) == {
            "raw_act",
            "admitted",
            "correction",
            "accepted",
            "reason",
        }
    assert size > 0
    with pytest.raises(ValueError, match="byte budget"):
        save_prediction(tmp_path, "too-big", raw, result, bytes_used=0, max_bytes=1)


def test_optional_temporal_arrays_are_saved_in_the_same_artifact(tmp_path: Path):
    raw = np.zeros((60, 16), dtype=np.float32)
    result = admit_action_chunk(raw, NAMES, LOWER, UPPER, GRIPPER_QUOTA)
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
