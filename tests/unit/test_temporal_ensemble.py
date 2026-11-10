"""Focused tests for absolute-time ACT temporal ensembling."""

import math

import numpy as np
import pytest
from dual_arm_learning.temporal_ensemble import (
    CHUNK_SIZE,
    MAX_HISTORY,
    PERIOD_NS,
    TemporalEnsemble,
    validate_coefficient,
)


def chunk(value: float = 0.0) -> np.ndarray:
    return np.full((CHUNK_SIZE, 16), value, dtype=np.float64)


def ramp() -> np.ndarray:
    return np.arange(CHUNK_SIZE * 16, dtype=np.float64).reshape(CHUNK_SIZE, 16)


def test_first_prediction_is_returned_exactly_with_unit_counts():
    actions = ramp().astype(np.float32)
    ensemble = TemporalEnsemble(0.01)
    result = ensemble.preview(7_000_000_000, 1, actions)
    np.testing.assert_array_equal(result.targets, actions.astype(np.float64))
    np.testing.assert_array_equal(result.counts, np.ones(CHUNK_SIZE, dtype=np.int64))
    assert result.history_observation_ids.size == 0
    assert result.history_observation_time_ns.size == 0
    assert result.targets.dtype == np.float64
    assert result.counts.dtype == np.int64
    assert ensemble.history_length == 0


@pytest.mark.parametrize("step", [2, 20])
def test_points_align_by_absolute_time_across_100ms_and_one_second(step):
    start = 10_000_000_000
    history = ramp()
    current = chunk(0.0)
    ensemble = TemporalEnsemble(0.0)
    ensemble.commit(start - step * PERIOD_NS, 11, history)

    result = ensemble.preview(start, 12, current)
    overlap = CHUNK_SIZE - step
    expected = np.zeros((CHUNK_SIZE, 16), dtype=np.float64)
    expected[:overlap] = history[step:] / 2.0
    np.testing.assert_allclose(result.targets, expected)
    np.testing.assert_array_equal(result.counts[:overlap], 2)
    np.testing.assert_array_equal(result.counts[overlap:], 1)
    np.testing.assert_array_equal(result.history_observation_ids, [11])
    np.testing.assert_array_equal(result.history_observation_time_ns, [start - step * PERIOD_NS])

    committed = ensemble.commit(start, 12, current)
    np.testing.assert_allclose(committed.targets, expected)
    assert ensemble.history_length == 2


def test_positive_coefficient_favours_older_prediction_and_zero_is_equal_weight():
    start = 1_000_000_000
    older = chunk(1.0)
    current = chunk(0.0)

    positive = TemporalEnsemble(0.5)
    positive.commit(start - PERIOD_NS, 1, older)
    weighted = positive.preview(start, 2, current)
    weight = math.exp(0.5)
    expected = weight / (1.0 + weight)
    assert weighted.targets[0, 0] == pytest.approx(expected)
    assert weighted.targets[0, 0] > 0.5
    assert weighted.targets[-1, 0] == 0.0

    neutral = TemporalEnsemble(0.0)
    neutral.commit(start - PERIOD_NS, 1, older)
    equal = neutral.preview(start, 2, current)
    assert equal.targets[0, 0] == pytest.approx(0.5)


def test_each_future_point_is_normalised_independently_and_tail_uses_new_prediction():
    start = 5_000_000_000
    ensemble = TemporalEnsemble(0.0)
    ensemble.commit(start - 2 * PERIOD_NS, 1, chunk(2.0))
    ensemble.commit(start - PERIOD_NS, 2, chunk(1.0))

    result = ensemble.preview(start, 3, chunk(0.0))
    np.testing.assert_allclose(result.targets[:58], 1.0)
    np.testing.assert_allclose(result.targets[58], 0.5)
    np.testing.assert_allclose(result.targets[59], 0.0)
    np.testing.assert_array_equal(result.counts[:58], 3)
    np.testing.assert_array_equal(result.counts[58], 2)
    np.testing.assert_array_equal(result.counts[59], 1)
    np.testing.assert_array_equal(result.history_observation_ids, [1, 2])


def test_convex_average_preserves_exact_boundary_values():
    ensemble = TemporalEnsemble(0.01)
    boundary = np.full((60, 16), 0.08, dtype=np.float64)
    for step in range(65):
        result = ensemble.commit(step * PERIOD_NS, step + 1, boundary)
        np.testing.assert_array_equal(result.targets, boundary)


def test_history_at_or_beyond_three_seconds_is_discarded():
    start = 20_000_000_000
    ensemble = TemporalEnsemble(0.01)
    ensemble.commit(start, 1, chunk(0.0))
    ensemble.commit(start + 1_000_000_000, 2, chunk(1.0))

    result = ensemble.commit(start + 4_000_000_000, 3, chunk(2.0))
    assert ensemble.history_length == 1
    assert result.history_observation_ids.size == 0
    np.testing.assert_array_equal(result.counts, 1)
    np.testing.assert_allclose(result.targets, 2.0)


def test_history_is_bounded_and_only_recent_observations_contribute():
    start = 30_000_000_000
    ensemble = TemporalEnsemble(0.01)
    for index in range(MAX_HISTORY + 10):
        ensemble.commit(start + index * PERIOD_NS, index + 1, chunk(float(index)))

    assert ensemble.history_length == MAX_HISTORY
    result = ensemble.preview(
        start + (MAX_HISTORY + 10) * PERIOD_NS,
        MAX_HISTORY + 11,
        chunk(100.0),
    )
    assert result.counts.max() == MAX_HISTORY
    assert result.history_observation_ids.size == MAX_HISTORY - 1


def test_preview_does_not_commit_but_commit_rejects_repeated_time():
    start = 2_000_000_000
    ensemble = TemporalEnsemble(0.01)
    ensemble.preview(start, 1, chunk(0.0))
    assert ensemble.history_length == 0
    ensemble.commit(start, 1, chunk(0.0))
    with pytest.raises(ValueError, match="does not advance"):
        ensemble.preview(start, 2, chunk(0.0))


@pytest.mark.parametrize(
    "actions",
    [
        np.zeros((CHUNK_SIZE - 1, 16)),
        np.zeros((CHUNK_SIZE, 15)),
        np.full((CHUNK_SIZE, 16), np.nan),
        np.full((CHUNK_SIZE, 16), np.inf),
    ],
)
def test_invalid_chunks_are_rejected(actions):
    with pytest.raises(ValueError):
        TemporalEnsemble(0.0).preview(0, 1, actions)


@pytest.mark.parametrize("value", [True, "0.1", float("nan"), float("inf"), -0.1, 1.1])
def test_invalid_coefficients_are_rejected(value):
    with pytest.raises(ValueError):
        TemporalEnsemble(value)
    with pytest.raises(ValueError):
        validate_coefficient(value)


def test_time_must_be_integer_strictly_increasing_and_on_the_50ms_grid():
    start = 3_000_000_000

    for value in (True, 1.0, "3000000000"):
        with pytest.raises(ValueError, match="integer"):
            TemporalEnsemble(0.0).preview(value, 1, chunk())

    ensemble = TemporalEnsemble(0.0)
    ensemble.commit(start, 1, chunk())
    with pytest.raises(ValueError, match="does not advance"):
        ensemble.preview(start - PERIOD_NS, 2, chunk())
    with pytest.raises(ValueError, match="50 ms grid"):
        ensemble.preview(start + 1, 2, chunk())
    with pytest.raises(ValueError, match="50 ms grid"):
        ensemble.commit(start + 1, 2, chunk())
    assert ensemble.history_length == 1
