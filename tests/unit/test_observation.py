from dataclasses import replace

import pytest
from dual_arm_runtime.observation import ObservationBuffer
from robot_contracts.models import ContractError


def make_buffer(context):
    buffer = ObservationBuffer(("front",), max_skew_ns=20, max_age_ns=100)
    buffer.set_context(context)
    return buffer


def test_assembly_uses_latest_past_frame(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 90)
    buffer.add("front", context, 2, 110)
    observation = buffer.assemble(120)
    assert observation.frames["front"].sequence == 1
    assert buffer.assemble(120).observation_id == observation.observation_id


def test_only_future_or_too_old_frames_rejected(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 50)
    buffer.add("front", context, 2, 110)
    with pytest.raises(ContractError, match="NOT_READY"):
        buffer.assemble(120)


def test_stale_state_rejected(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 100)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(201)


def test_new_context_clears_samples_and_rejects_late_frames(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 100)
    newer = replace(context, clock_epoch=1, episode_id="next")
    buffer.set_context(newer)
    assert not buffer.add("front", context, 2, 110)
    with pytest.raises(ContractError, match="NOT_READY"):
        buffer.assemble(120)
    buffer.add("state", newer, 1, 0)
    buffer.add("front", newer, 1, 0)
    assert buffer.assemble(1).context == newer


def test_duplicate_or_reversed_sample_rejected(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    assert buffer.add("state", context, 2, 100)
    assert not buffer.add("state", context, 2, 100)
    assert not buffer.add("state", context, 1, 101)
    assert not buffer.add("state", context, 3, 99)
    assert buffer.dropped_samples == 3
