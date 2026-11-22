import pytest
from dual_arm_runtime.observation import ObservationBuffer, SampleRef
from model_helpers import replace
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


def test_late_frame_does_not_change_allocated_observation(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 90)
    first = buffer.assemble(100)
    assert first.observation_id == 1
    assert first.frames["front"].sequence == 1

    assert buffer.add("front", context, 2, 95)
    second = buffer.assemble(105)
    assert second.observation_id == first.observation_id
    assert second.state == first.state
    assert second.frames["front"] == first.frames["front"]


def test_new_state_gets_new_id_and_newest_eligible_frame(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 90)
    first = buffer.assemble(100)
    buffer.add("front", context, 2, 95)
    buffer.add("state", context, 2, 110)
    buffer.add("front", context, 3, 120)

    second = buffer.assemble(110)
    assert second.observation_id == first.observation_id + 1
    assert second.state.sequence == 2
    assert second.state.stamp_ns == 110
    assert second.frames["front"].sequence == 2
    assert list(second.frames) == ["front"]
    assert [sample.sequence for sample in second.frames.values()] == [2]


def test_cached_observation_does_not_bypass_stale_check(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 100)
    buffer.assemble(100)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(201)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(99)


def test_missing_frame_does_not_consume_observation_id(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    with pytest.raises(ContractError, match="NOT_READY"):
        buffer.assemble(100)
    buffer.add("front", context, 1, 95)

    observation = buffer.assemble(101)
    assert observation.observation_id == 1
    assert observation.frames["front"].sequence == 1


def test_caller_cannot_mutate_allocated_frame_references(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100)
    buffer.add("front", context, 1, 90)
    first = buffer.assemble(100)
    first.frames["front"] = SampleRef(99, 99)

    second = buffer.assemble(100)
    assert second.observation_id == first.observation_id
    assert second.frames["front"].sequence == 1
    assert second.frames["front"].stamp_ns == 90


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
    assert buffer.assemble(100).observation_id == 1
    newer = replace(context, clock_epoch=1, episode_id="next")
    buffer.set_context(newer)
    assert not buffer.add("front", context, 2, 110)
    with pytest.raises(ContractError, match="NOT_READY"):
        buffer.assemble(120)
    buffer.add("state", newer, 1, 0)
    buffer.add("front", newer, 1, 0)
    observation = buffer.assemble(1)
    assert observation.context == newer
    assert observation.observation_id == 1


def test_duplicate_or_reversed_sample_rejected(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    assert buffer.add("state", context, 2, 100)
    assert not buffer.add("state", context, 2, 100)
    assert not buffer.add("state", context, 1, 101)
    assert not buffer.add("state", context, 3, 99)
    assert buffer.dropped_samples == 3
