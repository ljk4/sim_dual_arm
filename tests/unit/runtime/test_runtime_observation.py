import pytest
from dual_arm_runtime.observation import ExactCaptureBuffer
from model_helpers import replace
from robot_contracts.models import ContractError

CAMS = ("global", "left", "right")


def make_buffer(context, *, capacity=8, wall_timeout=500):
    buffer = ExactCaptureBuffer(
        CAMS, max_age_ns=1000, wall_arrival_timeout_ns=wall_timeout, capacity=capacity
    )
    buffer.set_context(context)
    return buffer


def capture(buffer, context, sequence, stamp, *, payload="same", arrival=None):
    buffer.add(
        "state",
        context,
        sequence,
        stamp,
        (payload, "state"),
        wall_arrival_ns=stamp if arrival is None else arrival,
        fingerprint=(stamp, payload),
    )
    for name in CAMS:
        buffer.add(
            name,
            context,
            sequence,
            stamp,
            (payload, name),
            wall_arrival_ns=stamp if arrival is None else arrival,
            fingerprint=(stamp, payload),
        )


def test_complete_capture_is_stable_across_reads(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    capture(buffer, context, 1, 100)
    first = buffer.assemble(110, 110)
    second = buffer.assemble(120, 120)
    assert first.observation_id == 1
    assert second.observation_id == first.observation_id
    assert second.state is first.state
    assert second.frames["global"].payload is first.frames["global"].payload
    assert buffer.stats().published == 1


def test_new_state_waits_for_its_own_frames_not_old_ones(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    capture(buffer, context, 1, 100)
    assert buffer.assemble(100, 100).observation_id == 1

    buffer.add(
        "state", context, 2, 200, ("new", "state"), wall_arrival_ns=200, fingerprint=(200, "new")
    )
    # The newest complete capture is still the old one; the new state is never
    # paired with the old frames and no second observation is allocated.
    assert buffer.assemble(200, 200).observation_id == 1
    for name in CAMS:
        buffer.add(
            name, context, 2, 200, ("new", name), wall_arrival_ns=200, fingerprint=(200, "new")
        )
    second = buffer.assemble(201, 201)
    assert second.observation_id == 2
    assert second.state.sequence == 2
    assert {sample.sequence for sample in second.frames.values()} == {2}


def test_late_old_frame_does_not_replace_latest_complete_capture(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 2, 200, "s2", wall_arrival_ns=200, fingerprint=(200, "s2"))
    for name in CAMS:
        buffer.add(name, context, 2, 200, name, wall_arrival_ns=200, fingerprint=(200, "f2"))
    buffer.add("global", context, 1, 100, "old", wall_arrival_ns=201, fingerprint=(100, "old"))
    observation = buffer.assemble(202, 202)
    assert observation.state.sequence == 2
    assert observation.frames["global"].sequence == 2


def test_out_of_order_duplicate_and_conflict_are_counted(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100, "s1", wall_arrival_ns=100, fingerprint=(100, "s1"))
    buffer.add("right", context, 1, 100, "r1", wall_arrival_ns=100, fingerprint=(100, "r1"))
    buffer.add("left", context, 1, 100, "l1", wall_arrival_ns=100, fingerprint=(100, "l1"))
    buffer.add("global", context, 1, 100, "g1", wall_arrival_ns=100, fingerprint=(100, "g1"))
    assert (
        buffer.add("global", context, 1, 100, "g1", wall_arrival_ns=100, fingerprint=(100, "g1"))
        is False
    )
    assert (
        buffer.add(
            "global", context, 1, 100, "tampered", wall_arrival_ns=100, fingerprint=(100, "bad")
        )
        is False
    )
    stats = buffer.stats()
    assert stats.duplicates == 1
    assert stats.conflicts == 1
    assert "same sequence" in buffer.conflict_reason
    assert buffer.assemble(101, 101).frames["global"].payload == "g1"


def test_incomplete_group_is_never_published(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100, "s1", wall_arrival_ns=100, fingerprint=(100, "s1"))
    buffer.add("global", context, 1, 100, "g1", wall_arrival_ns=100, fingerprint=(100, "g1"))
    buffer.add("left", context, 1, 100, "l1", wall_arrival_ns=100, fingerprint=(100, "l1"))
    with pytest.raises(ContractError, match="NOT_READY"):
        buffer.assemble(100, 100)
    assert buffer.stats().published == 0


def test_wrong_stamp_or_sequence_is_not_paired(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    buffer.add("state", context, 1, 100, "s1", wall_arrival_ns=100, fingerprint=(100, "s1"))
    buffer.add("global", context, 1, 100, "g1", wall_arrival_ns=100, fingerprint=(100, "g1"))
    buffer.add("left", context, 1, 100, "l1", wall_arrival_ns=100, fingerprint=(100, "l1"))
    buffer.add("right", context, 1, 101, "r1", wall_arrival_ns=100, fingerprint=(101, "r1"))
    with pytest.raises(ContractError, match="NOT_READY"):
        buffer.assemble(101, 101)


def test_capacity_evicts_oldest_and_keeps_bounded(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, capacity=2)
    capture(buffer, context, 1, 100)
    capture(buffer, context, 2, 101)
    capture(buffer, context, 3, 102)
    assert buffer.stats().evicted == 4
    observation = buffer.assemble(103, 103)
    assert observation.state.sequence == 3
    assert observation.observation_id == 1


def test_context_change_rejects_and_resets_identity(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context)
    capture(buffer, context, 1, 100)
    assert buffer.assemble(100, 100).observation_id == 1
    other = replace(context, clock_epoch=context.clock_epoch + 1)
    assert buffer.add("state", other, 2, 200, "x", wall_arrival_ns=200) is False
    assert buffer.stats().rejected_context == 1
    buffer.set_context(other)
    capture(buffer, other, 1, 300)
    assert buffer.assemble(300, 300).observation_id == 1


def test_incomplete_capture_also_expires_on_the_wall_clock(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, wall_timeout=50)
    for sequence, stamp in ((1, 100), (2, 101), (3, 102)):
        buffer.add(
            "state",
            context,
            sequence,
            stamp,
            f"s{sequence}",
            wall_arrival_ns=stamp,
            fingerprint=(stamp, sequence),
        )
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(102, 160)


def test_wall_arrival_deadline_is_independent_of_simulation_clock(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, wall_timeout=50)
    capture(buffer, context, 1, 100, arrival=100)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(101, 151)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(1101, 1101)


def test_live_identity_survives_other_stream_activity(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, capacity=4)
    for sequence in range(1, 5):
        capture(buffer, context, sequence, sequence * 10)
    assert (
        buffer.add("state", context, 2, 20, "tampered", wall_arrival_ns=100, fingerprint="tampered")
        is False
    )
    assert buffer.stats().conflicts == 1
    assert buffer.assemble(40, 40).observation_id == 1


def test_late_old_state_does_not_evict_newer_useful_state(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, capacity=2)
    buffer.add("state", context, 10, 100, "s10", wall_arrival_ns=100, fingerprint=10)
    buffer.add("state", context, 11, 110, "s11", wall_arrival_ns=100, fingerprint=11)
    assert buffer.add("state", context, 9, 90, "s9", wall_arrival_ns=100, fingerprint=9) is False
    assert buffer.stats().dropped_old == 1
    for name in CAMS:
        buffer.add(name, context, 10, 100, name, wall_arrival_ns=100, fingerprint=10)
    observation = buffer.assemble(110, 100)
    assert observation.state_seq == 10


def test_expired_camera_component_does_not_revive_with_new_state(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, wall_timeout=500)
    for name in CAMS:
        buffer.add(name, context, 1, 100, name, wall_arrival_ns=0, fingerprint=100)
    buffer.add("state", context, 1, 100, "state", wall_arrival_ns=1000, fingerprint=100)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(100, 1000)


def test_published_capture_never_rolls_back_to_fresh_older_capture(controlled):
    _, context, _ = controlled
    buffer = make_buffer(context, wall_timeout=500)
    capture(buffer, context, 2, 200, arrival=0)
    first = buffer.assemble(200, 0)
    assert first.state_seq == 2
    capture(buffer, context, 1, 100, arrival=800)
    with pytest.raises(ContractError, match="STALE_OBSERVATION"):
        buffer.assemble(200, 1000)
    assert buffer.assemble(200, 0).observation_id == first.observation_id
