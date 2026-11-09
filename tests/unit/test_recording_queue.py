"""Behavior tests for the bounded MCAP write-behind queue."""

import threading
import time
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.native_config import load_native_configuration
from dual_arm_runtime.recording_queue import (
    BoundedRecordingQueue,
    RecordingQueueError,
    RecordingSettings,
)
from robot_contracts.models import ContractError

TOPICS = {"/state": "RobotState", "/frame": "CameraFrame"}
ROOT = Path(__file__).resolve().parents[2]
BASE_CONFIG = ROOT / "runtime/ros2/configs/native_teacher.yaml"


def _resolve_paths(document, source: Path) -> None:
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_constraints",
    ):
        if field in document:
            document[field] = str((source.parent / document[field]).resolve())


class _Writer:
    def __init__(self, *, open_hook=None, write_hook=None, close_hook=None):
        self.open_hook = open_hook
        self.write_hook = write_hook
        self.close_hook = close_hook
        self.opened = False
        self.closed = False
        self.writes = []

    def open(self):
        if self.open_hook is not None:
            self.open_hook()
        self.opened = True

    def write(self, topic, payload, timestamp_ns):
        if self.write_hook is not None:
            self.write_hook(topic, payload, timestamp_ns)
        self.writes.append((topic, payload, timestamp_ns))

    def close(self):
        if self.close_hook is not None:
            self.close_hook()
        self.closed = True


def _queue(settings, writer, name):
    return BoundedRecordingQueue(settings, lambda: writer, TOPICS, thread_name=name)


def _wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_settings_reject_bool_and_non_positive_values():
    with pytest.raises(ValueError):
        RecordingSettings(queue_capacity=True)
    with pytest.raises(ValueError):
        RecordingSettings(max_pending_bytes=0)
    with pytest.raises(ValueError):
        RecordingSettings(startup_timeout_s=0)
    with pytest.raises(ValueError):
        RecordingSettings(close_timeout_s=float("inf"))


def test_normal_and_multi_producer_writes_are_complete_and_ordered_per_producer():
    writer = _Writer()
    producers = 4
    per_producer = 25
    # This test verifies ordering. Allow the entire finite burst even if the
    # scheduler runs every producer before the writer; overflow has its own tests.
    queue = _queue(
        RecordingSettings(queue_capacity=producers * per_producer, max_pending_bytes=1 << 20),
        writer,
        "test-quiet-writer",
    )

    def produce(producer):
        for sequence in range(per_producer):
            queue.append("/state", f"{producer}:{sequence}".encode(), sequence)

    threads = [threading.Thread(target=produce, args=(index,)) for index in range(producers)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    stats = queue.close(2.0)

    assert stats.accepted == stats.written == producers * per_producer
    assert stats.pending_items == stats.pending_bytes == 0
    assert stats.failure_reason == ""
    assert len(writer.writes) == producers * per_producer
    for producer in range(producers):
        observed = [
            int(payload.split(b":")[1])
            for topic, payload, _ in writer.writes
            if payload.startswith(f"{producer}:".encode())
        ]
        assert observed == list(range(per_producer))
    assert all(topic == "/state" for topic, _, _ in writer.writes)
    assert writer.closed
    assert queue.close(2.0).written == stats.written


def test_count_capacity_includes_the_item_currently_being_written():
    entered = threading.Event()
    release = threading.Event()

    def slow_write(*_):
        entered.set()
        assert release.wait(2.0)

    writer = _Writer(write_hook=slow_write)
    queue = _queue(
        RecordingSettings(queue_capacity=2, max_pending_bytes=1 << 20),
        writer,
        "test-count-writer",
    )
    queue.append("/state", b"one", 1)
    assert entered.wait(1.0)
    queue.append("/state", b"two", 2)
    with pytest.raises(RecordingQueueError, match="full"):
        queue.append("/state", b"three", 3)
    assert queue.stats().pending_items == 2
    release.set()
    with pytest.raises(RecordingQueueError, match="full"):
        queue.check_health()
    with pytest.raises(RecordingQueueError, match="full"):
        queue.close(2.0)
    stats = queue.stats()
    assert stats.accepted == 2
    assert stats.written == 1
    assert stats.pending_items == 1


def test_byte_budget_keeps_in_flight_payload_until_write_returns():
    entered = threading.Event()
    release = threading.Event()

    def slow_write(*_):
        entered.set()
        assert release.wait(2.0)

    writer = _Writer(write_hook=slow_write)
    queue = _queue(
        RecordingSettings(queue_capacity=10, max_pending_bytes=100),
        writer,
        "test-byte-writer",
    )
    queue.append("/frame", b"a" * 60, 1)
    assert entered.wait(1.0)
    with pytest.raises(RecordingQueueError, match="full"):
        queue.append("/frame", b"b" * 50, 2)
    assert queue.stats().pending_bytes == 60
    release.set()
    with pytest.raises(RecordingQueueError, match="full"):
        queue.close(2.0)
    assert queue.stats().written == 1


def test_single_payload_larger_than_budget_fails_without_enqueueing():
    writer = _Writer()
    queue = _queue(
        RecordingSettings(max_pending_bytes=10),
        writer,
        "test-oversize-writer",
    )
    with pytest.raises(RecordingQueueError, match="exceeds max_pending_bytes"):
        queue.append("/frame", b"x" * 11, 1)
    with pytest.raises(RecordingQueueError, match="exceeds max_pending_bytes"):
        queue.check_health()
    with pytest.raises(RecordingQueueError, match="exceeds max_pending_bytes"):
        queue.close(2.0)
    assert queue.stats().accepted == queue.stats().written == 0


@pytest.mark.parametrize(
    ("arguments", "needle"),
    [
        (("/other", b"x", 1), "unregistered"),
        (("/state", b"x", True), "timestamp"),
        (("/state", b"x", -1), "timestamp"),
        (("/state", bytearray(b"x"), 1), "payload"),
    ],
)
def test_invalid_topic_time_and_payload_are_permanent_failures(arguments, needle):
    queue = _queue(RecordingSettings(), _Writer(), "test-invalid-writer")
    with pytest.raises(RecordingQueueError, match=needle):
        queue.append(*arguments)
    with pytest.raises(RecordingQueueError):
        queue.check_health()
    with pytest.raises(RecordingQueueError):
        queue.close(2.0)


def test_writer_failure_is_latched_and_repeated_close_keeps_failing():
    def broken_write(*_):
        raise OSError("disk gone")

    writer = _Writer(write_hook=broken_write)
    queue = _queue(RecordingSettings(), writer, "test-failure-writer")
    queue.append("/state", b"x", 1)
    assert _wait_for(lambda: _health_error(queue) is not None)
    error = _health_error(queue)
    assert "disk gone" in error
    with pytest.raises(RecordingQueueError, match="disk gone"):
        queue.append("/state", b"y", 2)
    with pytest.raises(RecordingQueueError, match="disk gone"):
        queue.close(2.0)
    with pytest.raises(RecordingQueueError, match="disk gone"):
        queue.close(2.0)
    assert writer.closed


def _health_error(queue):
    try:
        queue.check_health()
    except RecordingQueueError as error:
        return str(error)
    return None


def test_close_timeout_is_bounded_and_repeated_close_stays_failed():
    entered = threading.Event()
    release = threading.Event()

    def blocked_write(*_):
        entered.set()
        assert release.wait(5.0)

    writer = _Writer(write_hook=blocked_write)
    queue = _queue(RecordingSettings(), writer, "test-timeout-writer")
    queue.append("/frame", b"data", 1)
    assert entered.wait(1.0)
    started = time.monotonic()
    with pytest.raises(RecordingQueueError, match="timed out"):
        queue.close(0.1)
    assert time.monotonic() - started < 1.0
    with pytest.raises(RecordingQueueError):
        queue.append("/frame", b"late", 2)
    release.set()
    with pytest.raises(RecordingQueueError, match="timed out"):
        queue.close(2.0)
    assert writer.closed
    assert not any(
        thread.name == "test-timeout-writer" and thread.is_alive()
        for thread in threading.enumerate()
    )


def test_append_during_close_is_rejected_and_close_still_completes():
    entered = threading.Event()
    release = threading.Event()

    def slow_write(*_):
        entered.set()
        assert release.wait(2.0)

    writer = _Writer(write_hook=slow_write)
    queue = _queue(RecordingSettings(), writer, "test-close-race-writer")
    queue.append("/state", b"first", 1)
    assert entered.wait(1.0)
    closed = {}

    def close():
        try:
            closed["stats"] = queue.close(2.0)
        except Exception as error:
            closed["error"] = error

    closer = threading.Thread(target=close)
    closer.start()
    time.sleep(0.05)
    with pytest.raises(RecordingQueueError):
        queue.append("/state", b"late", 2)
    release.set()
    closer.join(2.0)
    assert not closer.is_alive()
    assert isinstance(closed.get("error"), RecordingQueueError)
    assert queue.stats().written == 1
    with pytest.raises(RecordingQueueError):
        queue.check_health()


def test_startup_timeout_is_bounded_and_writer_thread_ends():
    release = threading.Event()
    entered = threading.Event()

    def blocked_open():
        entered.set()
        assert release.wait(2.0)

    writer = _Writer(open_hook=blocked_open)
    name = "test-startup-writer"
    with pytest.raises(RecordingQueueError, match="startup timed out"):
        BoundedRecordingQueue(
            RecordingSettings(startup_timeout_s=0.1),
            lambda: writer,
            TOPICS,
            thread_name=name,
        )
    assert entered.is_set()
    release.set()
    assert _wait_for(
        lambda: not any(
            thread.name == name and thread.is_alive() for thread in threading.enumerate()
        )
    )
    assert writer.closed


def test_configuration_fills_recording_defaults_into_snapshot(tmp_path):
    document = yaml.safe_load(BASE_CONFIG.read_text())
    _resolve_paths(document, BASE_CONFIG)
    document.pop("recording")
    path = tmp_path / "runtime-without-recording.yaml"
    path.write_text(yaml.safe_dump(document))

    configuration = load_native_configuration(path, ROOT)

    expected = RecordingSettings()
    assert configuration.recipe.recording == expected
    assert configuration.snapshot["recipe"]["recording"] == {
        "queue_capacity": 1024,
        "max_pending_bytes": 67108864,
        "startup_timeout_s": 5.0,
        "close_timeout_s": 5.0,
    }


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("queue_capacity", True),
        ("queue_capacity", 0),
        ("max_pending_bytes", -1),
        ("startup_timeout_s", True),
        ("close_timeout_s", 0),
    ],
)
def test_configuration_rejects_invalid_recording_limits(tmp_path, field, value):
    document = yaml.safe_load(BASE_CONFIG.read_text())
    _resolve_paths(document, BASE_CONFIG)
    document["recording"][field] = value
    path = tmp_path / "runtime-invalid-recording.yaml"
    path.write_text(yaml.safe_dump(document))

    with pytest.raises(ContractError, match="SCHEMA_MISMATCH"):
        load_native_configuration(path, ROOT)


def test_explicit_producer_fail_latches_and_rejects_later_appends():
    queue = _queue(RecordingSettings(), _Writer(), "test-explicit-fail-writer")
    queue.fail("producer rejected")
    with pytest.raises(RecordingQueueError, match="producer rejected"):
        queue.append("/state", b"x", 1)
    with pytest.raises(RecordingQueueError, match="producer rejected"):
        queue.check_health()
    with pytest.raises(RecordingQueueError, match="producer rejected"):
        queue.close(2.0)
